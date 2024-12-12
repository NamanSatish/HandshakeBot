import mediapipe as mp
from mediapipe.tasks import python
from mediapipe.tasks.python import vision

from mediapipe import solutions
from mediapipe.framework.formats import landmark_pb2

import numpy as np
import cv2
import time
import sys
import logging
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

import pyrealsense2 as rs

import rospy
from geometry_msgs.msg import PoseStamped, PointStamped

BaseOptions = mp.tasks.BaseOptions
HandLandmarker = mp.tasks.vision.HandLandmarker
HandLandmarkerOptions = mp.tasks.vision.HandLandmarkerOptions
HandLandmarkerResult = mp.tasks.vision.HandLandmarkerResult
VisionRunningMode = mp.tasks.vision.RunningMode


def draw_hand_landmarks_on_image(rgb_image, detection_result):
    hand_landmarks_list = detection_result.hand_landmarks
    annotated_image = np.copy(rgb_image)

    if hand_landmarks_list is not None:
        # Loop through the detected hands to visualize.
        for idx in range(len(hand_landmarks_list)):
            hand_landmarks = hand_landmarks_list[idx]

            # Draw the hand landmarks.
            hand_landmarks_proto = landmark_pb2.NormalizedLandmarkList()
            hand_landmarks_proto.landmark.extend(
                [
                    landmark_pb2.NormalizedLandmark(
                        x=landmark.x, y=landmark.y, z=landmark.z
                    )
                    for landmark in hand_landmarks
                ]
            )
            solutions.drawing_utils.draw_landmarks(
                annotated_image,
                hand_landmarks_proto,
                solutions.hands.HAND_CONNECTIONS,
                solutions.drawing_styles.get_default_hand_landmarks_style(),
                solutions.drawing_styles.get_default_hand_connections_style(),
            )

    return annotated_image


def callback(result: HandLandmarkerResult, output_image: mp.Image, timestamp_ms: int):
    global rendered_image
    global wrist_points

    try:
        rendered_image = draw_hand_landmarks_on_image(output_image.numpy_view(), result)
        wrist_points[timestamp_ms % 50] = [
            result.hand_landmarks[0][0].x,
            result.hand_landmarks[0][0].y,
            result.hand_landmarks[0][0].z,
        ]

    except Exception as e:
        logging.error(f"Error in callback function: {e}")


def get3DCoord(pts, intrinsics):
    x = pts[:, 2] * (pts[:, 0] - intrinsics.ppx) / intrinsics.fx
    y = pts[:, 2] * (pts[:, 1] - intrinsics.ppy) / intrinsics.fy
    coords = np.vstack([x, y, pts[:, 2]]).T
    return coords


def get3DCoordSingle(pt, intrinsics):
    x = pt[2] * (pt[0] - intrinsics.ppx) / intrinsics.fx
    y = pt[2] * (pt[1] - intrinsics.ppy) / intrinsics.fy
    return np.array([x, y, pt[2]]) * 0.001


def publishCoord(pt, publisher):
    pose_msg = PointStamped()
    pose_msg.header.stamp = rospy.Time.now()
    # pose_msg.header.frame_id = "ar_marker_5"  #TODO Replace with your camera frame name (realsense?)

    # set position
    pose_msg.point.x = pt[0]
    pose_msg.point.y = pt[1]
    pose_msg.point.z = pt[2]

    # orientation (not applicable here, so setting it as default)
    # pose_msg.pose.orientation.x = 0.0
    # pose_msg.pose.orientation.y = 0.0
    # pose_msg.pose.orientation.z = 0.0
    # pose_msg.pose.orientation.w = 1.0
    #
    # publish the message
    publisher.publish(pose_msg)


def main():
    # init variables
    last_timestamp = 0
    rendered_image = None

    global wrist_points
    wrist_points = np.zeros((0, 3))

    timestamps = []

    depths = {}

    # Initialize the ROS node
    rospy.init_node("hand_pose_tracking")

    # Init rate
    rate = rospy.Rate(30)  # 10 Hz

    # Init options
    options = HandLandmarkerOptions(
        base_options=BaseOptions(model_asset_path="hand_landmarker.task"),
        running_mode=VisionRunningMode.VIDEO,
        num_hands=1,
    )

    # Init publisher
    pose_publisher = rospy.Publisher("/hand_pose", PointStamped, queue_size=1)

    with HandLandmarker.create_from_options(options) as landmarker:
        try:
            pipeline = rs.pipeline()

            config = rs.config()
            config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
            config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
            align = rs.align(rs.stream.color)

            pipeline.start(config)
            start_time = time.time()

            while True:

                frames = pipeline.wait_for_frames()
                frames = align.process(frames)
                depth = frames.get_depth_frame()
                color = frames.get_color_frame()

                if not depth:
                    continue
                if not color:
                    continue

                intrinsics = depth.profile.as_video_stream_profile().intrinsics

                depth_im = np.asanyarray(depth.get_data())
                color_im = np.asanyarray(color.get_data())

                print("Width:", intrinsics.width)
                print("Height:", intrinsics.height)
                print("Principal Point (PPX, PPY):", intrinsics.ppx, intrinsics.ppy)
                print("Focal Length (fx, fy):", intrinsics.fx, intrinsics.fy)
                print("Distortion Coefficients:", intrinsics.coeffs)

                mp_image = mp.Image(
                    image_format=mp.ImageFormat.SRGB,
                    data=cv2.cvtColor(color_im, cv2.COLOR_BGR2RGB),
                )

                timestamp = time.time()

                result = landmarker.detect_for_video(
                    mp_image, mp.Timestamp.from_seconds(timestamp).value
                )
                rendered_image = draw_hand_landmarks_on_image(
                    mp_image.numpy_view(), result
                )

                if len(result.hand_landmarks) > 0:
                    x = int(result.hand_landmarks[0][0].x * 640)
                    y = int(result.hand_landmarks[0][0].y * 480)

                    if x >= 0 and x < 640 and y >= 0 and y < 480:

                        wrist_points = np.vstack(
                            [
                                wrist_points,
                                [
                                    640 * result.hand_landmarks[0][0].x,
                                    480 * result.hand_landmarks[0][0].y,
                                    result.hand_landmarks[0][0].z + depth_im[y, x],
                                ],
                            ]
                        )
                        rlt_pt = get3DCoordSingle(
                            [
                                640 * result.hand_landmarks[0][0].x,
                                480 * result.hand_landmarks[0][0].y,
                                result.hand_landmarks[0][0].z + depth_im[y, x],
                            ],
                            intrinsics=intrinsics,
                        )
                        publishCoord(rlt_pt, pose_publisher)

                if rendered_image is not None:
                    cv2.imshow(
                        "hand landmarks",
                        cv2.cvtColor(rendered_image, cv2.COLOR_RGB2BGR),
                    )

                key = cv2.waitKey(1)
                if key == 27:

                    n_pts = get3DCoord(wrist_points, intrinsics=intrinsics)

                    sample_window = 2
                    m_pts = np.zeros(n_pts.shape)
                    for r in range(sample_window, n_pts.shape[0] - sample_window):
                        # print(f'[{np.median(n_pts[r - sample_window:r+sample_window,0])*0.001}, {np.median(n_pts[r - sample_window:r+sample_window,1])*0.001}, {np.median(n_pts[r - sample_window:r+sample_window,2])*0.001}],')
                        m_pts[r, 0] = (
                            np.median(n_pts[r - sample_window : r + sample_window, 0])
                            * 0.001
                        )
                        m_pts[r, 1] = (
                            np.median(n_pts[r - sample_window : r + sample_window, 1])
                            * 0.001
                        )
                        m_pts[r, 2] = (
                            np.median(n_pts[r - sample_window : r + sample_window, 2])
                            * 0.001
                        )
                    fig = plt.figure()
                    axes = fig.add_subplot(projection="3d")
                    axes.plot3D(m_pts[:, 0], m_pts[:, 1], m_pts[:, 2], marker="o")

                    # Set all axis to have the same scale
                    # axes.set_box_aspect([np.ptp(n_pts[:,0]), np.ptp(n_pts[:,1]), np.ptp(n_pts[:,2])])

                    axes.set_xlabel("X")
                    axes.set_ylabel("Y")
                    axes.set_zlabel("Z")

                    plt.show()
                    break

                rate.sleep()
        except Exception as e:
            print(e)
            pass


if __name__ == "__main__":
    main()

"""
options = HandLandmarkerOptions(
    base_options=BaseOptions(model_asset_path='hand_landmarker.task'),
    running_mode= VisionRunningMode.VIDEO,
    num_hands = 1
)

rospy.init_node('hand_pose_publisher', anonymous=True)
pose_publisher = rospy.Publisher('/hand_pose', PoseStamped, queue_size=10)


with HandLandmarker.create_from_options(options) as landmarker:
    try:
        pipeline = rs.pipeline()

        config = rs.config()
        config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        align = rs.align(rs.stream.color)

        pipeline.start(config)
        start_time = time.time()
        prev_pt = np.array([0,0,0])

        min_threshold = .05
        max_threshold = .5
        first_point = True

 
        while True:
            
            frames = pipeline.wait_for_frames()
            frames = align.process(frames)
            depth = frames.get_depth_frame()
            color = frames.get_color_frame()

            if not depth: continue
            if not color: continue

            intrinsics = depth.profile.as_video_stream_profile().intrinsics

            depth_im = np.asanyarray(depth.get_data())
            color_im = np.asanyarray(color.get_data())

            print("Width:", intrinsics.width)
            print("Height:", intrinsics.height)
            print("Principal Point (PPX, PPY):", intrinsics.ppx, intrinsics.ppy)
            print("Focal Length (fx, fy):", intrinsics.fx, intrinsics.fy)
            print("Distortion Coefficients:", intrinsics.coeffs)
                        
            
            mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=cv2.cvtColor(color_im, cv2.COLOR_BGR2RGB))

            timestamp = time.time()

            result = landmarker.detect_for_video(mp_image, mp.Timestamp.from_seconds(timestamp).value)
            rendered_image = draw_hand_landmarks_on_image(mp_image.numpy_view(), result)

            if len(result.hand_landmarks) > 0:
                x = int(result.hand_landmarks[0][0].x * 640)
                y = int(result.hand_landmarks[0][0].y * 480)
                
                if x >= 0 and x < 640 and y >= 0 and y < 480:
             
                    wrist_points = np.vstack([wrist_points, [640*result.hand_landmarks[0][0].x, 480*result.hand_landmarks[0][0].y, result.hand_landmarks[0][0].z + depth_im[y, x]]])
                    rlt_pt = get3DCoordSingle([640*result.hand_landmarks[0][0].x, 480*result.hand_landmarks[0][0].y, result.hand_landmarks[0][0].z + depth_im[y, x]], intrinsics=intrinsics)
                    print(f"rlt_pt{rlt_pt}, prev_pt {prev_pt}, distance: {np.linalg.norm(prev_pt - rlt_pt)}")
                    if ((np.linalg.norm(prev_pt - rlt_pt) > min_threshold and np.linalg.norm(prev_pt - rlt_pt) < max_threshold) or first_point):
                        publishCoord(rlt_pt, pose_publisher)
                        prev_pt = rlt_pt
                        first_point = False
                    
        
           
            if rendered_image is not None:
                cv2.imshow('hand landmarks', cv2.cvtColor(rendered_image, cv2.COLOR_RGB2BGR))

            key = cv2.waitKey(1)
            if key == 27:
                
                
                n_pts = get3DCoord(wrist_points, intrinsics=intrinsics)
                
                sample_window = 2
                m_pts = np.zeros(n_pts.shape)
                for r in range(sample_window, n_pts.shape[0] - sample_window):
                    #print(f'[{np.median(n_pts[r - sample_window:r+sample_window,0])*0.001}, {np.median(n_pts[r - sample_window:r+sample_window,1])*0.001}, {np.median(n_pts[r - sample_window:r+sample_window,2])*0.001}],')
                    m_pts[r, 0] = np.median(n_pts[r - sample_window:r+sample_window,0])*0.001
                    m_pts[r, 1] = np.median(n_pts[r - sample_window:r+sample_window,1])*0.001
                    m_pts[r, 2] = np.median(n_pts[r - sample_window:r+sample_window,2])*0.001
                fig = plt.figure()
                axes = fig.add_subplot(projection='3d')
                axes.plot3D(m_pts[:,0], m_pts[:,1], m_pts[:,2], marker='o')

                #Set all axis to have the same scale
                #axes.set_box_aspect([np.ptp(n_pts[:,0]), np.ptp(n_pts[:,1]), np.ptp(n_pts[:,2])])

                axes.set_xlabel('X')
                axes.set_ylabel('Y')
                axes.set_zlabel('Z')

                
                plt.show()
                break
    
    except Exception as e:
        print(e)
        pass
        

    # Define the translation (position of the new frame in the base frame)
    transform.transform.translation.x = 0.9168308570235706
    transform.transform.translation.y = 0.7110835508043919
    transform.transform.translation.z = 0.35409153525141457
    transform.transform.rotation.x = -0.5372949769275619
    transform.transform.rotation.y = -0.5311668177057733
    transform.transform.rotation.z = 0.47656178354926226
    transform.transform.rotation.w = 0.4495161687827037
        """
