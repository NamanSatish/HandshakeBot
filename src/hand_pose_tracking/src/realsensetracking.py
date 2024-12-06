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

import pyrealsense2 as rs


last_timestamp = 0
rendered_image = None

BaseOptions = mp.tasks.BaseOptions
HandLandmarker = mp.tasks.vision.HandLandmarker
HandLandmarkerOptions = mp.tasks.vision.HandLandmarkerOptions
HandLandmarkerResult = mp.tasks.vision.HandLandmarkerResult
VisionRunningMode = mp.tasks.vision.RunningMode

global wrist_points
wrist_points = np.zeros((0,3))

timestamps = []

depths = {}

def draw_hand_landmarks_on_image(rgb_image, detection_result):
  hand_landmarks_list = detection_result.hand_landmarks  
  annotated_image = np.copy(rgb_image)

  if hand_landmarks_list is not None:
    # Loop through the detected hands to visualize.
    for idx in range(len(hand_landmarks_list)):
        hand_landmarks = hand_landmarks_list[idx]

        # Draw the hand landmarks.
        hand_landmarks_proto = landmark_pb2.NormalizedLandmarkList()
        hand_landmarks_proto.landmark.extend([
        landmark_pb2.NormalizedLandmark(x=landmark.x, y=landmark.y, z=landmark.z) for landmark in hand_landmarks
        ])
        solutions.drawing_utils.draw_landmarks(
        annotated_image,
        hand_landmarks_proto,
        solutions.hands.HAND_CONNECTIONS,
        solutions.drawing_styles.get_default_hand_landmarks_style(),
        solutions.drawing_styles.get_default_hand_connections_style())
    
  return annotated_image

def callback(result: HandLandmarkerResult, output_image: mp.Image, timestamp_ms: int):
    global rendered_image
    global wrist_points
    
    try:
        rendered_image = draw_hand_landmarks_on_image(output_image.numpy_view(), result)
        wrist_points[timestamp_ms % 50] = [result.hand_landmarks[0][0].x, result.hand_landmarks[0][0].y, result.hand_landmarks[0][0].z]

    except Exception as e:
        logging.error(f"Error in callback function: {e}")

def get3DCoord(pts, intrinsics):
    x = pts[:, 2] * (pts[:, 0] - intrinsics.ppx) / intrinsics.fx
    y = pts[:, 2] * (pts[:, 1] - intrinsics.ppy) / intrinsics.fy
    coords = np.vstack([x, y, pts[:, 2]]).T
    return coords



options = HandLandmarkerOptions(
    base_options=BaseOptions(model_asset_path='hand_landmarker.task'),
    running_mode= VisionRunningMode.VIDEO,
    num_hands = 1
)


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
            print(depth_im.shape)
            if len(result.hand_landmarks) > 0:
                x = int(result.hand_landmarks[0][0].x * 640)
                y = int(result.hand_landmarks[0][0].y * 480)
                print(result.hand_landmarks[0][0].x, result.hand_landmarks[0][0].y)
                print(x, y)
                if x >= 0 and x < 640 and y >= 0 and y < 480:
                    wrist_points = np.vstack([wrist_points, [640*result.hand_landmarks[0][0].x, 480*result.hand_landmarks[0][0].y, result.hand_landmarks[0][0].z + depth_im[y, x]]])

           
            if rendered_image is not None:
                cv2.imshow('hand landmarks', cv2.cvtColor(rendered_image, cv2.COLOR_RGB2BGR))

            key = cv2.waitKey(1)
            if key == 27:
                #3d plot wrist points
                #print(min(timestamps), max(timestamps))
                
                n_pts = get3DCoord(wrist_points, intrinsics=intrinsics)
                print(n_pts.shape)
                print(n_pts[:, 2])
                fig = plt.figure()
                axes = fig.add_subplot(projection='3d')
                axes.plot3D(n_pts[:,0], n_pts[:,1], n_pts[:,2], marker='o')
                #Set all axis to have the same scale
                axes.set_box_aspect([np.ptp(n_pts[:,0]), np.ptp(n_pts[:,1]), np.ptp(n_pts[:,2])])
                #Color x, y, z axis
                axes.set_xlabel('X')
                axes.set_ylabel('Y')
                axes.set_zlabel('Z')

                
                plt.show()
                break
    except Exception as e:
        print(e)
        pass