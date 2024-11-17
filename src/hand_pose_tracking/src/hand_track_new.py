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

last_timestamp = 0
rendered_image = None

BaseOptions = mp.tasks.BaseOptions
HandLandmarker = mp.tasks.vision.HandLandmarker
HandLandmarkerOptions = mp.tasks.vision.HandLandmarkerOptions
HandLandmarkerResult = mp.tasks.vision.HandLandmarkerResult
VisionRunningMode = mp.tasks.vision.RunningMode

global wrist_points 
wrist_points = np.zeros((0,3))

global pointer_points
pointer_points = np.zeros((0,3))

global wrist_points_world
wrist_points_world = np.zeros((0,3))

global pointer_points_world
pointer_points_world = np.zeros((0,3))
# Create a face landmarker instance with the live stream mode:
def callback(result: HandLandmarkerResult, output_image: mp.Image, timestamp_ms: int):
    global rendered_image
    global wrist_points
    global pointer_points
    global wrist_points_world
    global pointer_points_world
    try:
        rendered_image = draw_hand_landmarks_on_image(output_image.numpy_view(), result)
        print(result)
        wrist_points = np.vstack([wrist_points, [result.hand_landmarks[0][0].x, result.hand_landmarks[0][0].y, result.hand_landmarks[0][0].z]])
        pointer_points = np.vstack([pointer_points, [result.hand_landmarks[0][8].x, result.hand_landmarks[0][8].y, result.hand_landmarks[0][8].z]])
        wrist_points_world = np.vstack([wrist_points_world, [result.hand_world_landmarks[0][0].x, result.hand_world_landmarks[0][0].y, result.hand_world_landmarks[0][0].z]])
        pointer_points_world = np.vstack([pointer_points_world, [result.hand_world_landmarks[0][8].x, result.hand_world_landmarks[0][8].y, result.hand_world_landmarks[0][8].z]])
    except Exception as e:
        logging.error(f"Error in callback function: {e}")

options = HandLandmarkerOptions(
    base_options=BaseOptions(model_asset_path='hand_landmarker.task'),
    running_mode= VisionRunningMode.LIVE_STREAM,
    num_hands = 1,
    result_callback=callback
)

with HandLandmarker.create_from_options(options) as landmarker:
    cap = cv2.VideoCapture(0)
    cap.set(6, cv2.VideoWriter.fourcc('M', 'J', 'P', 'G'))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    print("Hand Landmarker created")
    while cap.isOpened():
        start_time = time.time()

        success, image = cap.read()
        if not success:
            print("Ignoring empty camera frame.")
            continue

        #image = cv2.flip(image, 1)
        mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=cv2.cvtColor(image, cv2.COLOR_BGR2RGB))

        timestamp = time.time()
        if timestamp <= last_timestamp:
           continue

        landmarker.detect_async(mp_image, mp.Timestamp.from_seconds(timestamp).value)
        last_timestamp = timestamp

        if rendered_image is not None:
            cv2.imshow('hand landmarks', cv2.cvtColor(rendered_image, cv2.COLOR_RGB2BGR))

        key = cv2.waitKey(1)
        if key == 27:
            #3d plot wrist points
            print(wrist_points)
            fig, axes = plt.subplots(1, 2)
            axes[0] = fig.add_subplot(projection='3d')
            axes[0].plot3D(wrist_points[:,0], wrist_points[:,1], wrist_points[:,2], 'green')
            axes[0].plot3D(pointer_points[:,0], pointer_points[:,1], pointer_points[:,2], 'red')
            #Set all axis to have the same scale
            axes[0].set_box_aspect([np.ptp(wrist_points[:,0]), np.ptp(wrist_points[:,1]), np.ptp(wrist_points[:,2])])
            #Color x, y, z axis
            axes[0].set_xlabel('X')
            axes[0].set_ylabel('Y')
            axes[0].set_zlabel('Z')

            #3d plot wrist points world
            axes[1] = fig.add_subplot(projection='3d')
            axes[1].plot3D(wrist_points_world[:,0], wrist_points_world[:,1], wrist_points_world[:,2], 'green')
            axes[1].plot3D(pointer_points_world[:,0], pointer_points_world[:,1], pointer_points_world[:,2], 'red')
            #Set all axis to have the same scale
            axes[1].set_box_aspect([np.ptp(wrist_points_world[:,0]), np.ptp(wrist_points_world[:,1]), np.ptp(wrist_points_world[:,2])])
            #Color x, y, z axis
            axes[1].set_xlabel('X')
            axes[1].set_ylabel('Y')
            axes[1].set_zlabel('Z')

            
            plt.show()
            break
