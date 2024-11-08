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

# Create a face landmarker instance with the live stream mode:
def callback(result: HandLandmarkerResult, output_image: mp.Image, timestamp_ms: int):
    global rendered_image
    try:
        rendered_image = draw_hand_landmarks_on_image(output_image.numpy_view(), result)
    except Exception as e:
        logging.error(f"Error in callback function: {e}")

options = HandLandmarkerOptions(
    base_options=BaseOptions(model_asset_path='hand_landmarker.task'),
    running_mode= VisionRunningMode.LIVE_STREAM,
    num_hands = 2,
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

        image = cv2.flip(image, 1)
        mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=image)

        timestamp = time.time()
        if timestamp <= last_timestamp:
           continue

        landmarker.detect_async(mp_image, mp.Timestamp.from_seconds(timestamp).value)
        last_timestamp = timestamp

        if rendered_image is not None:
            cv2.imshow('hand landmarks', rendered_image)

        key = cv2.waitKey(1)
        if key == 27:
            sys.exit(1)