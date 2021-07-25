import cv2
import mediapipe as mp
import rtmidi
import math
import csv
import os
from datetime import datetime


# Clean up existing files
if os.path.exists("points.csv"):
    os.remove("points.csv") 

# Init Mediapipe
mp_drawing = mp.solutions.drawing_utils
mp_holistic = mp.solutions.holistic


#########
## SET ##
## UP  ##
#########

#this should be a reference to your camera (number)
#               or a video reference (file)
#cam_ref=3
cam_ref="arm_move_f_2.mp4"


########################################################
# init empty string if we don't have wrists recognised #
########################################################

empty_json=[]
for i in range(21):
    empty_json.append("{'X': 0.0, 'Y': 0.0, 'Z': 0.0, 'Visibility': 0.0}")

##################
# Init Variables #
##################

cap = cv2.VideoCapture(cam_ref)
timestamps = [cap.get(cv2.CAP_PROP_POS_MSEC)]
calc_timestamps = [0.0]


##############################
# Loop over the video Frames #
##############################

with mp_holistic.Holistic(
    min_detection_confidence=0.5,
    min_tracking_confidence=0.5) as holistic:
  while cap.isOpened():
    success, image = cap.read()
    if not success:
      # Stop when video done
      break

    # Flip the image horizontally for a later selfie-view display, and convert
    # the BGR image to RGB.
    image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    # To improve performance, optionally mark the image as not writeable to
    # pass by reference.
    image.flags.writeable = False
    results = holistic.process(image)

    # Draw landmark annotation on the image.
    image.flags.writeable = True
    image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
    mp_drawing.draw_landmarks(
        image, results.face_landmarks, mp_holistic.FACE_CONNECTIONS)
    mp_drawing.draw_landmarks(
        image, results.left_hand_landmarks, mp_holistic.HAND_CONNECTIONS)
    mp_drawing.draw_landmarks(
        image, results.right_hand_landmarks, mp_holistic.HAND_CONNECTIONS)
    mp_drawing.draw_landmarks(
        image, results.pose_landmarks, mp_holistic.POSE_CONNECTIONS)
    cv2.imshow('MediaPipe Holistic', image)
    
    
    #############
    # Calc Time #
    #############
    
    # in theory use fps to adjust the frame rate when playback lags
    # needs testing
    fps = cap.get(cv2.CAP_PROP_FPS)
    frame_exists, curr_frame = cap.read()
    if frame_exists:
        timestamps.append(cap.get(cv2.CAP_PROP_POS_MSEC))
        calc_timestamps.append(calc_timestamps[-1] + 1000/fps)
    
    ########################################################
    # Transform variables, so they are easier to work with #
    ########################################################
    
    #wrist recognised
    if results.right_hand_landmarks != None:
        keypoints = []
        for data_point in results.right_hand_landmarks.landmark:
            keypoints.append({
                                 "X": data_point.x,
                                 "Y": data_point.y,
                                 "Z": data_point.z,
                                 "Visibility": data_point.visibility,
                                 })
    #Body recognised
    if results.pose_landmarks != None:
        keypoints_body = []
        for data_point in results.pose_landmarks.landmark:
            keypoints_body.append({
                                 "X": data_point.x,
                                 "Y": data_point.y,
                                 "Z": data_point.z,
                                 "Visibility": data_point.visibility,
                                 })
    
    ########################
    # Save the Data Points #
    ########################
    
    # Calculate time delay between frames
    if len(timestamps) > 0:
        timestamps[-1] = timestamps[-1] - timestamps[-2]
    
    # If we found both body and wrist...
    if results.right_hand_landmarks != None and results.pose_landmarks != None:
        with open('points.csv', 'a', newline='') as csvfile:
            spamwriter = csv.writer(csvfile, delimiter=',')
            spamwriter.writerow([datetime.now(tz=None)] + keypoints_body + keypoints + [calc_timestamps[-1]])
    # If we found body only, add [0,..,0] for wrist
    if results.pose_landmarks != None:
        with open('points.csv', 'a', newline='') as csvfile:
            spamwriter = csv.writer(csvfile, delimiter=',')
            spamwriter.writerow([datetime.now(tz=None)] + keypoints_body + empty_json + [calc_timestamps[-1]])
    
    ###################
    # Quit on ESC key #
    ###################
    
    if cv2.waitKey(1) & 0xFF == 27:
      break

# Cleanup
cap.release()
