# -*- coding: utf-8 -*-
"""
Created on Sun May 30 01:02:16 2021

@author: moroz
"""

import cv2
import mediapipe as mp
from mediapipe.framework.formats import landmark_pb2
import math

import rtmidi


mp_drawing = mp.solutions.drawing_utils
mp_pose = mp.solutions.pose

midiout = rtmidi.MidiOut()
available_ports = midiout.get_ports()
midiout.open_port(1)






def remap(x, in_min, in_max, out_min, out_max):
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

def myAngle(cx, cy, ex, ey):
  dy = ey - cy
  dx = ex - cx
  theta = math.atan2(dy, dx) 
  theta *= 180 / math.pi
  return round(theta)


# For webcam input:
cap = cv2.VideoCapture(3)
x=0
tmp_ind=0
tmp_ind2=0
with mp_pose.Pose(
    min_detection_confidence=0.5,
    min_tracking_confidence=0.5) as pose:
  while cap.isOpened():
    success, image = cap.read()
    if not success:
      print("Ignoring empty camera frame.")
      # If loading a video, use 'break' instead of 'continue'.
      continue

    # Flip the image horizontally for a later selfie-view display, and convert
    # the BGR image to RGB.
    image = cv2.cvtColor(cv2.flip(image, 1), cv2.COLOR_BGR2RGB)
    # To improve performance, optionally mark the image as not writeable to
    # pass by reference.
    image.flags.writeable = False
    results = pose.process(image)

    # Draw the pose annotation on the image.
    image.flags.writeable = True
    image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
    mp_drawing.draw_landmarks(
        image, results.pose_landmarks, mp_pose.POSE_CONNECTIONS)
    cv2.imshow('MediaPipe Pose', image)
    
    keypoints = []
    for data_point in results.pose_landmarks.landmark:
        keypoints.append({
                             'X': data_point.x,
                             'Y': data_point.y,
                             'Z': data_point.z,
                             'Visibility': data_point.visibility,
                             })
    
    if len(keypoints) > 16:
        #print("1 - ",keypoints[12].get("X")," 2 - ",keypoints[12].get("Y")," 3 - ",keypoints[14].get("X")," 4 - ",keypoints[14].get("Y"))
        my_shoulder = myAngle(keypoints[12].get("X"),keypoints[12].get("Y"),keypoints[14].get("X"),keypoints[14].get("Y"))
        #print("Right Shoulder : ", my_shoulder)
        
        a1 = myAngle(keypoints[12].get("X"),keypoints[12].get("Y"),keypoints[14].get("X"),keypoints[14].get("Y"))
        a2 = myAngle(keypoints[14].get("X"),keypoints[14].get("Y"),keypoints[16].get("X"),keypoints[16].get("Y"))
        
        if a1 > a2 :
           sign =  1
        else:
           sign = -1
        
        result_angle = a1 - a2
        K = -sign * math.pi * 2
        
        if (abs(K + result_angle) < abs(result_angle)): 
            result_angle = K + result_angle
        else:
            result_angle = result_angle
        
        #print("Right Elbow : ",round(result_angle))
        my_elbow = round(result_angle)
        
        
        if int(my_shoulder) < 165 and int(my_shoulder) > 0:
            if abs(tmp_ind2 - int(my_shoulder)) > 2:
                NewValue = remap(int(my_shoulder), 0, 165, 62, 127)
                tmp_ind2 = int(my_shoulder)
                midiout.send_message([176, 21, int(NewValue)])    
                #time.sleep(0.1)
                print ("Elbow - ", NewValue)
        if int(my_shoulder) < 0 and int(my_shoulder) > -90:
            if abs(tmp_ind2 - int(my_shoulder)) > 2:
                NewValue = remap(int(my_shoulder)+90, 0, 90, 31, 61)
                tmp_ind2 = int(my_shoulder)
                midiout.send_message([176, 21, int(NewValue)])
                #time.sleep(0.1)
                print ("Elbow - ", NewValue)
        if int(my_shoulder) < 270 and int(my_shoulder) > 179:
            if abs(tmp_ind2 - int(my_shoulder)) > 2:                
                NewValue = abs(remap(int(my_shoulder), 180, 270, 0, 30))
                tmp_ind2 = int(my_shoulder)
                midiout.send_message([176, 21, int(NewValue)])
                #time.sleep(0.1)
                print ("Elbow - ", NewValue)
                
        if abs(tmp_ind - int(my_elbow)) > 2:
            NewValue = remap(int(my_elbow)+180, 0, 360, 0, 127)
            tmp_ind  = int(my_elbow)
            midiout.send_message([176, 20, int(NewValue)])
            print ("Shoulder - ", int(NewValue))
            #time.sleep(0.1)
    
    if cv2.waitKey(5) & 0xFF == 27:
      break
cap.release()
