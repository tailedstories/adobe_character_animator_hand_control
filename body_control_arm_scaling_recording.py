import cv2
import mediapipe as mp
import rtmidi
import math
import csv
from datetime import datetime

mp_drawing = mp.solutions.drawing_utils
mp_holistic = mp.solutions.holistic

midiout = rtmidi.MidiOut()
available_ports = midiout.get_ports()


#########
## SET ##
## UP  ##
#########

#this should be a reference to a loopmidi (number)
midiout.open_port(1)
#this should be a reference to your camera (number)
#cam_ref=3
cam_ref="015_dance.mp4"

# midi values

#Right Arm
right_shoulder_scaling_midi = 11
right_shoulder_midi   = 12
right_elbow_midi      = 18
right_wrist_midi      = 80
right_wrist_flip_midi = 81
right_elbow_scaling_midi = 10
#center
center_wrist_near = 7
#Left Arm
left_shoulder_midi   = 17
left_elbow_midi      = 21
left_wrist_midi      = 19
left_wrist_flip_midi = 83
#Screen Position (walking behavior px)
screen_position_midi = 22
#Screen Position Standing (x)
main_position_midi   = 23

#<3QT face note
l3qt_midi = 84
#Front face note
center_midi = 89
#>3QT face note
r3qt_midi = 86

# Right turn
right_midi = 87
# Left turn
left_midi = 88
# Front turn
front_midi = 89



def changeRate(x1,x2):
    if x1 == 0 or x2 == 0:
        x1+=1
        x2+=1
    return (x2-x1)/x1

def prepangle(x1,y1,x2,y2,x3,y3):
    a1 = myAngle(x1,y1,x2,y2)
    a2 = myAngle(x2,y2,x3,y3)
    
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
    return round(result_angle)

def remap(x, in_min, in_max, out_min, out_max):
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

def myAngle(cx, cy, ex, ey):
  dy = ey - cy
  dx = ex - cx
  theta = math.atan2(dy, dx) 
  theta *= 180 / math.pi
  return round(theta)


empty_json=[]
for i in range(21):
    empty_json.append("{'X': 0.0, 'Y': 0.0, 'Z': 0.0, 'Visibility': 0.0}")



tmp_ind_l= 0
tmp_ind_r= 0
x_mid=0
New_val_old=0

# For webcam input:
cap = cv2.VideoCapture(cam_ref)
with mp_holistic.Holistic(
    min_detection_confidence=0.5,
    min_tracking_confidence=0.5) as holistic:
  while cap.isOpened():
    success, image = cap.read()
    if not success:
      print("Ignoring empty camera frame.")
      # If loading a video, use 'break' instead of 'continue'.
      continue

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
    
    
    
    if results.right_hand_landmarks != None:
        keypoints = []
        for data_point in results.right_hand_landmarks.landmark:
            keypoints.append({
                                 "X": data_point.x,
                                 "Y": data_point.y,
                                 "Z": data_point.z,
                                 "Visibility": data_point.visibility,
                                 })
    
    if results.pose_landmarks != None:
        keypoints_body = []
        for data_point in results.pose_landmarks.landmark:
            keypoints_body.append({
                                 "X": data_point.x,
                                 "Y": data_point.y,
                                 "Z": data_point.z,
                                 "Visibility": data_point.visibility,
                                 })
    
    # Save the Data Points
    if results.right_hand_landmarks != None and results.pose_landmarks != None:
        #Column 33 end keypoints_body
        with open('points.csv', 'a', newline='') as csvfile:
            spamwriter = csv.writer(csvfile, delimiter=',')
            spamwriter.writerow([datetime.now(tz=None)] + keypoints_body + keypoints)
        #with open('points_hands.csv', 'a', newline='') as csvfile:
        #    spamwriter = csv.writer(csvfile, delimiter=',')
        #    spamwriter.writerow(keypoints)
    if results.pose_landmarks != None:
        #Column 33 end keypoints_body
        with open('points.csv', 'a', newline='') as csvfile:
            spamwriter = csv.writer(csvfile, delimiter=',')
            spamwriter.writerow([datetime.now(tz=None)] + keypoints_body + empty_json)
    
    
    # Near Wrist
    if results.right_hand_landmarks != None and results.pose_landmarks != None:
        NewValue = myAngle(keypoints[12].get("X"),keypoints[12].get("Y"),keypoints_body[16].get("X"),keypoints_body[16].get("Y"))
        if NewValue < 0:
            NewValue = 0
        if NewValue > 110:
            NewValue = 110        
        NewValue = int(remap(int(NewValue), 0, 110, 0, 127))
        
        #print(NewValue)
        midiout.send_message([176, left_wrist_midi, int(NewValue)])
    
    
    # Near elbow
    if results.pose_landmarks != None:
        my_elbow_r = prepangle(keypoints_body[12].get("X"),keypoints_body[12].get("Y"),
                               keypoints_body[14].get("X"),keypoints_body[14].get("Y"),
                               keypoints_body[16].get("X"),keypoints_body[16].get("Y"))
        
        # Near elbow
        if int(my_elbow_r) < 165 and int(my_elbow_r) > 0:
            if abs(tmp_ind_r - int(my_elbow_r)) > 2:
                NewValue = remap(int(my_elbow_r), 0, 165, 62, 127)
                tmp_ind_r = int(my_elbow_r)
                midiout.send_message([176, right_elbow_midi, int(NewValue)])    
                #time.sleep(0.1)
                #print ("Elbow - ", NewValue)
        if int(my_elbow_r) < 0 and int(my_elbow_r) > -90:
            if abs(tmp_ind_r - int(my_elbow_r)) > 2:
                NewValue = remap(int(my_elbow_r)+90, 0, 90, 31, 61)
                tmp_ind_r = int(my_elbow_r)
                midiout.send_message([176, right_elbow_midi, int(NewValue)])
                #time.sleep(0.1)
                #print ("Elbow - ", NewValue)
        if int(my_elbow_r) < 270 and int(my_elbow_r) > 179:
            if abs(tmp_ind_r - int(my_elbow_r)) > 2:                
                NewValue = abs(remap(int(my_elbow_r), 180, 270, 0, 30))
                tmp_ind_r = int(my_elbow_r)
                midiout.send_message([176, right_elbow_midi, int(NewValue)])
                #time.sleep(0.1)
                #print ("Elbow - ", NewValue)
    
    # Near elbow scaling
    if results.pose_landmarks != None:
        # Near Shoulder Scaling
        my_x = int(abs(keypoints_body[12].get("X")-keypoints_body[14].get("X"))*1000)
        my_y = int(abs(keypoints_body[12].get("Y")-keypoints_body[14].get("Y"))*1000)
        
        New_val = (my_x+my_y)/2
        if New_val < 60:
            New_val = 60
        elif New_val > 200:
            New_val = 200
        New_val = int(remap(int(New_val), 60, 200, 0, 127))
        
        if abs(New_val - New_val_old)> 5:
            midiout.send_message([176, right_shoulder_scaling_midi, New_val])
        
        print(New_val)
        
        if New_val - New_val_old < 10:
            #center wrist
            my_x = int(abs(keypoints_body[14].get("X")-keypoints_body[16].get("X"))*1000)
            my_y = int(abs(keypoints_body[14].get("Y")-keypoints_body[16].get("Y"))*1000)
            if my_y < 70 and my_x < 70 and x_mid != 1:
                print("in")
                x_mid = 1
                midiout.send_message([0x90, center_wrist_near, 100])
            elif my_y >= 50 and my_x >= 50 and x_mid != 0:
                print("out")
                x_mid = 0
                midiout.send_message([0x90, center_wrist_near, 100])
                #print(my_x,  "  ", my_y)
            
            #scaling
            if my_y >= 70 and my_x >= 70:
                #right_elbow_scaling_midi
                if my_x > 200:
                    my_x=200
                midiout.send_message([176, right_elbow_scaling_midi, int(remap(int(my_x), 50, 200, 0, 127))])
                
                #print(remap(int(my_x), 50, 200, 0, 127))
    
        
        New_val_old = New_val
        
    if cv2.waitKey(5) & 0xFF == 27:
      break
cap.release()
midiout.close_port()