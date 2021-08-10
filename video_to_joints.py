import cv2
import mediapipe as mp
import math
import csv
import os
import time
from datetime import datetime
from math import atan2, cos, sin, degrees

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
cam_ref=1
#cam_ref="arm_move_f_2.mp4"

send_midi_bool = True


            ###############
           ## MIDI Values ##
#####################################\
# Far                               ##
######################################
# Far | Shoulder                     #
far_shoulder_midi = 70               #
# Far | Shoulder Scaling             #
far_shoulder_scaling_midi = 71       #
# Far | Elbow                        #
far_elbow_midi = 72                  #
# Far | Elbow Scaling                #
far_elbow_scaling_midi = 73          #
# Far | Wrist                        #
far_wrist_midi = 74                  #
# Far | Center Flip                  #
center_wrist_far = 75                #
# Far | Buttons ↑→↓                  #
far_wrist_flip_midi = [78,77,76,79]  #
#####################################\
# Near                              ##
######################################
# Near | Shoulder                    #
near_shoulder_midi   = 60            #
# Near | Shoulder Scaling            #
near_shoulder_scaling_midi = 61      #
# Near | Elbow                       #
near_elbow_midi      = 62            #
# Near | Elbow Scaling               #
near_elbow_scaling_midi = 63         #
# Near | Wrist                       #
near_wrist_midi      = 64            #
# Near | Center Flip                 #
center_wrist_near = 65               #
# Near | Buttons ↑→↓                 #
near_wrist_flip_midi = [68,67,66,69]##
#####################################\
# Position                          ##
######################################
#Screen Position(walking behavior px)#
screen_position_midi = 22            #
#Screen Position Standing (x)        #
main_position_midi   = 23            #
#####################################\
# Character Turn                    ##
######################################
# Front turn                         #
front_midi = 80                      #
#<3QT face note                      #
l3qt_midi = 81                       #
#>3QT face note                      #
r3qt_midi = 82                       #
# Right turn                         #
right_midi = 83                      #
# Left turn                          #
left_midi = 84                       #
#####################################/
##  Threshholds                     ##
######################################
#↓                                   #
down_arm_midi_top = 120              #
#←                                   #
#left_arm_midi = 32                  #
#↑                                   #
up_arm_midi = 75                     #
#→                                   #
right_arm_midi = 60                  #
#↓                                   #
down_arm_midi_bottom = 10            #
######################################


##################
# Init variables #
##################

if send_midi_bool:
    import rtmidi
    #this should be a reference to a loopmidi (number)
    midiout = rtmidi.MidiOut()
    midiout.open_port(1)

dist_max = 163
dist_min = 100
dist_max_should = 264
dist_min_should = 220

my_arr_NearWrist = [0]
my_arr_NearShoulder = [0]
my_arr_NearShoulderScaling = [0]

near_elbow_flip_status = 3
near_elbow_flip_status_b = 2

my_arr_FarWrist = [0]
my_arr_FarShoulder = [0]
my_arr_FarShoulderScaling = [0]

far_elbow_flip_status = 3
far_elbow_flip_status_b = 2

###################
# Functions Setup #
###################

#remap x from input min/max to output min/max
def remap(x, in_min, in_max, out_min, out_max):
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

#0-360 for the line based on coordinates
def myAngle(cx, cy, ex, ey):
  angle = atan2(cos(cx)*sin(ex)-sin(cx) * cos(ex)*cos(ey-cy), sin(ey-cy)*cos(ex))
  bearing = (degrees(angle) + 360) % 360
  return bearing 

def myAngleRad(cx, cy, ex, ey):
  angle = atan2(cos(cx)*sin(ex)-sin(cx) * cos(ex)*cos(ey-cy), sin(ey-cy)*cos(ex))
  return angle

def rotate(cx, cy, ex, ey, angle):
    qx = cx + math.cos(angle) * (ex - cx) - math.sin(angle) * (ey - cy)
    qy = cy + math.sin(angle) * (ex - cx) + math.cos(angle) * (ey - cy)
    return qx, qy


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
      #continue
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
        keypoints_wrist_near = []
        for data_point in results.right_hand_landmarks.landmark:
            keypoints_wrist_near.append({
                                 "X": data_point.x,
                                 "Y": data_point.y,
                                 "Z": data_point.z,
                                 "Visibility": data_point.visibility,
                                 })
    #wrist recognised
    if results.left_hand_landmarks != None:
        keypoints_wrist_far = []
        for data_point in results.left_hand_landmarks.landmark:
            keypoints_wrist_far.append({
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
            spamwriter.writerow([datetime.now(tz=None)] + keypoints_body + keypoints_wrist_near + [calc_timestamps[-1]])
    # If we found body only, add [0,..,0] for wrist
    if results.pose_landmarks != None:
        with open('points.csv', 'a', newline='') as csvfile:
            spamwriter = csv.writer(csvfile, delimiter=',')
            spamwriter.writerow([datetime.now(tz=None)] + keypoints_body + empty_json + [calc_timestamps[-1]])
    
    if results.left_hand_landmarks != None and results.right_hand_landmarks != None and results.pose_landmarks != None:
        ######################
        ## Near - Midi Send ##
        ######################
        
        my_n_hip = keypoints_body[24]
        my_n_shoulder = keypoints_body[12]
        my_n_elbow = keypoints_body[14]
        my_n_wrist = keypoints_body[16]
        my_n_h_wrist = keypoints_wrist_near[12]
        
        my_f_hip = keypoints_body[23]
        my_f_shoulder = keypoints_body[11]
        my_f_elbow = keypoints_body[13]
        my_f_wrist = keypoints_body[15]
        my_f_h_wrist = keypoints_wrist_far[12] 
        
        
        ##############
        # Near Wrist #
        ##############
        
        NewValue = myAngle(my_n_wrist.get("X"),my_n_wrist.get("Y"),my_n_h_wrist.get("X"),my_n_h_wrist.get("Y"))
        
        if my_n_h_wrist.get("X") != 0 or my_n_h_wrist.get("Y") != 0:
            if NewValue < 0:
                NewValue = 0
            if NewValue > 110:
                NewValue = 110        
            NearWrist = int(remap(int(NewValue), 0, 110, 0, 127))
            
        
        ##############
        # Near elbow #
        ##############
        
        my_elbow_tmp = myAngleRad(my_n_shoulder.get("X"),my_n_shoulder.get("Y"),my_n_elbow.get("X"),my_n_elbow.get("Y"))
        rot_my_elbow_n = rotate(my_n_elbow.get("X"),my_n_elbow.get("Y"),my_n_wrist.get("X"),my_n_wrist.get("Y"), my_elbow_tmp)
        my_elbow_n = myAngle(my_n_elbow.get("X"),my_n_elbow.get("Y"),rot_my_elbow_n[0],rot_my_elbow_n[1])
        #my_elbow_n = myAngle(my_n_elbow.get("X"),my_n_elbow.get("Y"),my_n_wrist.get("X"),my_n_wrist.get("Y"))
        NearElbow = int(remap(int(my_elbow_n), 0, 360, 0, 127))
        
        #################
        # Near shoulder #
        #################
        
        my_shoulder_n = myAngle(my_n_shoulder.get("X"),my_n_shoulder.get("Y"),my_n_elbow.get("X"),my_n_elbow.get("Y"))
        my_shoulder_n = remap(int(my_shoulder_n), 0, 360, 0, 127)
        
        #########################
        # Near Elbow Scaling    #
        #########################
        
        my_x = abs(my_n_wrist.get("X")-my_n_elbow.get("X"))*1000
        my_y = abs(my_n_wrist.get("Y")-my_n_elbow.get("Y"))*1000
        dist = math.hypot(my_x, my_y)
        if dist > dist_max:
            dist = dist_max
        if dist < dist_min:
            dist = dist_min
        my_n_elbow_scale_values = int(remap(int(dist), dist_min, dist_max, 0, 127))
        if my_n_elbow_scale_values >= 119:
            my_n_elbow_scale_values = 127
        if my_n_elbow_scale_values < 0:
            my_n_elbow_scale_values = 0
        
        #########################
        # Near Shoulder Scaling #
        #########################
        
        my_x = abs(my_n_elbow.get("X")-my_n_shoulder.get("X"))*1000
        my_y = abs(my_n_elbow.get("Y")-my_n_shoulder.get("Y"))*1000
        dist = math.hypot(my_x, my_y)
        if dist > dist_max_should:
            dist_max_should = dist
        if dist < dist_min_should:
            dist_min_should = dist
        my_n_shoulder_scale_values = int(remap(int(dist), dist_min_should, dist_max_should, 0, 127))
        if my_n_shoulder_scale_values >= 120:
            my_n_shoulder_scale_values = 127
        if my_n_shoulder_scale_values < 0:
            my_n_shoulder_scale_values = 0
        
        
        #############
        # Far Wrist #
        #############
        
        NewValue = myAngle(my_f_wrist.get("X"),my_f_wrist.get("Y"),my_f_h_wrist.get("X"),my_f_h_wrist.get("Y"))
        
        if my_f_h_wrist.get("X") != 0 or my_f_h_wrist.get("Y") != 0:
            if NewValue < 0:
                NewValue = 0
            if NewValue > 110:
                NewValue = 110        
            FarWrist = int(remap(int(NewValue), 0, 110, 0, 127))
        else:
            FarWrist = my_arr_FarWrist[-1]
        
        #############
        # Far elbow #
        #############
           
        my_elbow_f = myAngle(my_f_elbow.get("X"),my_f_elbow.get("Y"),my_f_wrist.get("X"),my_f_wrist.get("Y"))
        FarElbow = int(remap(int(my_elbow_f), 0, 360, 0, 127))
        
        ################
        # Far shoulder #
        ################
        
        my_shoulder_f = myAngle(my_f_shoulder.get("X"),my_f_shoulder.get("Y"),my_f_elbow.get("X"),my_f_elbow.get("Y"))
        my_shoulder_f = remap(int(my_shoulder_f), 0, 360, 0, 127)
        
        ########################
        # Far Elbow Scaling    #
        ########################
        
        my_x = abs(my_f_wrist.get("X")-my_f_elbow.get("X"))*1000
        my_y = abs(my_f_wrist.get("Y")-my_f_elbow.get("Y"))*1000
        dist = math.hypot(my_x, my_y)
        if dist > dist_max:
            dist = dist_max
        if dist < dist_min:
            dist = dist_min
        my_f_elbow_scale_values = int(remap(int(dist), dist_min, dist_max, 0, 127))
        if my_f_elbow_scale_values >= 119:
            my_f_elbow_scale_values = 127
        if my_f_elbow_scale_values < 0:
            my_f_elbow_scale_values = 0
        
        ########################
        # Far Shoulder Scaling #
        ########################
        
        my_x = abs(my_f_elbow.get("X")-my_f_shoulder.get("X"))*1000
        my_y = abs(my_f_elbow.get("Y")-my_f_shoulder.get("Y"))*1000
        dist = math.hypot(my_x, my_y)
        if dist > dist_max_should:
            dist = dist_max_should
        if dist < dist_min_should:
            dist = dist_min_should
        my_f_shoulder_scale_values = int(remap(int(dist), dist_min, dist_max, 0, 127))
        if my_f_shoulder_scale_values >= 120:
            my_f_shoulder_scale_values = 127
        if my_f_shoulder_scale_values < 0:
            my_f_shoulder_scale_values = 0
        
        if send_midi_bool:
            my_delay = 0.01
            ################
            ## Midi Notes ##
            #########################################################
            ## button press                                         #
            ## --> midiout.send_message([0x90, midi_ref, 100])      #
            ## knob value send                                      #
            ## --> midiout.send_message([176, midi_ref, midi_value])#
            #########################################################
            
            # Near | Elbow - knob
            if abs(my_arr_NearWrist[-1] - NearElbow) > 2:
                midiout.send_message([176, near_elbow_midi, NearElbow])
                my_arr_NearWrist[-1] = NearElbow
                time.sleep(my_delay)
            # Near | Elbow Scaling - knob
            #if abs(my_arr_NearElbowScaling[-1] - my_n_elbow_scale_values) > 4:
                #midiout.send_message([176, near_elbow_scaling_midi, my_n_elbow_scale_values])
            #    time.sleep(my_delay)
            # Near | Shoulder - knob
            if abs(my_arr_NearShoulder[-1] - my_shoulder_n) > 1:
                midiout.send_message([176, near_shoulder_midi, my_shoulder_n])
                my_arr_NearShoulder[-1] = my_shoulder_n
                time.sleep(my_delay)
            # Near | Shoulder Scaling - knob
            #if near_elbow_flip_status == 1 and abs(my_arr_NearShoulderScaling[-1] - my_n_shoulder_scale_values) > 4:
            #    midiout.send_message([176, near_shoulder_scaling_midi, my_n_shoulder_scale_values])
            #    my_arr_NearShoulderScaling[-1] = my_n_shoulder_scale_values
            #    time.sleep(my_delay)
            # Near | Wrist - knob
            #if abs(my_arr_NearWrist[-1] - NearWrist) > 2:
            #    midiout.send_message([176, near_wrist_midi, NearWrist])
            #    time.sleep(my_delay)
            # Near | Wrist Center Flip
            if my_n_elbow_scale_values < 30 and near_elbow_flip_status_b != 1:
                midiout.send_message([0x90, center_wrist_near, 100])
                near_elbow_flip_status_b = 1
                time.sleep(my_delay)
            elif my_n_elbow_scale_values >= 30 and near_elbow_flip_status_b != 2:
                midiout.send_message([0x90, center_wrist_near, 100])
                near_elbow_flip_status_b = 2
                time.sleep(my_delay)
            # Near | Wrist Switch Buttons      
            # Near | ↓ | Down sideways
            if near_elbow_flip_status_b != 1:
                if NearElbow >= down_arm_midi_top and near_elbow_flip_status != 1 or NearElbow < down_arm_midi_bottom and near_elbow_flip_status != 1:
                    midiout.send_message([0x90, near_wrist_flip_midi[2], 100])
                    #print("down - ", NearElbow)
                    near_elbow_flip_status = 1
                    time.sleep(my_delay)
                # Near | ← | Left Out
                elif NearElbow < down_arm_midi_top and NearElbow >= up_arm_midi and near_elbow_flip_status != 3:
                    if near_elbow_flip_status == 1:
                        tmp_send = near_wrist_flip_midi[2]
                    elif near_elbow_flip_status == 2:
                        tmp_send = near_wrist_flip_midi[1]
                    elif near_elbow_flip_status == 4:
                        tmp_send = near_wrist_flip_midi[0]
           
                    #midiout.send_message([0x90, tmp_send, 100])
                    midiout.send_message([0x90, near_wrist_flip_midi[3], 100])
                    #print("out - ", NearElbow)
                    near_elbow_flip_status = 3
                    time.sleep(my_delay)
                # Near | ↑ | Up sideways
                elif NearElbow < up_arm_midi and NearElbow >= right_arm_midi and near_elbow_flip_status != 4:
                    midiout.send_message([0x90, near_wrist_flip_midi[0], 100])
                    #print("up - ", NearElbow)
                    near_elbow_flip_status = 4 
                    time.sleep(my_delay)
                # Near | → | Right In
                elif NearElbow < right_arm_midi and NearElbow >= down_arm_midi_bottom and near_elbow_flip_status != 2:
                    midiout.send_message([0x90, near_wrist_flip_midi[1], 100])
                    #print("in - ", NearElbow)
                    near_elbow_flip_status = 2
                    time.sleep(my_delay)
           
            #####################
            ## Far - Midi Send ##
            #####################
            #if False:
            # Far | Elbow - knob
            if abs(my_arr_FarWrist[-1] - FarElbow) > 2:
                midiout.send_message([176, far_elbow_midi, FarElbow])
                time.sleep(my_delay)
            # Far | Elbow Scaling - knob
            #if abs(my_arr_FarElbowScaling[-1] - my_f_elbow_scale_values) > 4:
                #midiout.send_message([176, far_elbow_scaling_midi, my_f_elbow_scale_values])
            #    time.sleep(my_delay)
            # Far | Shoulder - knob
            if abs(my_arr_FarShoulder[-1] - my_shoulder_f) > 1:
                time.sleep(my_delay)
                midiout.send_message([176, far_shoulder_midi, my_shoulder_f])
            # Far | Shoulder Scaling - knob
            #if far_elbow_flip_status == 1 and abs(my_arr_FarShoulderScaling[-1] - my_f_shoulder_scale_values) > 4:
            #    time.sleep(my_delay)
            #    midiout.send_message([176, far_shoulder_scaling_midi, my_f_shoulder_scale_values])
            # Far | Wrist - knob
            #if abs(my_arr_FarWrist[-1] - FarWrist) > 2:
            #    midiout.send_message([176, far_wrist_midi, FarWrist])
            #    time.sleep(my_delay)
            # Far | Wrist Center Flip
            if my_f_elbow_scale_values < 30 and far_elbow_flip_status_b != 1:
                time.sleep(my_delay)
                midiout.send_message([0x90, center_wrist_far, 100])
                far_elbow_flip_status_b = 1
            elif my_f_elbow_scale_values >= 30 and far_elbow_flip_status_b != 2:
                time.sleep(my_delay)
                midiout.send_message([0x90, center_wrist_far, 100])
                far_elbow_flip_status_b = 2
            # Far | Wrist Switch Buttons      
            # Far | ↓ | Down sideways
            if far_elbow_flip_status_b != 1:
                if FarElbow >= down_arm_midi_top and far_elbow_flip_status != 1 or FarElbow < down_arm_midi_bottom and far_elbow_flip_status != 1:
                    time.sleep(my_delay)
                    midiout.send_message([0x90, far_wrist_flip_midi[2], 100])
                    #print("down - ", FarElbow)
                    far_elbow_flip_status = 1
                # Far | ← | Left Out
                elif FarElbow < down_arm_midi_top and FarElbow >= up_arm_midi and far_elbow_flip_status != 3:
                    time.sleep(my_delay)                             
                    midiout.send_message([0x90, far_wrist_flip_midi[1], 100])
                    #print("out - ", NearElbow)
                    far_elbow_flip_status = 3
                # Far | ↑ | Up sideways
                elif FarElbow < up_arm_midi and FarElbow >= right_arm_midi and far_elbow_flip_status != 4:
                    time.sleep(my_delay)
                    midiout.send_message([0x90, far_wrist_flip_midi[0], 100])
                    #print("up - ", FarElbow)
                    far_elbow_flip_status = 4 
                # Far | → | Right In
                elif FarElbow < right_arm_midi and FarElbow >= down_arm_midi_bottom and far_elbow_flip_status != 2:
                    time.sleep(my_delay)
                    midiout.send_message([0x90, far_wrist_flip_midi[3], 100])
                    #print("in - ", FarElbow)
                    far_elbow_flip_status = 2
                             
            
    
    
    
    
    
    ###################
    # Quit on ESC key #
    ###################
    
    if cv2.waitKey(1) & 0xFF == 27:
      break

# Cleanup
cap.release()

if send_midi_bool:            
    midiout.close_port()
