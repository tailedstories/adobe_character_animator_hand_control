# Importing Exported Midports
import csv
import json
import time
import math
import matplotlib.pyplot as plt
import numpy as np
import cv2,os
from scipy.ndimage.filters import gaussian_filter1d
from math import atan2, cos, sin, degrees
from tsmoothie.smoother import ConvolutionSmoother



#########
## SET ##
## UP  ##
#########

#width and height for output window
my_height = 960
my_width  = 1280

# send midi while playing
send_midi_bool = False
# display joints
display_bool = True
my_delay = 0.02

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
# Far | Hip                          #
far_hip_midi = 50                    #
# Far | Knee                         #
far_knee_midi = 51                   #
# Far | Foot                         #
far_foot_midi = 52                   #
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
# Near | Hip                         #
near_hip_midi = 53                   #
# Near | Knee                        #
near_knee_midi = 54                  #
# Near | Foot                        #
near_foot_midi = 55                  #
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
######################################
# Rotation                           #
my_body_rot_midi = 85                #
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

##################
# Init variables #
##################


if send_midi_bool:
    import rtmidi
    #this should be a reference to a loopmidi (number)
    midiout = rtmidi.MidiOut()
    midiout.open_port(3)

tmp_ind_r=0
NearElbow=None



dist_max = 0
dist_min = 99999
dist_max_should = 0
dist_min_should = 99999

my_arr_NearElbow = [0]
my_arr_NearShoulder = [0]
my_arr_NearShoulderInside = [0]
my_arr_NearShoulderScaling = [0]
my_arr_NearElbowScaling = [0]
my_arr_NearWrist = [0]
my_arr_NearWristSwitch = [0]

my_arr_NearHip = [0]
my_arr_NearKnee = [0]
my_arr_NearFoot = [0]

my_arr_FarElbow = [0]
my_arr_FarShoulder = [0]
my_arr_FarShoulderInside = [0]
my_arr_FarShoulderScaling = [0]
my_arr_FarElbowScaling = [0]
my_arr_FarWrist = [0]

my_arr_FarHip = [0]
my_arr_FarKnee = [0]
my_arr_FarFoot = [0]

my_arr_BodyRot = [0]

near_elbow_flip_status_b = 2
near_elbow_flip_status = 3
far_elbow_flip_status_b = 2
far_elbow_flip_status = 3

sl_arr = [0]
sl = 0

New_val_old = 0
x_mid = 0
my_n_shoulder_scale = 0

i_dt = 0


min_y = 0

####################################################
# Calibrate the distance to calculate arm scalling #
####################################################

with open('points.csv', newline='') as csvfile:
     spamreader = csv.reader(csvfile, delimiter=',')
     for row in spamreader:
         my_n_shoulder = json.loads(row[13].replace("'",'"'))
         my_n_elbow = json.loads(row[15].replace("'",'"'))
         my_n_wrist = json.loads(row[17].replace("'",'"'))

         my_x = abs(my_n_wrist.get("X")-my_n_elbow.get("X"))*1000
         my_y = abs(my_n_wrist.get("Y")-my_n_elbow.get("Y"))*1000
            
         dist = math.hypot(my_x, my_y)
         
         if dist > dist_max:
             dist_max = dist
         if dist < dist_min:
             dist_min = dist
             
         if float(json.loads(row[31].replace("'",'"')).get("Y")) > min_y:
            min_y = float(json.loads(row[31].replace("'",'"')).get("Y"))
             
         my_x = abs(my_n_elbow.get("X")-my_n_shoulder.get("X"))*1000
         my_y = abs(my_n_elbow.get("Y")-my_n_shoulder.get("Y"))*1000
         dist = math.hypot(my_x, my_y)
         if dist > dist_max_should:
             dist_max_should = dist
         if dist < dist_min_should:
             dist_min_should = dist

#adjust distance min/max threshhold
dist_max -= 60
#dist_min += 80

dist_max_should -= 60

min_y -= 0.04
print("Y - ", min_y)
print("Dist min - ", dist_min_should)
print("Dist max - ", dist_max_should)

print("Dist min - ", dist_min)
print("Dist max - ", dist_max)


######################
# Open CSV File      #
# Loop over the Rows #
######################


with open('points.csv', newline='') as csvfile:
     spamreader = csv.reader(csvfile, delimiter=',')
     for row in spamreader:
         #x = row
         if len(row) > 0:
             
             #parse columns into variables
             # _n - near
             # _f - far
             # _h - hand (wrist dots)
             my_n_foot_line_1 = json.loads(row[31].replace("'",'"'))
             my_n_foot_line_2 = json.loads(row[33].replace("'",'"'))
             my_n_foot = json.loads(row[29].replace("'",'"'))
             my_n_knee = json.loads(row[27].replace("'",'"'))
             my_n_hip = json.loads(row[25].replace("'",'"'))
             my_n_shoulder = json.loads(row[13].replace("'",'"'))
             my_n_elbow = json.loads(row[15].replace("'",'"'))
             my_n_wrist = json.loads(row[17].replace("'",'"'))
             my_n_h_wrist = json.loads(row[34+13].replace("'",'"'))
             
             my_f_foot_line_1 = json.loads(row[30].replace("'",'"'))
             my_f_foot_line_2 = json.loads(row[32].replace("'",'"'))
             my_f_foot = json.loads(row[28].replace("'",'"'))
             my_f_knee = json.loads(row[26].replace("'",'"'))
             my_f_hip = json.loads(row[24].replace("'",'"'))
             my_f_shoulder = json.loads(row[12].replace("'",'"'))
             my_f_elbow = json.loads(row[14].replace("'",'"'))
             my_f_wrist = json.loads(row[16].replace("'",'"'))
             my_f_h_wrist = json.loads(row[34+13].replace("'",'"')) # how near anf far wrist work? --> to fix
             
             
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
             else:
                 NearWrist = my_arr_NearWrist[-1]
                 
             
             ##############
             # Near elbow #
             ##############
             
             my_elbow_tmp = myAngleRad(my_n_shoulder.get("X"),my_n_shoulder.get("Y"),my_n_elbow.get("X"),my_n_elbow.get("Y"))
             rot_my_elbow_n = rotate(my_n_elbow.get("X"),my_n_elbow.get("Y"),my_n_wrist.get("X"),my_n_wrist.get("Y"), my_elbow_tmp)
             
             my_elbow_n = myAngle(my_n_elbow.get("X"),my_n_elbow.get("Y"),rot_my_elbow_n[0],rot_my_elbow_n[1])
             NearElbow = int(remap(int(my_elbow_n), 0, 360, 0, 127))
             
             #my_elbow_n = myAngle(my_n_elbow.get("X"),my_n_elbow.get("Y"),my_n_wrist.get("X"),my_n_wrist.get("Y"))
             #NearElbow  = int(remap(int(my_elbow_n), 0, 360, 0, 127))
             
             #################
             # Near shoulder #
             #################
             
             my_shoulder_tmp = myAngleRad(my_n_shoulder.get("X"),my_n_shoulder.get("Y"),my_n_hip.get("X"),my_n_hip.get("Y"))
             rot_my_shoulder_n = rotate(my_n_shoulder.get("X"),my_n_shoulder.get("Y"),my_n_elbow.get("X"),my_n_elbow.get("Y"), my_shoulder_tmp)
             
             my_shoulder_n = myAngle(my_n_shoulder.get("X"),my_n_shoulder.get("Y"),rot_my_shoulder_n[0],rot_my_shoulder_n[1])
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
                 dist = dist_max_should
             if dist < dist_min_should:
                 dist = dist_min_should
             my_n_shoulder_scale_values = int(remap(int(dist), dist_min_should, dist_max_should, 0, 127))
             if my_n_shoulder_scale_values >= 120:
                 my_n_shoulder_scale_values = 127
             if my_n_shoulder_scale_values < 0:
                 my_n_shoulder_scale_values = 0
                
             #############
             # Near Hip  #
             #############   
             
             my_hip_tmp = myAngleRad(my_n_shoulder.get("X"),my_n_shoulder.get("Y"),my_n_hip.get("X"),my_n_hip.get("Y"))
             rot_my_hip_n = rotate(my_n_hip.get("X"),my_n_hip.get("Y"),my_n_knee.get("X"),my_n_knee.get("Y"), my_hip_tmp)
             
             my_hip_n = myAngle(my_n_hip.get("X"),my_n_hip.get("Y"),rot_my_hip_n[0],rot_my_hip_n[1])
             my_hip_n = remap(int(my_hip_n), 0, 360, 0, 127)
             
             #my_hip_n = myAngle(my_n_hip.get("X"),my_n_hip.get("Y"),my_n_knee.get("X"),my_n_knee.get("Y"))
             #my_hip_n = remap(int(my_hip_n), 0, 360, 0, 127)
             
             #############
             # Near Knee #
             #############   
             
             my_knee_tmp = myAngleRad(my_n_hip.get("X"),my_n_hip.get("Y"),my_n_knee.get("X"),my_n_knee.get("Y"))
             rot_my_knee_n = rotate(my_n_knee.get("X"),my_n_knee.get("Y"),my_n_foot.get("X"),my_n_foot.get("Y"), my_knee_tmp)
             
             my_knee_n = myAngle(my_n_knee.get("X"),my_n_knee.get("Y"),rot_my_knee_n[0],rot_my_knee_n[1])
             my_knee_n = int(remap(int(my_knee_n), 0, 360, 0, 127))
             
             #my_knee_n = myAngle(my_n_knee.get("X"),my_n_knee.get("Y"),my_n_foot.get("X"),my_n_foot.get("Y"))
             #my_knee_n = remap(int(my_knee_n), 0, 360, 0, 127)
             
             
             #############
             # Near Foot #
             #############   
             foot_n_status = ""
             if min_y - my_n_foot_line_1.get("Y") < 0 or min_y - my_n_foot_line_2.get("Y") < 0:
                 # Legs are under ground
                 # Put on the ground and IK the joints above
                 foot_n_status = "under ground"
             elif min_y - my_n_foot_line_1.get("Y") < 0 or min_y - my_n_foot_line_2.get("Y") >= 0 and min_y - my_n_foot_line_1.get("Y") < 0.03 or min_y - my_n_foot_line_2.get("Y") > 0 and min_y - my_n_foot_line_2.get("Y") < 0.03 :
                 foot_n_status = "on ground"
                 #print(foot_status)
             else:
                 foot_n_status = "flying"
                 #print(foot_status)
                 
             
             
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
                
             my_elbow_tmp = myAngleRad(my_f_shoulder.get("X"),my_f_shoulder.get("Y"),my_f_elbow.get("X"),my_f_elbow.get("Y"))
             rot_my_elbow_f = rotate(my_f_elbow.get("X"),my_f_elbow.get("Y"),my_f_wrist.get("X"),my_f_wrist.get("Y"), my_elbow_tmp)
             
             my_elbow_f = myAngle(my_f_elbow.get("X"),my_f_elbow.get("Y"),rot_my_elbow_f[0],rot_my_elbow_f[1])
             FarElbow = int(remap(int(my_elbow_f), 0, 360, 0, 127))
             
             #my_elbow_f = myAngle(my_f_elbow.get("X"),my_f_elbow.get("Y"),my_f_wrist.get("X"),my_f_wrist.get("Y"))
             #FarElbow = int(remap(int(my_elbow_f), 0, 360, 0, 127))
             
             ################
             # Far shoulder #
             ################
             
             my_shoulder_tmp = myAngleRad(my_f_shoulder.get("X"),my_f_shoulder.get("Y"),my_f_hip.get("X"),my_f_hip.get("Y"))
             rot_my_shoulder_f = rotate(my_f_shoulder.get("X"),my_f_shoulder.get("Y"),my_f_elbow.get("X"),my_f_elbow.get("Y"), my_shoulder_tmp)
             
             my_shoulder_f = myAngle(my_f_shoulder.get("X"),my_f_shoulder.get("Y"),rot_my_shoulder_f[0],rot_my_shoulder_f[1])
             my_shoulder_f = remap(int(my_shoulder_f), 0, 360, 0, 127)
             
             #my_shoulder_f = myAngle(my_f_shoulder.get("X"),my_f_shoulder.get("Y"),my_f_elbow.get("X"),my_f_elbow.get("Y"))
             #my_shoulder_f = remap(int(my_shoulder_f), 0, 360, 0, 127)
             
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
                
             ############
             # Far Hip  #
             ############   
             
             
             my_hip_tmp = myAngleRad(my_f_shoulder.get("X"),my_f_shoulder.get("Y"),my_f_hip.get("X"),my_f_hip.get("Y"))
             rot_my_hip_f = rotate(my_f_hip.get("X"),my_f_hip.get("Y"),my_f_knee.get("X"),my_f_knee.get("Y"), my_hip_tmp)
             
             my_hip_f = myAngle(my_f_hip.get("X"),my_f_hip.get("Y"),rot_my_hip_f[0],rot_my_hip_f[1])
             my_hip_f = remap(int(my_hip_f), 0, 360, 0, 127)
             
             
             #my_hip_f = myAngle(my_f_hip.get("X"),my_f_hip.get("Y"),my_f_knee.get("X"),my_f_knee.get("Y"))
             #my_hip_f = remap(int(my_hip_f), 0, 360, 0, 127)
             
             ############
             # Far Knee #
             ############   
             
             my_knee_tmp = myAngleRad(my_f_hip.get("X"),my_f_hip.get("Y"),my_f_knee.get("X"),my_f_knee.get("Y"))
             rot_my_elbow_f = rotate(my_f_knee.get("X"),my_f_knee.get("Y"),my_f_foot.get("X"),my_f_foot.get("Y"), my_knee_tmp)
             
             my_knee_f = myAngle(my_f_knee.get("X"),my_f_knee.get("Y"),rot_my_elbow_f[0],rot_my_elbow_f[1])
             my_knee_f = int(remap(int(my_knee_f), 0, 360, 0, 127))
             
             #my_knee_f = myAngle(my_f_knee.get("X"),my_f_knee.get("Y"),my_f_foot.get("X"),my_f_foot.get("Y"))
             #my_knee_f = remap(int(my_knee_f), 0, 360, 0, 127)
             
             
             ############
             # Far Foot #
             ############   
             
             foot_f_status = ""
             if min_y - my_f_foot_line_1.get("Y") < 0 or min_y - my_f_foot_line_2.get("Y") < 0:
                 # Legs are under ground
                 # Put on the ground and IK the joints above
                 foot_f_status = "under ground"
             elif min_y - my_f_foot_line_1.get("Y") < 0 or min_y - my_f_foot_line_2.get("Y") >= 0 and min_y - my_f_foot_line_1.get("Y") < 0.01 or min_y - my_f_foot_line_2.get("Y") > 0 and min_y - my_f_foot_line_2.get("Y") < 0.01 :
                 foot_f_status = "on ground"
                 #print(foot_status)
             else:
                 foot_f_status = "flying"
                 #print(foot_status)
                 
             
             ####################
             # Body Rotation    #
             ####################
             
             my_body_rot_1 = myAngle(my_n_shoulder.get("X"),my_n_shoulder.get("Y"),my_n_hip.get("X"),my_n_hip.get("Y"))
             #my_body_rot_2 = myAngle(my_f_shoulder.get("X"),my_f_shoulder.get("Y"),my_f_hip.get("X"),my_f_hip.get("Y"))
             #my_body_rot = (my_body_rot_1 + my_body_rot_2) / 2
             my_body_rot = my_body_rot_1
             my_body_rot = remap(int(my_body_rot), 0, 360, 0, 127)
                
             ##################
             # Display Joints #
             ##################
             
             #set to false to disable player
             if display_bool:
                 #Prepare Green Screen
                 image = np.zeros((my_height,my_width,3), np.uint8)
                 image[:] = [0,255,0]
                 image.flags.writeable = False
                 results = image
                 image.flags.writeable = True
                 image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
                 
                 # Draw Far Arm
                 if True:    
                     #Draw Landmarks (joint lines)
                     # near | elbow --> wrist
                     cv2.line(image, (int(my_width*my_f_wrist.get("X")), int(my_height*my_f_wrist.get("Y"))), (int(my_width*my_f_elbow.get("X")), int(my_height*my_f_elbow.get("Y"))), (200,150,50),5)
                     # near | shoulder --> elbow
                     cv2.line(image, (int(my_width*my_f_shoulder.get("X")), int(my_height*my_f_shoulder.get("Y"))), (int(my_width*my_f_elbow.get("X")), int(my_height*my_f_elbow.get("Y"))), (200,100,50),5)
                     # near | hip --> shoulder
                     cv2.line(image, (int(my_width*my_f_shoulder.get("X")), int(my_height*my_f_shoulder.get("Y"))), (int(my_width*my_f_hip.get("X")), int(my_height*my_f_hip.get("Y"))), (255,155,150),5)
                    
                 # Draw Far Leg
                 if True:    
                     #Draw Landmarks (joint lines)
                     # far | hip --> knee
                     cv2.line(image, (int(my_width*my_f_hip.get("X")), int(my_height*my_f_hip.get("Y"))), (int(my_width*my_f_knee.get("X")), int(my_height*my_f_knee.get("Y"))), (200,150,0),5)
                     # far | knee --> foot
                     cv2.line(image, (int(my_width*my_f_knee.get("X")), int(my_height*my_f_knee.get("Y"))), (int(my_width*my_f_foot.get("X")), int(my_height*my_f_foot.get("Y"))), (100,100,0),5)
                     # far | foot
                     cv2.line(image, (int(my_width*my_f_foot_line_1.get("X")), int(my_height*my_f_foot_line_1.get("Y"))), (int(my_width*my_f_foot_line_2.get("X")), int(my_height*my_f_foot_line_2.get("Y"))), (255,100,255),5)
                     
                 # Draw Near Leg
                 if True:    
                     #Draw Landmarks (joint lines)
                     # near | hip --> knee
                     cv2.line(image, (int(my_width*my_n_hip.get("X")), int(my_height*my_n_hip.get("Y"))), (int(my_width*my_n_knee.get("X")), int(my_height*my_n_knee.get("Y"))), (200,50,50),5)
                     # near | knee --> foot
                     cv2.line(image, (int(my_width*my_n_knee.get("X")), int(my_height*my_n_knee.get("Y"))), (int(my_width*my_n_foot.get("X")), int(my_height*my_n_foot.get("Y"))), (100,0,50),5)
                     # near | foot
                     cv2.line(image, (int(my_width*my_n_foot_line_1.get("X")), int(my_height*my_n_foot_line_1.get("Y"))), (int(my_width*my_n_foot_line_2.get("X")), int(my_height*my_n_foot_line_2.get("Y"))), (255,0,255),5)

                     
                     
                     #cv2.line(image, (int(my_width*my_f_elbow.get("X")), int(my_height*my_f_elbow.get("Y"))), (int(my_width*my_f_hip.get("X")), int(my_height*my_f_hip.get("Y"))), (0,0,255),2)   
                 
                 # Draw Near Arm
                 if True:    
                     #Draw Landmarks (joint lines)
                     # near | hip --> shoulder
                     cv2.line(image, (int(my_width*my_n_shoulder.get("X")), int(my_height*my_n_shoulder.get("Y"))), (int(my_width*my_n_hip.get("X")), int(my_height*my_n_hip.get("Y"))), (255,55,150),5)
                     # near | elbow --> wrist
                     cv2.line(image, (int(my_width*my_n_wrist.get("X")), int(my_height*my_n_wrist.get("Y"))), (int(my_width*my_n_elbow.get("X")), int(my_height*my_n_elbow.get("Y"))), (200,50,50),5)
                     # near | shoulder --> elbow
                     cv2.line(image, (int(my_width*my_n_shoulder.get("X")), int(my_height*my_n_shoulder.get("Y"))), (int(my_width*my_n_elbow.get("X")), int(my_height*my_n_elbow.get("Y"))), (200,0,50),5)
                     
                     
                     #cv2.line(image, (int(my_width*my_n_elbow.get("X")), int(my_height*my_n_elbow.get("Y"))), (int(my_width*my_n_hip.get("X")), int(my_height*my_n_hip.get("Y"))), (0,0,255),2)
                 

                 # Draw Ground
                 cv2.line(image, (0,int(my_height*min_y)), (int(my_width),int(my_height*min_y)), (0,0,0),5)
                 
                 # Draw Midi Values
                 # near
                 i=0
                 if True:
                     # adjust position for Far distance
                     i+=50
                     # near | Shoulder
                     image = cv2.putText(image, "s" + str(int(round(my_shoulder_n,0))), (int(my_width*my_n_shoulder.get("X")), int(my_height*my_n_shoulder.get("Y"))), cv2.FONT_HERSHEY_DUPLEX, 1, (50,0,255), 2, cv2.LINE_AA)
                     # near | Elbow
                     image = cv2.putText(image, "e" + str(int(round(NearElbow,0))), (int(my_width*my_n_elbow.get("X")), int(my_height*my_n_elbow.get("Y"))), cv2.FONT_HERSHEY_DUPLEX, 1, (50,0,255), 2, cv2.LINE_AA)
                     # near | Shoulder Scaling
                     image = cv2.putText(image, "Near Elbow Dist : " + str(my_n_elbow_scale_values), (10,100), cv2.FONT_HERSHEY_DUPLEX, 1, (50,0,255), 2, cv2.LINE_AA)
                     # near | Shoulder Distance
                     image = cv2.putText(image, "Near Shoulder Dist : " + str(my_n_shoulder_scale_values), (600,100), cv2.FONT_HERSHEY_DUPLEX, 1, (50,0,255), 2, cv2.LINE_AA)
                 # far
                 if True:    
                     if abs(my_f_shoulder.get("X")-my_n_shoulder.get("X")) < 0.05:
                         if  my_f_shoulder.get("X")-my_n_shoulder.get("X") < 0:
                             my_f_shoulder["X"] = my_f_shoulder.get("X") - 0.05
                         else:
                             my_f_shoulder["X"] = my_f_shoulder.get("X") + 0.05
                     # far | Shoulder
                     image = cv2.putText(image, "s" + str(int(round(my_shoulder_f,0))), (int(my_width*my_f_shoulder.get("X")), int(my_height*my_f_shoulder.get("Y"))), cv2.FONT_HERSHEY_DUPLEX, 1, (50,100,255), 2, cv2.LINE_AA)
                     
                     
                     if abs(my_f_elbow.get("X")-my_n_elbow.get("X")) < 0.05:
                         if  my_f_elbow.get("X")-my_n_elbow.get("X") < 0:
                             my_f_elbow["X"] = my_f_elbow.get("X") - 0.05
                         else:
                             my_f_elbow["X"] = my_f_elbow.get("X") + 0.05
                     if abs(my_f_elbow.get("Y")-my_n_elbow.get("Y")) < 0.05:
                         if  my_f_elbow.get("Y")-my_n_elbow.get("Y") < 0:
                             my_f_elbow["Y"] = my_f_elbow.get("Y") - 0.05
                         else:
                             my_f_elbow["Y"] = my_f_elbow.get("Y") + 0.05
                                     
                     # far | Elbow
                     image = cv2.putText(image, "e" + str(int(round(FarElbow,0))), (int(my_width*my_f_elbow.get("X")), int(my_height*my_f_elbow.get("Y"))), cv2.FONT_HERSHEY_DUPLEX, 1, (50,100,255), 2, cv2.LINE_AA)
                     # far | Shoulder Scaling
                     image = cv2.putText(image, "Far Elbow Dist : " + str(my_f_elbow_scale_values), (40,100+i), cv2.FONT_HERSHEY_DUPLEX, 1, (50,0,255), 2, cv2.LINE_AA)
                     # far | Shoulder Distance
                     image = cv2.putText(image, "Far Shoulder Dist : " + str(my_f_shoulder_scale_values), (630,100+i), cv2.FONT_HERSHEY_DUPLEX, 1, (50,0,255), 2, cv2.LINE_AA)
                 
                 image = cv2.putText(image, "Foot near status : " + str(foot_n_status), (10,int(my_height*(min_y+0.05))), cv2.FONT_HERSHEY_DUPLEX, 1.3, (50,0,255), 2, cv2.LINE_AA)
                 
                 image = cv2.putText(image, "Foot far status : " + str(foot_f_status), (10,int(my_height*(min_y+0.1))), cv2.FONT_HERSHEY_DUPLEX, 1.3, (50,0,255), 2, cv2.LINE_AA)
                 
                 
                 image = cv2.putText(image, "Rotation : " + str(int(my_body_rot)), (900,int(my_height*(min_y+0.05))), cv2.FONT_HERSHEY_DUPLEX, 1.3, (50,0,255), 2, cv2.LINE_AA)
                 
                 
                 # show image
                 cv2.imshow('MediaPipe Holistic', image)
                 
                 # Send MIDI
                 if send_midi_bool:
                     my_delay = 0.1
                     
                     ################
                     ## Midi Notes ##
        #########################################################
        ## button press                                         #
        ## --> midiout.send_message([0x90, midi_ref, 100])      #
        ## knob value send                                      #
        ## --> midiout.send_message([176, midi_ref, midi_value])#
        #########################################################
                     
                     
                     ######################
                     ## Near - Midi Send ##
                     ######################
                     
                     ########
                     # Arms #
                     ######/
                     # Near | Elbow - knob
                     if abs(my_arr_NearWrist[-1] - NearElbow) > 0:
                         time.sleep(my_delay)
                         midiout.send_message([176, near_elbow_midi, NearElbow])
                         
                     # Near | Elbow Scaling - knob
                     #if abs(my_arr_NearElbowScaling[-1] - my_n_elbow_scale_values) > 4:
                     #    midiout.send_message([176, near_elbow_scaling_midi, my_n_elbow_scale_values])
                     #    time.sleep(my_delay)
                     # Near | Shoulder - knob
                     if abs(my_arr_NearShoulder[-1] - my_shoulder_n) > 0:
                         time.sleep(my_delay)
                         midiout.send_message([176, near_shoulder_midi, my_shoulder_n])
                     # Near | Shoulder Scaling - knob
                     #if near_elbow_flip_status == 1 and abs(my_arr_NearShoulderScaling[-1] - my_n_shoulder_scale_values) > 4:
                     #    time.sleep(my_delay)
                     #    midiout.send_message([176, near_shoulder_scaling_midi, my_n_shoulder_scale_values])
                     # Near | Wrist - knob
                     #if abs(my_arr_NearWrist[-1] - NearWrist) > 2:
                     #    midiout.send_message([176, near_wrist_midi, NearWrist])
                     #    time.sleep(my_delay)
                     # Near | Wrist Center Flip
                     if False:
                         if my_n_elbow_scale_values < 30 and near_elbow_flip_status_b != 1:
                             time.sleep(my_delay)
                             midiout.send_message([0x90, center_wrist_near, 100])
                             near_elbow_flip_status_b = 1
                         elif my_n_elbow_scale_values >= 30 and near_elbow_flip_status_b != 2:
                             time.sleep(my_delay)
                             midiout.send_message([0x90, center_wrist_near, 100])
                             near_elbow_flip_status_b = 2
                         # Near | Wrist Switch Buttons    
                             # Near | ↓ | Down sideways
                             if near_elbow_flip_status_b != 1:
                                 if NearElbow >= down_arm_midi_top and near_elbow_flip_status != 1 or NearElbow < down_arm_midi_bottom and near_elbow_flip_status != 1:
                                     time.sleep(my_delay)
                                     midiout.send_message([0x90, near_wrist_flip_midi[2], 100])
                                     #print("down - ", NearElbow)
                                     near_elbow_flip_status = 1
                                 # Near | ← | Left Out
                                 elif NearElbow < down_arm_midi_top and NearElbow >= up_arm_midi and near_elbow_flip_status != 3:
                                     time.sleep(my_delay)                             
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
                                 # Near | ↑ | Up sideways
                                 elif NearElbow < up_arm_midi and NearElbow >= right_arm_midi and near_elbow_flip_status != 4:
                                     time.sleep(my_delay)
                                     midiout.send_message([0x90, near_wrist_flip_midi[0], 100])
                                     #print("up - ", NearElbow)
                                     near_elbow_flip_status = 4 
                                 # Near | → | Right In
                                 elif NearElbow < right_arm_midi and NearElbow >= down_arm_midi_bottom and near_elbow_flip_status != 2:
                                     time.sleep(my_delay)
                                     midiout.send_message([0x90, near_wrist_flip_midi[1], 100])
                                     #print("in - ", NearElbow)
                                     near_elbow_flip_status = 2
                     ########
                     # Legs #
                     ######/
                     
                     if abs(my_arr_NearHip[-1] - int(my_hip_n)) > 0:
                         time.sleep(my_delay)
                         midiout.send_message([176, near_hip_midi, int(my_hip_n)])
                     
                     if abs(my_arr_NearKnee[-1] - my_knee_n) > 0:
                         time.sleep(my_delay)
                         print(my_knee_n)
                         midiout.send_message([176, near_knee_midi, my_knee_n])
                     
                     
                     #####################
                     ## Far - Midi Send ##
                     #####################
                     
                     ########
                     # Arms #
                     ######/
                     
                     # Far | Elbow - knob
                     if abs(my_arr_FarWrist[-1] - FarElbow) > 0:
                         time.sleep(my_delay)
                         midiout.send_message([176, far_elbow_midi, FarElbow])
                         
                     # Far | Elbow Scaling - knob
                     #if abs(my_arr_FarElbowScaling[-1] - my_f_elbow_scale_values) > 4:
                     #    midiout.send_message([176, far_elbow_scaling_midi, my_f_elbow_scale_values])
                     #    time.sleep(my_delay)
                     # Far | Shoulder - knob
                     if abs(my_arr_FarShoulder[-1] - my_shoulder_f) > 0:
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
                     if False:
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
                             
                     ########
                     # Legs #
                     ######/
                     
                     if abs(my_arr_FarHip[-1] - int(my_hip_f)) > 0:
                         time.sleep(my_delay)
                         midiout.send_message([176, far_hip_midi, int(my_hip_f)])
                     
                     if abs(my_arr_FarKnee[-1] - int(my_knee_f)) > 0:
                         time.sleep(my_delay)
                         midiout.send_message([176, far_knee_midi, int(my_knee_f)])
           
                     ######################
                     ## Body - Midi Send ##
                     ######################
                     #if abs(my_arr_BodyRot[-1] - my_body_rot) > 2:
                     time.sleep(my_delay)
                     midiout.send_message([176, my_body_rot_midi, my_body_rot])
                 
                 #Quit on ESC
                 if cv2.waitKey(1) & 0xFF == 27:
                     break
                           
                 #Pause to have correct play speed
                 if i_dt != 0:
                     #sl =  (datetime.fromisoformat(row[0]) - i_dt).total_seconds() 
                     #time.sleep(sl)
                     #print((float(row[55]) - i_dt)/1000)
                     sleep_time = ((float(row[55]) - i_dt) / 1000) - my_delay
                     if sleep_time < 0:
                         sleep_time = 0
                     time.sleep((float(row[55]) - i_dt) / 1000)
                     #print(sleep_time)
                 #i_dt = datetime.fromisoformat(row[0])
                 i_dt = float(row[55])
                 
             # update arrays with results for further processing
             my_arr_NearElbow.append(NearElbow)
             my_arr_NearShoulder.append(int(my_shoulder_n))
             my_arr_NearElbowScaling.append(int(my_n_elbow_scale_values)) #dist
             my_arr_NearShoulderScaling.append(int(my_n_shoulder_scale_values)) #dist
             my_arr_NearWrist.append(int(NearWrist))          
             
             my_arr_NearHip.append(int(my_hip_n))          
             my_arr_NearKnee.append(int(my_knee_n))          
             my_arr_NearFoot.append(foot_n_status)          
             
             
             my_arr_FarElbow.append(FarElbow)
             my_arr_FarShoulder.append(int(my_shoulder_f))
             my_arr_FarElbowScaling.append(int(my_f_elbow_scale_values)) #dist
             my_arr_FarShoulderScaling.append(int(my_f_shoulder_scale_values)) #dist
             my_arr_FarWrist.append(int(FarWrist))          
             
             my_arr_FarHip.append(int(my_hip_f))          
             my_arr_FarKnee.append(int(my_knee_f))          
             my_arr_FarFoot.append(foot_f_status)         
             
             my_arr_BodyRot.append(my_body_rot)
             
             
             sl_arr.append(row[55])


##########################################
# Buttons to trigger inside zone for Arm #
##########################################

# Smooth Scalling Values
my_arr_NearShoulderScaling_s = gaussian_filter1d(my_arr_NearElbowScaling, sigma=8)

# Loop over the smoothed scaled values
for i in range(len(my_arr_NearShoulderScaling_s)):
    if my_arr_NearShoulderScaling_s[i] < 40 and x_mid != 1:
       #print("wrist_in_zone")
       x_mid = 1
       my_n_shoulder_scale = 127
    elif my_arr_NearShoulderScaling_s[i] >= 40 and x_mid != 0:
       #print("wrist_out_zone")
       x_mid = 0
       my_n_shoulder_scale = 0
    my_arr_NearShoulderInside.append(int(my_n_shoulder_scale))

################
# Smooth Wrist #
################

my_arr_NearWrist = gaussian_filter1d(my_arr_NearWrist, sigma=1)

# optional smoothing
if False:
    smoother = ConvolutionSmoother(window_len=20, window_type='ones')
    smoother.smooth(my_arr_NearWrist)
    my_arr_NearWrist = smoother.smooth_data[0]


#################################
# Shoulder Scaling Post Process #
#################################

#smoothing
my_arr_NearElbowScaling = gaussian_filter1d(my_arr_NearElbowScaling, sigma=8)
#ensure top scaled is snapped all the way
for i in range(len(my_arr_NearShoulderScaling_s)):
    if my_arr_NearElbowScaling[i] > 120:
        my_arr_NearElbowScaling[i] = 127


####################################
# Elbow in/out Button Post Process #
####################################


#fix false wrist ins
smoother = ConvolutionSmoother(window_len=20, window_type='ones')
smoother.smooth(my_arr_NearShoulderInside)
i=0
for xx in smoother.smooth_data[0]:
    if int(xx) < 70:
       my_arr_NearShoulderInside[i] = 0
    else:
        my_arr_NearShoulderInside[i] = 127
    i+=1


my_arr_NearShoulderInside = gaussian_filter1d(my_arr_NearShoulderInside, sigma=5)

#fix smoothing 
for i in range(len(my_arr_NearShoulderInside)):
    if my_arr_NearShoulderInside[i] > 120:
        my_arr_NearShoulderInside[i]=127
    if my_arr_NearShoulderInside[i] < 10:
        my_arr_NearShoulderInside[i]=0




###############################
# Elbow Rotation Post Process #
###############################

        
#smooth path; avoid smoothing transition 127 <--> 0
i=1
for x in range(len(my_arr_NearElbow)):
    if my_arr_NearElbow[x] == 127 and my_arr_NearElbow[x+1] == 0:
        my_arr_NearElbow[i+3:x-3] = gaussian_filter1d(my_arr_NearElbow[i+3:x-3], sigma=0.4)



#######################
# Wrist Switch Button #
#######################


for x in range(len(my_arr_NearElbow)):
    # ←
    if my_arr_NearElbow[x] < 32:
            tmp_ind_r = 0
    # →
    elif my_arr_NearElbow[x] < 64:
            tmp_ind_r = 32
    # ↓
    elif my_arr_NearElbow[x] < 96:
            tmp_ind_r = 96
    # ↑
    else:
        tmp_ind_r = 127
    
    Wrist_switch = tmp_ind_r
    my_arr_NearWristSwitch.append(Wrist_switch)





################
# Smooth Legs  #
################

if True:
    
    #smooth path; avoid smoothing transition 127 <--> 0
    i=1
    for x in range(len(my_arr_FarKnee)):
        if my_arr_FarKnee[x] == 127 and my_arr_FarKnee[x+1] == 0:
            my_arr_FarKnee[i+3:x-3] = gaussian_filter1d(my_arr_FarKnee[i+3:x-3], sigma=0.4)

    i=1
    for x in range(len(my_arr_FarHip)):
        if my_arr_FarHip[x] == 127 and my_arr_FarHip[x+1] == 0:
            my_arr_FarHip[i+3:x-3] = gaussian_filter1d(my_arr_FarHip[i+3:x-3], sigma=0.4)

    i=1
    for x in range(len(my_arr_NearKnee)):
        if my_arr_NearKnee[x] == 127 and my_arr_NearKnee[x+1] == 0:
            my_arr_NearKnee[i+3:x-3] = gaussian_filter1d(my_arr_NearKnee[i+3:x-3], sigma=0.4)

    i=1
    for x in range(len(my_arr_NearHip)):
        if my_arr_NearHip[x] == 127 and my_arr_NearHip[x+1] == 0:
            my_arr_NearHip[i+3:x-3] = gaussian_filter1d(my_arr_NearHip[i+3:x-3], sigma=0.4)



########################
# Smooth Body Rotation #
########################

if True:
    
    i=1
    for x in range(len(my_arr_BodyRot)):
        if my_arr_BodyRot[x] == 127 and my_arr_BodyRot[x+1] == 0:
            my_arr_BodyRot[i+3:x-3] = gaussian_filter1d(my_arr_BodyRot[i+3:x-3], sigma=0.4)
            


#############
# The Graph #
#############

# Wrist Rotation
#plt.plot(my_arr_NearWrist)

# Elbow Rotation
#plt.plot(my_arr_NearElbow)

# Shoulder Rotation
#plt.plot(my_arr_NearShoulder)

# Wrist/Elbow zone switch
#plt.plot(my_arr_NearWristSwitch)

# Elbow Scalling
#plt.plot(my_arr_NearElbowScaling)

# Inside zone wrist flip
#plt.plot(my_arr_NearShoulderInside)

# Body Rotation
plt.plot(my_arr_BodyRot)

# Far Hip Rotation
#plt.plot(my_arr_FarHip)

# Far Knee Rotation
#plt.plot(my_arr_FarKnee)



################
# Save to File #
################

# Clean up existing files
if os.path.exists("frontflip.csv"):
    os.remove("frontflip.csv") 



'''
int(my_arr_NearElbow[x]),
my_arr_NearShoulder[x],
my_arr_NearWristSwitch[x],
my_arr_NearElbowScaling[x],
my_arr_NearShoulderInside[x],
my_arr_FarKnee[x],
my_arr_FarHip[x],
my_arr_NearKnee[x],
my_arr_NearHip[x],
int(my_arr_BodyRot[x])
'''
    
    
with open('frontflip.csv', 'w', newline='') as csvfile:
            spamwriter = csv.writer(csvfile, delimiter=',')
            for x in range(len(my_arr_NearElbow)):
                    spamwriter.writerow([sl_arr[x],
                                         int(my_arr_NearElbow[x]),
                                         int(my_arr_NearShoulder[x]),
                                         int(my_arr_FarElbow[x]),
                                         int(my_arr_FarShoulder[x]),
                                         int(my_arr_NearHip[x]),
                                         int(my_arr_NearKnee[x]),
                                         int(my_arr_FarHip[x]),
                                         int(my_arr_FarKnee[x]),
                                         int(my_arr_BodyRot[x])
                                       ])
                    

if send_midi_bool:            
    midiout.close_port()







