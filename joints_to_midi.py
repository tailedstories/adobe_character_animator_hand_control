# Importing Exported Midports
import csv
import json
import time
import math
import matplotlib.pyplot as plt
import numpy as np
import cv2
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


##################
# Init variables #
##################

tmp_ind_r=0
NearElbow=None



dist_max = 0
dist_min = 0

my_arr_NearElbow = []
my_arr_NearShoulder = []
my_arr_NearShoulderInside = []
my_arr_NearShoulderScaling = []
my_arr_NearWrist = []
my_arr_NearWristSwitch = []

sl_arr = []
sl=0

New_val_old=0
x_mid=0
my_n_shoulder_scale = 0

i_dt=0


####################################################
# Calibrate the distance to calculate arm scalling #
####################################################

with open('points.csv', newline='') as csvfile:
     spamreader = csv.reader(csvfile, delimiter=',')
     for row in spamreader:
         my_n_elbow = json.loads(row[15].replace("'",'"'))
         my_n_wrist = json.loads(row[17].replace("'",'"'))

         my_x = abs(my_n_wrist.get("X")-my_n_elbow.get("X"))*1000
         my_y = abs(my_n_wrist.get("Y")-my_n_elbow.get("Y"))*1000
            
         dist = math.hypot(my_x, my_y)
         
         if dist > dist_max:
             dist_max = dist
         if dist < dist_min:
             dist_min = dist

#adjust distance min/max threshhold
dist_max -= 60
dist_min += 80

#print("Dist min - ", dist_min)
#print("Dist max - ", dist_max)


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
             my_n_hip = json.loads(row[25].replace("'",'"'))
             my_f_shoulder = json.loads(row[12].replace("'",'"'))
             my_n_shoulder = json.loads(row[13].replace("'",'"'))
             my_n_elbow = json.loads(row[15].replace("'",'"'))
             my_n_wrist = json.loads(row[17].replace("'",'"'))
             my_n_h_wrist = json.loads(row[34+13].replace("'",'"'))
             
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
                
             my_elbow_r = myAngle(my_n_elbow.get("X"),my_n_elbow.get("Y"),my_n_wrist.get("X"),my_n_wrist.get("Y"))
             NearElbow = int(remap(int(my_elbow_r), 0, 360, 0, 127))
             
             #################
             # Near shoulder #
             #################
             
             my_shoulder_r = myAngle(my_n_shoulder.get("X"),my_n_shoulder.get("Y"),my_n_elbow.get("X"),my_n_elbow.get("Y"))
             my_shoulder_r = remap(int(my_shoulder_r), 0, 360, 0, 127)
             
             #########################
             # Near Shoulder Scaling #
             #########################
             
             my_x = abs(my_n_wrist.get("X")-my_n_elbow.get("X"))*1000
             my_y = abs(my_n_wrist.get("Y")-my_n_elbow.get("Y"))*1000
             dist = math.hypot(my_x, my_y)
             if dist > dist_max:
                 dist = dist_max
             if dist < dist_min:
                 dist = dist_min
             my_n_shoulder_scale_values = int(remap(int(dist), dist_min, dist_max, 0, 127))
             if my_n_shoulder_scale_values >= 120:
                 my_n_shoulder_scale_values = 127
             
                
             ##################
             # Display Joints #
             ##################
             
             #set to false to disablle player
             if True:
                 #Prepare Green Screen
                 image = np.zeros((my_height,my_width,3), np.uint8)
                 image[:] = [0,255,0]
                 image.flags.writeable = False
                 results = image
                 image.flags.writeable = True
                 image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
                   
                 #Draw Landmarks (joint lines)
                 # near | elbow --> wrist
                 cv2.line(image, (int(my_width*my_n_wrist.get("X")), int(my_height*my_n_wrist.get("Y"))), (int(my_width*my_n_elbow.get("X")), int(my_height*my_n_elbow.get("Y"))), (200,50,50),5)
                 # near | shoulder --> elbow
                 cv2.line(image, (int(my_width*my_n_shoulder.get("X")), int(my_height*my_n_shoulder.get("Y"))), (int(my_width*my_n_elbow.get("X")), int(my_height*my_n_elbow.get("Y"))), (200,0,50),5)
                 # near | hip --> shoulder
                 cv2.line(image, (int(my_width*my_n_shoulder.get("X")), int(my_height*my_n_shoulder.get("Y"))), (int(my_width*my_n_hip.get("X")), int(my_height*my_n_hip.get("Y"))), (255,55,150),5)
                 
                 # near | Midi value -- Shoulder
                 image = cv2.putText(image, str(int(round(my_shoulder_r,0))), (int(my_width*my_n_shoulder.get("X")), int(my_height*my_n_shoulder.get("Y"))), cv2.FONT_HERSHEY_DUPLEX, 1.3, (50,0,255), 2, cv2.LINE_AA)
                 # near | Midi value -- Elbow
                 image = cv2.putText(image, str(int(round(NearElbow,0))), (int(my_width*my_n_elbow.get("X")), int(my_height*my_n_elbow.get("Y"))), cv2.FONT_HERSHEY_DUPLEX, 1.3, (50,0,255), 2, cv2.LINE_AA)
                 # near | Midi value -- Shoulder Scaling
                 image = cv2.putText(image, "Elbow Dist: " + str(my_n_shoulder_scale_values), (10,100), cv2.FONT_HERSHEY_DUPLEX, 1.3, (50,0,255), 2, cv2.LINE_AA)
                 image = cv2.putText(image, "Shoulder Dist: " + str(my_n_shoulder_scale_values), (600,100), cv2.FONT_HERSHEY_DUPLEX, 1.3, (50,0,255), 2, cv2.LINE_AA)
                 
                 # show image
                 cv2.imshow('MediaPipe Holistic', image)
                    
                 #Quit on ESC
                 if cv2.waitKey(1) & 0xFF == 27:
                     break
                           
                 #Pause to have correct play speed
                 if i_dt != 0:
                     #sl =  (datetime.fromisoformat(row[0]) - i_dt).total_seconds() 
                     #time.sleep(sl)
                     #print((float(row[55]) - i_dt)/1000)
                     time.sleep((float(row[55]) - i_dt)/1000)
                 #i_dt = datetime.fromisoformat(row[0])
                 i_dt = float(row[55])
                 
             # update arrays with results for further processing
             my_arr_NearElbow.append(NearElbow)
             my_arr_NearShoulder.append(int(my_shoulder_r))
             my_arr_NearShoulderScaling.append(int(my_n_shoulder_scale_values)) #dist
             my_arr_NearWrist.append(int(NearWrist))             
             sl_arr.append(row[0])


##########################################
# Buttons to trigger inside zone for Arm #
##########################################

# Smooth Scalling Values
my_arr_NearShoulderScaling_s = gaussian_filter1d(my_arr_NearShoulderScaling, sigma=8)

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
my_arr_NearShoulderScaling = gaussian_filter1d(my_arr_NearShoulderScaling, sigma=8)
#ensure top scaled is snapped all the way
for i in range(len(my_arr_NearShoulderScaling_s)):
    if my_arr_NearShoulderScaling[i] > 120:
        my_arr_NearShoulderScaling[i]=127




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
#plt.plot(my_arr_NearShoulderScaling)

# Inside zone wrist flip
#plt.plot(my_arr_NearShoulderInside)



################
# Save to File #
################

with open('elbow.csv', 'w', newline='') as csvfile:
            spamwriter = csv.writer(csvfile, delimiter=',')
            for x in range(len(my_arr_NearElbow)):
                    spamwriter.writerow([sl_arr[x],
                                         int(my_arr_NearElbow[x]),
                                         my_arr_NearShoulder[x],
                                         my_arr_NearWristSwitch[x],
                                         my_arr_NearShoulderScaling[x],
                                         my_arr_NearShoulderInside[x]
                                       ])
                    

            







