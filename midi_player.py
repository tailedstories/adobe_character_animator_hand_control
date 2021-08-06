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

# send midi while playing
send_midi_bool = False


###############
# MIDI Values #
###############

far_shoulder_scaling_midi = 11
far_shoulder_midi   = 17
far_elbow_midi      = 18
far_wrist_midi      = 80
far_wrist_flip_midi = 81
far_elbow_scaling_midi = 10
#center
center_wrist_far = 7
#Left Arm
near_shoulder_midi   = 16
near_elbow_midi      = 21
near_wrist_midi      = 19
near_wrist_flip_midi = 83

near_elbow_scaling_midi = 10    # wrong ref
near_shoulder_scaling_midi = 10 # wrong ref

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


if send_midi_bool:
    import rtmidi
    #this should be a reference to a loopmidi (number)
    midiout = rtmidi.MidiOut()
    midiout.open_port(1)

tmp_ind_r=0
NearElbow=None



dist_max = 0
dist_min = 99999
dist_max_should = 0
dist_min_should = 99999

my_arr_NearElbow = []
my_arr_NearShoulder = []
my_arr_NearShoulderInside = []
my_arr_NearShoulderScaling = []
my_arr_NearElbowScaling = []
my_arr_NearWrist = []
my_arr_NearWristSwitch = []

my_arr_FarElbow = []
my_arr_FarShoulder = []
my_arr_FarShoulderInside = []
my_arr_FarShoulderScaling = []
my_arr_FarElbowScaling = []
my_arr_FarWrist = []


sl_arr = []
sl = 0

New_val_old = 0
x_mid = 0
my_n_shoulder_scale = 0

i_dt = 0


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
             
         my_x = abs(my_n_elbow.get("X")-my_n_shoulder.get("X"))*1000
         my_y = abs(my_n_elbow.get("Y")-my_n_shoulder.get("Y"))*1000
         dist = math.hypot(my_x, my_y)
         if dist > dist_max_should:
             dist_max_should = dist
         if dist < dist_min_should:
             dist_min_should = dist

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
             my_n_shoulder = json.loads(row[13].replace("'",'"'))
             my_n_elbow = json.loads(row[15].replace("'",'"'))
             my_n_wrist = json.loads(row[17].replace("'",'"'))
             my_n_h_wrist = json.loads(row[34+13].replace("'",'"'))
             
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
                
             my_elbow_n = myAngle(my_n_elbow.get("X"),my_n_elbow.get("Y"),my_n_wrist.get("X"),my_n_wrist.get("Y"))
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
                 dist = dist_max_should
             if dist < dist_min_should:
                 dist = dist_min_should
             my_n_shoulder_scale_values = int(remap(int(dist), dist_min, dist_max, 0, 127))
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
                 
                 # Draw Near Arm
                 if True:    
                     #Draw Landmarks (joint lines)
                     # near | elbow --> wrist
                     cv2.line(image, (int(my_width*my_n_wrist.get("X")), int(my_height*my_n_wrist.get("Y"))), (int(my_width*my_n_elbow.get("X")), int(my_height*my_n_elbow.get("Y"))), (200,50,50),5)
                     # near | shoulder --> elbow
                     cv2.line(image, (int(my_width*my_n_shoulder.get("X")), int(my_height*my_n_shoulder.get("Y"))), (int(my_width*my_n_elbow.get("X")), int(my_height*my_n_elbow.get("Y"))), (200,0,50),5)
                     # near | hip --> shoulder
                     cv2.line(image, (int(my_width*my_n_shoulder.get("X")), int(my_height*my_n_shoulder.get("Y"))), (int(my_width*my_n_hip.get("X")), int(my_height*my_n_hip.get("Y"))), (255,55,150),5)
                     
                     cv2.line(image, (int(my_width*my_n_elbow.get("X")), int(my_height*my_n_elbow.get("Y"))), (int(my_width*my_n_hip.get("X")), int(my_height*my_n_hip.get("Y"))), (0,0,255),2)
                 
                 # Draw Far Arm
                 if True:    
                     #Draw Landmarks (joint lines)
                     # near | elbow --> wrist
                     cv2.line(image, (int(my_width*my_f_wrist.get("X")), int(my_height*my_f_wrist.get("Y"))), (int(my_width*my_f_elbow.get("X")), int(my_height*my_f_elbow.get("Y"))), (200,50,50),5)
                     # near | shoulder --> elbow
                     cv2.line(image, (int(my_width*my_f_shoulder.get("X")), int(my_height*my_f_shoulder.get("Y"))), (int(my_width*my_f_elbow.get("X")), int(my_height*my_f_elbow.get("Y"))), (200,0,50),5)
                     # near | hip --> shoulder
                     cv2.line(image, (int(my_width*my_f_shoulder.get("X")), int(my_height*my_f_shoulder.get("Y"))), (int(my_width*my_f_hip.get("X")), int(my_height*my_f_hip.get("Y"))), (255,55,150),5)
                     
                     cv2.line(image, (int(my_width*my_f_elbow.get("X")), int(my_height*my_f_elbow.get("Y"))), (int(my_width*my_f_hip.get("X")), int(my_height*my_f_hip.get("Y"))), (0,0,255),2)   
                 
                 # Draw Midi Values
                 # near
                 i=0
                 if True:
                     # adjust position for Far distance
                     i+=50
                     # near | Shoulder
                     image = cv2.putText(image, str(int(round(my_shoulder_n,0))), (int(my_width*my_n_shoulder.get("X")), int(my_height*my_n_shoulder.get("Y"))), cv2.FONT_HERSHEY_DUPLEX, 1.3, (50,0,255), 2, cv2.LINE_AA)
                     # near | Elbow
                     image = cv2.putText(image, str(int(round(NearElbow,0))), (int(my_width*my_n_elbow.get("X")), int(my_height*my_n_elbow.get("Y"))), cv2.FONT_HERSHEY_DUPLEX, 1.3, (50,0,255), 2, cv2.LINE_AA)
                     # near | Shoulder Scaling
                     image = cv2.putText(image, "Near Elbow Dist : " + str(my_n_elbow_scale_values), (10,100), cv2.FONT_HERSHEY_DUPLEX, 1.3, (50,0,255), 2, cv2.LINE_AA)
                     # near | Shoulder Distance
                     image = cv2.putText(image, "Near Shoulder Dist : " + str(my_n_shoulder_scale_values), (600,100), cv2.FONT_HERSHEY_DUPLEX, 1.3, (50,0,255), 2, cv2.LINE_AA)
                 # far
                 if True:    
                     # far | Shoulder
                     image = cv2.putText(image, str(int(round(my_shoulder_f,0))), (int(my_width*my_f_shoulder.get("X")), int(my_height*my_f_shoulder.get("Y"))), cv2.FONT_HERSHEY_DUPLEX, 1.3, (50,0,255), 2, cv2.LINE_AA)
                     # far | Elbow
                     image = cv2.putText(image, str(int(round(FarElbow,0))), (int(my_width*my_f_elbow.get("X")), int(my_height*my_f_elbow.get("Y"))), cv2.FONT_HERSHEY_DUPLEX, 1.3, (50,0,255), 2, cv2.LINE_AA)
                     # far | Shoulder Scaling
                     image = cv2.putText(image, "Far Elbow Dist : " + str(my_f_elbow_scale_values), (40,100+i), cv2.FONT_HERSHEY_DUPLEX, 1.3, (50,0,255), 2, cv2.LINE_AA)
                     # far | Shoulder Distance
                     image = cv2.putText(image, "Far Shoulder Dist : " + str(my_f_shoulder_scale_values), (630,100+i), cv2.FONT_HERSHEY_DUPLEX, 1.3, (50,0,255), 2, cv2.LINE_AA)
                     
                 # show image
                 cv2.imshow('MediaPipe Holistic', image)
                 
                 # Send MIDI
                 if send_midi_bool:
                     my_delay = 0.001
                     
                     ################
                     ## Midi Notes ##
                     ################
                     ## button press
                     ## --> midiout.send_message([0x90, midi_ref, 100])
                     ## knob value send
                     ## --> midiout.send_message([176, midi_ref, midi_value])
                     ################
                     
                     midiout.send_message([176, near_elbow_midi, NearElbow])
                     time.sleep(my_delay)
                     midiout.send_message([176, near_shoulder_midi, my_shoulder_n])
                     time.sleep(my_delay)
                     midiout.send_message([176, near_elbow_scaling_midi, my_n_elbow_scale_values])
                     time.sleep(my_delay)
                     midiout.send_message([176, near_shoulder_scaling_midi, my_n_shoulder_scale_values])
                     time.sleep(my_delay)
                     midiout.send_message([176, near_wrist_midi, NearWrist])
                     time.sleep(my_delay)
                     

                     
                 
                 #Quit on ESC
                 if cv2.waitKey(1) & 0xFF == 27:
                     break
                           
                 #Pause to have correct play speed
                 if i_dt != 0:
                     #sl =  (datetime.fromisoformat(row[0]) - i_dt).total_seconds() 
                     #time.sleep(sl)
                     #print((float(row[55]) - i_dt)/1000)
                     time.sleep((float(row[55]) - i_dt) / 1000)
                 #i_dt = datetime.fromisoformat(row[0])
                 i_dt = float(row[55])
                 
             # update arrays with results for further processing
             my_arr_NearElbow.append(NearElbow)
             my_arr_NearShoulder.append(int(my_shoulder_n))
             my_arr_NearElbowScaling.append(int(my_n_elbow_scale_values)) #dist
             my_arr_NearShoulderScaling.append(int(my_n_shoulder_scale_values)) #dist
             my_arr_NearWrist.append(int(NearWrist))          
             
             my_arr_FarElbow.append(FarElbow)
             my_arr_FarShoulder.append(int(my_shoulder_f))
             my_arr_FarElbowScaling.append(int(my_f_elbow_scale_values)) #dist
             my_arr_FarShoulderScaling.append(int(my_f_shoulder_scale_values)) #dist
             my_arr_FarWrist.append(int(FarWrist))          
             
             
             
             
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
plt.plot(my_arr_NearElbowScaling)

# Inside zone wrist flip
plt.plot(my_arr_NearShoulderInside)



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
                                         my_arr_NearElbowScaling[x],
                                         my_arr_NearShoulderInside[x]
                                       ])
                    

            







