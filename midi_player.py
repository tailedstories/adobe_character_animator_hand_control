import rtmidi
import csv
import json
import time
from datetime import datetime
import math
import matplotlib.pyplot as plt
import numpy as np
import cv2
from tsmoothie.smoother import *
from tsmoothie.bootstrap import BootstrappingWrapper
from scipy.signal import argrelextrema
from scipy.ndimage.filters import gaussian_filter1d
from scipy.signal import find_peaks
from math import atan2, cos, sin, degrees
# max speed 


# min movement angle





midiout = rtmidi.MidiOut()
available_ports = midiout.get_ports()



#########
## SET ##
## UP  ##
#########

#this should be a reference to a loopmidi (number)
midiout.open_port(1)
#this should be a reference to your camera (number)
cam_ref=3

my_height = 960
my_width  = 1280

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
    #a1 = myAngle(x1,y1,x2,y2)
    #a2 = myAngle(x2,y2,x3,y3)
    
    
    points = np.array([[x1, y1], [x2, y2], [x3, y3]])
    
    A = points[1] - points[0]
    B = points[2] - points[1]
    C = points[0] - points[2]
    
    angles = []
    for e1, e2 in ((A, -B), (B, -C), (C, -A)):
        num = np.dot(e1, e2)
        denom = np.linalg.norm(e1) * np.linalg.norm(e2)
        angles.append(np.arccos(num/denom) * 180 / np.pi)
    #print angles
    #print sum(angles)
    
    #if a1 > a2 :
    #   sign =  1
    #else:
    #   sign = -1
    
    #result_angle = a1 - a2
    #K = -sign * math.pi * 2
    
    #if (abs(K + result_angle) < abs(result_angle)): 
    #    result_angle = K + result_angle
    #else:
    #    result_angle = result_angle
    #return round(result_angle)
    return angles[0]

def remap(x, in_min, in_max, out_min, out_max):
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

def myAngle(cx, cy, ex, ey):
  #Vmyradians = math.atan2(cy-ey, cx-ex)
  #theta = math.degrees(Vmyradians)
  
  #distance = [cx - ex, cy - ey]
  #norm = math.sqrt(distance[0] ** 2 + distance[1] ** 2)
  #direction = [distance[0] / norm, distance[1] / norm]
  #bullet_vector = [direction[0] * math.sqrt(2), direction[1] * math.sqrt(2)]
  
  
  angle = atan2(cos(cx)*sin(ex)-sin(cx) * cos(ex)*cos(ey-cy), sin(ey-cy)*cos(ex))
  bearing = (degrees(angle) + 360) % 360

  return bearing 


def deftreePoinAngle(p0, p1=np.array([0,0]), p2=None):
    if p2 is None:
        p2 = p1 + np.array([1, 0])
    v0 = np.array(p0) - np.array(p1)
    v1 = np.array(p2) - np.array(p1)
  
    angle = np.math.atan2(np.linalg.det([v0,v1]),np.dot(v0,v1))
    return np.degrees(angle)

def myNewAngle(cx, cy, ex, ey):
  dy = ey - cy
  dx = ex - cx
  theta = math.atan2(dy, dx) 
  theta *= 180 / math.pi
  theta = math.degrees(theta)
  return round(theta)


tmp_ind_r=0
NearElbow=None


elbow_min = 99999
elbow_max = -99999
elbow_min_side = 99999
elbow_max_side = -99999

shoulder_min = 99999
shoulder_max = -99999

dist_max = 0
dist_min = 0

my_arr_1_x = []
my_arr_1_y = []
my_arr_2_x = []
my_arr_2_y = []
my_arr_3_x = []
my_arr_3_y = []

my_arr_1_xy = []
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


with open('points.csv', newline='') as csvfile:
     spamreader = csv.reader(csvfile, delimiter=',')
     for row in spamreader:
         my_n_hip = json.loads(row[25].replace("'",'"'))
         my_f_shoulder = json.loads(row[12].replace("'",'"'))
         my_n_shoulder = json.loads(row[13].replace("'",'"'))
         my_n_elbow = json.loads(row[15].replace("'",'"'))
         my_n_wrist = json.loads(row[17].replace("'",'"'))
         
         my_shoulder_r = myAngle(my_n_shoulder.get("X"),my_n_shoulder.get("Y"),my_n_elbow.get("X"),my_n_elbow.get("Y"))
         
         #my_shoulder_r = deftreePoinAngle([my_n_shoulder.get("X"),my_n_shoulder.get("Y")],
         #                                     [my_n_elbow.get("X"),my_n_elbow.get("Y")],
         #                                     [my_f_shoulder.get("X"),my_f_shoulder.get("Y")])
         
         
             #my_shoulder_r = prepangle(my_f_shoulder.get("X") , my_f_shoulder.get("Y"),
             #                          my_n_elbow.get("X")    , my_n_elbow.get("Y"),
             #                          my_n_shoulder.get("X") , my_n_shoulder.get("Y"))
         #else:
         #    my_shoulder_r = prepangle(my_f_shoulder.get("X") , my_f_shoulder.get("Y"),
         #                              my_n_elbow.get("X")    , my_n_elbow.get("Y"),
         #                              my_n_shoulder.get("X") , my_n_shoulder.get("Y"))
         #if my_n_elbow.get("X") < my_n_shoulder.get("X") and my_n_elbow.get("Y") < my_n_shoulder.get("Y"):
         #    my_shoulder_r = prepangle(my_n_shoulder.get("X") , my_n_shoulder.get("Y"),
         #                          my_f_shoulder.get("X") , my_f_shoulder.get("Y"),
         #                          my_n_elbow.get("X")    , my_n_elbow.get("Y"))
         #else:
         #    my_shoulder_r = 0
             #my_shoulder_r *= -1
         
         #if my_shoulder_r > 0:
         #    my_shoulder_r = 0
         #if my_shoulder_r < -100:
         #    my_shoulder_r = -100
         #my_shoulder_r = remap(int(my_shoulder_r), -110, 80, 0, 127)
         my_arr_NearShoulder.append(int(my_shoulder_r))
my_arr_NearShoulder_save = my_arr_NearShoulder
shoulder_min = min(my_arr_NearShoulder)
shoulder_max = max(my_arr_NearShoulder)
print(min(my_arr_NearShoulder))
print(max(my_arr_NearShoulder))

for x in range(len(my_arr_NearShoulder)-1):
     #print(abs(my_arr_NearShoulder[x] - my_arr_NearShoulder[x+1]))
     if abs(my_arr_NearShoulder[x] - my_arr_NearShoulder[x+1])>5:
         my_arr_NearShoulder[x+1] = my_arr_NearShoulder[x]

#smoother = ConvolutionSmoother(window_len=80, window_type='ones')
#smoother.smooth(my_arr_NearShoulder)

#my_arr_NearShoulder = smoother.smooth_data[0]
i_dt=0
i=0
with open('points.csv', newline='') as csvfile:
     spamreader = csv.reader(csvfile, delimiter=',')
     for row in spamreader:
         my_n_shoulder = json.loads(row[13].replace("'",'"'))
         my_n_elbow = json.loads(row[15].replace("'",'"'))
         my_n_wrist = json.loads(row[17].replace("'",'"'))
         #my_elbow_r = prepangle(my_n_shoulder.get("X"),my_n_shoulder.get("Y"),
         #                              my_n_elbow.get("X"),my_n_elbow.get("Y"),
         #                              my_n_wrist.get("X"),my_n_wrist.get("Y"))
         my_elbow_r = myAngle(my_n_elbow.get("X"),my_n_elbow.get("Y"),my_n_wrist.get("X"),my_n_wrist.get("Y"))
         
         if my_elbow_r < elbow_min_side and my_arr_NearShoulder[i] >= 50:
             elbow_min_side = my_elbow_r
         if my_elbow_r > elbow_max_side and my_arr_NearShoulder[i] >= 50:
             elbow_max_side = my_elbow_r
             
         if my_elbow_r < elbow_min and my_arr_NearShoulder[i] < 50:
             elbow_min = my_elbow_r
         if my_elbow_r > elbow_max and my_arr_NearShoulder[i] < 50:
             elbow_max = my_elbow_r
         #print(my_arr_NearShoulder[i] )
         
         my_x = abs(my_n_wrist.get("X")-my_n_elbow.get("X"))*1000
         my_y = abs(my_n_wrist.get("Y")-my_n_elbow.get("Y"))*1000
            
            
         dist = math.hypot(my_x, my_y)
         
         if dist > dist_max:
             dist_max = dist
         if dist < dist_min:
             dist_min = dist
         i+=1

print("Elbow min - ", elbow_min)
print("Elbow max - ", elbow_max)

print("Elbow min - ", elbow_min_side)
print("Elbow max - ", elbow_max_side)

dist_max -= 60
dist_min += 80

print("Dist min - ", dist_min)
print("Dist max - ", dist_max)



my_arr_NearShoulder_b = my_arr_NearShoulder
my_arr_NearShoulder=[]


with open('points.csv', newline='') as csvfile:
     spamreader = csv.reader(csvfile, delimiter=',')
     i=0
     i_shoulder =  0
     for row in spamreader:
         #x = row
         if len(row) > 0:
             if i != 0:
                 sl =  (datetime.fromisoformat(row[0]) - i).total_seconds()
                 #time.sleep(sl)
                 #print(sl)
             json_process = json.loads(row[1].replace("'",'"'))
             #print(row[1])
             
             i = datetime.fromisoformat(row[0])
         
             
             #+34 +1
             my_n_hip = json.loads(row[25].replace("'",'"'))
             
             my_f_shoulder = json.loads(row[12].replace("'",'"'))
            
             my_n_shoulder = json.loads(row[13].replace("'",'"'))
             my_n_elbow = json.loads(row[15].replace("'",'"'))
             my_n_wrist = json.loads(row[17].replace("'",'"'))
             
             my_n_h_wrist = json.loads(row[34+13].replace("'",'"'))
             
             
             # Near Wrist
             #NewValue = myAngle(my_n_h_wrist.get("X"),my_n_h_wrist.get("Y"),my_n_wrist.get("X"),my_n_wrist.get("Y"))
             NewValue = prepangle( my_n_elbow.get("X"),my_n_elbow.get("Y"),
                                   my_n_wrist.get("X"),my_n_wrist.get("Y"),
                                   my_n_h_wrist.get("X"),my_n_h_wrist.get("Y"))
             
             if my_n_h_wrist.get("X") != 0 or my_n_h_wrist.get("Y") != 0:
                 if NewValue < 0:
                     NewValue = 0
                 if NewValue > 110:
                     NewValue = 110        
                 NearWrist = int(remap(int(NewValue), 0, 110, 0, 127))
             else:
                 NearWrist = my_arr_NearWrist[-1]
             #print(NewValue)
             #if NearWrist > 100:
             #    NearWrist = 127
             #print(NearWrist)
             #print(my_n_shoulder)
             
             # Near elbow
             #my_elbow_r = prepangle(my_n_shoulder.get("X"),my_n_shoulder.get("Y"),
             #                          my_n_elbow.get("X"),my_n_elbow.get("Y"),
             #                          my_n_wrist.get("X"),my_n_wrist.get("Y"))
             my_elbow_r = myAngle(my_n_elbow.get("X"),my_n_elbow.get("Y"),my_n_wrist.get("X"),my_n_wrist.get("Y"))
             
             NewValue = None
             # Near elbow
             NewValue = remap(int(my_elbow_r), 0, 360, 0, 127)
             
             #if my_arr_NearShoulder_b[i_shoulder] < 50:
             #    NewValue = remap(int(my_elbow_r), elbow_min, elbow_max, 0, 127)
             #elif my_arr_NearShoulder_b[i_shoulder] >= 50:
             #    NewValue = remap(int(my_elbow_r), elbow_min_side, elbow_max_side, 0, 127)
             #midiout.send_message([176, right_elbow_midi, int(NewValue)])
             
             
             
             # Near shoulder 
             my_shoulder_r = myAngle(my_n_shoulder.get("X"),my_n_shoulder.get("Y"),my_n_elbow.get("X"),my_n_elbow.get("Y"))
             
             
             
             #my_shoulder_r = deftreePoinAngle([my_n_shoulder.get("X"),my_n_shoulder.get("Y")],
             #                                 [my_n_elbow.get("X"),my_n_elbow.get("Y")],
             #                                 [my_f_shoulder.get("X"),my_f_shoulder.get("Y")])
             #print(my_shoulder_r)
             #my_shoulder_r = prepangle(my_n_shoulder.get("X") , my_n_shoulder.get("Y"),
             #                      my_f_shoulder.get("X") , my_f_shoulder.get("Y"),
             #                      my_n_elbow.get("X")    , my_n_elbow.get("Y"))
             #my_shoulder_r2 = prepangle(my_n_shoulder.get("X") , my_n_shoulder.get("Y"),
             #                          my_n_hip.get("X") , my_n_hip.get("Y"),
             #                          my_n_elbow.get("X")    , my_n_elbow.get("Y"))
             
             #my_shoulder_r+=my_shoulder_r2
             
             
             #if my_n_elbow.get("Y") > my_n_shoulder.get("Y") or my_n_elbow.get("X") > my_n_shoulder.get("X"):
                 #my_shoulder_r = prepangle(my_n_shoulder.get("X") , my_n_shoulder.get("Y"),
                 #                      my_f_shoulder.get("X") , my_f_shoulder.get("Y"),
                 #                      my_n_elbow.get("X")    , my_n_elbow.get("Y"))
             #    my_shoulder_r = prepangle(my_f_shoulder.get("X") , my_f_shoulder.get("Y"),
             #                          my_n_elbow.get("X")    , my_n_elbow.get("Y"),
             #                          my_n_shoulder.get("X") , my_n_shoulder.get("Y"))
             #else:
             #    my_shoulder_r = prepangle(my_f_shoulder.get("X") , my_f_shoulder.get("Y"),
             #                          my_n_elbow.get("X")    , my_n_elbow.get("Y"),
             #                          my_n_shoulder.get("X") , my_n_shoulder.get("Y"))
             #    my_shoulder_r *= -1
             
             
             
             #if my_n_elbow.get("Y") < my_n_shoulder.get("Y"):
             #if my_n_elbow.get("X") < my_n_shoulder.get("X"):
             #if my_n_elbow.get("X") < my_n_shoulder.get("X") or my_n_elbow.get("Y") > my_n_shoulder.get("Y"):
             #    my_shoulder_r *= -1
             #if my_shoulder_r > shoulder_max:
             #    my_shoulder_r = shoulder_max
             #if my_shoulder_r < shoulder_min:
             #    my_shoulder_r = shoulder_min
             #print(my_shoulder_r)
             
                 #midiout.send_message([176, right_shoulder_midi, int(NewValue)])
                 #print ("Shoulder right - ", int(NewValue))
                 #time.sleep(0.1)
                
             
                
             # Near Shoulder Scaling
             my_x = abs(my_n_wrist.get("X")-my_n_elbow.get("X"))*1000
             my_y = abs(my_n_wrist.get("Y")-my_n_elbow.get("Y"))*1000
            
            
             dist = math.hypot(my_x, my_y)
             
             #print(dist)
             
             #New_val = (my_x+my_y)/2
             #if New_val < 60:
             #    New_val = 60
             #elif New_val > 200:
             #    New_val = 200
             #my_n_shoulder_scale_values = int(remap(int(New_val), 60, 200, 0, 127))
             
             #print(dist)
             if dist > dist_max:
                 dist = dist_max
             if dist < dist_min:
                 dist = dist_min
             my_n_shoulder_scale_values = int(remap(int(dist), dist_min, dist_max, 0, 127))
             #if abs(New_val - New_val_old)> 5:
             #    midiout.send_message([176, right_shoulder_scaling_midi, New_val])
             if my_n_shoulder_scale_values >= 120:
                 my_n_shoulder_scale_values = 127
             #print(dist)
             
        
             
             
             
             if NewValue != None:
                 NearElbow = int(NewValue)
 
             #print(NearElbow)
             
             
             my_shoulder_r = remap(int(my_shoulder_r), 0, 360, 0, 127)
             
             if True:
                 image = np.zeros((my_height,my_width,3), np.uint8)
                 image[:] = [0,255,0]
                 image.flags.writeable = False
                 results = image
                    
                 # Draw landmark annotation on the image.
                 image.flags.writeable = True
                 image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
                   
                    
                 cv2.line(image, (int(my_width*my_n_shoulder.get("X")), int(my_height*my_n_shoulder.get("Y"))), (int(my_width*my_n_elbow.get("X")), int(my_height*my_n_elbow.get("Y"))), (200,0,50),5)
                   
                 cv2.line(image, (int(my_width*my_n_shoulder.get("X")), int(my_height*my_n_shoulder.get("Y"))), (int(my_width*my_n_hip.get("X")), int(my_height*my_n_hip.get("Y"))), (255,55,150),5)
                           
                 cv2.line(image, (int(my_width*my_n_wrist.get("X")), int(my_height*my_n_wrist.get("Y"))), (int(my_width*my_n_elbow.get("X")), int(my_height*my_n_elbow.get("Y"))), (200,50,50),5)
                    
                 #connector lines
                 #cv2.line(image, (int(my_width*my_n_hip.get("X")), int(my_height*my_n_hip.get("Y"))), (int(my_width*my_n_elbow.get("X")), int(my_height*my_n_elbow.get("Y"))), (0,0,255),2)
                 #cv2.line(image, (int(my_width*my_n_wrist.get("X")), int(my_height*my_n_wrist.get("Y"))), (int(my_width*my_n_shoulder.get("X")), int(my_height*my_n_shoulder.get("Y"))), (0,0,255),2)
                 #cv2.line(image, (int(my_width*my_f_shoulder.get("X")), int(my_height*my_f_shoulder.get("Y"))), (int(my_width*my_n_elbow.get("X")), int(my_height*my_n_elbow.get("Y"))), (0,0,255),3)
                 
                 image = cv2.putText(image, str(int(round(my_shoulder_r,0))), (int(my_width*my_n_shoulder.get("X")), int(my_height*my_n_shoulder.get("Y"))), cv2.FONT_HERSHEY_SIMPLEX, 1.3, (100,0,255), 2, cv2.LINE_AA)
                 
                 #image = cv2.putText(image, str(int(round(my_elbow_r,0))), (int(my_width*my_n_elbow.get("X")), int(my_height*my_n_elbow.get("Y"))), cv2.FONT_HERSHEY_SIMPLEX, 1.3, (100,0,255), 2, cv2.LINE_AA)
                 image = cv2.putText(image, str(int(round(NewValue,0))), (int(my_width*my_n_elbow.get("X")), int(my_height*my_n_elbow.get("Y"))), cv2.FONT_HERSHEY_SIMPLEX, 1.3, (100,0,255), 2, cv2.LINE_AA)
                 
                 image = cv2.putText(image, "Dist Val - " + str(my_n_shoulder_scale_values), (10,100), cv2.FONT_HERSHEY_SIMPLEX, 1.3, (100,0,255), 2, cv2.LINE_AA)
                 #image = cv2.putText(image, "Dist Val - " + str(round(dist)), (10,100), cv2.FONT_HERSHEY_SIMPLEX, 1.3, (100,0,255), 2, cv2.LINE_AA)
                 cv2.imshow('MediaPipe Holistic', image)
                    
                 #time.sleep(100)
                 if cv2.waitKey(5) & 0xFF == 27:
                     break
                           
             
                 if i_dt != 0:
                     sl =  (datetime.fromisoformat(row[0]) - i).total_seconds() 
                     time.sleep(sl)
                 
                 i_dt = datetime.fromisoformat(row[0])
             
             
             
             
             
             
             my_arr_1_x.append(my_n_shoulder.get("X"))
             my_arr_1_y.append(my_n_shoulder.get("Y"))
             my_arr_2_x.append(my_n_elbow.get("X"))
             my_arr_2_y.append(my_n_elbow.get("Y"))
             my_arr_3_x.append(my_n_wrist.get("X"))
             my_arr_3_y.append(my_n_wrist.get("Y"))
                                       
             my_arr_1_xy.append([float(my_n_wrist.get("X")),float(my_n_wrist.get("Y"))])
             
             my_arr_NearElbow.append(NearElbow)
             my_arr_NearShoulder.append(int(my_shoulder_r))
             
             my_arr_NearShoulderScaling.append(int(my_n_shoulder_scale_values)) #dist
             my_arr_NearWrist.append(int(NearWrist))
             
             sl_arr.append(row[0])
             
             i_shoulder+=1
midiout.close_port()


my_arr_NearShoulderScaling_s = gaussian_filter1d(my_arr_NearShoulderScaling, sigma=8)

for i in range(len(my_arr_NearShoulderScaling_s)):
    
    #center wrist
    #my_x = int(abs(my_n_elbow.get("X")-my_n_wrist.get("X"))*1000)
    #my_y = int(abs(my_n_elbow.get("Y")-my_n_wrist.get("Y"))*1000)
    if my_arr_NearShoulderScaling_s[i] < 40 and x_mid != 1:
       #print("in")
       x_mid = 1
       my_n_shoulder_scale = 127
       #midiout.send_message([0x90, center_wrist_near, 100])
    elif my_arr_NearShoulderScaling_s[i] >= 40 and x_mid != 0:
       #print("out")
       x_mid = 0
       my_n_shoulder_scale = 0
       #midiout.send_message([0x90, center_wrist_near, 100])
       #print(my_x,  "  ", my_y)
    #else:
    #   my_n_shoulder_scale = 0
    
    my_arr_NearShoulderInside.append(int(my_n_shoulder_scale))

   
#scaling
if my_y >= 70 and my_x >= 70:
    #right_elbow_scaling_midi
    if my_x > 200:
        my_x=200
    midiout.send_message([176, right_elbow_scaling_midi, int(remap(int(my_x), 50, 200, 0, 127))])
   
    #print(remap(int(my_x), 50, 200, 0, 127))
   



# smooth wrist

my_arr_NearWrist = gaussian_filter1d(my_arr_NearWrist, sigma=1)
#smoother = ConvolutionSmoother(window_len=20, window_type='ones')
#smoother.smooth(my_arr_NearWrist)

#my_arr_NearWrist = smoother.smooth_data[0]

#plt.plot(my_arr_NearWrist)



my_arr_NearShoulderScaling = gaussian_filter1d(my_arr_NearShoulderScaling, sigma=8)

for i in range(len(my_arr_NearShoulderScaling_s)):
    if my_arr_NearShoulderScaling[i] > 120:
        my_arr_NearShoulderScaling[i]=127
#smooth forearm scaling
#scale_peaks = find_peaks(my_arr_NearShoulderScaling)


#for xx in range(len(my_arr_NearShoulderScaling)):
#    if my_arr_NearShoulderScaling[xx] >= 110:
#        my_arr_NearShoulderScaling[xx] = 127

plt.plot(my_arr_NearShoulderScaling)


#fix false wrist ins
#smoother = ConvolutionSmoother(window_len=20, window_type='ones')
#smoother.smooth(my_arr_NearShoulderInside)

#i=0
#for xx in smoother.smooth_data[0]:
#    if int(xx) < 70:
#        my_arr_NearShoulderInside[i] = 0
#    else:
#        my_arr_NearShoulderInside[i] = 127
#    i+=1


my_arr_NearShoulderInside = gaussian_filter1d(my_arr_NearShoulderInside, sigma=5)

for i in range(len(my_arr_NearShoulderInside)):
    if my_arr_NearShoulderInside[i] > 120:
        my_arr_NearShoulderInside[i]=127
    if my_arr_NearShoulderInside[i] < 10:
        my_arr_NearShoulderInside[i]=0
plt.plot(my_arr_NearShoulderInside)

    



#heal dips
#for x in range(len(my_arr_NearElbow)):
    
#    if abs(my_arr_NearElbow[x-1]-my_arr_NearElbow[x]) > 20 and abs(my_arr_NearElbow[x-1]-my_arr_NearElbow[x]) < 100 and my_arr_NearElbow[x] < 110 and my_arr_NearElbow[x] > 10:
#        my_arr_NearElbow[x] =  my_arr_NearElbow[x-1]      #int((my_arr_NearElbow[x-1]+my_arr_NearElbow[x+1]+my_arr_NearElbow[x+2])/3)
        #print(my_arr_NearElbow[x])

#fix min/max
#for x in range(len(my_arr_NearElbow)):
#    if x+2 < len(my_arr_NearElbow):
#        if my_arr_NearElbow[x] > 100 and abs(my_arr_NearElbow[x]-my_arr_NearElbow[x+1])>70 or my_arr_NearElbow[x] > 100 and abs(my_arr_NearElbow[x]-my_arr_NearElbow[x+2])>70:
#           my_arr_NearElbow[x] = 127
#           my_arr_NearElbow[x+1] = 0
#           
#        if my_arr_NearElbow[x] < 20 and abs(my_arr_NearElbow[x]-my_arr_NearElbow[x+1])>70 or my_arr_NearElbow[x] < 20 and abs(my_arr_NearElbow[x]-my_arr_NearElbow[x+2])>70:
#           my_arr_NearElbow[x] = 0
#           my_arr_NearElbow[x+1] = 127


        
#smooth path
i=1
for x in range(len(my_arr_NearElbow)):
    if my_arr_NearElbow[x] == 127 and my_arr_NearElbow[x+1] == 0:
        # smooth form i to x
        #smoother = ConvolutionSmoother(window_len=20, window_type='ones')
        #smoother.smooth(my_arr_NearElbow[i:x-1])
        
        #for xx in smoother.smooth_data[0]:
        #    my_arr_NearElbow[i] = int(xx)
        #    i+=1
        #print(i," - ", x)
        #i+=1
        my_arr_NearElbow[i+3:x-3] = gaussian_filter1d(my_arr_NearElbow[i+3:x-3], sigma=0.4)


plt.plot(my_arr_NearElbow)




# wrist switch

for x in range(len(my_arr_NearElbow)):
    if my_arr_NearElbow[x] < 32:
        #if abs(tmp_ind_r - int(my_elbow_r)) > 2:
            #NewValue = remap(int(my_elbow_r), 0, 165, 62, 127)
            tmp_ind_r = 0
    #        #midiout.send_message([176, right_elbow_midi, int(NewValue)])
    #        #time.sleep(0.1)
    #        #print ("Elbow - ", NewValue)
    elif my_arr_NearElbow[x] < 64:
        #if abs(tmp_ind_r - int(my_elbow_r)) > 2:
            #NewValue = remap(int(my_elbow_r)+90, 0, 90, 31, 61)
            tmp_ind_r = 32
    #        #midiout.send_message([176, right_elbow_midi, int(NewValue)])
    #        #time.sleep(0.1)
    #        #print ("Elbow - ", NewValue)
    elif my_arr_NearElbow[x] < 96:
        #if abs(tmp_ind_r - int(my_elbow_r)) > 2:                
            #NewValue = abs(remap(int(my_elbow_r), 180, 270, 0, 30))
            tmp_ind_r = 96
    #        #midiout.send_message([176, right_elbow_midi, int(NewValue)])
    #        #time.sleep(0.1)
    #        #print ("Elbow - ", NewValue)
    else:
        tmp_ind_r = 127
    
    Wrist_switch = tmp_ind_r
    
    my_arr_NearWristSwitch.append(Wrist_switch)
    

plt.plot(my_arr_NearWristSwitch)




        #print(my_arr_NearElbow[x])





#for x in range(len(my_arr_NearShoulder)-1):
     #print(abs(my_arr_NearShoulder[x] - my_arr_NearShoulder[x+1]))
#     if abs(my_arr_NearShoulder[x] - my_arr_NearShoulder[x+1])>5:
#         my_arr_NearShoulder[x+1] = my_arr_NearShoulder[x]

#smoother = ConvolutionSmoother(window_len=80, window_type='ones')
#smoother.smooth(my_arr_NearShoulder)

#my_arr_NearShoulder = smoother.smooth_data[0]

#my_arr_NearShoulder = gaussian_filter1d(my_arr_NearShoulder, sigma=8)

plt.plot(my_arr_NearShoulder)

# operate smoothing
#smoother = DecomposeSmoother(smooth_type='lowess', periods=24,smooth_fraction=0.3)
#smoother = ConvolutionSmoother(window_len=20, window_type='ones')
#smoother.smooth(my_arr_NearElbow)


#bts = BootstrappingWrapper(ConvolutionSmoother(window_len=8, window_type='ones'), 
#                           bootstrap_type='mbb', block_length=24)
#bts_samples = bts.sample(my_arr_NearElbow, n_samples=100)

# plot the bootstrapped timeseries
#plt.figure(figsize=(13,5))
#plt.plot(bts_samples.T, alpha=0.3, c='orange')
#plt.plot(data[0], c='blue', linewidth=2)

# generate intervals
#low, up = smoother.get_intervals('sigma_interval', n_sigma=2)

# plot the smoothed timeseries with intervals
#plt.figure(figsize=(11,6))
#plt.plot(smoother.smooth_data[0], linewidth=3, color='blue')
#plt.plot(smoother.data[0], '.k')
#plt.fill_between(range(len(smoother.data[0])), low[0], up[0], alpha=0.3)


mins = argrelextrema(np.array(my_arr_NearElbow), np.greater)
maxes = argrelextrema(np.array(my_arr_NearElbow), np.less)

c_min = [my_arr_NearElbow[i] for i in mins[0]]
c_max = [my_arr_NearElbow[i] for i in maxes[0]]

#for cc in c_min:
#    if cc > 100:
        #print(cc)

#for cc in range(len(c_max)):
#    if c_max[cc] < 40:
        #print(maxes[0][cc], " - ", c_max[cc])

#plt.plot(c)

#plt.plot(my_arr_1_x,my_arr_1_y)
#plt.show()
#plt.plot(my_arr_2_x,my_arr_2_y)
#plt.show()
#plt.plot(my_arr_3_x,my_arr_3_y)
#plt.plot(my_arr_NearElbow)
#plt.show()







with open('elbow.csv', 'w', newline='') as csvfile:
            spamwriter = csv.writer(csvfile, delimiter=',')
            for x in range(len(my_arr_NearElbow)):
                #if abs(my_arr_NearElbow[x-1]-my_arr_NearElbow[x]) > 10:
                #    spamwriter.writerow([int((my_arr_NearElbow[x-1]+my_arr_NearElbow[x+1]+my_arr_NearElbow[x+2])/3),sl_arr[x]])
                #else:
                    #print(my_arr_NearElbow[x])
                    spamwriter.writerow([int(my_arr_NearElbow[x]),sl_arr[x],my_arr_NearShoulder[x]])
                    

            







