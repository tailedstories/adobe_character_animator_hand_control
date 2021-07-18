import rtmidi
import csv
import json
import time
from datetime import datetime
import math
import matplotlib.pyplot as plt
import numpy as np
from tsmoothie.smoother import *
from tsmoothie.bootstrap import BootstrappingWrapper
from scipy.signal import argrelextrema

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



tmp_ind_r=0
NearElbow=None


elbow_min = 99999
elbow_max = -99999

my_arr_1_x = []
my_arr_1_y = []
my_arr_2_x = []
my_arr_2_y = []
my_arr_3_x = []
my_arr_3_y = []

my_arr_1_xy = []
my_arr_NearElbow = []
my_arr_NearShoulder = []
sl_arr = []
sl=0

with open('points.csv', newline='') as csvfile:
     spamreader = csv.reader(csvfile, delimiter=',')
     i=0
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
            
             my_n_shoulder = json.loads(row[13].replace("'",'"'))
             my_n_elbow = json.loads(row[15].replace("'",'"'))
             my_n_wrist = json.loads(row[17].replace("'",'"'))
             
             my_n_h_wrist = json.loads(row[34+13].replace("'",'"'))
             
             
             # Near Wrist
             NewValue = myAngle(my_n_h_wrist.get("X"),my_n_h_wrist.get("Y"),my_n_wrist.get("X"),my_n_wrist.get("Y"))
             if NewValue < 0:
                 NewValue = 0
             if NewValue > 110:
                 NewValue = 110        
             NearWrist = int(remap(int(NewValue), 0, 110, 0, 127))
             
             #print(NearWrist)
             #print(my_n_shoulder)
             
             # Near elbow
             my_elbow_r = prepangle(my_n_shoulder.get("X"),my_n_shoulder.get("Y"),
                                       my_n_elbow.get("X"),my_n_elbow.get("Y"),
                                       my_n_wrist.get("X"),my_n_wrist.get("Y"))
                
             
             NewValue = None
             # Near elbow
             
             NewValue = remap(int(my_elbow_r), -84, 303, 0, 127)
             midiout.send_message([176, right_elbow_midi, int(NewValue)])
             
             #if int(my_elbow_r) < 165 and int(my_elbow_r) > 0:
             #    if abs(tmp_ind_r - int(my_elbow_r)) > 2:
             #        NewValue = remap(int(my_elbow_r), 0, 165, 62, 127)
             #        tmp_ind_r = int(my_elbow_r)
             #        #midiout.send_message([176, right_elbow_midi, int(NewValue)])    
             #        #time.sleep(0.1)
             #        #print ("Elbow - ", NewValue)
             #if int(my_elbow_r) < 0 and int(my_elbow_r) > -90:
             #    if abs(tmp_ind_r - int(my_elbow_r)) > 2:
             #        NewValue = remap(int(my_elbow_r)+90, 0, 90, 31, 61)
             #        tmp_ind_r = int(my_elbow_r)
             #        #midiout.send_message([176, right_elbow_midi, int(NewValue)])
             #        #time.sleep(0.1)
             #        #print ("Elbow - ", NewValue)
             #if int(my_elbow_r) < 270 and int(my_elbow_r) > 179:
             #    if abs(tmp_ind_r - int(my_elbow_r)) > 2:                
             #        NewValue = abs(remap(int(my_elbow_r), 180, 270, 0, 30))
             #        tmp_ind_r = int(my_elbow_r)
             #        #midiout.send_message([176, right_elbow_midi, int(NewValue)])
             #        #time.sleep(0.1)
             #        #print ("Elbow - ", NewValue)
             
             
             
             # Right shoulder 
             my_shoulder_r = myAngle(my_n_shoulder.get("X"),my_n_shoulder.get("Y"),my_n_elbow.get("X"),my_n_elbow.get("Y"))
            
             
             #if abs(tmp_ind - int(my_shoulder_r)) > 5:
             my_shoulder_r = remap(int(my_shoulder_r)+180, 0, 360, 0, 127)
             
                 #midiout.send_message([176, right_shoulder_midi, int(NewValue)])
                 #print ("Shoulder right - ", int(NewValue))
                 #time.sleep(0.1)
                
             
             
             if NewValue != None:
                 NearElbow = int(NewValue)
 
             #print(NearElbow)
             if my_elbow_r < elbow_min:
                 elbow_min = my_elbow_r
             if my_elbow_r > elbow_max:
                 elbow_max = my_elbow_r
             
             my_arr_1_x.append(my_n_shoulder.get("X"))
             my_arr_1_y.append(my_n_shoulder.get("Y"))
             my_arr_2_x.append(my_n_elbow.get("X"))
             my_arr_2_y.append(my_n_elbow.get("Y"))
             my_arr_3_x.append(my_n_wrist.get("X"))
             my_arr_3_y.append(my_n_wrist.get("Y"))
                                       
             my_arr_1_xy.append([float(my_n_wrist.get("X")),float(my_n_wrist.get("Y"))])
             
             my_arr_NearElbow.append(NearElbow)
             my_arr_NearShoulder.append(int(my_shoulder_r))
             sl_arr.append(row[0])
midiout.close_port()



    
print("Elbow min - ", elbow_min)
print("Elbow max - ", elbow_max)


#heal dips
for x in range(len(my_arr_NearElbow)):
    
    if abs(my_arr_NearElbow[x-1]-my_arr_NearElbow[x]) > 20 and abs(my_arr_NearElbow[x-1]-my_arr_NearElbow[x]) < 100 and my_arr_NearElbow[x] < 110 and my_arr_NearElbow[x] > 10:
        my_arr_NearElbow[x] =  my_arr_NearElbow[x-1]      #int((my_arr_NearElbow[x-1]+my_arr_NearElbow[x+1]+my_arr_NearElbow[x+2])/3)
        #print(my_arr_NearElbow[x])

#fix min/max
for x in range(len(my_arr_NearElbow)):
    if my_arr_NearElbow[x] > 100 and abs(my_arr_NearElbow[x]-my_arr_NearElbow[x+1])>70:
       my_arr_NearElbow[x] = 127
       my_arr_NearElbow[x+1] = 0
       
    if my_arr_NearElbow[x] < 20 and abs(my_arr_NearElbow[x]-my_arr_NearElbow[x+1])>70:
       my_arr_NearElbow[x] = 0
       my_arr_NearElbow[x+1] = 127

        
#smooth path
i=1
for x in range(len(my_arr_NearElbow)):
    if my_arr_NearElbow[x] == 127 and my_arr_NearElbow[x+1] == 0:
        # smooth form i to x
        smoother = ConvolutionSmoother(window_len=20, window_type='ones')
        smoother.smooth(my_arr_NearElbow[i:x-1])
        
        for xx in smoother.smooth_data[0]:
            my_arr_NearElbow[i] = int(xx)
            i+=1
        i+=1


#fix zero bug
for x in range(len(my_arr_NearElbow)):
    if my_arr_NearElbow[x] > 126:
        my_arr_NearElbow[x+1] = 0


        #print(my_arr_NearElbow[x])





for x in range(len(my_arr_NearShoulder)-1):
     #print(abs(my_arr_NearShoulder[x] - my_arr_NearShoulder[x+1]))
     if abs(my_arr_NearShoulder[x] - my_arr_NearShoulder[x+1])>5:
         my_arr_NearShoulder[x+1] = my_arr_NearShoulder[x]

smoother = ConvolutionSmoother(window_len=80, window_type='ones')
smoother.smooth(my_arr_NearShoulder)

my_arr_NearShoulder = smoother.smooth_data[0]

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
                    

            







