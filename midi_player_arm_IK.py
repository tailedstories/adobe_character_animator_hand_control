import rtmidi
import os

import csv
import json
import time
from datetime import datetime
import math
import matplotlib.pyplot as plt
import numpy as np
from tsmoothie.smoother import *


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


i=0

#os.system("start " + os.path.dirname(os.path.realpath(__file__)) + "\\record_ch_start.exe")
#os.system("start " + os.path.dirname(os.path.realpath(__file__)) + "\\record_ch_stop.exe")
#time.sleep(0.02)

#print(os.path.dirname(os.path.realpath(__file__)))

'''

with open('wheel_points.csv', newline='') as csvfile:
     spamreader = csv.reader(csvfile, delimiter=',')
     i=0
     for row in spamreader:
         if len(row) > 0:
             midiout.send_message([176, far_elbow_midi, int(float(row[2]))])
             time.sleep(0.02)
             midiout.send_message([176, far_shoulder_midi, int(float(row[3]))])
             if i != 0:
                 time.sleep((float(row[0]) - i)/1000)             
         i = float(row[0])

'''

midiout.close_port()
