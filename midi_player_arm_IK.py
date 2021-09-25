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
midiout.open_port(3)
#this should be a reference to your camera (number)
cam_ref=3
# how long to wait between sending next MIDI
time_scale = 0.01

# True - enable hitting record button for Ch
enable_recording = True

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




# midi order to send
midi_store = [far_shoulder_midi , far_elbow_midi, 
              near_shoulder_midi, near_elbow_midi]

    
for column in range(len(midi_store)):
        
    
    # skip first frame time delay
    #i=0
    input("Press Enter to continue...")
    
    
    if enable_recording:
            # run the character animattor recording
            os.system("start " + os.path.dirname(os.path.realpath(__file__)) + "\\record_ch_start.exe")
            # wait till timer is running
            time.sleep(5)
        
    # loop over the file with saved midi values
    with open('wheel_points.csv', newline='') as csvfile:
         # loop over the columns
         # read csv file
         spamreader = csv.reader(csvfile, delimiter=',')     
         print(column)
         # get rows from the file
         for row in spamreader:
             # check if row is not empty
             #if len(row) > 0:
                 # send midi
                 
                 print("Midi : ", midi_store[column], " Value : ", int(float(row[column+1])) )
                 midiout.send_message([176, midi_store[column], int(float(row[column+1]))])
                 #if i != 0:
                     #time.sleep((float(row[0]) - i)/1000)
                     #time.sleep(float(row[0])/1000)
                 time.sleep(time_scale)
             # save time delay for next frame
             #i = float(row[0])
             
    if enable_recording:
        # stop recording + rewind the timeline
        time.sleep(1)
        os.system("start " + os.path.dirname(os.path.realpath(__file__)) + "\\record_ch_stop.exe")
        # delay just in case
        time.sleep(5)
# close midi sending port
midiout.close_port()
