import rtmidi
import csv
import json
import time
from datetime import datetime
import math
import matplotlib.pyplot as plt
import numpy as np
from tsmoothie.smoother import *

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
right_shoulder_midi   = 17
right_elbow_midi      = 18
right_wrist_midi      = 80
right_wrist_flip_midi = 81
right_elbow_scaling_midi = 10
#center
center_wrist_near = 7
#Left Arm
left_shoulder_midi   = 16
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


i=0

with open('elbow.csv', newline='') as csvfile:
     spamreader = csv.reader(csvfile, delimiter=',')
     i=0
     for row in spamreader:
         if len(row) > 0:
             midiout.send_message([176, right_elbow_midi, int(float(row[0]))])
             time.sleep(0.01)
             midiout.send_message([176, right_shoulder_midi, int(float(row[2]))])
             if i != 0:
                 sl =  (datetime.fromisoformat(row[1]) - i).total_seconds() 
                 time.sleep(sl)
             
         i = datetime.fromisoformat(row[1])



midiout.close_port()








