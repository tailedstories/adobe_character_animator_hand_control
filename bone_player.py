import rtmidi
import csv
import json
import time
import cv2
from datetime import datetime
import math
import matplotlib.pyplot as plt
import numpy as np
from tsmoothie.smoother import *



import mediapipe as mp
import rtmidi
import math
import csv
from datetime import datetime

mp_drawing = mp.solutions.drawing_utils
mp_holistic = mp.solutions.holistic



image = np.zeros((960,1280,3), np.uint8)

image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)


with open('points.csv', newline='') as csvfile:
     spamreader = csv.reader(csvfile, delimiter=',')
     i=0
     for row in spamreader:
        image = np.zeros((960,1280,3), np.uint8)
         
        my_n_hip = json.loads(row[25].replace("'",'"'))
        my_f_shoulder = json.loads(row[12].replace("'",'"'))
        my_n_shoulder = json.loads(row[13].replace("'",'"'))
        my_n_elbow = json.loads(row[15].replace("'",'"'))
        my_n_wrist = json.loads(row[17].replace("'",'"'))
        
        # To improve performance, optionally mark the image as not writeable to
        # pass by reference.
        image.flags.writeable = False
        results = image
        
        # Draw landmark annotation on the image.
        image.flags.writeable = True
        image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
        
        
        cv2.line(image, (int(1280*my_n_shoulder.get("X")), int(960*my_n_shoulder.get("Y"))), (int(1280*my_n_elbow.get("X")), int(960*my_n_elbow.get("Y"))), (255,0,0),5)
        
        cv2.line(image, (int(1280*my_n_shoulder.get("X")), int(960*my_n_shoulder.get("Y"))), (int(1280*my_n_hip.get("X")), int(960*my_n_hip.get("Y"))), (0,255,0),5)
               
        cv2.line(image, (int(1280*my_n_wrist.get("X")), int(960*my_n_wrist.get("Y"))), (int(1280*my_n_elbow.get("X")), int(960*my_n_elbow.get("Y"))), (0,255,255),5)
        
        #connector lines
        cv2.line(image, (int(1280*my_n_hip.get("X")), int(960*my_n_hip.get("Y"))), (int(1280*my_n_elbow.get("X")), int(960*my_n_elbow.get("Y"))), (0,0,255),3)
        cv2.line(image, (int(1280*my_n_wrist.get("X")), int(960*my_n_wrist.get("Y"))), (int(1280*my_n_shoulder.get("X")), int(960*my_n_shoulder.get("Y"))), (0,0,255),3)
        #cv2.line(image, (int(1280*my_f_shoulder.get("X")), int(960*my_f_shoulder.get("Y"))), (int(1280*my_n_elbow.get("X")), int(960*my_n_elbow.get("Y"))), (0,0,255),3)
        cv2.imshow('MediaPipe Holistic', image)
        
        #time.sleep(100)
        if cv2.waitKey(5) & 0xFF == 27:
            break