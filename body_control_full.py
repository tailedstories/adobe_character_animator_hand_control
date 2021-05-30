import cv2
import mediapipe as mp
import math
import rtmidi


mp_drawing = mp.solutions.drawing_utils
mp_pose = mp.solutions.pose

midiout = rtmidi.MidiOut()
available_ports = midiout.get_ports()


#########
## SET ##
## UP  ##
#########

#this should be a reference to a loopmidi (number)
midiout.open_port(1)
#this should be a reference to your camera (number)
cam_ref=0



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


# For webcam input:
cap = cv2.VideoCapture(cam_ref)
x=0
tmp_ind=0
tmp_ind2=0
tmp_ind_r=0
tmp_ind_l=0
with mp_pose.Pose(
    min_detection_confidence=0.5,
    min_tracking_confidence=0.5) as pose:
  while cap.isOpened():
    success, image = cap.read()
    if not success:
      print("Ignoring empty camera frame.")
      # If loading a video, use 'break' instead of 'continue'.
      continue

    # Flip the image horizontally for a later selfie-view display, and convert
    # the BGR image to RGB.
    image = cv2.cvtColor(cv2.flip(image, 1), cv2.COLOR_BGR2RGB)
    # To improve performance, optionally mark the image as not writeable to
    # pass by reference.
    image.flags.writeable = False
    results = pose.process(image)

    # Draw the pose annotation on the image.
    image.flags.writeable = True
    image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
    mp_drawing.draw_landmarks(
        image, results.pose_landmarks, mp_pose.POSE_CONNECTIONS)
    cv2.imshow('MediaPipe Pose', image)
    
    keypoints = []
    if results.pose_landmarks != None:
        for data_point in results.pose_landmarks.landmark:
            keypoints.append({
                                 'X': data_point.x,
                                 'Y': data_point.y,
                                 'Z': data_point.z,
                                 'Visibility': data_point.visibility,
                                 })
        
        if len(keypoints) > 16:
            #print("1 - ",keypoints[12].get("X")," 2 - ",keypoints[12].get("Y")," 3 - ",keypoints[14].get("X")," 4 - ",keypoints[14].get("Y"))
            my_shoulder_r = myAngle(keypoints[12].get("X"),keypoints[12].get("Y"),keypoints[14].get("X"),keypoints[14].get("Y"))
            
            my_shoulder_l = myAngle(keypoints[11].get("X"),keypoints[11].get("Y"),keypoints[13].get("X"),keypoints[13].get("Y"))
            #print("Right Shoulder : ", my_shoulder)
            
            my_elbow_r = prepangle(keypoints[12].get("X"),keypoints[12].get("Y"),
                                   keypoints[14].get("X"),keypoints[14].get("Y"),
                                   keypoints[16].get("X"),keypoints[16].get("Y"))
            
            my_elbow_l = prepangle(keypoints[11].get("X"),keypoints[11].get("Y"),
                                   keypoints[13].get("X"),keypoints[13].get("Y"),
                                   keypoints[15].get("X"),keypoints[15].get("Y"))
            
            
            #right elbow
            if int(my_elbow_r) < 165 and int(my_elbow_r) > 0:
                if abs(tmp_ind_r - int(my_elbow_r)) > 2:
                    NewValue = remap(int(my_elbow_r), 0, 165, 62, 127)
                    tmp_ind_r = int(my_elbow_r)
                    midiout.send_message([176, 21, int(NewValue)])    
                    #time.sleep(0.1)
                    #print ("Elbow - ", NewValue)
            if int(my_elbow_r) < 0 and int(my_elbow_r) > -90:
                if abs(tmp_ind_r - int(my_elbow_r)) > 2:
                    NewValue = remap(int(my_elbow_r)+90, 0, 90, 31, 61)
                    tmp_ind_r = int(my_elbow_r)
                    midiout.send_message([176, 21, int(NewValue)])
                    #time.sleep(0.1)
                    #print ("Elbow - ", NewValue)
            if int(my_elbow_r) < 270 and int(my_elbow_r) > 179:
                if abs(tmp_ind_r - int(my_elbow_r)) > 2:                
                    NewValue = abs(remap(int(my_elbow_r), 180, 270, 0, 30))
                    tmp_ind_r = int(my_elbow_r)
                    midiout.send_message([176, 21, int(NewValue)])
                    #time.sleep(0.1)
                    #print ("Elbow - ", NewValue)
                    
            #left elbow
            if int(my_elbow_l) < 165 and int(my_elbow_l) > 0:
                if abs(tmp_ind_l - int(my_elbow_l)) > 2:
                    NewValue = remap(int(my_elbow_l), 0, 165, 62, 127)
                    tmp_ind_l = int(my_elbow_l)
                    midiout.send_message([176, 18, int(NewValue)])    
                    #time.sleep(0.1)
                    #print ("Elbow - ", NewValue)
            if int(my_elbow_l) < 0 and int(my_elbow_l) > -90:
                if abs(tmp_ind_l - int(my_elbow_l)) > 2:
                    NewValue = remap(int(my_elbow_l)+90, 0, 90, 31, 61)
                    tmp_ind_l = int(my_elbow_l)
                    midiout.send_message([176, 18, int(NewValue)])
                    #time.sleep(0.1)
                    #print ("Elbow - ", NewValue)
            if int(my_elbow_l) < 270 and int(my_elbow_l) > 179:
                if abs(tmp_ind_l - int(my_elbow_l)) > 2:                
                    NewValue = abs(remap(int(my_elbow_l), 180, 270, 0, 30))
                    tmp_ind_l = int(my_elbow_l)
                    midiout.send_message([176, 18, int(NewValue)])
                    #time.sleep(0.1)
                    #print ("Elbow - ", NewValue)
            
            
            
            #right shoulder 
            if abs(tmp_ind - int(my_shoulder_r)) > 2:
                NewValue = remap(int(my_shoulder_r)+180, 0, 360, 0, 127)
                tmp_ind  = int(my_shoulder_r)
                midiout.send_message([176, 20, int(NewValue)])
                #print ("Shoulder right - ", int(NewValue))
                #time.sleep(0.1)
                
            #left shoulder 
            if abs(tmp_ind2 - int(my_shoulder_l)) > 2:
                NewValue = abs(remap(int(my_shoulder_l)+180, 0, 360, 0, 127)-127)
                
                tmp_ind2  = int(my_shoulder_l)
                midiout.send_message([176, 17, int(NewValue)])
                #print ("Shoulder left - ", int(NewValue))
                #time.sleep(0.1)
            
            
            
            #right wrist auto flip
            if(keypoints[16].get("X") > keypoints[12].get("X")) :
                if x != 1:
                    x=1
                    midiout.send_message([0x90, 80, 100])
                    #print ("Right In")
            else:
                if x != 2:
                    x=2
                    midiout.send_message([0x90, 81, 100])
                    #print ("Right Out")
                    
            #left wrist auto flip
            if(keypoints[15].get("X") > keypoints[11].get("X")) :
                if x != 1:
                    x=1
                    midiout.send_message([0x90, 83, 100])
                    #print ("Left In")
            else:
                if x != 2:
                    x=2
                    midiout.send_message([0x90, 82, 100])
                    #print ("Left Out")
        
    if cv2.waitKey(5) & 0xFF == 27:
      break
cap.release()
