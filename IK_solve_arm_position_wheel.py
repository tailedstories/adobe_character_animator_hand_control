################
## References #################################################################################################################################################
# IK solver from https://github.com/ekorudiawan/CCD-Inverse-Kinematics-2D/blob/master/sources/CCD-Inverse-Kinematics-2D.py                                    #
# Sort coordinates in circle from https://stackoverflow.com/questions/41855695/sorting-list-of-two-dimensional-coordinates-by-clockwise-angle-using-python    #
###############################################################################################################################################################
###########
# Imports #
###########
import numpy as np 
import matplotlib.pyplot as plt 
import math
from skimage import draw
import time,os
import csv
from math import atan2, cos, sin, degrees
from datetime import datetime

############
# Settings #
############

# clockwise   --> true
# couterclock --> false
clockwise = False

# True will send midi signal for character animator
# False will not
sendmidi = False

# Set to false to not play animation on graph
# just record joints to file
animated_graph = True

# the ranges to change gpaph size
graph_X_min = -100
graph_X_max = 150
graph_Y_min = -50
graph_Y_max = 150

# wheel X position on graph
wheel_x = 25
# wheel Y position on graph
wheel_y = 30
# wheel Radius on graph
wheel_r = 30

# loopmidi port number (system)
loopmidi_num = 3

#############
# Right Arm #
#############

# Right Arm joint length (shoulder ; forearm)
r_arm = [50, 55]
# Right Arm Initial Joint Values (degree)
angle = [-90, -90]


#############
# Left Arm #
#############

# Left Arm joint length (shoulder ; forearm)
l_arm = [50, 50]
# Left Arm Initial Joint Values (degree)
# angling elbow down allow better circular path 
angle2 = [-100, 0]

############################
# Define the target & Plot #
############################
# Target End of Effector Position
target = [0, 0, 0] 
# Create figure to plot
fig = plt.figure() 
ax = fig.add_subplot(1,1,1)

################
# Draw a wheel #
################
rr, cc = draw.circle_perimeter(wheel_x,wheel_y,wheel_r)

#####################
# init midi sending #
#####################
if sendmidi:
    import rtmidi
    #this should be a reference to a loopmidi (number)
    midiout = rtmidi.MidiOut()
    midiout.open_port(loopmidi_num)

###############
# Init Values #
###############
pts = np.stack((rr, cc), axis=-1)
origin = [40, 20]
refvec = [0, 1]

########################################
# Clean up existing files to Save MIDI #
########################################
if os.path.exists("wheel_points.csv"):
    os.remove("wheel_points.csv") 


###############################################
# We have the points for cirle                #
# Arrange them in motion followin the circle  #
###############################################

def clockwiseangle_and_distance(point):
    # Vector between point and the origin: v = p - o
    vector = [point[0]-origin[0], point[1]-origin[1]]
    # Length of vector: ||v||
    lenvector = math.hypot(vector[0], vector[1])
    # If length is zero there is no angle
    if lenvector == 0:
        return -math.pi, 0
    # Normalize vector: v/||v||
    normalized = [vector[0]/lenvector, vector[1]/lenvector]
    dotprod  = normalized[0]*refvec[0] + normalized[1]*refvec[1]     # x1*x2 + y1*y2
    diffprod = refvec[1]*normalized[0] - refvec[0]*normalized[1]     # x1*y2 - y1*x2
    angle = math.atan2(diffprod, dotprod)
    # Negative angles represent counter-clockwise angles so we need to subtract them 
    # from 2*pi (360 degrees)
    if angle < 0:
        return 2*math.pi+angle, lenvector
    # I return first the angle because that's the primary sorting criterium
    # but if two vectors have the same angle then the shorter distance should come first.
    return angle, lenvector

# sort the coordinates, to movi in circle
coorinput = sorted(pts, key=clockwiseangle_and_distance)

# based on what whe choose in settings 
if clockwise:
    #clock
    rr,cc=np.array(coorinput).T
else:
    #counter clock
    rr,cc=np.array(coorinput)[::-1].T


#remap x from input min/max number range to output min/max
def remap(x, in_min, in_max, out_min, out_max):
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

#0-360 for the line based on coordinates
def myAngle(cx, cy, ex, ey):
  angle = atan2(cos(cx)*sin(ex)-sin(cx) * cos(ex)*cos(ey-cy), sin(ey-cy)*cos(ex))
  bearing = (degrees(angle) + 360) % 360
  return bearing 

# rotate jounts
def rotateZ(theta):
    rz = np.array([[math.cos(theta), - math.sin(theta), 0, 0],
                   [math.sin(theta), math.cos(theta), 0, 0],
                   [0, 0, 1, 0],
                   [0, 0, 0, 1]])
    return rz

def translate(dx, dy, dz):
    t = np.array([[1, 0, 0, dx],
                  [0, 1, 0, dy],
                  [0, 0, 1, dz],
                  [0, 0, 0, 1]])
    return t

# Forward Kinematics
# Input initial angles and length of links
# Output positions each points
def FK(angle, link, myind=1):
    n_links = len(link)
    P = []
    P.append(np.eye(4))
    if myind == 1:
        P[0][0][-1] = 40
        P[0][1][-1] = 65
    else:
        P[0][0][-1] = 0
        P[0][1][-1] = 65
    for i in range(0, n_links):
        R = rotateZ(angle[i]/180*math.pi)
        T = translate(link[i], 0, 0)
        P.append(P[-1].dot(R).dot(T))
    return P

# IK solver to position the arms to target position
def IK(target, angle, link, max_iter = 10000, err_min = 0.1,myind=1):
    # init variables
    solved = False
    err_end_to_target = math.inf
    
    # loop over up to max_iter times
    for loop in range(max_iter):
        # loop over all joints
        for i in range(len(link)-1, -1, -1):
            # use function input for separate handling of the left and right arms
            if myind==1:
                P = FK(angle, link)
            else:
                P = FK(angle, link,2)
                
            # do math to calc joint position
            end_to_target = target - P[-1][:3, 3]
            err_end_to_target = math.sqrt(end_to_target[0] ** 2 + end_to_target[1] ** 2)
            if err_end_to_target < err_min:
                solved = True
            else:
                cur_to_end = P[-1][:3, 3] - P[i][:3, 3]
                cur_to_end_mag = math.sqrt(cur_to_end[0] ** 2 + cur_to_end[1] ** 2)
                cur_to_target = target - P[i][:3, 3]
                cur_to_target_mag = math.sqrt(cur_to_target[0] ** 2 + cur_to_target[1] ** 2)

                end_target_mag = cur_to_end_mag * cur_to_target_mag

                if end_target_mag <= 0.0001:    
                    cos_rot_ang = 1
                    sin_rot_ang = 0
                else:
                    cos_rot_ang = (cur_to_end[0] * cur_to_target[0] + cur_to_end[1] * cur_to_target[1]) / end_target_mag
                    sin_rot_ang = (cur_to_end[0] * cur_to_target[1] - cur_to_end[1] * cur_to_target[0]) / end_target_mag

                rot_ang = math.acos(max(-1, min(1,cos_rot_ang)))

                if sin_rot_ang < 0.0:
                    rot_ang = -rot_ang

                # Update current joint angle values
                angle[i] = angle[i] + (rot_ang * 180 / math.pi)

                if angle[i] >= 360:
                    angle[i] = angle[i] - 360
                if angle[i] < 0:
                    angle[i] = 360 + angle[i]
            
        if solved:
            break
    return angle, err_end_to_target, solved, loop

# Run animation when clicked on the graph
def onclick(event):
    # import global values 
    global target, r_arm, angle, link2, angle2, ax,rr,cc
    
    ######################################
    # Loop Over all points in the circle #
    ######################################
    for ind in range(len(cc)):
        # read X and Y
        target[0] = rr[ind]
        target[1] = cc[ind]
        
        # clear the plot (existing graph)
        plt.cla()
        # define the size of the graph
        ax.set_xlim(graph_X_min, graph_X_max)
        ax.set_ylim(graph_Y_min, graph_Y_max)
        
        
        #################
        # Prep Left Arm #
        #################
        # init array for joint info
        P_left = FK(angle2, l_arm,2)
        
        # do only once index
        ii=0
        #for every joint in left arm
        for i in range(len(l_arm)):
            start_point = P_left[i]
            end_point = P_left[i+1]
            if ii==0:
                # draw a circle for the elbow
                circle2 = plt.Circle((end_point[0,3],end_point[1,3]), 5, color='r')
                # do once
                ii+=1
            # draw the joint on the graph
            ax.plot([start_point[0,3], end_point[0,3]], [start_point[1,3], end_point[1,3]], linewidth=5)     
            
        # put the elbow circle on the graph
        plt.gca().add_patch(circle2)
        
        
        #################
        # Left Shoulder #
        #################
        
        # get variables for (x,y) shoulder and elbow
        cx = P_left[0][0][3]+100
        cy = P_left[0][1][3]+100
        ex = P_left[1][0][3]+100
        ey = P_left[1][1][3]+100
        
        # calculate absolute rotation on screen
        my_shoulder_angle_l = degrees(atan2((ey - cy), (ex - cx)))
        # 0 to 360
        my_shoulder_angle_l = (my_shoulder_angle_l + 360) % 360
        # scale 0 to 360 to ==> 0 to 127
        my_l_shoulder = int(remap(int(my_shoulder_angle_l), 0, 360, 0, 127))
        if sendmidi:
            # send midi signal
            midiout.send_message([176, 60, my_l_shoulder])
            # wait, so othe midi works
            time.sleep(0.05)
        
        ##############
        # Left Elbow #
        ##############
        
        # get variables for (x,y) elbow and wrist
        ex = P_left[2][0][3]+100
        ey = P_left[2][1][3]+100
        cx = P_left[1][0][3]+100
        cy = P_left[1][1][3]+100
        
        # calculate absolute rotation on screen
        my_l_elbow = degrees(atan2((ey - cy), (ex - cx)))
        # 0 to 360
        my_l_elbow = (my_l_elbow + 360) % 360
        # scale 0 to 360 to ==> 0 to 127
        my_l_elbow = int(remap(int(my_l_elbow), 0, 360, 0, 127))
        if sendmidi:
            # send midi signal
            midiout.send_message([176, 62, my_l_elbow])
            # wait, so othe midi works
            time.sleep(0.05)
        
        
        #######################
        # Solve the Right Arm #
        # Inverse Kinematics  #
        #######################
        angle, err, solved, iteration = IK(target, angle, r_arm, max_iter=1000)
        
        
        ##################
        # Prep Right Arm #
        ##################
        
        # init array for joint info
        P_right = FK(angle, r_arm)
        
        # do only once index
        ii=0
        # for every joint in right arm
        for i in range(len(r_arm)):
            start_point = P_right[i]
            end_point = P_right[i+1]
            if ii==0:
                # draw a circle for the elbow
                circle1 = plt.Circle((end_point[0,3],end_point[1,3]), 5, color='r')
                # don once
                ii+=1
            # draw the joint on the graph
            ax.plot([start_point[0,3], end_point[0,3]], [start_point[1,3], end_point[1,3]], linewidth=5)
        
        # refresh target
        target[0] = rr[ind-int(len(cc)/2)]
        target[1] = cc[ind-int(len(cc)/2)]
        
        ######################
        # Solve the Left Arm #
        # Inverse Kinematics #
        ######################
        angle2, err, solved, iteration = IK(target, angle2, r_arm, max_iter=1000,myind=2)
        
        # put the elbow circle on the graph
        plt.gca().add_patch(circle1)
        
        ##################
        # Right Shoulder #
        ##################
        
        # get variables for (x,y) shoulder and elbow
        cx = P_right[0][0][3]+100
        cy = P_right[0][1][3]+100
        ex = P_right[1][0][3]+100
        ey = P_right[1][1][3]+100
        
        # calculate absolute rotation on screen
        my_shoulder_angle_r = degrees(atan2((ey - cy), (ex - cx)))
        # 0 to 360
        my_shoulder_angle_r = (my_shoulder_angle_r + 360) % 360
        # scale 0 to 360 to ==> 0 to 127
        my_r_shoulder = int(remap(int(my_shoulder_angle_r), 0, 360, 0, 127))
        if sendmidi:
            # send midi signal
            midiout.send_message([176, 70, my_r_shoulder])
            # wait, so othe midi works
            time.sleep(0.05)
        
        ###############
        # Right Elbow #
        ###############
        
        # get variables for (x,y) shoulder and elbow
        ex = P_right[2][0][3]+100
        ey = P_right[2][1][3]+100
        cx = P_right[1][0][3]+100
        cy = P_right[1][1][3]+100
        
        # calculate absolute rotation on screen
        my_elbow_angle_r = degrees(atan2((ey - cy), (ex - cx)))
        # 0 to 360
        my_elbow_angle_r = (my_elbow_angle_r + 360) % 360
        # scale 0 to 360 to ==> 0 to 127
        my_r_elbow = int(remap(int(my_elbow_angle_r), 0, 360, 0, 127))
        # flip the values to opposite midi range
        my_r_elbow = abs(my_r_elbow - 127)
        # if we fo below the zero, recover to 0-127 range
        if my_r_elbow < 0:
            my_r_elbow = 127 + my_r_elbow
        if sendmidi:
            # send midi signal
            midiout.send_message([176, 72, my_r_elbow])
            # wait, so othe midi works
            time.sleep(0.05)
        
        ################
        # Draw a Wheel #
        ################
        plt.scatter(rr,cc, color='y')

        # Draw Everything on Graph
        fig.canvas.draw()
        # Pause for visibility in the loop
        if animated_graph:
            plt.pause(0.01)
        
        
        ####################################
        # Save Joints MIDI Data for Replay #
        ####################################
        with open('wheel_points.csv', 'a', newline='') as csvfile:
            spamwriter = csv.writer(csvfile, delimiter=',')
            spamwriter.writerow([datetime.now(tz=None)] + [ind, my_r_shoulder, my_r_elbow, my_l_shoulder, my_l_elbow])
            
        
        
        
        
        
        
    if sendmidi:
        # close midi sending signal    
        midiout.close_port()

#########################
# When we start the App #
#########################
def main():
    # Set up Graph Titles
    fig.canvas.mpl_connect('button_press_event', onclick)
    fig.suptitle("Cyclic Coordinate Descent - Inverse Kinematics", fontsize=12)
    # Set up graph Ranges -------> attach to settings
    ax.set_xlim(graph_X_min, graph_X_max)
    ax.set_ylim(graph_Y_min, graph_Y_max)

    # Forward Kinematics - right arm
    P = FK(angle, r_arm)
    # init do once
    ii=0
    # loop over the joints
    for i in range(len(r_arm)):
        start_point = P[i]
        end_point = P[i+1]
        if ii==0:
            # draw circle
            circle1 = plt.Circle((end_point[0,3],end_point[1,3]), 5, color='r')
            # do once
            ii += 1
        # plot the arm dots on the graph
        ax.plot([start_point[0,3], end_point[0,3]], [start_point[1,3], end_point[1,3]], linewidth=5)
    
    # Forward Kinematics - left arm
    P2 = FK(angle2, l_arm,2)
    # init do once
    ii=0
    # loop over the joints
    for i in range(len(l_arm)):
        start_point = P2[i]
        end_point = P2[i+1]
        if ii==0:
            # draw circle
            circle2 = plt.Circle((end_point[0,3],end_point[1,3]), 5, color='r')
            # do once
            ii+=1
        # plot the arm dots on the graph
        ax.plot([start_point[0,3], end_point[0,3]], [start_point[1,3], end_point[1,3]], linewidth=5)
    
    ################
    # Draw a Wheel #
    ################
    plt.scatter(rr,cc, color='y')
    # draw elbow circles on the gpraph
    plt.gca().add_patch(circle2)
    plt.gca().add_patch(circle1)
    # draw a graph
    fig.canvas.draw()
    # show a graph
    plt.show()

if __name__ == "__main__":
    main()