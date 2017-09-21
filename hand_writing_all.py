# -*- coding: utf-8 -*-
"""
@author: zttara
"""

################################################################################
# Copyright (C) 2012-2013 Leap Motion, Inc. All rights reserved.               #
# Leap Motion proprietary and confidential. Not for distribution.              #
# Use subject to the terms of the Leap Motion SDK Agreement available at       #
# https://developer.leapmotion.com/sdk_agreement, or another agreement         #
# between Leap Motion and you, your company or other organization.             #
################################################################################

from __future__ import division 
import sys, time
import matplotlib.pyplot as plt

sys.path.insert(0, "E:\Leap\leap_python")
import Leap

import matplotlib.pyplot as plt  
from mpl_toolkits.mplot3d import Axes3D  


import numpy as np 
from matplotlib import cm
import math


#import xlwt

co_list_x = []
co_list_y = []
co_list_z = []

class SampleListener(Leap.Listener):
    finger_names = ['Thumb', 'Index', 'Middle', 'Ring', 'Pinky']
    bone_names = ['Metacarpal', 'Proximal', 'Intermediate', 'Distal']
    
    def on_init(self, controller):
        print "Initialized"

    def on_connect(self, controller):
        print "Connected"

    def on_disconnect(self, controller):
        # Note: not dispatched when running in a debugger.
        print "Disconnected"

    def on_exit(self, controller):
        print "Exited"

    def on_frame(self, controller):
        # Get the most recent frame and report some basic information
        frame = controller.frame()

        print "Frame id: %d, timestamp: %d, hands: %d, fingers: %d" % (
              frame.id, frame.timestamp, len(frame.hands), len(frame.fingers))
        
        # Get hands
        for hand in frame.hands:

            handType = "Left hand" if hand.is_left else "Right hand"

            # print "  %s, id %d, position: %s" % (
            #     handType, hand.id, hand.palm_position)

            # Get the hand's normal vector and direction
            normal = hand.palm_normal
            direction = hand.direction

            # Calculate the hand's pitch, roll, and yaw angles
            # print "  pitch: %f degrees, roll: %f degrees, yaw: %f degrees" % (
            #     direction.pitch * Leap.RAD_TO_DEG,
            #     normal.roll * Leap.RAD_TO_DEG,
            #     direction.yaw * Leap.RAD_TO_DEG)

            # Get arm bone
            # arm = hand.arm
            # print "  Arm direction: %s, wrist position: %s, elbow position: %s" % (
            #     arm.direction,
            #     arm.wrist_position,
            #     arm.elbow_position)

            # Get fingers
            # hand.fingers[1] is the pointer
            finger = hand.fingers[1]
            # for finger in hand.fingers:
                
            #     print "    %s finger" % (self.finger_names[finger.type])
                # print "    %s finger, id: %d, length: %fmm, width: %fmm" % (
                #     self.finger_names[finger.type],
                #     finger.id,
                #     finger.length,
                #     finger.width)

            # Get end of pointer bone
            bone = finger.bone(3)
            co_list_x.append(bone.next_joint.x)
            co_list_y.append(bone.next_joint.y)
            co_list_z.append(bone.next_joint.z)
            
            '''
            #capping height of vertical distance from the leap motion 
            if bone.next_joint[1]<100:
                arr.append((bone.next_joint[0], bone.next_joint[2]))
                print 'tracking'

            '''
                # for b in range(0, 4):
                #     bone = finger.bone(b)
                #     print "      Bone: %s, start: %s, end: %s, direction: %s" % (
                #         self.bone_names[bone.type],
                #         bone.prev_joint,
                #         bone.next_joint,
                #         bone.direction)

        # if not frame.hands.is_empty:
        #     print ""

def main():
    # Create a sample listener and controller
    listener = SampleListener()
    controller = Leap.Controller()
    
    # Have the sample listener receive events from the controller
    controller.add_listener(listener)

    # Keep this process running until Enter is pressed
    print "Press Enter to quit..."
    try:
        sys.stdin.readline()
    except KeyboardInterrupt:
        pass
    finally:
        # Remove the sample listener when done
        controller.remove_listener(listener)

#----------------------------------------------------------------------------------------------------
#Define three space points to make a plane,they are O(x1,y1,z1),P(x2,y2,z2),Q(x3,y3,z3),respectively.
#The coordinate of start point when writing in the air.
    #The coordinate of start point when writing in the air.
    x1 = co_list_x[0]
    y1 = co_list_y[0]
    z1 = co_list_z[0]

    #The coordinate of middle point when writing in the air.
    x2 = co_list_x[int(len(co_list_x)/2)]
    y2 = co_list_y[int(len(co_list_y)/2)]
    z2 = co_list_z[int(len(co_list_z)/2)]

    #The coordinate of end point when writing in the air.
    x3 = co_list_x[len(co_list_x)-1]
    y3 = co_list_y[len(co_list_y)-1]
    z3 = co_list_z[len(co_list_z)-1]

    #Calculate the coefficient of the plane equation which made by O,P and Q.
    A = (y3-y1)*(z3-z1)-(z2-z1)*(y3-y1)
    B = (x3-x1)*(z2-z1)-(x2-x1)*(z3-z1)
    C = (x2-x1)*(y3-y1)-(x3-x1)*(y2-y1)


    new_x = []
    new_y = []
    new_z = []

#--------------------------------------------------------------------
#Project the 3D space points into the 3D space plane which made by O,P and Q.
    for i in range(len(co_list_x)):
        Xp = co_list_x[i]
        Yp = co_list_y[i]
        Zp = co_list_z[i]

        #Calculate the coordinates of projection points which will be projected into 3D space plane.
        x0 = (A*x2+B*B*Xp/A-B*Yp+B*y2+C*C*Xp/A-C*Zp-C*z2)/(A+B*B/A+C*C/A)
        y0 = B*(x0-Xp)/A+Yp
        z0 = C*(x0-Xp)/A+Zp
        new_x.append(x0)
        new_y.append(y0)
        new_z.append(z0)

#---------------------------------------------------------
#The visualization of hand writing and projection results.
    fig1 = plt.figure('fig1')  
    ax1 = fig1.add_subplot(111, projection='3d')   
    x11 = co_list_x
    y11 = co_list_y
    z11 = co_list_z
    ax1.w_xaxis.set_pane_color((1.0, 1.0, 1.0, 1.0))
    ax1.w_yaxis.set_pane_color((1.0, 1.0, 1.0, 1.0))
    ax1.w_zaxis.set_pane_color((1.0, 1.0, 1.0, 1.0))
    ax1.scatter(z11,x11,y11)

    fig2 = plt.figure('fig2')  
    ax2 = fig2.add_subplot(111, projection='3d')   
    x21 = new_x
    y21 = new_y
    z21 = new_z
    ax2.w_xaxis.set_pane_color((1.0, 1.0, 1.0, 1.0))
    ax2.w_yaxis.set_pane_color((1.0, 1.0, 1.0, 1.0))
    ax2.w_zaxis.set_pane_color((1.0, 1.0, 1.0, 1.0))
    ax2.scatter(z21,x21,y21)

#--------------------------------------------------------
#Transfor the 3D space plane into 2D plane.
    O=np.array([x1,y1,z1])
    P=np.array([x2,y2,z2])
    OP=[x-y for x,y in zip(P,O)]
    module_OP=math.sqrt(np.dot(OP,OP))
    u=[i * (1/module_OP) for i in OP]#vector unitization

    Q=np.array([x3,y3,z3])
    OQ=[x-y for x,y in zip(Q,O)]
    M=[j * np.dot(OQ,u) for j in u]
    V=[x-y for x,y in zip(OQ,M)]
    module_V=math.sqrt(np.dot(V,V))
    v=[t * (1/module_V) for t in V]

    x_2d=[]
    y_2d=[]
    for c in range(len(new_x)):
        K=np.array([new_x[c],new_y[c],new_z[c]])
        OK=[x-y for x,y in zip(K,O)]
        c_x=np.dot(OK,u)
        x_2d.append(c_x)
        c_y=np.dot(OK,v)
        y_2d.append(c_y)

    fig3 = plt.figure('fig3')
    plt.plot(x_2d,y_2d,linewidth=5)

    plt.show()


if __name__ == "__main__":
    main()