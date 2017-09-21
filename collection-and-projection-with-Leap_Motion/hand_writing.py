# -*- coding: utf-8 -*-
"""
Created on Fri Mar 24 15:26:38 2017

@author: zttara
"""

################################################################################
# Copyright (C) 2012-2013 Leap Motion, Inc. All rights reserved.               #
# Leap Motion proprietary and confidential. Not for distribution.              #
# Use subject to the terms of the Leap Motion SDK Agreement available at       #
# https://developer.leapmotion.com/sdk_agreement, or another agreement         #
# between Leap Motion and you, your company or other organization.             #
################################################################################

import sys, time
import matplotlib.pyplot as plt

sys.path.insert(0, "E:\Leap\leap_python")
import Leap

import matplotlib.pyplot as plt  
from mpl_toolkits.mplot3d import Axes3D  

import xlwt

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


    f = xlwt.Workbook() #创建工作簿
    sheet1 = f.add_sheet('sheet1',cell_overwrite_ok=True) #创建sheet
    print co_list_x
    print co_list_y
    print co_list_z
    for i in range(0,len(co_list_x)):
        sheet1.write(0,i,co_list_x[i])
        sheet1.write(1,i,co_list_y[i])
        sheet1.write(2,i,co_list_z[i])
    
  #生成第一列和最后一列(合并4行)
    f.save('demo1.xlsx') #保存文件

    fig = plt.figure()  
    ax = fig.add_subplot(111, projection='3d')   


    x = co_list_x
    y = co_list_y
    z = co_list_z

    ax.w_xaxis.set_pane_color((1.0, 1.0, 1.0, 1.0))
    ax.w_yaxis.set_pane_color((1.0, 1.0, 1.0, 1.0))
    ax.w_zaxis.set_pane_color((1.0, 1.0, 1.0, 1.0))
    #fig.set_size_inches(10,8)
    #ax.set_xlabel('x')  
    #ax.set_ylabel('y')  
    #ax.set_zlabel('z') 
    ax.scatter(z,x,y)
    plt.show()



if __name__ == "__main__":
    main()