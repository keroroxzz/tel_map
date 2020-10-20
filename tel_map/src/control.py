#!/usr/bin/env python

#Default Sakura
#Author : RTU
#Version : 1.0

import sys
import rospy
import math
import cv2
import numpy as np

#native msgs
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist

class Sakura:

    #Speed control
    vel = rospy.Publisher('/sakura/cmd_vel', Twist, queue_size=1)

    #Shovel (0.0 ~ 2.0?)
    shovel = rospy.Publisher('/sakura/shovel/command', Float64, queue_size=1)

    #Roller arm
    rol_arm = rospy.Publisher('/sakura/roller/arm/command', Float64, queue_size=1)

    #Slider of the arm (0.0~0.16)
    rol_slid = rospy.Publisher('/sakura/roller/slider/command', Float64, queue_size=1)

    #Not funtional yet
    rol_rol = rospy.Publisher('/sakura/roller/roller', Float64, queue_size=1)

    #rotation platform (0 ~ +pi*3/2) and yaw of camera (-pi/2 ~ +pi/2)
    rot = rospy.Publisher('/sakura/rot/command', Float64, queue_size=1)
    cam_y = rospy.Publisher('/sakura/cam_y/command', Float64, queue_size=1)

    #Robot arm (all -pi/2 ~ +pi/2)
    arm_base = rospy.Publisher('/sakura/arm/base/command', Float64, queue_size=1)
    arm_j1 = rospy.Publisher('/sakura/arm/j1/command', Float64, queue_size=1)
    arm_j2 = rospy.Publisher('/sakura/arm/j2/command', Float64, queue_size=1)
    arm_j3 = rospy.Publisher('/sakura/arm/j3/command', Float64, queue_size=1)
    arm_j4 = rospy.Publisher('/sakura/arm/j4/command', Float64, queue_size=1)

    #Gripper (both 0.0 ~ 0.025)
    arm_f1 = rospy.Publisher('/sakura/arm/f1/command', Float64, queue_size=1)
    arm_f2 = rospy.Publisher('/sakura/arm/f2/command', Float64, queue_size=1)

    def __init__(self):
        pass

class Dragbar:

    def __init__(self, barname, winname, min, max, resolution, topic):
        self.barname = barname
        self.winname = winname
        self.min = min
        self.max = max
        self.resolution = resolution
        
        self.pub = rospy.Publisher(topic, Float64, queue_size=1)

        self.val = 0.0

    def show(self, img, pos, size):
        cv2.createTrackbar(self.barname, self.winname, 0, self.resolution, self.callback)

        cv2.putText(img, '{0} : {1:.2f}'.format(self.barname, self.val), pos, cv2.FONT_HERSHEY_SIMPLEX, size, (255,255,255),2, cv2.LINE_AA)
    
    def callback(self, v):
        self.val = (self.max-self.min)*v/self.resolution + self.min
        self.pub.publish(self.val)
    


class Panel:

    def __init__(self):
        self.sakura = Sakura()

        self.shovel = Dragbar('shovel', 'panel', -0.5, 3.14159, 1024, '/sakura/shovel/command')
        self.roller_arm = Dragbar('roller_arm', 'panel', -2.0, 1.0, 1024, '/sakura/roller/arm/command')
        self.roller_slider = Dragbar('roller_slider', 'panel', 0.0, 0.25, 1024, '/sakura/roller/slider/command')
        self.roller = Dragbar('roller', 'panel', -2000, 2000, 1024, '/sakura/roller/roller')
        self.rot = Dragbar('rot', 'panel', 0.0, 3.14159+1.57, 1024, '/sakura/rot/command')
        self.cam_y = Dragbar('cam_y', 'panel', -1.57, 1.57, 1024, '/sakura/cam_y/command')
        self.arm_base = Dragbar('arm_base', 'panel', -1.57, 1.57, 1024, '/sakura/arm/base/command')
        self.arm_j1 = Dragbar('arm_j1', 'panel', -1.57, 1.57, 1024, '/sakura/arm/j1/command')
        self.arm_j2 = Dragbar('arm_j2', 'panel', -1.57, 1.57, 1024, '/sakura/arm/j2/command')
        self.arm_j3 = Dragbar('arm_j3', 'panel', -1.57, 1.57, 1024, '/sakura/arm/j3/command')
        self.arm_j4 = Dragbar('arm_j4', 'panel', -1.57, 1.57, 1024, '/sakura/arm/j4/command')
        self.arm_f1 = Dragbar('arm_f1', 'panel', 0.0, 0.03, 1024, '/sakura/arm/f1/command')
        self.arm_f2 = Dragbar('arm_f2', 'panel', 0.0, 0.03, 1024, '/sakura/arm/f2/command')

        cv2.namedWindow('panel', cv2.WINDOW_NORMAL)
        cv2.namedWindow('control', cv2.WINDOW_NORMAL)

    def loop(self):
        while not rospy.is_shutdown():
            try:
                img = np.zeros((420,300))

                size = 0.8
                cv2.namedWindow('panel', cv2.WINDOW_NORMAL)
                cv2.resizeWindow('panel',(420,300))
                self.shovel.show(img, (20, 30), size)
                self.roller_arm.show(img, (20, 60), size)
                self.roller_slider.show(img, (20, 90), size)
                self.roller.show(img, (20, 120), size)
                self.rot.show(img, (20, 150), size)
                self.cam_y.show(img, (20, 180), size)
                self.arm_base.show(img, (20, 210), size)
                self.arm_j1.show(img, (20, 240), size)
                self.arm_j2.show(img, (20, 270), size)
                self.arm_j3.show(img, (20, 300), size)
                self.arm_j4.show(img, (20, 330), size)
                self.arm_f1.show(img, (20, 360), size)
                self.arm_f2.show(img, (20, 390), size)
                cv2.waitKey(1)

                cv2.namedWindow('control', cv2.WINDOW_NORMAL)
                cv2.resizeWindow('control',(300,420))
                cv2.imshow('control', img)
                cv2.waitKey(1)

                rospy.Rate(100).sleep()
            except:
                pass

rospy.init_node('control')
panel = Panel()

try:
    panel.loop()
except rospy.ROSInterruptException:
    pass