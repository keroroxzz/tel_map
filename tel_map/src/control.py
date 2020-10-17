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
    rol_rol = rospy.Publisher('/sakura/roller/roller/command', Float64, queue_size=1)

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
