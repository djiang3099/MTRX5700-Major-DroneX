#!/usr/bin/env python

# MTRX5700 Major Project DroneX 
# 470355499 470355503

import cv2
import numpy as np
import argparse
import rospy
import copy
from cv_bridge import CvBridge

import geometry_msgs.msg
import sensor_msgs.msg
import nav_msgs.msg
import std_msgs
import geometry_msgs.msg
from mtrx_major.msg import Navdata
from geometry_msgs.msg import Twist, Vector3, Pose, PoseWithCovariance, Point, Quaternion
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, Image, CompressedImage
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from std_msgs.msg import Int8

class CvDroneController():
    def __init__(self, time, kp=0.0005, ki=0.00005, kd=0.0003):
        print("Initialising drone controller...", time)
        # Track the time the controller is called at and initial time
        self.lastTime = time

        # Actuation limit
        self.ctrlLimit = 0.1    # Max control output
        self.errThresh = 0.05   # Threshold to use hover functionality
        
        # Gains for drone moving up/down/left/right
        self.kp_zy = kp
        self.ki_zy = ki
        self.kd_zy = kd

        # Gains for in and out of the frame
        self.kp_x = kp*10
        self.ki_x = ki*10
        self.kd_x = kd*10

        # Store previous error for derivative control
        self.prevErrorX = 0
        self.prevErrorY = 0
        self.prevErrorZ = 0
        self.prevErrorYaw = 0

        # Integral control history
        self.i_linX = 0
        self.i_linY = 0
        self.i_linZ = 0
        self.i_angZ = 0

        # Integral controller saturation
        self.intSat = 300

        # Default target settings
        self.centreZ = 240
        self.centreY = 300
        self.refHeight = 14
        self.refWidth = 10

        return

    def set_centre(self, centreY, centreZ):
        self.centreZ = centreZ
        self.centreY = centreY
        return

    def set_target_size(self, w, h):
        self.refHeight = h
        self.refWidth = w
        return

    def hover(self):
        command = Twist()
        command.linear.x = 0.0
        command.linear.y = 0.0
        command.linear.z = 0.0
        command.angular.x = 0.0
        command.angular.y = 0.0
        command.angular.z = 0.0
        print("----------------------- Hovering!!!")

        return command

    def compute(self, time, y, z, w, h):
        command = Twist()
        dt = time - self.lastTime
        print("dt", dt,  h/self.refHeight)
        self.lastTime = time


        # Compute Proportional error
        linXErr = (1 - float(h)/self.refHeight)*100
        linYErr = self.centreY - y
        linZErr = self.centreZ - z
        # angZErr = self.goalYaw - self.yaw

        # Compute Derivative error
        d_linXErr = (linXErr - self.prevErrorX)/dt
        d_linYErr = (linYErr - self.prevErrorY)/dt
        d_linZErr = (linZErr - self.prevErrorZ)/dt
        # d_angZErr = (angZErr - self.prevErrorYaw)/dt

        # Update previous error
        self.prevErrorX = linXErr
        self.prevErrorY = linYErr
        self.prevErrorZ = linZErr
        # self.prevErrorYaw = angZErr

        # Compute Integral error with saturation
        self.i_linX = np.sign(self.i_linX + linXErr*dt) * min(self.intSat, \
            abs(self.i_linX + linXErr*dt))
        print("Integral ", self.i_linY + linYErr*dt, self.i_linZ + linZErr*dt)
        if abs(dt) < 50:
            # self.i_linY = self.i_linY + linYErr*dt
            # self.i_linZ  =self.i_linZ + linZErr*dt
            self.i_linY = np.sign(self.i_linY + linYErr*dt) * min(self.intSat, \
                abs(self.i_linY + linYErr*dt))
            self.i_linZ = np.sign(self.i_linZ + linZErr*dt) * min(self.intSat, \
                abs(self.i_linZ + linZErr*dt))
            # self.i_angZ = np.sign(self.i_angZ + angZErr*dt) * min(self.intSat, \
            #     abs(self.i_angZ + angZErr*dt))

        # If very close to the goal, hover
        if abs(linXErr) < self.errThresh and abs(linYErr) < self.errThresh and \
            abs(linZErr) < self.errThresh and abs(angZErr) < self.errThresh:
            command.linear.x = 0.0
            command.linear.y = 0.0
            command.linear.z = 0.0
            command.angular.x = 0.0
            command.angular.y = 0.0
            command.angular.z = 0.0
            print("----------------------- Hovering!!!")

        else:
            controlX = (self.kp_zy * linXErr) + (self.kd_zy * d_linXErr) + (self.ki_zy * self.i_linX)
            command.linear.x = np.sign(controlX) * min(self.ctrlLimit, abs(controlX))

            controlY = (self.kp_zy * linYErr) + (self.kd_zy * d_linYErr) + (self.ki_zy * self.i_linY)
            command.linear.y = np.sign(controlY)* min(self.ctrlLimit, abs(controlY))
            
            controlZ = (self.kp_zy * linZErr) + (self.kd_zy * d_linZErr) + (self.ki_zy * self.i_linZ)
            command.linear.z = np.sign(controlZ) * min(self.ctrlLimit, abs(controlZ))
            
            # controlYaw = (self.kp * angZErr) + (self.kd * d_angZErr) + (self.ki * self.i_angZ)
            # command.angular.z = np.sign(controlYaw) * min(self.ctrlLimit, abs(controlYaw))

            command.angular.x = 0.0
            command.angular.y = 0.0
            command.angular.z = 0.0
            print("PID X:   {:1.4}, {:1.4}, {:1.4}".format((self.kp_zy * linXErr), \
                (self.ki_zy * self.i_linX), (self.kd_zy * d_linXErr)))
            print("PID Y:   {:1.4}, {:1.4}, {:1.4}".format((self.kp_zy * linYErr), \
                (self.ki_zy * self.i_linY), (self.kd_zy * d_linYErr)))
            print("PID Z:   {:1.4}, {:1.4}, {:1.4}".format((self.kp_zy * linZErr), \
                (self.ki_zy * self.i_linZ), (self.kd_zy * d_linZErr)))
            

        print("Error:   {}, {}, {}, Size: {}, {}".format(linXErr, linYErr, linZErr, w, h))
        print("Command: {:.2}, {:.2}, {:.2}, {:.2}, {:.2}, {:.2}\n".format(command.linear.x,\
            command.linear.y, command.linear.z, command.angular.x ,command.angular.y,\
                command.angular.z))
        return command