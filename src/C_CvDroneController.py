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
    def __init__(self, time, kp=0.001, ki=0.0000, kd=0.0001):
        print("Initialising drone controller...", time)
        # Track the time the controller is called at and initial time
        self.lastTime = time
        self.yaw = 0.0

        # Actuation limit
        self.ctrlLimit = 0.1    # Max control output
        self.yawLimit = 0.3     # Max yaw rate
        self.errThresh = 0.1   # Threshold to use hover functionality
        
        # Gains for drone moving up/down/left/right
        self.kp_zy = kp
        self.ki_zy = ki
        self.kd_zy = kd

        # Gains for in and out of the frame
        self.kp_x = kp*2
        self.ki_x = ki
        self.kd_x = kd

        # Gains for drone yaw
        self.kp_ang = kp*100
        self.ki_ang = ki*100
        self.kd_ang = kd*100

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
        self.goalYaw = 0

        # For result plots
        self.plotErrorX = []
        self.plotErrorY = []
        self.plotErrorZ = []
        self.plotCommandX = []
        self.plotCommandY = []
        self.plotCommandZ = []
        self.plotTime = []

        return

    def set_centre(self, centreY, centreZ):
        # Limit the centre to be within a margin
        if centreY > 520:
            centreY = 520
        elif centreY < 120:
            centreY = 120
        
        if centreZ > 300:
            centreZ = 300
        elif centreZ < 60:
            centreZ = 60

        self.centreZ = centreZ
        self.centreY = centreY
        return

    def get_centre(self):
        return self.centreY, self.centreZ

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
        # print("dt", dt,  h/self.refHeight)
        self.lastTime = time


        # Compute Proportional error
        linXErr = (1 - float(h)/self.refHeight) * 100
        linYErr = self.centreY - (y+w/2)
        linZErr = self.centreZ - (z+h/2)
        angZErr = self.goalYaw - self.yaw

        # Compute Derivative error
        d_linXErr = (linXErr - self.prevErrorX)/dt
        d_linYErr = (linYErr - self.prevErrorY)/dt
        d_linZErr = (linZErr - self.prevErrorZ)/dt
        d_angZErr = (angZErr - self.prevErrorYaw)/dt

        # Update previous error
        self.prevErrorX = linXErr
        self.prevErrorY = linYErr
        self.prevErrorZ = linZErr
        self.prevErrorYaw = angZErr

        # Compute Integral error with saturation
        self.i_linX = np.sign(self.i_linX + linXErr*dt) * min(self.intSat, \
            abs(self.i_linX + linXErr*dt))
        # print("Integral ", self.i_linY + linYErr*dt, self.i_linZ + linZErr*dt)
        if abs(dt) < 50:
            # self.i_linY = self.i_linY + linYErr*dt
            # self.i_linZ  =self.i_linZ + linZErr*dt
            self.i_linY = np.sign(self.i_linY + linYErr*dt) * min(self.intSat, \
                abs(self.i_linY + linYErr*dt))
            self.i_linZ = np.sign(self.i_linZ + linZErr*dt) * min(self.intSat, \
                abs(self.i_linZ + linZErr*dt))
            self.i_angZ = np.sign(self.i_angZ + angZErr*dt) * min(self.intSat, \
                abs(self.i_angZ + angZErr*dt))

        # If very close to the goal, hover
        if abs(linXErr) < 15 and abs(linYErr) < 30 and \
            abs(linZErr) < 20 and abs(angZErr) < 0.35:
            command.linear.x = 0.0
            command.linear.y = 0.0
            command.linear.z = 0.0
            command.angular.x = 0.0
            command.angular.y = 0.0
            command.angular.z = 0.0
            print("----------------------- At Goal Hovering!!!")

        else:
            command.linear.x = 0.0
            command.linear.y = 0.0
            command.linear.z = 0.0
            command.angular.x = 0.0
            command.angular.y = 0.0
            command.angular.z = 0.0

            if linXErr < 0:
                controlX = (self.kp_x*0.3 * linXErr) + 0*(self.kd_x * d_linXErr) + (self.ki_x * self.i_linX)
            else: 
                controlX = (self.kp_x*0.6 * linXErr) + 0*(self.kd_x * d_linXErr) + (self.ki_x * self.i_linX)
            command.linear.x = np.sign(controlX) * min(self.ctrlLimit, abs(controlX))

            controlY = (self.kp_zy*0.15 * linYErr) + (self.kd_zy * d_linYErr) + (self.ki_zy * self.i_linY)
            command.linear.y = np.sign(controlY)* min(self.ctrlLimit, abs(controlY))
            
            controlZ = (self.kp_zy * linZErr) + (self.kd_zy * d_linZErr) + (self.ki_zy * self.i_linZ)
            command.linear.z = np.sign(controlZ) * min(self.ctrlLimit, abs(controlZ))
            
            ## Yaw odometry too inaccurate for useful control
            # controlYaw = (self.kp_ang * angZErr) + (self.kd_ang * d_angZErr) + (self.ki_ang * self.i_angZ)
            # command.angular.z = np.sign(controlYaw) * min(self.yawLimit, abs(controlYaw))

            # print("PID X:   {:1.4}, {:1.4}, {:1.4}".format((self.kp_zy * linXErr), \
            #     (self.ki_zy * self.i_linX), (self.kd_zy * d_linXErr)))
            # print("PID Y:   {:1.4}, {:1.4}, {:1.4}".format((self.kp_zy*0.15 * linYErr), \
            #     (self.ki_zy * self.i_linY), (self.kd_zy * d_linYErr)))
            # print("PID Z:   {:+05.4f}, {:+05.4f}, {:+05.4f}".format((self.kp_zy * linZErr), \
            #     (self.ki_zy * self.i_linZ), (self.kd_zy * d_linZErr)))
            

        print("Error:   {:+05.2f}, {:+05.2f}, {:+05.2f}, Size: {}, {}".format(linXErr, linYErr, linZErr, w, h))
        print("Command: {:+05.4f}, {:+05.4f}, {:+05.4f}, {:+05.4f}, {:+05.4f}, {:+05.4f}\n".format(command.linear.x,\
            command.linear.y, command.linear.z, command.angular.x ,command.angular.y,\
                command.angular.z))

        # For result plots
        # self.plotErrorX.append(linXErr/50)
        # self.plotErrorY.append(linYErr/100)
        # self.plotErrorZ.append(linZErr/50)
        self.plotErrorX.append(linXErr/1)
        self.plotErrorY.append(linYErr/1)
        self.plotErrorZ.append(linZErr/1)
        self.plotCommandX.append(command.linear.x)
        self.plotCommandY.append(command.linear.y)
        self.plotCommandZ.append(command.linear.z)
        self.plotTime.append(time)

        return command

    ################### FOR ABSOLUTE POSE CONTROL ############################

    # Setter for current drone pose
    # Takes a geometry_msgs.msg.pose.pose type as input.
    def set_pose(self, pose):
        self.x = pose.position.x
        self.y = pose.position.y
        self.z = pose.position.z
        quat = pose.orientation
        self.roll, self.pitch, self.yaw = \
            euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])
        return
        
    # Takes a geometry_msgs.msg.pose.pose type as input.
    def set_goal(self, pose):
        self.goalX = pose.position.x
        self.goalY = pose.position.y
        self.goalZ = pose.position.z
        quat = pose.orientation
        self.goalRoll, self.goalPitch, self.goalYaw = \
            euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])
        return

    def compute_absolute(self, time, pose=None, goal=None):
        if pose is not None:
            self.set_pose(pose)
        if goal is not None:
            self.set_goal(goal)

        command = Twist()
        dt = time - self.lastTime
        self.lastTime = time

        # Compute Proportional error
        linXErr = self.goalX - self.x
        linYErr = self.goalY - self.y
        linZErr = self.goalZ - self.z
        angZErr = self.goalYaw - self.yaw

        # Transform the error into the 'world' frame
        realX = linXErr*np.cos(-self.yaw) - linYErr*np.sin(-self.yaw)
        realY = linXErr*np.sin(-self.yaw) + linYErr*np.cos(-self.yaw)

        # Compute Derivative error
        d_linXErr = (realX - self.prevErrorX)/dt
        d_linYErr = (realY - self.prevErrorY)/dt
        d_linZErr = (linZErr - self.prevErrorZ)/dt
        d_angZErr = (angZErr - self.prevErrorYaw)/dt

        # Update previous error
        self.prevErrorX = realX
        self.prevErrorY = realY
        self.prevErrorZ = linZErr
        self.prevErrorYaw = angZErr

        # Compute Integral error with saturation
        self.i_linX = np.sign(self.i_linX + realX*dt) * min(self.intSat, \
            abs(self.i_linX + realX*dt))
        self.i_linY = np.sign(self.i_linY + realY*dt) * min(self.intSat, \
            abs(self.i_linY + realY*dt))
        self.i_linZ = np.sign(self.i_linZ + linZErr*dt) * min(self.intSat, \
            abs(self.i_linZ + linZErr*dt))
        self.i_angZ = np.sign(self.i_angZ + angZErr*dt) * min(self.intSat, \
            abs(self.i_angZ + angZErr*dt))

        # If very close to the goal, hover
        if abs(linXErr) < 20 and abs(linYErr) < 20 and \
            abs(linZErr) < 10 and abs(angZErr) < 0.35:
            command.linear.x = 0.0
            command.linear.y = 0.0
            command.linear.z = 0.0
            command.angular.x = 0.0
            command.angular.y = 0.0
            command.angular.z = 0.0

        else:
            controlX = (self.kp_zy * realX) + (self.kd_zy * d_linXErr) + (self.ki_zy * self.i_linX)
            command.linear.x = np.sign(controlX) * min(self.ctrlLimit, abs(controlX))

            controlY = (self.kp_zy * realY) + (self.kd_zy * d_linYErr) + (self.ki_zy * self.i_linY)
            command.linear.y = np.sign(controlY)* min(self.ctrlLimit, abs(controlY))
            
            controlZ = (self.kp_zy * linZErr) + (self.kd_zy * d_linZErr) + (self.ki_zy * self.i_linZ)
            command.linear.z = np.sign(controlZ) * min(self.ctrlLimit, abs(controlZ))
            
            controlYaw = (self.kp_ang * angZErr) + (self.kd_ang * d_angZErr) + (self.ki_ang * self.i_angZ)
            command.angular.z = np.sign(controlYaw) * min(self.yawLimit, abs(controlYaw))

            command.angular.x = 0.0
            command.angular.y = 0.0

        print(command)

        return command