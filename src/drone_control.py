#!/usr/bin/env python

# MTRX5700 Major Project DroneX 
# 470355499 470355503

import rospy

import math
import numpy as np
import copy

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

class DroneController():
    def __init__(self, kp=1, ki=0, kd=0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        return

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

    def compute(self):
        command = Twist()
        command.linear.x = self.kp * (self.goalX - self.x)
        command.linear.y = self.kp * (self.goalY - self.y)
        command.linear.z = self.kp * (self.goalZ - self.z)
        command.angular.x = self.kp * (self.goalZ - self.z)
        print(command)
        return command

class DroneX():
    # In the initialisation:
    # - Set up subscribers and publishers
    # - Zero all velocities
    def __init__(self, controller=None):
        print('Initialising a drone...')
        self.odomOffset = None
        self.pos = None
        self.orient = None
        self.battery = 100
        self.initPos = None
        self.initOrient = None

        self.goalSet = 0

        if controller is not None: 
            self.PID = controller
        else: 
            self.PID = DroneController()

        # Define subscribers
        self.odomSub = rospy.Subscriber("ardrone/odometry", nav_msgs.msg.Odometry, self.odom_callback, queue_size=100)
        self.navdataSub = rospy.Subscriber("/ardrone/navdata", Navdata, self.navdata_callback, queue_size=100)

        # Define publishers
        self.takeoffPub = rospy.Publisher('/ardrone/takeoff', std_msgs.msg.Empty, queue_size=10)
        self.landingPub = rospy.Publisher('ardrone/land', std_msgs.msg.Empty, queue_size=10)
        self.estopPub = rospy.Publisher('ardrone/reset', std_msgs.msg.Empty, queue_size=10)
        
        self.zeroOdomPub = rospy.Publisher('dronex/odom', nav_msgs.msg.Odometry, queue_size=100)
        self.commandPub = rospy.Publisher('cmd_vel', geometry_msgs.msg.Twist, queue_size=100)    

        r = rospy.Rate(30)
        while not rospy.is_shutdown():

            if self.goalSet == 1:
                self.move()
            r.sleep()

    def odom_callback(self, odomMsg):
        print("odom")
        # Store the first odom
        if (self.pos is None):
            print("FIRST ODOM######################")
            self.initPos = copy.copy(odomMsg.pose.pose.position)
            quat = odomMsg.pose.pose.orientation
            self.initRPY = euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])
            
            print(odomMsg)
        else:
            print("here")
        print(self.initPos.x)
        # Subtract current odom by first odom
        odomMsg.pose.pose.position.x = odomMsg.pose.pose.position.x- self.initPos.x
        odomMsg.pose.pose.position.y = odomMsg.pose.pose.position.y- self.initPos.y
        odomMsg.pose.pose.position.z = odomMsg.pose.pose.position.z- self.initPos.z
        
        quat = odomMsg.pose.pose.orientation
        roll, pitch, yaw = euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])
        newRoll = roll - self.initRPY[0]
        newPitch = pitch - self.initRPY[1]
        newYaw = yaw - self.initRPY[2]
        x,y,z,w = quaternion_from_euler(roll,pitch,yaw,'rxyz')
        odomMsg.pose.pose.orientation = Quaternion(x,y,z,w)
        
        # zeroOdomMsg = geometry_msg.msg.poseStamped()
        # zeroOdomMsg.header.frame_id = "odom"
        # zeroOdomMsg.pose.position = msg.pose.pose.posiion
        # zeroOdomMsg.pose.orientation = msg.pose.pose.orientation

        # Publish the subtracted odom
        self.pose = odomMsg.pose.pose
        self.pos = odomMsg.pose.pose.position

        self.zeroOdomPub.publish(odomMsg)   

        self.PID.set_pose(odomMsg.pose.pose)
        self.PID.set_goal(odomMsg.pose.pose)
        self.goalSet = 1     
        
        return

    def navdata_callback(self, navdataMsg):
        print("     nav")
        # update the battery status
        self.battery = navdataMsg.batteryPercent

        return


    def move(self):
        command = self.PID.compute()
        self.commandPub.publish(command)
        print("PUBLISHED!")

        return

    
if __name__ == '__main__':
    print('Drone Control started...')
    rospy.init_node('drone_control')

    DroneController = DroneController()
    Drone = DroneX(DroneController)
