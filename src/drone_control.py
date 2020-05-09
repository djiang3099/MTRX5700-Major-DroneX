#!/usr/bin/env python

# MTRX5700 Major Project DroneX 
# 470355499 470355503

import rospy

import math
import numpy as np

import sensor_msgs.msg
import nav_msgs.msg
import std_msgs
import geometry_msgs.msg
from mtrx_major.msg import Navdata
from geometry_msgs.msg import Twist, Vector3, Pose, PoseWithCovariance, Point, Quaternion
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, Image, CompressedImage
from tf.transformations import euler_from_quaternion

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
        print("Odom Stub")
        self.PID.set_pose(odomMsg.pose.pose)
        self.PID.set_goal(odomMsg.pose.pose)
        self.goalSet = 1

        return

    def navdata_callback(self, navdataMsg):
        print("Navdata Stub")
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