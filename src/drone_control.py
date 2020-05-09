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

class DroneX():
    # In the initialisation:
    # - Set up subscribers and publishers
    # - Zero all velocities
    def __init__(self):
        print('Initialising a drone...')
        self.odomOffset = None
        self.pos = None
        self.orient = None
        self.battery = 100

        # Define subscribers
        odomSub = rospy.Subscriber("ardrone/odometry", nav_msgs.msg.Odometry, self.odom_callback, queue_size=100)
        navdataSub = rospy.Subscriber("/ardrone/navdata", Navdata, self.navdata_callback, queue_size=100)

        # Define publishers
        takeoffPub = rospy.Publisher('/ardrone/takeoff', std_msgs.msg.Empty, queue_size=10)
        landingPub = rospy.Publisher('ardrone/land', std_msgs.msg.Empty, queue_size=10)
        estopPub = rospy.Publisher('ardrone/reset', std_msgs.msg.Empty, queue_size=10)
        
        zeroOdomPub = rospy.Publisher('dronex/odom', nav_msgs.msg.Odometry, queue_size=100)
        commandPub = rospy.Publisher('cmd_vel', geometry_msgs.msg.Twist, queue_size=100)    

        r = rospy.Rate(30)
        while not rospy.is_shutdown():
            r.sleep()

    def odom_callback(self, odomMsg):
        print("Stub")
        return

    def navdata_callback(self, navdataMsg):
        print("Stub")
        return

    
if __name__ == '__main__':
    print('Drone Control started...')
    rospy.init_node('drone_control')

    Drone = DroneX()