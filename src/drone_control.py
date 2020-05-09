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
        self.initPos = None
        self.initOrient = None

        # Define subscribers
        odomSub = rospy.Subscriber("ardrone/odometry", nav_msgs.msg.Odometry, self.odom_callback, queue_size=100)
        navdataSub = rospy.Subscriber("/ardrone/navdata", Navdata, self.navdata_callback, queue_size=100)

        # Define publishers
        self.takeoffPub = rospy.Publisher('/ardrone/takeoff', std_msgs.msg.Empty, queue_size=10)
        self.landingPub = rospy.Publisher('ardrone/land', std_msgs.msg.Empty, queue_size=10)
        self.estopPub = rospy.Publisher('ardrone/reset', std_msgs.msg.Empty, queue_size=10)
        
        self.zeroOdomPub = rospy.Publisher('dronex/odom', nav_msgs.msg.Odometry, queue_size=100)
        self.commandPub = rospy.Publisher('cmd_vel', geometry_msgs.msg.Twist, queue_size=100)    

        r = rospy.Rate(30)
        while not rospy.is_shutdown():
            r.sleep()

    def odom_callback(self, odomMsg):
        print("odom")
        # Store the first odom
        if (self.pos is None):
            print("FIRST ODOM######################")
            self.initPos = copy.copy(odomMsg.pose.pose.position)
            self.initOrient = copy.copy(odomMsg.pose.pose.orientation)
            
            print(odomMsg)
        else:
            print("here")
        print(self.initPos.x)
        # Subtract current odom by first odom
        odomMsg.pose.pose.position.x = odomMsg.pose.pose.position.x- self.initPos.x
        odomMsg.pose.pose.position.y = odomMsg.pose.pose.position.y- self.initPos.y
        odomMsg.pose.pose.position.z = odomMsg.pose.pose.position.z- self.initPos.z
        
        odomMsg.pose.pose.orientation.x = odomMsg.pose.pose.orientation.x- self.initOrient.x
        odomMsg.pose.pose.orientation.y = odomMsg.pose.pose.orientation.y- self.initOrient.y
        odomMsg.pose.pose.orientation.z = odomMsg.pose.pose.orientation.z- self.initOrient.z
        
        # zeroOdomMsg = geometry_msg.msg.poseStamped()
        # zeroOdomMsg.header.frame_id = "odom"
        # zeroOdomMsg.pose.position = msg.pose.pose.posiion
        # zeroOdomMsg.pose.orientation = msg.pose.pose.orientation

        # Publish the subtracted odom
        self.pose = odomMsg.pose.pose
        self.pos = odomMsg.pose.pose.position

        self.zeroOdomPub.publish(odomMsg)        
        
        return

    def navdata_callback(self, navdataMsg):
        print("     nav")
        # update the battery status
        self.battery = navdataMsg.batteryPercent

        return

    
if __name__ == '__main__':
    print('Drone Control started...')
    rospy.init_node('drone_control')

    Drone = DroneX()
    