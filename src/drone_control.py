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
    def __init__(self, kp=0.15, ki=0, kd=0):
        self.limit = 0.2
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.goalX = 0
        self.goalY = 0
        self.goalZ = 0.5
        self.goalRoll = 0
        self.goalPitch = 0
        self.goalYaw = 0
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
        thresh = 0.05
        command = Twist()

        linxErr = self.goalX - self.x
        linyErr = self.goalY - self.y
        linzErr = self.goalZ - self.z
        angzErr = self.goalYaw - self.yaw

        if abs(linxErr) > thresh:
            realX = linxErr*np.cos(self.initYaw) - linyErr*np.sin(self.initYaw)
            command.linear.x = min(self.limit, self.kp * realX)
        else:
            command.linear.x = 0
        if abs(linyErr) > thresh:
            realY = linxErr*np.sin(self.initYaw) + linyErr*np.cos(self.initYaw)
            command.linear.y = min(self.limit, self.kp * linyErr)
        else:
            command.linear.y = 0
        if abs(linzErr) > thresh:
            command.linear.z = min(self.limit, self.kp * linzErr)
            print('Z-Kp Conrol - {}'.format(linzErr))
        else:
            command.linear.z = 0
            print("Z - zero")
        if abs(angzErr) > thresh:
            command.angular.z = min(self.limit, self.kp * angzErr)
        else:
            command.angular.z = 0

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
        self.takeoffFlag = -1
        self.prevAltitude = -1

        self.goalSet = 0

        if controller is not None: 
            self.PID = controller
        else: 
            self.PID = DroneController()

        # Define subscribers
        self.odomSub = rospy.Subscriber("ardrone/odometry", nav_msgs.msg.Odometry, self.odom_callback, queue_size=100)
        self.navdataSub = rospy.Subscriber("/ardrone/navdata", Navdata, self.navdata_callback, queue_size=100)
        self.droneGoalSub = rospy.Subscriber("droneGoal", geometry_msgs.msg.Pose, self.droneGoal_callback, queue_size=1000)
        self.takeoffSub = rospy.Subscriber("/ardrone/takeoff", std_msgs.msg.Empty, self.takeoff_callback, queue_size=1000)
        self.landSub = rospy.Subscriber("/ardrone/land", std_msgs.msg.Empty, self.land_callback, queue_size=1000)

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

        print("Battery state: {}".format(self.battery))

    def odom_callback(self, odomMsg):
        if self.takeoffFlag == 0:   # Drone taking off
            # Reset where we 'zero' the 'world' frame.
            self.initPos = copy.copy(odomMsg.pose.pose.position)
            quat = odomMsg.pose.pose.orientation
            self.initRPY = euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])
            self.PID.initYaw = self.initRPY[2]
            # print("Prev alt: {:.4f}, curr alt: {:.4f}".format(self.prevAltitude))
            if self.prevAltitude > self.initPos.z:
                self.takeoffFlag = 1
            else: 
                self.prevAltitude = self.initPos.z
            
            print('Drone taking off!! Odom Pose:{:.2f},{:.2f},{:.2f},{:.2f},{:.2f},{:.2f}'\
                .format(self.initPos.x,self.initPos.y,self.initPos.z,\
                    self.initRPY[0],self.initRPY[1],self.initRPY[2]))
        
        elif self.takeoffFlag == 1:     # Drone is in flight
            # Subtract current odom by first odom
            posX = odomMsg.pose.pose.position.x- self.initPos.x
            posY = odomMsg.pose.pose.position.y- self.initPos.y
            posZ = odomMsg.pose.pose.position.z- self.initPos.z
            odomMsg.pose.pose.position.x = posX
            odomMsg.pose.pose.position.y = posY
            odomMsg.pose.pose.position.z = posZ
            
            quat = odomMsg.pose.pose.orientation
            roll, pitch, yaw = euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])
            newRoll = roll - self.initRPY[0]
            newPitch = pitch - self.initRPY[1]
            newYaw = yaw - self.initRPY[2]
            x,y,z,w = quaternion_from_euler(roll,pitch,yaw,'rxyz')
            odomMsg.pose.pose.orientation = Quaternion(x,y,z,w)
        
            print('Drone flying!! Zeroed Pose:{:.2f},{:.2f},{:.2f},{:.2f},{:.2f},{:.2f}'\
                .format(posX, posY, posZ, newRoll, newPitch, newYaw))

            # Publish the subtracted odom
            self.pose = odomMsg.pose.pose
            self.pos = odomMsg.pose.pose.position

            self.zeroOdomPub.publish(odomMsg)   

            self.PID.set_pose(odomMsg.pose.pose)
            
        else:   # Waiting for takeoff, just publish status
            pos = copy.copy(odomMsg.pose.pose.position)
            quat = odomMsg.pose.pose.orientation
            rpy = euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])
            print('Waiting for takeoff... Battery: {}, Odom Pose:{:.2f},{:.2f},{:.2f},{:.2f},{:.2f},{:.2f}'\
                .format(self.battery,pos.x,pos.y,pos.z,rpy[0],rpy[1],rpy[2]))
        return

    def navdata_callback(self, navdataMsg):
        # print("     nav")
        # update the battery status
        self.battery = navdataMsg.batteryPercent
        return

    def droneGoal_callback(self, droneGoalMsg):
        print("######################################")
        print(droneGoalMsg.position)
        self.PID.set_goal(droneGoalMsg)
        self.goalSet = 1 
        return

    def takeoff_callback(self, takeoffMsg):
        print('Taking off!')
        # Reset takeoff flag, need to rezero the 'world' frame.
        self.takeoffFlag = 0

        # Wait for 4 seconds to allow drone to takeoff uninterrupted
        rospy.sleep(4.)
        print("Finished sleeping")
        return

    def land_callback(self, landMsg):
        print ("Land")

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
