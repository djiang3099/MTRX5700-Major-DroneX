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
from geometry_msgs.msg import Twist, Vector3, Pose, PoseWithCovariance, Point, Quaternion
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, Image, CompressedImage
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from std_msgs.msg import Int8, Empty
from openpose_ros_msgs.msg import OpenPoseHumanList

from mtrx_major.msg import Navdata
from ardrone_autonomy.srv import CamSelect

from get_gradient import get_grad
from C_CvDroneController import CvDroneController

NECK = 1
RSH = 2
RELB = 3
RRST = 4
LSH = 5
LELB = 6
LRST = 7
MHIP = 8
RHIP = 9
LHIP = 12

class CvDrone:
    def __init__(self, time, controller=None):
        print("Initialised CV Drone")


        self.initTime = time
        self.battery = -1
        self.takeoffFlag = -1
        self.waitReady = 0
        self.prevAltitude = -1

        if controller is not None: 
            self.PID = controller
        else: 
            self.PID = DroneController()

        # Subscribers and Publishers
        self.bridge = CvBridge()
        self.cam_sub = rospy.Subscriber('/ardrone/front/image_raw', \
            Image, self.cam_callback, queue_size=100)
        self.odomSub = rospy.Subscriber("ardrone/odometry", nav_msgs.msg.Odometry, self.odom_callback, queue_size=100)
        self.navdataSub = rospy.Subscriber("/ardrone/navdata", Navdata, self.navdata_callback, queue_size=100)
        self.takeoffSub = rospy.Subscriber("/ardrone/takeoff", std_msgs.msg.Empty, self.takeoff_callback, queue_size=1000)
        self.landSub = rospy.Subscriber("/ardrone/land", std_msgs.msg.Empty, self.land_callback, queue_size=1000)
        self.openposeSub = rospy.Subscriber("/openpose_ros/human_list",OpenPoseHumanList, self.openpose_callback, queue_size=1000)


        self.commandPub = rospy.Publisher('cmd_vel', geometry_msgs.msg.Twist, queue_size=100)    
        self.zeroOdomPub = rospy.Publisher('dronex/odom', nav_msgs.msg.Odometry, queue_size=100)
        self.landingPub = rospy.Publisher('ardrone/land', std_msgs.msg.Empty, queue_size=10)

        # CV Thresholds
        self.greenLower = (49, 21, 42)
        self.greenUpper = (103, 176, 160)

        self.first = True
        self.centreX = None
        self.centreY = None


        self.output = np.array([360,640,3])


        self.box_x1 = 0
        self.box_x2 = 0
        self.box_y1 = 0
        self.box_y2 = 0

        # self.switch_camera(0)

        self.missingBodyParts = True
        self.hovering = False

        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            # cv2.imshow("Output", self.output)
            r.sleep()

        print("Battery state: {}".format(self.battery))
        return

    def odom_callback(self, odomMsg):
        if self.takeoffFlag == 0:   # Drone taking off
            # Reset where we 'zero' the 'world' frame.
            self.initPos = copy.copy(odomMsg.pose.pose.position)
            quat = odomMsg.pose.pose.orientation
            self.initRPY = euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])
            # print("Prev alt: {:.4f}, curr alt: {:.4f}".format(self.prevAltitude))
            if self.prevAltitude > self.initPos.z and self.waitReady == 1:
                self.takeoffFlag = 1
            elif self.prevAltitude < self.initPos.z: 
                self.prevAltitude = self.initPos.z
            
            # print('Drone taking off!! Odom Pose:{:.2f},{:.2f},{:.2f},{:.2f},{:.2f},{:.2f}'\
            #     .format(self.initPos.x,self.initPos.y,self.initPos.z,\
            #         self.initRPY[0],self.initRPY[1],self.initRPY[2]))
        
        elif self.takeoffFlag == 1:     # Drone is in flight
            # Subtract current odom by first odom
            posX = odomMsg.pose.pose.position.x- self.initPos.x
            posY = odomMsg.pose.pose.position.y- self.initPos.y
            posZ = odomMsg.pose.pose.position.z #- self.initPos.z
            odomMsg.pose.pose.position.x = posX
            odomMsg.pose.pose.position.y = posY
            odomMsg.pose.pose.position.z = posZ
            
            quat = odomMsg.pose.pose.orientation
            roll, pitch, yaw = euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])
            newRoll = roll - self.initRPY[0]
            newPitch = pitch - self.initRPY[1]
            newYaw = yaw - self.initRPY[2]
            x,y,z,w = quaternion_from_euler(newRoll,newPitch,newYaw,'rxyz')
            odomMsg.pose.pose.orientation = Quaternion(x,y,z,w)
        
            self.PID.yaw = newYaw

            # print('Drone flying!! Zeroed Pose:{:.2f},{:.2f},{:.2f},{:.2f},{:.2f},{:.2f}'\
            #     .format(posX, posY, posZ, newRoll, newPitch, newYaw))

            # Publish the subtracted odom
            self.pose = odomMsg.pose.pose
            self.pos = odomMsg.pose.pose.position

            self.zeroOdomPub.publish(odomMsg)   

            # Check if the video stream is lagging, hover if so
            if rospy.get_time() - self.lastImgTime > 0.5:   # 2 FPS
                print("---------------------------------- Video stream lagging!")
                self.command = self.PID.hover()

            if self.hovering:
                self.command = self.PID.hover()
            
            self.publishCommand(self.command)
               
            
        else:   # Waiting for takeoff, just publish status
            pos = copy.copy(odomMsg.pose.pose.position)
            quat = odomMsg.pose.pose.orientation
            rpy = euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])
            # print('Waiting for takeoff... Battery: {}, Odom Pose:{:.2f},{:.2f},{:.2f},{:.2f},{:.2f},{:.2f}'\
            #     .format(self.battery,pos.x,pos.y,pos.z,rpy[0],rpy[1],rpy[2]))
            
        return

    def navdata_callback(self, navdataMsg):
        # update the battery status
        self.battery = navdataMsg.batteryPercent
        return

    def cam_callback(self, image_message):
        # print("Image received")        
        # Convert from ROS image to opencv image
        ###### For non compressed images
        image = self.bridge.imgmsg_to_cv2(image_message, desired_encoding="bgr8")
        # image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        
        ###### For compressed images
        # np_arr = np.fromstring(image_message.data, np.uint8)  
        # image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        frame = self.preprocess(image)

        # self.output, targetY, targetZ, targetW, targetH = self.get_contours(frame)
        # self.output, _, _, _, _ = self.get_contours(frame)
        cv2.rectangle(image,(self.box_x1, self.box_y1), (self.box_x2, self.box_y2), (255, 0, 0), 2)
        # print("size", self.box_x2 - self.box_x1, self.box_y2 - self.box_y1)
        # Makr the drone goal and centre of the rectangle
        cv2.circle(image, (self.PID.centreY,self.PID.centreZ), 10, (0,0,255), -1)
        cv2.rectangle(image,( self.PID.centreY - self.PID.refWidth/2 , self.PID.centreZ - self.PID.refHeight/2 ), \
                            ( self.PID.centreY + self.PID.refWidth/2 , self.PID.centreZ + self.PID.refHeight/2 ), (0, 0, 255), 1)

        
        cv2.circle(image, ( (self.box_x1 + self.box_x2)/2  , (self.box_y1 + self.box_y2)/2  ), 3, (0,255,0), 2)

        cv2.imshow("Output", image)

        cv2.waitKey(1)

        if self.first:
            size = frame.shape      
            self.PID.set_centre(320, 180)
            self.PID.set_target_size(130, 160)
            # was 130 height
            print("Frame size: {}, {}".format(size[1], size[0])) # 640 wide, 360 high
            self.first = 0

        # if not self.targetFound:
        #     print("----------------------- Target lost")
        #     self.command = self.PID.hover()
        # else:
        #     time = rospy.get_time() - self.initTime
        #     self.command = self.PID.compute(time, targetY, targetZ, targetW, targetH)

        

        # print("##############missing body parts?", self.missingBodyParts)

        # if self.takeoffFlag == 1:
        #     # command = self.PID.hover()
        #     self.commandDrone(command)

        return

    def takeoff_callback(self, takeoffMsg):
        print('----------------------------------- Taking off!')
        # Reset takeoff flag, need to rezero the 'world' frame.
        self.takeoffFlag = 0

        # Wait for 4 seconds to allow drone to takeoff uninterrupted
        rospy.sleep(8.)
        self.waitReady = 1
        print("----------------------------------- Finished sleeping")
        return

    def land_callback(self, landMsg):
        print ("---------------------------------- Land, Battery: {}, Yaw: {}".format(self.battery, self.PID.yaw))

        return

    def openpose_callback(self, openposeMsg):
        # print ("Openpose msg received! no. of humans in frame: ", openposeMsg.num_humans)
        maxHeight = 0
        self.lastImgTime = rospy.get_time()
        self.missingBodyParts = True
        mainPerson = None       # To store the main person in the frame

        if openposeMsg.human_list:
            for person in openposeMsg.human_list:
                opNeck = person.body_key_points_with_prob[NECK]
                opRSh = person.body_key_points_with_prob[RSH]
                opLSh = person.body_key_points_with_prob[LSH]
                opMHip = person.body_key_points_with_prob[MHIP]
                opRHip = person.body_key_points_with_prob[RHIP]
                opLHip = person.body_key_points_with_prob[LHIP]
                person_x1 = int(min(opRSh.x, opRHip.x))
                person_x2 = int(max(opLSh.x, opLHip.x))
                person_y1 = int(min(opNeck.y, opRSh.y, opLSh.y))
                person_y2 = int(max(opMHip.y, opRHip.y, opLHip.y))
                personHeight = person_y2 - person_y1

                # Check that all the key joints exist
                if  opRSh.x != 0 and opLSh.x != 0 and opRHip != 0 and opRHip != 0 :
                    if personHeight > maxHeight:
                        mainPerson = person
                        maxHeight = personHeight
                        self.box_x1 = person_x1
                        self.box_x2 = person_x2
                        self.box_y1 = person_y1
                        self.box_y2 = person_y2
                        self.missingBodyParts = False

        # Control the drone towards the desired position 
        if self.missingBodyParts:
            print("MISSING BODY PARTS")
            self.command = self.PID.hover()
        elif abs( self.box_x2 - self.box_x1 ) > abs( self.box_y2 - self.box_y1 ):
            print("Aspect Ratio")
            self.command = self.PID.hover()
        else:
            targetY = self.box_x1
            targetZ = self.box_y1
            targetW = self.box_x2 - self.box_x1
            targetH = self.box_y2 - self.box_y1
            time = rospy.get_time() - self.initTime
            self.command = self.PID.compute(time, targetY, targetZ, targetW, targetH)

        #     # Left hand points
        #     lSh = person.body_key_points_with_prob[LSH]
        #     lSh = Point(lSh.x, lSh.y, 0)
        #     lElb = person.body_key_points_with_prob[LELB]
        #     lElb = Point(lElb.x, lElb.y, 0)
        #     lRst = person.body_key_points_with_prob[LRST]
        #     lRst = Point(lRst.x, lRst.y, 0)
        #     leftOut = 0
        #     leftLand = 0
            
        #     # Right hand points
        #     rSh = person.body_key_points_with_prob[RSH]
        #     rSh = Point(rSh.x, rSh.y, 0)
        #     rElb = person.body_key_points_with_prob[RELB]
        #     rElb = Point(rElb.x, rElb.y, 0)
        #     rRst = person.body_key_points_with_prob[RRST]
        #     rRst = Point(rRst.x, rRst.y, 0)
        #     rightOut = 0
        #     rightLand = 0

        #     rlShiftThr = targetH/4
        #     unhoverThr = targetH/8
        #     unhoverUDThr = targetH/10.6
        #     unhoverLRThr = targetH/7
        #     landUDThr = targetH/2

        #     # Check for any gestures
        #     # Left hand outstretched? Segments in line and not stacked
        #     if lSh.x and lElb.x and lRst.x != 0:
        #         if abs( get_grad(lSh, lElb) - get_grad(lSh, lRst) ) < 0.3 and \
        #             abs(get_grad(lSh, lRst)) < 0.6:
        #             if lSh.x < lElb.x < lRst.x and (lRst.x - lSh.x) > rlShiftThr:
        #                 leftOut = 1
        #                 print("shift left")
        #         if (abs(lSh.x - lRst.x) < unhoverThr)  and ( lSh.y - lRst.y > unhoverUDThr ) and \
        #             (lElb.x - lSh.x > unhoverLRThr) and (lElb.x - lRst.x > unhoverLRThr):
        #             leftOut = 2
        #             print("Un-hover left")

        #         if (abs(lSh.x - lRst.x) < unhoverThr)  and ( lRst.y - lSh.y > landUDThr ) and \
        #             (lElb.x - lSh.x > unhoverLRThr) and (lElb.x - lRst.x > unhoverLRThr):
        #             leftLand = 1

        #     # Right hand outstretched?   
        #     if rSh.x and rElb.x and rRst.x != 0:
        #         # print(abs( get_grad(rSh, rElb) - get_grad(rSh, rRst) ), "< 0.3??  ",  abs(get_grad(rSh, rRst)), "< 0.6?")
        #         if abs( get_grad(rSh, rElb) - get_grad(rSh, rRst) ) < 0.3 and \
        #             abs( get_grad(rSh, rRst) ) < 0.6:
        #             # print (rSh.x, rElb.x, rRst.x, "< < <     ", (rRst.x - rSh.x), "<", rlShiftThr)
        #             if rSh.x > rElb.x > rRst.x and (rSh.x - rRst.x) > rlShiftThr:
        #                 rightOut = 1
        #                 print("shift right")
            
        #         if (abs(rSh.x - rRst.x) < unhoverThr) and ( rSh.y - rRst.y > unhoverUDThr ) and \
        #             (rSh.x - rElb.x > unhoverLRThr) and (rRst.x - rElb.x > unhoverLRThr):
        #             rightOut = 2
        #             print("Un-hover right")

        #         if (abs(rSh.x - rRst.x) < unhoverThr) and ( rRst.y - rSh.y > landUDThr ) and \
        #             (rSh.x - rElb.x > unhoverLRThr) and (rRst.x - rElb.x > unhoverLRThr):
        #             rightLand = 1

        #     # Both hands outstreched
        #     if rightOut == 1 and leftOut == 1:
        #         print("Hover Gesture!!")
        #         self.command = self.PID.hover()
        #         self.publishCommand(self.command)
        #         self.hovering = True;
        #     elif rightOut == 2 and leftOut == 2:
        #         print("Unhover Gesture!!")
        #         self.hovering = False;
        #     elif rightOut == 1 and not self.hovering:
        #         print("Right shift Gesture!!")
        #         y, z = self.PID.get_centre()
        #         self.PID.set_centre(y-5, z)
        #     elif leftOut == 1 and not self.hovering:
        #         print("Left shift Gesture!!")
        #         y, z = self.PID.get_centre()
        #         self.PID.set_centre(y+5, z)
            
        #     # Landing
        #     if leftLand == 1 and rightLand == 1:
        #         print("LAND")
        #         # self.goalController.compute_goal()
        #         self.landingPub.publish(Empty())
        return


    def switch_camera(self, num):    
        rospy.wait_for_service('CamSelect')
        try:
            if num == 1:
                print("------------------------------------ Switching to BACK camera!")
            elif num == 0:
                print("------------------------------------ Switching to FRONT camera!")
            CamSelect = rospy.ServiceProxy('CamSelect', CamSelect)
            resp1 = CamSelect(num)
            return resp1.result
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)
        return

    def preprocess(self, image):
        frame = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        kernel = np.ones((5,5), np.uint8)   # For Erosion/dilation
        frame = cv2.erode(frame, kernel, dst=frame, iterations = 1)
        frame = cv2.dilate(frame, kernel, dst=frame, iterations = 1)
        #frame = cv2.blur(frame,(10,10))
        return frame

    def get_contours(self, frame):
        # Mask
        maskline = cv2.inRange(frame, self.greenLower, self.greenUpper)
        contourImage,contours,_ = cv2.findContours(maskline, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        # Find largest contour
        maxContour = None
        maxArea = 0
        for contour in contours:
            if (cv2.contourArea(contour) > maxArea):
                maxContour = contour
                maxArea = cv2.contourArea(contour)

        # print("Max area", maxArea)

        if maxArea < 1000:
            self.targetFound = False
        else: 
            self.targetFound = True

        contourImage = cv2.drawContours(frame, maxContour, -1, (255,0,0), 2)
        maskline = cv2.cvtColor(maskline, cv2.COLOR_GRAY2RGB, dst=maskline)
        contourImage = cv2.cvtColor(contourImage, cv2.COLOR_HSV2BGR, dst=contourImage)  

        # Draw a big bounding rect around all valid contours
        # bigCont = np.concatenate(validCont)
        x,y,w,h = cv2.boundingRect(maxContour)
        cv2.rectangle(contourImage, (x,y), (x+w-1, y+h-1), (0,255,0), 2)

        # Check if 'landmark' (tape) is in the region of interest. 
        dim = frame.shape
        if abs(y+h/2 - dim[0]/2) < 30:
            valid = True
        else:
            valid = False
        targetY = x + w / 2
        targetZ = y + h / 2
        
        # print("Valid? {} | No. of Contours: {}".format(valid, len(validCont)))
        concat = cv2.vconcat([contourImage,maskline])
        return concat, targetY, targetZ, w, h

    def publishCommand(self, command):
        # print("---------------------------------------------- publish")
        self.commandPub.publish(command)
        return