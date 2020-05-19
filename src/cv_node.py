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
    def __init__(self, time, kp=0.0005, ki=0, kd=0.0005):
        print("Initialising drone controller...")
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
        self.intSat = 3

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

    def set_target_size(self, h, w):
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
        self.lastTime = time

        # Compute Proportional error
        linXErr = 0.0
        linYErr = self.centreY - y
        linZErr = self.centreZ - z
        # angZErr = self.goalYaw - self.yaw

        # Compute Derivative error
        # d_linXErr = (realX - self.prevErrorX)/dt
        d_linYErr = (linYErr - self.prevErrorY)/dt
        d_linZErr = (linZErr - self.prevErrorZ)/dt
        # d_angZErr = (angZErr - self.prevErrorYaw)/dt

        # Update previous error
        # self.prevErrorX = linXErr
        self.prevErrorY = linYErr
        self.prevErrorZ = linZErr
        # self.prevErrorYaw = angZErr

        # Compute Integral error with saturation
        # self.i_linX = np.sign(self.i_linX + realX*dt) * min(self.intSat, \
        #     abs(self.i_linX + realX*dt))
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
            # controlX = (self.kp * linXErr) + (self.kd * d_linXErr) + (self.ki * self.i_linX)
            # command.linear.x = np.sign(controlX) * min(self.ctrlLimit, abs(controlX))

            controlY = (self.kp_zy * linYErr) + (self.kd_zy * d_linYErr) + (self.ki_zy * self.i_linY)
            command.linear.y = np.sign(controlY)* min(self.ctrlLimit, abs(controlY))
            
            controlZ = (self.kp_zy * linZErr) + (self.kd_zy * d_linZErr) + (self.ki_zy * self.i_linZ)
            command.linear.z = np.sign(controlZ) * min(self.ctrlLimit, abs(controlZ))
            
            # controlYaw = (self.kp * angZErr) + (self.kd * d_angZErr) + (self.ki * self.i_angZ)
            # command.angular.z = np.sign(controlYaw) * min(self.ctrlLimit, abs(controlYaw))

            command.angular.x = 0.0
            command.angular.y = 0.0
            command.linear.x = 0.0
            command.angular.z = 0.0
            print("PD Y:   P{}, D{}".format((self.kp_zy * linYErr), (self.kd_zy * d_linYErr)))
            print("PD Z:   P{}, D{}".format((self.kp_zy * linZErr), (self.kd_zy * d_linZErr)))
            

        print("Error:   {}, {}, {}".format(linXErr, linYErr, linZErr))
        print("Command: {:.2}, {:.2}, {:.2}, {:.2}, {:.2}, {:.2}".format(command.linear.x,\
            command.linear.y, command.linear.z, command.angular.x ,command.angular.y,\
                command.angular.z))
        return command

class CvDroneController():
    def __init__(self, time, kp=0.0005, ki=0, kd=0.05):
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
        self.intSat = 3

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

    def set_target_size(self, h, w):
        self.refHeight = h
        self.refWidth = w
        return

    def compute(self, time, y, z, w, h):
        command = Twist()
        dt = time - self.lastTime
        self.lastTime = time

        # Compute Proportional error
        linYErr = self.centreY - y
        linZErr = self.centreZ - z
        # angZErr = self.goalYaw - self.yaw

        # Compute Derivative error
        # d_linXErr = (realX - self.prevErrorX)/dt
        d_linYErr = (linYErr - self.prevErrorY)/dt
        d_linZErr = (linZErr - self.prevErrorZ)/dt
        # d_angZErr = (angZErr - self.prevErrorYaw)/dt

        # Update previous error
        # self.prevErrorX = linXErr
        self.prevErrorY = linYErr
        self.prevErrorZ = linZErr
        # self.prevErrorYaw = angZErr

        # Compute Integral error with saturation
        # self.i_linX = np.sign(self.i_linX + realX*dt) * min(self.intSat, \
        #     abs(self.i_linX + realX*dt))
        self.i_linY = np.sign(self.i_linY + linYErr*dt) * min(self.intSat, \
            abs(self.i_linY + linYErr*dt))
        self.i_linZ = np.sign(self.i_linZ + linZErr*dt) * min(self.intSat, \
            abs(self.i_linZ + linZErr*dt))
        # self.i_angZ = np.sign(self.i_angZ + angZErr*dt) * min(self.intSat, \
        #     abs(self.i_angZ + angZErr*dt))

        # If very close to the goal, hover
        if abs(linXErr) < self.errThresh and abs(linYErr) < self.errThresh and \
            abs(linZErr) < self.errThresh and abs(angZErr) < self.errThresh:
            command.linear.x = 0
            command.linear.y = 0
            command.linear.z = 0
            command.angular.x = 0
            command.angular.y = 0
            command.angular.z = 0

        else:
            # controlX = (self.kp * linXErr) + (self.kd * d_linXErr) + (self.ki * self.i_linX)
            # command.linear.x = np.sign(controlX) * min(self.ctrlLimit, abs(controlX))

            controlY = (self.kp * linYErr) + (self.kd * d_linYErr) + (self.ki * self.i_linY)
            command.linear.y = np.sign(controlY)* min(self.ctrlLimit, abs(controlY))
            
            controlZ = (self.kp * linZErr) + (self.kd * d_linZErr) + (self.ki * self.i_linZ)
            command.linear.z = np.sign(controlZ) * min(self.ctrlLimit, abs(controlZ))
            
            # controlYaw = (self.kp * angZErr) + (self.kd * d_angZErr) + (self.ki * self.i_angZ)
            # command.angular.z = np.sign(controlYaw) * min(self.ctrlLimit, abs(controlYaw))

            command.angular.x = 0
            command.angular.y = 0

        print(command)
        return command

class CvDrone:
    def __init__(self, time, controller=None):
        print("Initialised CV Drone")

<<<<<<< HEAD
        self.initTime = time
=======
>>>>>>> 0a2002c82ba25791481a0301e4fde9acd44bfdd1
        self.battery = -1
        self.takeoffFlag = -1
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


        self.commandPub = rospy.Publisher('cmd_vel', geometry_msgs.msg.Twist, queue_size=100)    
        self.zeroOdomPub = rospy.Publisher('dronex/odom', nav_msgs.msg.Odometry, queue_size=100)
        

        # CV Thresholds
        self.greenLower = (49, 21, 42)
        self.greenUpper = (103, 176, 160)

        self.first = True
        self.centreX = None
        self.centreY = None


        self.output = np.array([360,640,3])

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
            if self.prevAltitude > self.initPos.z:
                self.takeoffFlag = 1
            else: 
                self.prevAltitude = self.initPos.z
            
<<<<<<< HEAD
            # print('Drone taking off!! Odom Pose:{:.2f},{:.2f},{:.2f},{:.2f},{:.2f},{:.2f}'\
            #     .format(self.initPos.x,self.initPos.y,self.initPos.z,\
            #         self.initRPY[0],self.initRPY[1],self.initRPY[2]))
=======
            print('Drone taking off!! Odom Pose:{:.2f},{:.2f},{:.2f},{:.2f},{:.2f},{:.2f}'\
                .format(self.initPos.x,self.initPos.y,self.initPos.z,\
                    self.initRPY[0],self.initRPY[1],self.initRPY[2]))
>>>>>>> 0a2002c82ba25791481a0301e4fde9acd44bfdd1
        
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
        
<<<<<<< HEAD
            # print('Drone flying!! Zeroed Pose:{:.2f},{:.2f},{:.2f},{:.2f},{:.2f},{:.2f}'\
            #     .format(posX, posY, posZ, newRoll, newPitch, newYaw))
=======
            print('Drone flying!! Zeroed Pose:{:.2f},{:.2f},{:.2f},{:.2f},{:.2f},{:.2f}'\
                .format(posX, posY, posZ, newRoll, newPitch, newYaw))
>>>>>>> 0a2002c82ba25791481a0301e4fde9acd44bfdd1

            # Publish the subtracted odom
            self.pose = odomMsg.pose.pose
            self.pos = odomMsg.pose.pose.position

            self.zeroOdomPub.publish(odomMsg)   
            
        else:   # Waiting for takeoff, just publish status
            pos = copy.copy(odomMsg.pose.pose.position)
            quat = odomMsg.pose.pose.orientation
            rpy = euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])
<<<<<<< HEAD
            # print('Waiting for takeoff... Battery: {}, Odom Pose:{:.2f},{:.2f},{:.2f},{:.2f},{:.2f},{:.2f}'\
            #     .format(self.battery,pos.x,pos.y,pos.z,rpy[0],rpy[1],rpy[2]))
=======
            print('Waiting for takeoff... Battery: {}, Odom Pose:{:.2f},{:.2f},{:.2f},{:.2f},{:.2f},{:.2f}'\
                .format(self.battery,pos.x,pos.y,pos.z,rpy[0],rpy[1],rpy[2]))
>>>>>>> 0a2002c82ba25791481a0301e4fde9acd44bfdd1
                
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

        self.output, targetY, targetZ, targetW, targetH = self.get_contours(frame)

        cv2.imshow("Output", self.output)
        cv2.waitKey(1)

        if not self.targetFound:
            print("----------------------- Target lost")
            return


        if self.first:
            size = frame.shape
            self.PID.set_centre(size[1]/2, size[0]/2)
            self.PID.set_target_size(size[1]/10, size[0]/5)

<<<<<<< HEAD
        time = rospy.get_time() - self.initTime
        self.PID.compute(time, targetY, targetZ, targetW, targetH)
=======
        self.PID.compute(targetY, targetZ, targetW, targetH)
>>>>>>> 0a2002c82ba25791481a0301e4fde9acd44bfdd1

        return

    def takeoff_callback(self, takeoffMsg):
<<<<<<< HEAD
        print('----------------------- Taking off!')
=======
        print('Taking off!')
>>>>>>> 0a2002c82ba25791481a0301e4fde9acd44bfdd1
        # Reset takeoff flag, need to rezero the 'world' frame.
        self.takeoffFlag = 0

        # Wait for 4 seconds to allow drone to takeoff uninterrupted
        rospy.sleep(4.)
<<<<<<< HEAD
        print("----------------------- Finished sleeping")
        return

    def land_callback(self, landMsg):
        print ("----------------------- Land, Battery: {}", self.battery)
=======
        print("Finished sleeping")
        return

    def land_callback(self, landMsg):
        print ("Land, Battery: {}", self.battery)
>>>>>>> 0a2002c82ba25791481a0301e4fde9acd44bfdd1

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

    def commandDrone(self, targetY, targetZ, w, h):
        # Calculate offset from the target
        offsetY = self.centreY - targetY
        offsetZ = self.centreZ - targetZ

        velY = offsetY * 0.0005
        velZ = offsetZ * 0.0005

        if ( (abs(w-self.refWidth) > 10 ) and   (abs (h-self.refHeight) > 10 ) ):
            velX = (self.refHeight - h) * 0.005
        else:
            velX = 0.0

        # Print statements, for debugging
        if velY > 0:
            a = "Left "
        else:
            a = "Right"
        if velZ > 0:
            b = "Up  "
        else:
            b = "Down"
        if velX > 0:
            c = "Forward"
        else:
            c = "Back   "

        print("{}, {:.2}, {}, {:.2}, {}, {:.2}".format(a, velY, b, velZ, c, velX))


        command = Twist()
        command.linear.x = np.sign(velX) * min(self.ctrlLimit, abs(velX))
        command.linear.y = np.sign(velY) * min(self.ctrlLimit, abs(velY))
        command.linear.z = np.sign(velZ) * min(self.ctrlLimit, abs(velZ))
        command.angular.x = 0
        command.angular.y = 0
        command.angular.z = 0
        print(command)

        # self.commandPub.publish(command)




        return

def main():

    print("----------------------- Starting CV_Node....")
    
    rospy.init_node("dronex_camera_test")

    # Track the time the controller is called at and initial time
    initTime = rospy.get_time()
    while rospy.get_time() == 0:    # For simulated time
        print( rospy.get_time())
        initTime = rospy.get_time()

<<<<<<< HEAD
    DroneController = CvDroneController(initTime)
=======
    DroneController = DroneController(initTime)
>>>>>>> 0a2002c82ba25791481a0301e4fde9acd44bfdd1
    drone = CvDrone(initTime, DroneController)
    

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Keyboard Interrupt")
    cv2.destroyAllWindows()    

	

    # Wait for escape key
    #while (cv2.waitKey(1) != 27):


if __name__ == '__main__':
    main()

