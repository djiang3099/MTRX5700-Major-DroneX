#!/usr/bin/env python

# MTRX5700 Major Project DroneX 
# 470355499 470355503

import cv2
import numpy as np
import argparse
import rospy
from cv_bridge import CvBridge
import sensor_msgs
from std_msgs.msg import Int8
from sensor_msgs.msg import(
Image,
CompressedImage,
)

class CvDrone:
    def __init__(self):
        print("Initialised CV Drone")

        self.bridge = CvBridge()
        self.cam_sub = rospy.Subscriber('/ardrone/front/image_raw', \
            Image, self.cam_callback, queue_size=100)

        # Thresholds
        self.greenLower = (49, 21, 42)
        self.greenUpper = (103, 150, 106)

        self.first = True
        self.centreX = None
        self.centreY = None


        self.output = np.array([360,640,3])

        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            # cv2.imshow("Output", self.output)
            r.sleep()
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
            return


        if self.first:
            size = frame.shape
            self.centreZ = size[0]/2
            self.centreY = size[1]/2
            self.refHeight = size[0]/5
            self.refWidth = size[1]/10

        self.commandDrone(targetY, targetZ, targetW, targetH)

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

        velY = offsetY * 0.01
        velZ = offsetZ * 0.01

        if ( (abs(w-self.refWidth) > 10 ) and   (abs (h-self.refHeight) > 10 ) ):
            velX = (self.refHeight - h) * 0.01
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

        return

def main():

    print("starting....")
    
    rospy.init_node("dronex_camera_test")

    
    drone = CvDrone()
    

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Keyboard Interrupt")
    cv2.destroyAllWindows()    

	

    # Wait for escape key
    #while (cv2.waitKey(1) != 27):


if __name__ == '__main__':
    main()

