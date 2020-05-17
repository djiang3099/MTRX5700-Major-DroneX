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
        self.cam_sub = rospy.Subscriber('/ardrone/front/image_raw/compressed', \
            CompressedImage, self.cam_callback, queue_size=100)

        # Thresholds
        self.greenLower = (49, 21, 42)
        self.greenUpper = (103, 150, 106)

        self.output = np.array([360,640,3])

        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            cv2.imshow("Output", self.output)
            r.sleep()
        return

    def cam_callback(self, image_message):
        print("Image received")
        ###### Not working rn, need to install scipy
        # np_arr = np.fromString(image_message.data, np.uint8)
        # image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        # cv2.imshow("image_np", image_np)

        # Convert from ROS image to opencv image
        # cv_image = self.bridge.imgmsg_to_cv2(image_message, desired_encoding="bgr8")
        # image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        
        np_arr = np.fromstring(image_message.data, np.uint8)  
        image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        frame = self.preprocess(image)

        cv2.imshow("Output", image)
        while (cv2.waitKey(1) != 27):
            a = 1

        self.output = self.get_contours(frame)


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
        
        # print("Valid? {} | No. of Contours: {}".format(valid, len(validCont)))
        concat = cv2.vconcat([contourImage,maskline])
        return concat

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

