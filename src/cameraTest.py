#!/usr/bin/env python

# MTRX5700 Major Project DroneX 
# 470355499 470355503

import cv2
import numpy as np
import argparse


def main():

    image_message = rospy.wait_for_message("/ardrone/front/image_raw", sensor_msgs.msg.Image, timeout=5)
    # Convert from ROS image to opencv image
    bridge = CvBridge()
    cv_image = bridge.imgmsg_to_cv2(image_message, desired_encoding="bgr8")

    frame = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

    cv2.imshow("frame", frame)
    cv2.waitKey(20)
	

    # Wait for escape key
    #while (cv2.waitKey(1) != 27):


if __name__ == '__main__':
    main()
