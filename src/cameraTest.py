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



def cam_callback(image_message):
    bridge = CvBridge()
    ###### Not working rn, need to install scipy
    # np_arr = np.fromString(image_message.data, np.uint8)
    # image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
    # cv2.imshow("image_np", image_np)

    # Convert from ROS image to opencv image
    cv_image = bridge.imgmsg_to_cv2(image_message, desired_encoding="bgr8")
    frame = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
    # cv2.imshow("cv_image", cv_image)

    cv2.imshow("frame", frame)
    cv2.waitKey(20)

    return frame


def main():

    print("starting....")
    
    rospy.init_node("dronex_camera_test")

    cam_sub = rospy.Subscriber('/ardrone/front/image_raw', Image, cam_callback, queue_size=100)
    
    r = rospy.Rate(10)
    while not rospy.is_shutdown():
        r.sleep()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Keyboard Interrupt")
    cv2.destroyAllWindows()    

	

    # Wait for escape key
    #while (cv2.waitKey(1) != 27):


if __name__ == '__main__':
    main()
