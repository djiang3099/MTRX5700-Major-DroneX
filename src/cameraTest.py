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

    commandDrone(frame)

    return frame

# Extract block coordinates from the image
def find_target(image):
    """
    This function extracts bounding boxes of the blocks from the input image
    :param image: OpenCV Image in rgb8 format
    :type image: OpenCV Image aka numpy array
    :return: List of bounding boxes of the blocks in the image plane
    """
    image_hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    # image_cropped = image_hsv[ 0:640, 0:580 ]
    greenLower = (49, 21, 42)
    greenUpper = (103, 150, 106)

    mask = cv2.inRange(image_cropped, greenLower, greenUpper)
    contourImage, contours, hierarchy = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    contourImage = cv2.drawContours(image, contours, -1, (0, 255,0), 2)

    xList = []
    yList = []
    wList = []
    hList = []


    for contour in contours:
	# x, y is the top-left corner
        x, y, w, h = cv2.boundingRect(contour)
        xList.append(x + w/2)
        yList.append(y + h/2)
        wList.append(w)
        hList.append(h)
        #contourImage = cv2.circle(contourImage, (x, y), 5, (255, 0, 0), 2)
        #contourImage = cv2.circle(contourImage, (x, y+h), 5, (0, 0, 255), 2)
    cv2.imshow("original image",image)
    cv2.waitKey(2000)
    cv2.imshow("original image",image_cropped)
    cv2.waitKey(2000)
    cv2.imshow("original image",whiteMasked)
    cv2.waitKey(2000)
    cv2.imshow("original image",contourImage)
    cv2.waitKey(2000)
    #print(contours)

    return xList, yList, wList, hList


def commandDrone(frame):
    print(frame.shape)
    print(frame.shape[0])
    print(frame.shape[1])
    print(frame.shape[2])



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
