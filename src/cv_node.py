#!/usr/bin/env python

# MTRX5700 Major Project DroneX 
# 470355499 470355503

import cv2
import numpy as np
import argparse
import rospy
import copy

from C_CvDroneController import CvDroneController
from C_CvDrone import CvDrone

def main():

    print("----------------------- Starting CV_Node....")
    
    rospy.init_node("dronex_camera_test")

    # Track the time the controller is called at and initial time
    initTime = rospy.get_time()
    while rospy.get_time() == 0:    # For simulated time
        print( rospy.get_time())
        initTime = rospy.get_time()

    DroneController = CvDroneController(initTime)
    drone = CvDrone(initTime, DroneController)
    

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Keyboard Interrupt")

    # Wait for escape key
    #while (cv2.waitKey(1) != 27):


if __name__ == '__main__':
    main()

