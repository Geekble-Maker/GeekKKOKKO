#!/usr/bin/env python

import rospy
import cv2
import numpy as np
import math

from cv_bridge import CvBridge, CvBridgeError

from sensor_msgs.msg import Image

BLUE_COLOR = (255,0,0)
RED_COLOR = (0,0,255)
ORANGE_COLOR = (255,0,255)
GREEN_COLOR = (0,255,0)
YELLOW_COLOR = (0,255,255)
WHITE_COLOR = (255,255,255)

bridge = CvBridge()

def nothing(x):
    pass

# ------------------CALLBACK FUNCTION----------------------

def RawCallback(image_msg): # Main Call Back
    global rawImage
    rawImage = bridge.imgmsg_to_cv2(image_msg, "bgr8")
    
    lower_blue = (120-10, 30, 30)
    upper_blue = (120+10, 255, 255)
    rangeImage = cv2.inRange(img_hsv, lower_blue, upper_blue)

    cv2.imshow('rangeImage', rangeImage)
    cv2.waitKey(1)


# --------------------MAIN FUNCTION------------------------

def initialize():
    print("Range Detect")

    cv2.namedWindow('rangeImage')

    cv2.createTrackbar('LOWER_H', 'rangeImage', 0, 255, nothing)
    cv2.createTrackbar('UPPER_H', 'rangeImage', 0, 255, nothing)

    cv2.createTrackbar('LOWER_S', 'rangeImage', 0, 255, nothing)
    cv2.createTrackbar('UPPER_S', 'rangeImage', 0, 255, nothing)

    cv2.createTrackbar('LOWER_V', 'rangeImage', 0, 255, nothing)
    cv2.createTrackbar('UPPER_V', 'rangeImage', 0, 255, nothing)

    cv2.setTrackbarPos('LOWER_H', 'rangeImage', 0)
    cv2.setTrackbarPos('LOWER_S', 'rangeImage', 0)
    cv2.setTrackbarPos('LOWER_V', 'rangeImage', 0)

    cv2.setTrackbarPos('UPPER_H', 'rangeImage', 255)
    cv2.setTrackbarPos('UPPER_S', 'rangeImage', 255)
    cv2.setTrackbarPos('UPPER_V', 'rangeImage', 255)

    while not rospy.is_shutdown():
        image_msg = rospy.wait_for_message('/d435/color/image_raw', Image)

        rawImage = bridge.imgmsg_to_cv2(image_msg, "bgr8")

        img_hsv = cv2.cvtColor(rawImage, cv2.COLOR_BGR2HSV)

        low = cv2.getTrackbarPos('low threshold', 'rangeImage')
        high = cv2.getTrackbarPos('high threshold', 'rangeImage')

        LOWER_H = cv2.getTrackbarPos('LOWER_H', 'rangeImage')
        LOWER_S = cv2.getTrackbarPos('LOWER_S', 'rangeImage')
        LOWER_V = cv2.getTrackbarPos('LOWER_V', 'rangeImage')

        UPPER_H = cv2.getTrackbarPos('UPPER_H', 'rangeImage')
        UPPER_S = cv2.getTrackbarPos('UPPER_S', 'rangeImage')
        UPPER_V = cv2.getTrackbarPos('UPPER_V', 'rangeImage')
        LOWER = (LOWER_H, LOWER_S, LOWER_V)
        UPPER = (UPPER_H, UPPER_S, UPPER_V)
    
        maskImage = cv2.inRange(img_hsv, LOWER, UPPER)

        kernel = np.ones((2, 2), np.uint8)
        maskImage = cv2.erode(maskImage, kernel, iterations = 2)
        maskImage = cv2.dilate(maskImage, kernel, iterations = 2)

        rangeImage = cv2.bitwise_and(rawImage,rawImage, mask= maskImage)

        cv2.imshow("rangeImage", rangeImage)

        cv2.waitKey(1)
# ---------------------------------------------------------------
rospy.init_node('range_node')

initialize()



