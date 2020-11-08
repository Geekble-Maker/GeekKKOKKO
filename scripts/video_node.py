#!/usr/bin/env python

import rospy
import cv2

from cv_bridge import CvBridge, CvBridgeError

from sensor_msgs.msg import Image
from sensor_msgs.msg import CompressedImage

rospy.init_node('video_node')
imagePub = rospy.Publisher('/d435/color/image_raw', Image, queue_size=100)

bridge = CvBridge()
videoFilePath = '../Files/test002.mp4'

cap = 0
while(True):
    cap = cv2.VideoCapture(videoFilePath)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
    while(cap.isOpened()):
        ret, frame = cap.read()
        if ret:
            image_message = bridge.cv2_to_imgmsg(frame, "bgr8")
            imagePub.publish(image_message)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
        else:
            break

cap.release()
cv2.destroyAllWindows()

