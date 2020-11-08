#!/usr/bin/env python

import rospy
import cv2
import numpy as np
import math

from pydub import AudioSegment
from pydub.playback import play
from threading import Thread

from cv_bridge import CvBridge, CvBridgeError

from std_msgs.msg import Int32
from sensor_msgs.msg import Image
from sensor_msgs.msg import CompressedImage
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

BLUE_COLOR = (255,0,0)
RED_COLOR = (0,0,255)
ORANGE_COLOR = (255,0,255)
GREEN_COLOR = (0,255,0)
YELLOW_COLOR = (0,255,255)
WHITE_COLOR = (255,255,255)

odomLength = 200
goalLength = 150
scanLength = 100

isMove = False
nowState = 0
isSongEnd = True

linearVelocity = 0
angularVelocity = 0

bridge = CvBridge()

def threaded_function(arg):
    global isSongEnd
    song = AudioSegment.from_mp3("../Files/test.mp3")
    play(song)
    isSongEnd = True

# ------------------CALLBACK FUNCTION----------------------
def LaserCallback(scan_msg):
    # rospy.loginfo("LaserCallback")
    global laserImage
    global odomRad
    global isLidarModeOn
    global isMove
    global isSongEnd

    # ----- Image Setting -----
    laserImage = np.zeros((500,500,3), np.uint8)
    c_x = 250
    c_y = 250

    angle=0
    avoidRadArray = []
    for scanData in scan_msg.ranges:
        COLOR = WHITE_COLOR
        scanRad = angle * math.pi / 360
        scanData *= scanLength

        if scanRad > 140.0*math.pi/180.0 and scanRad <220.0*math.pi/180.0: # front standard -pi/2 ~ pi/2
            COLOR = BLUE_COLOR
            if scanData < 100 and scanData !=0:
                COLOR = RED_COLOR
                avoidRadArray.append(scanRad)

        d_x = scanData*math.cos(scanRad)
        d_y = scanData*math.sin(scanRad)
        
        if scanRad == 0:
            laserImage = cv2.line(laserImage, (c_x,c_y),(c_x+100,c_y),YELLOW_COLOR,2)
        else:
            laserImage = cv2.line(laserImage, (c_x,c_y),(c_x+int(d_x),c_y+int(d_y)),COLOR,1)
        angle += 2

    if len(avoidRadArray)>15: # Failsafe Emergency Stop
        rospy.loginfo("!!!!!Stop: "+str(len(avoidRadArray))+"!!!!!")
        isMove = False
        
        if isSongEnd:
            isSongEnd = False
            thread1 = Thread(target = threaded_function, args = (10, ))
            thread1.start()

    else:
        rospy.loginfo("Len: "+str(len(avoidRadArray)))
        isMove = True
        
def OdomCallback(odom_msg):
    global odomRad
    # rospy.loginfo("OdomCallback")
    o_x = odom_msg.pose.pose.orientation.x
    o_y = odom_msg.pose.pose.orientation.y
    o_z = odom_msg.pose.pose.orientation.z
    o_w = odom_msg.pose.pose.orientation.w

    odomRad = -math.atan2(2*(o_y*o_x+o_w*o_z),o_w*o_w+o_x*o_x-o_y*o_y-o_z*o_z)
    # rospy.loginfo(math.degrees(odomRad))
    
def RawCallback(image_msg): # Main Call Back
    global rawImage
    rawImage = bridge.imgmsg_to_cv2(image_msg, "bgr8")

    LineTracing()
    RobotMovePublish()
    ImageShow()

prevTime = 0


totalFinalX = 0 
def LineTracing():
    global rawImage
    global angularVelocity
    global prevTime
    global edgesImage
    global error
    global totalFinalX

    now = rospy.get_rostime()    
    nowTime = now.nsecs
    errorTime = 0
    if nowTime<prevTime:
        errorTime = nowTime-prevTime+1000000000
    else:
        errorTime = nowTime-prevTime
    errorTime = errorTime / 100000000.0 
    prevTime = nowTime

    img_hsv = cv2.cvtColor(rawImage, cv2.COLOR_BGR2HSV)

    lower_color = (12, 87, 0)
    upper_color = (44, 255, 255)
    img_mask = cv2.inRange(img_hsv, lower_color, upper_color) 
   
    kernel = np.ones((3, 3), np.uint8)
    img_mask = cv2.erode(img_mask, kernel, iterations = 2)
    img_mask = cv2.dilate(img_mask, kernel, iterations = 2)

    edgesImage = cv2.Canny(img_mask,50,150,apertureSize = 3)
    lines = cv2.HoughLinesP(edgesImage, 0.8, np.pi / 180, 90, minLineLength = 50, maxLineGap = 100)

    if lines is None:
        return None


    x_array = []
    y_array = []

    for i in lines:

        x1=i[0][0]
        y1=i[0][1]
        x2=i[0][2]
        y2=i[0][3]

        if y1 != y2:
            length = math.sqrt(math.pow(int(x2 - x1), 2) + math.pow(int(y2 - y1),2))
            if length > 100:
                x_array.append(i[0][0])
                y_array.append(i[0][1])
                x_array.append(i[0][2])
                y_array.append(i[0][3])
                cv2.line(rawImage, (i[0][0], i[0][1]), (i[0][2], i[0][3]), (0, 0, 255), 2)
                

    if(len(x_array)>3):
        f1 = np.polyfit(y_array,x_array,1)
        
        final_y = 380
        final_x = int(f1[0]*380+f1[1])
        cv2.circle(rawImage, (final_x,final_y), 5 ,(255, 255, 0), 8)

        error = (320 - final_x)/1000.0

        K_P = 1.1
        
        goal = error*K_P

        max_value = 0.45

        if goal < -max_value:
            goal = -max_value
        if goal > max_value:
            goal = max_value
        # print("angular")
        # print(goal)

        angularVelocity = goal
        


# ------------------PUBLISH FUNCTION-----------------------

def RobotMovePublish():
    global isMove
    global linearVelocity
    global angularVelocity
    global isLidarModeOn

    twist = Twist()
    twist.linear.x = 0.26
    twist.linear.y = 0
    twist.linear.z = 0
    twist.angular.x = 0
    twist.angular.y = 0
    twist.angular.z = angularVelocity

    if not isMove:
        twist.linear.x = 0
        twist.angular.z = 0
    
    twistPub.publish(twist)
    
def ImageShow():
    global laserImage
    global rawImage
    global edgesImage

    cv2.imshow('rawImage', rawImage)
    cv2.imshow('edgesImage', edgesImage)   
    cv2.waitKey(1)
    cv2.imshow('laserImage', laserImage)

def initialize():
    print("Initialize")

# ---------------------------------------------------------------

initialize()

rospy.init_node('main_node')

twistPub = rospy.Publisher('/cmd_vel', Twist, queue_size=100)
movePub = rospy.Publisher('/move', Int32, queue_size=100)

rawSub = rospy.Subscriber('/d435/color/image_raw', Image, RawCallback, queue_size=1)
laserSub = rospy.Subscriber('/scan', LaserScan, LaserCallback, queue_size=1)
odomSub = rospy.Subscriber('/odom', Odometry, OdomCallback, queue_size=1)

rospy.spin()


