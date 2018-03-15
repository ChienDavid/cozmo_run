#!/usr/bin/env python
import sys
import numpy as np
import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


/****************************************************************
callback is to keep displaying the video feed on the displaying
****************************************************************/


def callback(data):
    bridge = CvBridge()
    color_min = np.array([90, 30, 120], np.uint8)
    color_max = np.array([135, 255, 255], np.uint8)
    rgb_cal=[]
    try:
        with open('wb.txt','r') as f:
            for line in f:
                rgb.append(map(float,line.split(' ')))
    exept:
            rgb_cal=[1.0,1.0,1.0]
    try:
            frame_raw = bridge.imgmsg_to_cv2(data, "bgr8")

            frame_hsv = cv2.cvtColor(frame_raw, cv2.COLOR_BGR2HSV)
            mask = cv2.inRange(frame_hsv, color_min, color_max)
            res = cv2.bitwise_and(frame_hsv, frame_hsv, mask=mask)
            canny = cv2.Canny(res, 300, 600)
            cv2.imshow('raw', frame_raw)
            cv2.imshow('res', res)
            cv2.imshow('canny', canny)
            cv2.waitKey(1)
    except CvBridgeError as e:
        print(e)


def main():
    rospy.init_node('img_process', anonymous=False)
    sub = rospy.Subscriber('/cozmo_camera/image', Image, callback)
    bridge = CvBridge()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
