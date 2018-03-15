#!/usr/bin/env python
import sys
import numpy as np
import rospy
import cv2
from sensor_msgs.msg import Image
from std_msgs.msg import Float64
from cv_bridge import CvBridge, CvBridgeError
# from evdev import InputDevice
# from select import select



def callback(data):
    global frame_raw
    global frame_cpy

    bridge = CvBridge()
    try:
        pub_head = rospy.Publisher('/head_angle', Float64, queue_size=1)
        head_angle = 5.0
        pub_head.publish(head_angle)

        with open("wb.txt","r") as f:
            bgr_cal = f.read().split()
        # print(type(bgr_cal[0]))
        for i in range(len(bgr_cal)):
            bgr_cal[i]=float(bgr_cal[i])
        # print(type(bgr_cal[0]))
        frame_raw = bridge.imgmsg_to_cv2(data, "bgr8")
        cv2.flip(frame_raw, 0 )
        frame_hsv = cv2.cvtColor(frame_raw, cv2.COLOR_BGR2HSV)
        frame_cpy = frame_raw.copy()
        b, g, r =cv2.split(frame_cpy)
        cv2.convertScaleAbs(b, b, bgr_cal[0])
        cv2.convertScaleAbs(g, g, bgr_cal[1])
        cv2.convertScaleAbs(r, r, bgr_cal[2])
        frame_cpy = cv2.merge((b,g,r))

        # frame_edit = frame_raw.copy()
        # cv2. rectangle(frame_edit,(80, 60), (240,180), (255,0,0), 2)
        cv2.imshow('aim target', frame_cpy)
        cv2.waitKey(1)
        # if keyboard.is_pressed('c'):
        #     cv2.imwrite("./test.jpg", frame_raw[60:180, 80:240])
        # else:
        #     pass
        # select([dev], [], [])
        # for event in dev.read():
        #     if (event.value == 1 or event.value == 0) and event.code == 44:
        #         cv2.imwrite("./test.jpg", frame_raw[60:180, 80:240])
    except CvBridgeError as e:
        print(e)
    # except KeyboardInterrupt:
    #     cv2.imwrite("test.jpg", frame_cpy)
    # # cv2.imwrite("./test.jpg", frame_raw[60:180, 80:240])
    # # cv2.destroyAllWindows()

def main():
    rospy.init_node('img_cap', anonymous=False)
    try:
        sub = rospy.Subscriber('/cozmo_camera/image', Image, callback)
        # rate = rospy.Rate(10)
        bridge = CvBridge()
        # rate.sleep()
        rospy.spin()
    except KeyboardInterrupt:

        print("Shutting down")


    cv2.imwrite("test.jpg", frame_cpy)

if __name__ == '__main__':
    main()
