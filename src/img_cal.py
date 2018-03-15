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
    cap=""
    shot=""
    bridge = CvBridge()
    # dev = InputDevice('/dev/input/event4')
    try:
            frame_raw = bridge.imgmsg_to_cv2(data, "bgr8")
            frame_hsv = cv2.cvtColor(frame_raw, cv2.COLOR_BGR2HSV)
            frame_edit = frame_raw.copy()
            cv2. rectangle(frame_edit,(80, 60), (240,180), (255,0,0), 2)
            cv2.imshow('aim target', frame_edit)
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
            # cv2.imwrite("./test.jpg", frame_raw[60:180, 80:240])
    # cv2.imwrite("./test.jpg", frame_raw[60:180, 80:240])
    # cv2.destroyAllWindows()

def main():
    sum_b=0.0
    sum_g=0.0
    sum_r=0.0
    rospy.init_node('img_calibration', anonymous=False)
    sub = rospy.Subscriber('/cozmo_camera/image', Image, callback)
    pub_head = rospy.Publisher('/head_angle', Float64)
    pub_head.publish(10.0)
    bridge = CvBridge()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    for i in range (79, 240):
        for j in range (59, 180):
            sum_b+=frame_raw[i, j, 0]
            sum_g+=frame_raw[i, j, 1]
            sum_r+=frame_raw[i, j, 2]
    avg_b = sum_b/(160*120)
    avg_g = sum_g/(160*120)
    avg_r = sum_r/(160*120)
    nor_b = min(avg_r,avg_g, avg_b)/avg_b
    nor_g = min(avg_r,avg_g, avg_b)/avg_g
    nor_r = min(avg_r,avg_g, avg_b)/avg_r
    # print(nor_r, nor_g, nor_b)
    with open("wb.txt", "w") as text_file:
        text_file.write('%4f' % nor_b)
        text_file.write(' ')
        text_file.write('%4f' % nor_g)
        text_file.write(' ')
        text_file.write('%4f' % nor_r)



if __name__ == '__main__':
    main()
