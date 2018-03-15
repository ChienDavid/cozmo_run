#!/usr/bin/env python
import sys
import numpy as np
import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import Float64,Float64MultiArray
from nav_msgs.msg import Odometry

min_track_size = 20
default_x =160
default_y =10

def callback(data):
    bridge = CvBridge()
    # road
    color_min = np.array([0, 15 , 20], np.uint8)
    color_max = np.array([60, 240, 200], np.uint8)
    # color_min = np.array([25, 100, 100],np.uint8)
    # color_max = np.array([35, 255, 255],np.uint8)
    #  home
    obj_color_min = np.array([150, 150, 100], np.uint8)
    obj_color_max = np.array([180, 240, 200], np.uint8)
    bgr_cal=[]

    # try:


    # pub_size = rospy.Publisher('/obj_size', Float64, queue_size=1)
    try:
        with open("/home/evan/catkin_ws/src/cozmo_run/src/wb.txt","r") as f:
            bgr_cal = f.read().split()
        # print(type(bgr_cal[0]))
        for i in xrange(len(bgr_cal)):
            bgr_cal[i]=float(bgr_cal[i])
        # print(type(bgr_cal[0]))

    except:
            # bgr_cal=[1,1,1]
            print('wb_error')
            pass
    try:
            pub_head = rospy.Publisher('/head_angle', Float64, queue_size=1)
            head_angle = -5
            pub_head.publish(head_angle)
            frame_raw = bridge.imgmsg_to_cv2(data, "bgr8")
            frame_raw =cv2.flip(frame_raw, 1)
            height, width = frame_raw.shape[:2]
            frame_cpy = frame_raw.copy()
            b, g, r =cv2.split(frame_cpy)
            cv2.convertScaleAbs(b, b, bgr_cal[0])
            cv2.convertScaleAbs(g, g, bgr_cal[1])
            cv2.convertScaleAbs(r, r, bgr_cal[2])
            frame_cpy = cv2.merge((b,g,r))
            # Original
            # frame_lower = frame_cpy[200:239, 30:305]
            # test


            # Frame cropping setting
            frame_lower =frame_cpy[120:160,15:305]
            # obj_frame = frame_cpy[140:160,30:290]
            frame_higher = frame_cpy[90:110,30:290]
            # obj_frame = frame_cpy[100:120, 40:280]
            obj_frame = frame_cpy[110:120, 40:280]
            frame_cpy = cv2.flip(frame_cpy, 1)


            kernel1 = np.ones((5,5),np.float32)/25
            kernel2 = np.ones((3,3),np.float32)/9


            # whole picture frame setting
            frame_hsv = cv2.cvtColor(frame_cpy, cv2.COLOR_BGR2HSV)
            mask = cv2.inRange(frame_hsv, color_min, color_max)
            res = cv2.bitwise_and(frame_hsv, frame_hsv, mask = mask)
            res = cv2.erode(res, kernel1, iterations=1)
            res = cv2.dilate(res, kernel1, iterations=1)
            gray = cv2.cvtColor(res, cv2.COLOR_HSV2BGR)
            gray = cv2.cvtColor(gray, cv2.COLOR_BGR2GRAY)
            canny = cv2.Canny(gray, 100, 200)
            _,contours,_ = cv2.findContours(gray,cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

            #  closer distance frame setting
            frame_hsv_lower = cv2.cvtColor(frame_lower, cv2.COLOR_BGR2HSV)
            mask = cv2.inRange(frame_hsv_lower, color_min, color_max)
            res_lower = cv2.bitwise_and(frame_hsv_lower, frame_hsv_lower, mask=mask)
            # res_lower = cv2.erode(res_lower, kernel1, iterations=1)
            res_lower = cv2.dilate(res_lower, kernel1, iterations=1)
            gray_lower = cv2.cvtColor(res_lower, cv2.COLOR_HSV2BGR)
            gray_lower = cv2.cvtColor(gray_lower, cv2.COLOR_BGR2GRAY)
            canny_lower = cv2.Canny(gray_lower, 100, 200)
            _,contours_lower,_ = cv2.findContours(gray_lower,cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)




            #  obj area frame setting
            obj_frame_hsv = cv2.cvtColor(obj_frame, cv2.COLOR_BGR2HSV)
            obj_mask = cv2.inRange(obj_frame_hsv, obj_color_min, obj_color_max)
            obj_res = cv2.bitwise_and(obj_frame_hsv, obj_frame_hsv, mask=obj_mask)
            obj_res = cv2.erode(obj_res, kernel2, iterations=1)
            obj_res = cv2.dilate(obj_res, kernel2, iterations=1)
            obj_gray = cv2.cvtColor(obj_res, cv2.COLOR_HSV2BGR)
            obj_gray = cv2.cvtColor(obj_gray, cv2.COLOR_BGR2GRAY)
            _,obj_contours,_ = cv2.findContours(obj_gray,cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

            # canny = cv2.Canny(gray, 100, 200)




            #  longer distance frame setting
            frame_hsv_h = cv2.cvtColor(frame_higher, cv2.COLOR_BGR2HSV)
            mask = cv2.inRange(frame_hsv_h, color_min, color_max)
            res_h = cv2.bitwise_and(frame_hsv_h, frame_hsv_h, mask=mask)
            res_h = cv2.erode(res_h, kernel2, iterations=1)
            res_h = cv2.dilate(res_h, kernel2, iterations=1)
            gray_h = cv2.cvtColor(res_h, cv2.COLOR_HSV2BGR)
            gray_h = cv2.cvtColor(gray_h, cv2.COLOR_BGR2GRAY)
            _,contours_h,_ = cv2.findContours(gray_h,cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

            #  lower area publish
            a = Float64MultiArray()
            if len(contours_lower) !=0:
                c = max(contours_lower, key = cv2.contourArea)
                # cnt = c
                # print(cnt)
                area = cv2.contourArea(c)
                M = cv2.moments(c)
                if area != 0:
                    cx = int(M['m10']/M['m00'])
                    cy = int(M['m01']/M['m00'])
                else:
                    cx = default_x
                    cy = default_y
                if area > min_track_size:

                    # a.data = [area,cx, cy]
                    a.data.append(0)
                    a.data.append(area)
                    a.data.append(cx)
                    a.data.append(cy)
                    # print(a.data, area, cx, cy)
                    # # pub_size.publish(a)
                    # print ("Obj found in lower area\n")
                    # cv2.drawContours(frame_cpy, contours_lower, -1, (0,255,0), 3)
                    if abs(cx - default_x)>20:
                        cv2.arrowedLine(frame_cpy, (default_x, 80),(width-cx, 80), (255, 0,100), thickness=8)
                else:
                    # a = Float64MultiArray()
                    # a.data = [0, 0.0,default_x,default_y]
                    a.data.append(0)
                    a.data.append(0.0)
                    a.data.append(default_x)
                    a.data.append(default_y)
                    # pub_size.publish(b)
                    # print ("Noise only at lower area\n")
                    # print(area, cx, cy)

            else:
                print("error")
                # a = Float64MultiArray()
                # a.data = [0, 0.0,default_x,default_y]
                a.data.append(0)
                a.data.append(0.0)
                a.data.append(0.0)
                a.data.append(0.0)
                # print ("No target found at lower\n")
                # # print(area)


            # higher area publish
            if len(contours_h) !=0:
                c = max(contours_h, key = cv2.contourArea)
                # cnt = c
                # print(cnt)
                area = cv2.contourArea(c)
                M = cv2.moments(c)
                if area != 0:
                    cx = int(M['m10']/M['m00'])
                    cy = int(M['m01']/M['m00'])
                else:
                    cx = default_x
                    cy = default_y
                if area > min_track_size:
                    a.data.append(1)
                    a.data.append(area)
                    a.data.append(cx)
                    a.data.append(cy)
                    # print(a.data, area, cx, cy)
                    # print(area, cx, cy)
                    # pub_size.publish(a)
                    # print ("Obj found in higher area\n")
                else:
                    a.data.append(1)
                    a.data.append(0.0)
                    a.data.append(0.0)
                    a.data.append(0.0)
                    # print ("Noise only at higher area\n")
                    # print(area, cx, cy)
            else:
                print("error")
                # a = Float64MultiArray()
                # a.data = [1, 0.0,default_x,default_y]
                a.data.append(1)
                a.data.append(0.0)
                a.data.append(0.0)
                a.data.append(0.0)
                # print ("No target found at higher\n")

            # Obj area publish
            if len(obj_contours) !=0:
                c = max(obj_contours, key = cv2.contourArea)
                # cnt = c
                # print(cnt)
                area = cv2.contourArea(c)
                M = cv2.moments(c)
                if area != 0:
                    cx = int(M['m10']/M['m00'])
                    cy = int(M['m01']/M['m00'])
                else:
                    cx = default_x
                    cy = default_y
                if area > min_track_size:
                    # a = Float64MultiArray()
                    # a.data = [area,cx, cy]
                    a.data.append(2)
                    a.data.append(area)
                    a.data.append(cx)
                    a.data.append(cy)
                    # print(area, cx, cy)
                    # pub_size.publish(a)
                    print ("Obj found in obj area\n")
                else:
                    # a = Float64MultiArray()
                    # a.data = [2, 0.0,default_x,default_y]
                    a.data.append(2)
                    a.data.append(0.0)
                    a.data.append(default_x)
                    a.data.append(default_y)
                    # pub_size.publish(b)
                    print ("Noise only at obj area\n")
                    # print(area, cx, cy)

            else:
                print("error")
                # a = Float64MultiArray()
                # a.data = [0, 0.0,default_x,default_y]
                a.data.append(0)
                a.data.append(0.0)
                a.data.append(0.0)
                a.data.append(0.0)
                print ("No target found at obj\n")
                # print(area)

            print(a)
            pub_size.publish(a)
            # cv2.imshow('raw', frame_raw)
            # cv2.imshow('res', res)


            cv2.imshow('gray', gray_lower)
            # cv2.imshow('gray_h', gray_h)
            # cv2.imshow('canny', canny)
            cv2.imshow('obj_gray', obj_gray)
            # cv2.imshow("obj_res",obj_frame_hsv)
            # cv2.imshow("original", frame_raw)

            cv2.namedWindow('wb',cv2.WINDOW_NORMAL)
            cv2.resizeWindow('wb', 1280, 960)
            cv2.imshow("wb", frame_cpy)
            cv2.waitKey(1)
            # print(a)
    except CvBridgeError as e:
        print(e)

def callback1(data):
    global x, y
    try:
        x = data.pose.pose.position.x
        y = data.pose.pose.position.y
        # print(x, y)
    except:
        print("No odom")



def main():
    rospy.init_node('obj_id', anonymous=False)
    global pub_size
    pub_size = rospy.Publisher('/obj_size', Float64MultiArray, queue_size=1)
    sub = rospy.Subscriber('/cozmo_camera/image', Image, callback)
    sub1 = rospy.Subscriber('/odom', Odometry, callback1)
    bridge = CvBridge()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
