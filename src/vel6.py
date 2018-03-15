#!/usr/bin/env python
import sys
import numpy as np
import rospy
import cv2
import time
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import Float64, Float64MultiArray, String
from collections import deque

max_size = 3000    #  object maximum size
default_x =140
default_y =10       # check this one

min_size = 50
deque_size = 2
auto_deque_size =5
size_deque = deque([max_size,max_size], deque_size)
x_deque = deque([default_x, default_x], deque_size)
y_deque = deque([default_y, default_y], deque_size)
auto_deque = deque([0,0,0,0,0], auto_deque_size)
msg=Twist()
msg.linear.x =0
normal_speed = 0.1
eint=0.0



class vel_ctrl(object):
    def __init__(self):
        self.say_flag =0
        rospy.init_node('vel_ctl', anonymous=False)
        self.vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        # self.say_pub = rospy.Publisher('/say', String, queue_size=1)
        rospy.Subscriber('/obj_size', Float64MultiArray, self.callback)


    def callback(self, data):
        eint = 0.0
        eprev=0.0
        ed=0
        # global u
        try:
            size = data.data
            print("Received size= ")
            print(len(size))
            # print(size[1])
            road_size = size[1]
            road_x = size[2]
            road_y = size[3]
            obs_flag = size[8]
            obs_size=size[9]
            obs_x = size[10]
            obs_y = size[11]
            sum_size = 0
            sum_x =0
            sum_y=0
            sum_auto =0
            msg=Twist()
            # print(obj_size)
            # print("pkg size", len(size))
            # auto_deque.append(road_size)
            # print("auto_deque size = ", len(auto_deque))
            print("with obs?", size[9])
            if obs_flag ==2.0 and obs_size > min_size:
                print("found")
                auto_deque.append(obs_size)

            # low pass filter
            else:
                if obs_flag ==0.0:
                    auto_deque.append(0)

            for i in xrange(auto_deque_size):
                sum_auto += auto_deque[i]
            if len(auto_deque)!=0:
                avg_auto = sum_auto/auto_deque_size
            else:
                avg_auto =0
            # print("avg_auto = ", avg_auto)
            # self.say_flag =0
            # say_pub.publish("")
            # Switch between stop and move
            print("obs size",avg_auto)
            if avg_auto !=0:
                msg.linear.x =0
                msg.angular.z=0
                self.vel_pub.publish(msg)



            else:
                if road_size >min_size:
                    size_deque.append(road_size)
                    x_deque.append(road_x)
                    y_deque.append(road_y)
                else:
                    size_deque.append(0)
                    x_deque.append(default_x)
                    y_deque.append(default_y)



                if len(size_deque) ==deque_size:
                    for i in xrange(deque_size):
                        sum_size+= size_deque[i]
                        sum_x+= x_deque[i]
                        sum_y+= y_deque[i]

                    avg_size = sum_size/deque_size
                    avg_x = sum_x/deque_size
                    avg_y = sum_y/deque_size
                    auto_flag =0
                    print("avg_x = ", avg_x)
                    msg.linear.x =normal_speed
                    # auto_deque.clear()


                else:
                    avg_size = 0
                    avg_x = default_x
                    avg_y = default_y


                delta_x = avg_x - default_x
                print("delta_x =", delta_x)
                # original
                # ke = 0.02
                # kd=0.001
                # ki=0
                ke =0.026
                kd=0.001
                ki=0

                if avg_size ==0:
                    msg.linear.x =0
                    msg.angular.z=0
                    self.vel_pub.publish(msg)
                else:
                    e = delta_x
                    ed=e-eprev
                    eint =eint + e
                    u = ke*e+ki*eint+kd*ed
                    eprev=e
                    print(u)
                    msg.linear.x = normal_speed
                    msg.angular.z =u
                    self.vel_pub.publish(msg)



        except:
            print('error!')
            # pass
        # rospy.sleep(0.1)

# Todo : find the pixel shift/ mm shift relationship at a certain distance says 10cm (linear_x =0.1, 1 second)








def main():
    # rospy.init_node('vel_ctl', anonymous=False)
    # sub = rospy.Subscriber('/obj_size', Float64MultiArray, callback)
    # vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    vc =vel_ctrl()
    bridge = CvBridge()
    try:
        # move_pub(u)
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
        # msg=Twist()
        # msg.linear.x =0
        # msg.linear.y=0
        # vel_pub.publish(msg)
    # cv2.destroyAllWindows()




if __name__ == '__main__':
    main()
