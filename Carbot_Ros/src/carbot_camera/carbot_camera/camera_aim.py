#!/usr/bin/env python
# encoding: utf-8

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
import numpy as np
import time
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge

# 摄像头内参
mtx = np.array([[1.35635605e+03, 0.00000000e+00, 6.46212314e+02],
       [0.00000000e+00, 1.35578874e+03, 3.78009469e+02],
       [0.00000000e+00, 0.00000000e+00, 1.00000000e+00]])

dist = np.array([[-2.52464877e-01, 1.92828476e-01, -6.55508668e-04, 2.48903668e-03, -7.45335496e-01]])

# HSV格式色表
color_dist = {'red': {'Lower': np.array([0, 60, 60]), 'Upper': np.array([6, 255, 255])},
              'gray': {'Lower': np.array([0, 0, 46]), 'Upper': np.array([180, 43, 220])},
              'yellow': {'Lower': np.array([26, 43, 46]), 'Upper': np.array([34, 255, 255])},          
              'blue': {'Lower': np.array([100, 80, 46]), 'Upper': np.array([124, 255, 255])},
              'green': {'Lower': np.array([35, 43, 35]), 'Upper': np.array([90, 255, 255])},}

class Carbot_Aim(Node):
    def __init__(self,name):
        super().__init__(name)
        # 参数声明
        self.declare_parameter("cap_id",0)
        self.declare_parameter("aim_x",320)
        self.declare_parameter("aim_y",240)
        self.declare_parameter("buffer",8)
        
        self.cap_id = self.get_parameter("cap_id").get_parameter_value().integer_value
        self.aim_x = self.get_parameter("aim_x").get_parameter_value().integer_value
        self.aim_y = self.get_parameter("aim_y").get_parameter_value().integer_value
        self.buffer = self.get_parameter("buffer").get_parameter_value().integer_value
        
        # 消息创建
        self.pub = self.create_publisher(Twist,'cmd_vel',1)
        # 打开摄像头
        self.cap = cv2.VideoCapture(self.cap_id)
    
    def get_frame(self):
        flag, frame = self.cap.read()
        if flag:
            dst = cv2.undistort(frame, mtx, dist, None, mtx)
            return True, dst
        else:
            return False
    
    def get_circle(self,img):
        type_test = np.zeros((1,1,3))
        hsv_img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        inRange_hsv = cv2.inRange(hsv_img, color_dist['green']['Lower'], color_dist['green']['Upper'])
        circles = cv2.HoughCircles(inRange_hsv, cv2.HOUGH_GRADIENT, 1, 50,
                         param1=100, param2=80, minRadius=60, maxRadius=200)
        if type(circles) == type(type_test):
            print(circles[0][0])
            return True, circles[0][0]
        else:
            return False, type_test
              
def main():
    rclpy.init()
    carbot_aim = Carbot_Aim("carbot_aim")
    aim_x = carbot_aim.aim_x
    aim_y = carbot_aim.aim_y
    buf = carbot_aim.buffer
    twist = Twist()
    try:
        while True:
            cir_x = 0
            cir_y = 0
            flag1 = False
            flag2 = False

            flag1, img = carbot_aim.get_frame()

            if flag1:
                flag2, circle = carbot_aim.get_circle(img)
            
            if flag2:
                cir_x = circle[0]
                cir_y = circle[1]

            
            if cir_x!=0 and cir_y != 0:
                if aim_x < cir_x - buf:
                    twist.linear.x = -0.01
                elif aim_x > cir_x + buf:
                    twist.linear.x = 0.01
                elif aim_y < cir_y - buf:
                    twist.linear.y = 0.01
                elif aim_y > cir_y + buf:
                    twist.linear.y = -0.01
                else:
                    twist.linear.x = 0.0
                    twist.linear.y = 0.0
                carbot_aim.pub.publish(twist)
                
    except Exception as e: print(e)
    finally:
        carbot_aim.pub.publish(Twist())
        carbot_aim.destroy_node()
    rclpy.shutdown() 