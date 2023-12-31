#!/usr/bin/env python
# encoding: utf-8

import rclpy
import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge

# 摄像头内参
mtx = np.array([[1.35635605e+03, 0.00000000e+00, 6.46212314e+02],
       [0.00000000e+00, 1.35578874e+03, 3.78009469e+02],
       [0.00000000e+00, 0.00000000e+00, 1.00000000e+00]])

dist = np.array([[-2.52464877e-01, 1.92828476e-01, -6.55508668e-04, 2.48903668e-03, -7.45335496e-01]])


class Image_Pub(Node):
    def __init__(self, name):
        super().__init__(name)
        # 摄像头编号参数
        self.declare_parameter("cap_id",0)
        self.cap_id = self.get_parameter("cap_id").get_parameter_value().integer_value
        # 创建画面发布者
        self.image_publisher = self.create_publisher(Image, "image_raw", 2)
        # 打开摄像头
        self.cap = cv2.VideoCapture(self.cap_id)
        # 消息频率
        self.time_period = 0.001
        self.timer = self.create_timer(self.time_period, self.timer_callback)
        
    def timer_callback(self):
        ret = False
        ret, frame = self.cap.read()
        if ret:
            dst = cv2.undistort(frame, mtx, dist, None, mtx)
            img = CvBridge().cv2_to_imgmsg(dst, encoding='passthrough')
            self.image_publisher.publish(img)
    
def main():
    rclpy.init()
    image_pub = Image_Pub('image_pub')
    rclpy.spin(image_pub)
    rclpy.shutdown()      