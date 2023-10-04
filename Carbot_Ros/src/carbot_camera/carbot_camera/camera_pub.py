#!/usr/bin/env python
# encoding: utf-8

import rclpy
from rclpy.node import Node
import numpy as np
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge, CvBridgeError

class Camera_Pub(Node):
    def __init__(self, name):
        super.__init__(name) 
        # 关于摄像头编号
        self.declare_parameter("cap_id",0)
        self.cap_id = self.get_parameter("cap_id").get_parameter_value().integer_value
        # 创建画面发布者
        self.image_publisher = self.create_publisher(Image, "/image_raw", 2)
        # 打开摄像头
        self.cap = cv2.VideoCapture(self.cap_id)
        self.bridge = CvBridge()
        self.count = 0
        
        self.time_period = 0.001
        self.timer = self.create_timer(time_period, self.timer_callback)
        
    def timer_callback(self):
        self.ret = False
        self.ret, self.frame = self.cap.read()
        if self.ret:
            self.msg = self.bridge.cv2_to_compressed_imgmsg(self.frame, 'jpg')
            self.image_publisher.publish(self.msg)
    
def main():
    rclpy.init()
    camera_pub = Camera_Pub('carbot_camera')
    rclpy.spin(camera_pub)
    rclpy.shutdown() 
        
            
        
        