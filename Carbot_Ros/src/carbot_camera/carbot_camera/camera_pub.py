#!/usr/bin/env python
# encoding: utf-8

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge


class Camera_Pub(Node):
    def __init__(self, name):
        super().__init__(name)
        # 摄像头编号参数
        self.declare_parameter("cap_id",0)
        self.cap_id = self.get_parameter("cap_id").get_parameter_value().integer_value
        # 创建画面发布者
        self.image_publisher = self.create_publisher(Image, "/image_raw", 2)
        # 打开摄像头
        self.cap = cv2.VideoCapture(self.cap_id)
        # 创建转换桥(Ros与Opencv)
        self.bridge = CvBridge()
        # 消息频率
        self.time_period = 0.001
        self.timer = self.create_timer(self.time_period, self.timer_callback)
        
    def timer_callback(self):
        self.ret = False
        self.ret, self.frame = self.cap.read()
        if self.ret:
            self.msg = self.bridge.cv2_to_imgmsg(self.frame, encoding='passthrough')
            self.image_publisher.publish(self.msg)
    
def main():
    rclpy.init()
    camera_pub = Camera_Pub('carbot_camera')
    rclpy.spin(camera_pub)
    rclpy.shutdown()      