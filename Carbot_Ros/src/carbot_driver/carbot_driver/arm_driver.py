#!/usr/bin/env python
# encoding: utf-8

import rclpy
from rclpy.node import Node
from carbot_lib import Carbot
from std_msgs.msg import Int64MultiArray

class Arm_Driver(Node):
    def __init__(self,name):
        super().__init__(name)
        # 订阅/发布
        self.arm_sub = self.create_subscription(Int64MultiArray,"arm_cmd",self.arm_callback,1)
        # 参数
        self.declare_parameter("arm_run_time", 800)
        self.run_time = self.get_parameter("arm_run_time").get_parameter_value().integer_value
        # 初始化
        self.arm = Carbot()
        self.arm_cmd = Int64MultiArray()
    
    def arm_callback(self,arm_msg):
        angle_01 = arm_msg.data[0]
        angle_02 = arm_msg.data[1]
        angle_03 = arm_msg.data[2]
        self.arm.set_uart_servo_angle_array([angle_01, angle_02, angle_03, 0, 0, 0],self.run_time)

def main():
    rclpy.init()
    arm_driver = Arm_Driver()
    rclpy.spin(arm_driver)
    arm_driver.destroy_node()
    rclpy.shutdown()
        