#!/usr/bin/env python
# encoding: utf-8

#public lib
import sys
import math
import random
import threading
from math import pi
from time import sleep
from carbot_lib import Carbot

#ros lib
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class Carbot_Driver(Node):
	def __init__(self, name):
		super().__init__(name)

        # 实例化底层控制库
		self.car = Carbot()

		# 创建订阅者
		self.sub_cmd_vel = self.create_subscription(Twist,"cmd_vel",self.cmd_vel_callback,1)

		self.declare_parameter("using_pid_ctrl",False)

		self.using_pid_ctrl = self.get_parameter("using_pid_ctrl").get_parameter_value().bool_value

	# 回调函数
	def cmd_vel_callback(self,msg):
        # 检查消息类型
		if not isinstance(msg, Twist): return
        # 下发线速度和角速度
		vx = msg.linear.x*1.0
		vy = msg.linear.y*1.0
		vz = msg.angular.z*1.0
		
		if self.using_pid_ctrl:
			pass
		else:
			self.car.set_car_motion(vx, vy, vz)
		
			
def main():
	rclpy.init() 
	carbot_driver = Carbot_Driver('driver_node')
	rclpy.spin(carbot_driver)
	rclpy.shutdown()