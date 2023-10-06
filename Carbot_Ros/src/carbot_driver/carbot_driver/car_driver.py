#!/usr/bin/env python
# encoding: utf-8

#public lib
from carbot_lib import Carbot

#ros lib
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class Car_Driver(Node):
	def __init__(self, name):
		super().__init__(name)

        # 实例化底层控制库
		self.car = Carbot()

		# self.car.set_uart_servo_angle_array([64, 209, 141, 90, 90, 90])

		# 创建订阅者
		self.sub_cmd_vel = self.create_subscription(Twist,"cmd_vel",self.cmd_callback,1)
		# 参数声明
		self.declare_parameter("pid_ctrl",False)
		self.pid_ctrl = self.get_parameter("pid_ctrl").get_parameter_value().bool_value

	# 回调函数
	def cmd_callback(self,msg):
        # 检查消息类型
		if not isinstance(msg, Twist): return
        # 下发线速度和角速度
		vx = msg.linear.x*1.0
		vy = msg.linear.y*1.0
		vz = msg.angular.z*1.0
		
		state = 0
		speed = 0

		if vx > 0:
			state = 1
			speed = vx
		elif vx < 0:
			state = 2
			speed = -vx
		elif vy > 0:
			state = 3
			speed = vy
		elif vy < 0:
			state = 4
			speed = -vy
		elif vz > 0:
			state = 5
			speed = vz
		elif vz < 0:
			state = 6
			speed = -vz
		else:
			state = 7

		# state=[0, 7],=0停止,=1前进,=2后退,=3向左,=4向右,=5左旋,=6右旋,=7停车
		self.car.set_car_run(state, speed, adjust=self.pid_ctrl)
		
			
def main():
	rclpy.init() 
	car_driver = Car_Driver('car_driver')
	rclpy.spin(car_driver)
	rclpy.shutdown()