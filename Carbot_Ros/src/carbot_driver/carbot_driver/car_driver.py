#!/usr/bin/env python
# encoding: utf-8

import rclpy
from rclpy.node import Node
from carbot_lib import Carbot
from geometry_msgs.msg import Twist


class Car_Driver(Node):
	def __init__(self, name):
		super().__init__(name)
        # 实例化底层控制板
		self.car = Carbot()
		# 创建订阅者
		self.twist_sub = self.create_subscription(Twist,"twist_cmd",self.twist_callback,1)
		# 参数声明
		self.declare_parameter("imu_pid_ctrl",False)
		self.imu_pid_ctrl = self.get_parameter("imu_pid_ctrl").get_parameter_value().bool_value

	# 回调函数
	def twist_callback(self,twist_msg):
        # 速度(v_x,v_y表示线速度, v_yaw为偏航角角速度)
		v_x = twist_msg.linear.x * 1.0
		v_y = twist_msg.linear.y * 1.0
		v_yaw = twist_msg.angular.z * 1.0
		# 移动状态和移动速度初始化
		# state = [0, 7],=0停止,=1前进,=2后退,=3向左,=4向右,=5左旋,=6右旋,=7停车
		state = 0
  		# speed = [0, 100] 线速度默认限制25,角速度默认限制20
		speed = 0
		# 移动判定
		# speed 在电机驱动中为 [0, 100] 而在控制消息twist中为[0,1] 故放大100倍
		if v_x > 0:
			state = 1
			speed = v_x * 100
		elif v_x < 0:
			state = 2
			speed = -v_x * 100
		elif v_y > 0:
			state = 3
			speed = v_y * 100
		elif v_y < 0:
			state = 4
			speed = -v_y * 100
		elif v_yaw > 0:
			state = 5
			speed = v_yaw * 100
		elif v_yaw < 0:
			state = 6
			speed = -v_yaw * 100
		else:
			state = 7
		# 下发运动控制(含imu的pid控制判断)
		self.car.set_car_run(state, speed, adjust=self.pid_ctrl)
		
def main():
	rclpy.init() 
	car_driver = Car_Driver('car_driver')
	rclpy.spin(car_driver)
	car_driver.destroy_node()
	rclpy.shutdown()