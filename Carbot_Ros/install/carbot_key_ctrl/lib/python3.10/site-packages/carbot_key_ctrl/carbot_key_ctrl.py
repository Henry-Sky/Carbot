#!/usr/bin/env python
# encoding: utf-8
#import public lib
from geometry_msgs.msg import Twist
import sys, select, termios, tty

#import ros lib
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

tip = """
Carbot Controler
--------------------------------------------------
x: w/s	y: a/d	z: q/e
space: force stop
--------------------------------------------------
u/j : increase/decrease max speeds by 10%
i/k : increase/decrease only linear speed by 10%
o/l : increase/decrease only angular speed by 10%
--------------------------------------------------
CTRL-C to quit
"""

moveBindings = {'w': (1, 0, 0),'a': (0, 1, 0),'s': (-1, 0, 0),'d': (0, -1, 0),'q': (0, 0, 1),'e': (0, 0, -1),'W': (1, 0, 0),'A': (0, 1, 0),'S': (-1, 0, 0),'D': (0, -1, 0),'Q': (0, 0, 1),'E': (0, 0, -1),}

speedBindings = {'u': (1.1, 1.1),'j': (.9, .9),'i': (1.1, 1),'k': (.9, 1),'o': (1, 1.1),'l': (1, .9),'U': (1.1, 1.1),'J': (.9, .9),'I': (1.1, 1),'K': (.9, 1),'O': (1, 1.1),'L': (1, .9),}

class Carbot_Key_Ctrl(Node):

	# 节点初始化
	def __init__(self,name):
		super().__init__(name)
        # 创建一个发布者 消息类型为 Twist 名为 cmd_vel 
		self.pub = self.create_publisher(Twist,'cmd_vel',1)
        # 创建一个名为 linear_speed_limit 默认值为 1.0 的参数
		self.declare_parameter("linear_speed_limit",1.0)
        # 创建一个名为 angular_speed_limit 默认值为 5.0 的参数
		self.declare_parameter("angular_speed_limit",5.0)
		# 创建一个自动刹车的参数
		self.declare_parameter("auto_stop",False)
        # 获取名为 linear_speed_limit 的参数，并存储在 self.linenar_speed_limit
		self.linenar_speed_limit = self.get_parameter("linear_speed_limit").get_parameter_value().double_value
		self.angular_speed_limit = self.get_parameter("angular_speed_limit").get_parameter_value().double_value
		self.auto_stop = self.get_parameter("auto_stop").get_parameter_value().bool_value
		# tcgetattr函数用于获取与终端相关的参数
		self.settings = termios.tcgetattr(sys.stdin)

	# 监测按键输入
	def getKey(self):
		tty.setraw(sys.stdin.fileno())
		rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
		if rlist: key = sys.stdin.read(1)
		else:
			if self.auto_stop:
				key = ' '
			else: 
				key = ''
		termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
		return key

	def vels(self, speed, turn):
		return "currently:\tspeed %s\tturn %s " % (speed,turn)		
	
def main():
	rclpy.init()
	# 实例化节点
	carbot_key_ctrl = Carbot_Key_Ctrl("carbot_key_ctrl")
	# 速度初始化
	(speed, turn) = (0.2, 1.0)
	# 方向系数初始化
	(x_linear, y_linear, z_angular) = (0, 0, 0)

	twist = Twist()
	try:
		# 操控提示
		print(tip)

		while (1):

			# 键盘监听
			key = carbot_key_ctrl.getKey()

			if key in moveBindings.keys():
				x_linear = moveBindings[key][0]
				y_linear = moveBindings[key][1]
				z_angular = moveBindings[key][2]	

			elif key in speedBindings.keys():
				speed = speed * speedBindings[key][0]
				turn = turn * speedBindings[key][1]
				# 速度上限监测
				if speed > carbot_key_ctrl.linenar_speed_limit: 
					speed = carbot_key_ctrl.linenar_speed_limit
				if turn > carbot_key_ctrl.angular_speed_limit: 
					turn = carbot_key_ctrl.angular_speed_limit

			elif key == ' ':
				(x_linear, y_linear, z_angular) = (0, 0, 0)
    
			elif key == '\x03':
				break

			else:
				pass

			twist.linear.x = speed * x_linear
			twist.linear.y = speed * y_linear
			twist.angular.z = turn * z_angular

			carbot_key_ctrl.pub.publish(twist)

	except Exception as e: print(e)
	finally: carbot_key_ctrl.pub.publish(Twist())
	termios.tcsetattr(sys.stdin, termios.TCSADRAIN, carbot_key_ctrl.settings)
	carbot_key_ctrl.destroy_node()
	rclpy.shutdown()
