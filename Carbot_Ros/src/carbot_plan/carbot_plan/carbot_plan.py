#!/usr/bin/env python
# encoding: utf-8

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry
from std_msgs.msg import Int64MultiArray
from rclpy.callback_groups import ReentrantCallbackGroup
import time

class Carbot_Plan(Node):
    def __init__(self,name):
        super().__init__(name)

        # 位置发送
        self.twist_pub = self.create_publisher(Twist,"twist_cmd",2)
        self.pose_pub = self.create_publisher(Pose,"navi_pose",10)
        # 位置初始化
        self.now_pose = Pose()
        # 位置反馈
        self.odom_sub = self.create_subscription(Odometry,"odom_data",self.odom_callback,2)

        # 机械臂控制
        self.arm_pub = self.create_publisher(Int64MultiArray,"arm_cmd",2)

        # 任务标记
        self.get_out = False
        self.get_qr_place = False
        self.get_pik_place = False

        # 任务初始化
        self.task_lists = [
            self.start_task(),
            self.scan_task(),
        ]
        
    def start_task(self):
        now_x = self.now_pose.position.x
        now_y = self.now_pose.position.y
        # 导航二维码前
        # step 1: 出站
        twist = Twist()
        twist.linear.x = 0.0
        twist.linear.y = 0.0
        if not self.get_out:
            if now_y > 0.2 and now_y < 0.3:
                twist.linear.y = 0.0
                self.get_out = True
            elif now_y <= 0.2:
                twist.linear.y = 0.1
            else:
                twist.linear.y = -0.1
        # step 2: 寻码
        if not self.get_qr_place and self.get_out:
            if now_x > 0.6 and now_x < 0.7:
                twist.linear.x = 0.0
                self.get_qr_place = True
            elif now_x <= 0.6:
                twist.linear.x = 0.1
            else:
                twist.linear.x = -0.1
        self.twist_pub.publish(twist)
    
    def scan_task(self):
        time.sleep(3)
        return True



    def odom_callback(self,odom_msg):
        self.now_pose = odom_msg.pose.pose
        self.start_task()
        
def main():
    rclpy.init()
    carbot_plan = Carbot_Plan("carbot_plan")
    rclpy.spin(carbot_plan)
    carbot_plan.destroy_node()
    rclpy.shutdown()