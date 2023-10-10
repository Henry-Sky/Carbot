#!/usr/bin/env python
# encoding: utf-8

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose
from std_msgs.msg import Int64MultiArray
import time

class Carbot_Plan(Node):
    def __init__(self,name):
        super().__init__(name)
        # 位置控制
        self.twist_pub = self.create_publisher(Twist,"twist_cmd",10)
        self.pose_pub = self.create_publisher(Pose,"goal_pose",10)
        # 机械臂控制
        self.arm_pub = self.create_publisher(Int64MultiArray,"arm_cmd",10)
        # 摄像头调用

        
    def publish_pose(self,pose):
        self.pose_pub.publish(pose)

    def publish_arm(self,arm):
        self.arm_pub.publish(arm)

    def publish_twist(self,twist):
        self.twist_pub.publish(twist)

        
def main():
    rclpy.init()
    carbot_plan = Carbot_Plan("carbot_plan")
    # Task01 : Carbot 出战,离开蓝色启停区
    start_twist = Twist()
    start_twist.linear.y = 0.1
    carbot_plan.publish_twist(start_twist)
    time.sleep(1.5)
    start_twist = Twist()
    carbot_plan.publish_twist(start_twist)
    # Task02 : 扫描二维码，获取任务
    # Task03 : 粗加工区拾取货物
    # Task04 : 
    carbot_plan.destroy_node()
    rclpy.shutdown()

main()