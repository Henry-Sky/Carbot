#!/usr/bin/env python
# encoding: utf-8

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry
from rclpy.callback_groups import ReentrantCallbackGroup
import time

class Carbot_Plan(Node):
    def __init__(self,name):
        super().__init__(name)

        self.twist_pub = self.create_publisher(Twist,"twist_cmd",2)
        self.now_pose = Pose()
        # 位置反馈
        self.odom_sub = self.create_subscription(Odometry,"odom_data",self.odom_callback,2)
        # 摄像头调取
        self.camera_pub = "请求摄像头任务"
        self.camera_sub = "取得任务完成信息"
        self.cam_task_info = "任务反馈"
        
        # 任务线程
        self.task_proc = self.create_timer(0.01,self.task_callback)
        
    def camera_task(self,task):
        if self.cam_task_info == "任务未完成":
            self.camera_pub = "任务类型"
        else:
            self.cam_task_info = "任务信息"
        set_step
        
    def task_callback(self):
        # 出站
        if not step01:
            self.get_pose("道路","北方")
        # 移动到二维码区
        if not step01 and step02:
            self.get_pose("二维码","北方")
        # 调摄像头扫码
        if not step01 and step02 and step03:
            self.camera_task("二维码扫描")
        # 移动到转盘区域
        if not step01 and step02 and step03 and step04:
            self.get_pose("转盘")
        # 调摄像头获取物块
        if not step01 and step02 and step03 and step04 and step05:
            self.camera_task("拾取物块")
        # 移动到单层放置区
        
        # 调摄像头放置物块
        # 移动到转盘区域
        # 调摄像头获取物块

    def odom_callback(self,odom_msg):
        self.now_pose = odom_msg.pose.pose
        self.start_task()
        
def main():
    rclpy.init()
    carbot_plan = Carbot_Plan("carbot_plan")
    rclpy.spin(carbot_plan)
    carbot_plan.destroy_node()
    rclpy.shutdown()