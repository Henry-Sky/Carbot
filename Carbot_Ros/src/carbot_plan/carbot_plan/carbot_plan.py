#!/usr/bin/env python
# encoding: utf-8

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry
from std_msgs.msg import Int64MultiArray
from carbot_interfaces.msg import Aimtask
from carbot_interfaces.msg import Eyestask

class Carbot_Plan(Node):
    def __init__(self,name):
        super().__init__(name)
        # 位置控制
        self.twist_pub = self.create_publisher(Twist,"twist_cmd",2)
        self.pose_pub = self.create_publisher(Pose,"navi_pose",2)
        self.odom_sub = self.create_subscription(Odometry,"odom_data",self.odom_callback,1)
        self.now_pose = Pose()
        # 机械臂控制
        self.arm_pub = self.create_publisher(Int64MultiArray,"arm_cmd",2)
        # 摄像头获取
        self.eyes_pub = self.create_publisher(Eyestask,"eyes_task",2)
        self.aim_sub = self.create_subscription(Aimtask,"aim_task",self.aim_callback,1)
        self.aimeye_name = " "
        self.aimeye_info = " "
        self.aim_x
        self.aim_y
        
        # 初始化
        self.task_list = [
            [False,self.start_task],
            "scan_task",
            "pick_task",
            "short_place_task",
            "long_place_task"
        ]
        self.task_timers = self.create_timer(0.001,self.task_callback)
        
    def task_callback(self):
        if self.task_list[0][0] != True:
            self.task_list[0][1]
    
    def start_task(self):
        now_x = self.now_pose.position.x
        now_y = self.now_pose.position.y
        twist = Twist()
        # 离开启停区
        get_ready = False
        if not get_ready:
            if now_y < 0.1:
                twist.linear.y = 0.1
            elif now_y > 0.2:
                twist.linear.y = -0.1
            else:
                twist.linear.y = 0.0
                get_ready = True
        # 导航二维码前
        aim_pose = Pose()
        aim_pose.position.x = 0.4
        aim_pose.position.y = 0.15
        self.pose_pub.publish(aim_pose)
        
    
    
    def eyes_publish(self, name, info):
        eyes_msg = Eyestask()
        eyes_msg.task_name = name
        eyes_msg.task_info = info
        while self.aimeye_name != name:
            self.eyes_pub.publish(eyes_msg)
            
    def odom_callback(self,odom_msg):
        self.now_pose = odom_msg.pose.pose
    
    def aim_callback(self,aim_msg):
        self.aimeye_name = aim_msg.task_name
        self.aimeye_info = aim_msg.task_info
        self.aim_x = aim_msg.aim_x
        self.aim_y = aim_msg.aim_y
        
               
        
        
        
def main():
    rclpy.init()
    carbot_plan = Carbot_Plan()
    rclpy.spin(carbot_plan)
    carbot_plan.destroy_node()
    rclpy.shutdown()