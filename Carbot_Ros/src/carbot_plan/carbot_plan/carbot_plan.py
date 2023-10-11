#!/usr/bin/env python
# encoding: utf-8

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose
from std_msgs.msg import Int64MultiArray
from carbot_interfaces.srv import Margin
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
        self.margin_cli = self.create_client(Margin,"margin_server")
        # 初始化
        self.margin_req = Margin.Request()
        
    def publish_pose(self,pose):
        self.pose_pub.publish(pose)

    def publish_arm(self,arm):
        self.arm_pub.publish(arm)

    def publish_twist(self,twist):
        self.twist_pub.publish(twist)
        
    def set_margin(self,upcolor,downcolor,aim_pix,flag):
        self.margin_req.upcolor = upcolor
        self.margin_req.downcolor = downcolor
        self.margin_req.aim_dis = aim_pix
        self.margin_req.send_flag = flag
        self.future = self.margin_cli.call_async(self.margin_req)
        
def set_margin(carbot_plan,upcolor,downcolor,aim_pix,flag):
    carbot_plan.set_margin(upcolor,downcolor,aim_pix,flag)
    while rclpy.ok():
        rclpy.spin_once(carbot_plan)
        if carbot_plan.future.done():
            response = carbot_plan.future.result()
            if not response.get_flag:
                continue
            break
        
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
    
    set_margin(carbot_plan,"gray","blue",120,True)
    
        
    
    
    # Task02 : 扫描二维码，获取任务
    # Task03 : 粗加工区拾取货物
    # Task04 : 
    carbot_plan.destroy_node()
    rclpy.shutdown()

main()