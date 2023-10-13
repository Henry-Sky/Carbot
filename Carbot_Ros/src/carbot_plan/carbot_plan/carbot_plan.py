#!/usr/bin/env python
# encoding: utf-8

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry
from rclpy.callback_groups import ReentrantCallbackGroup
from carbot_interfaces.msg import Camreq
from carbot_interfaces.msg import Camfed

class Carbot_Plan(Node):
    def __init__(self,name):
        super().__init__(name)

        self.twist_pub = self.create_publisher(Twist,"twist_cmd",2)
        self.now_pose = Pose()
        # 位置反馈
        self.odom_sub = self.create_subscription(Odometry,"odom_data",self.odom_callback,2)
        # 摄像头调取
        self.cam_pub = self.create_publisher(Camreq,"cam_req",2)
        self.cam_sub = self.create_subscription(Camfed,"cam_fed",self.camfed_callback,1)
        self.cam_status = Camfed()
        
        # (name,status,active,func)
        self.task_list = [
            ["move_out",False,False,self.task_moveout],
            ["move_qr",False,False,self.task_moveqr]
        ]
        # ["qrcode_scan",False,False, ],
        #     ["object_pick",False,False, ],
        
        # 任务线程
        self.task_name = " "
        self.task_proc = self.create_timer(0.01,self.task_callback)        
        
        
    def task_moveout(self):
        now_x = self.now_pose.position.x
        now_y = self.now_pose.position.y
        twist = Twist()
        if now_y < 0.3:
            twist.linear.y = 0.1
            self.twist_pub.publish(twist)
            return False
        elif now_y > 0.35:
            twist.linear.y = -0.1
            self.twist_pub.publish(twist)
            return False
        else:
            twist.linear.y = 0.0
            self.twist_pub.publish(twist)
            return True
        
    def task_moveqr(self):
        return True
            
        
    def task_callback(self):
        # 获取一个未完成的任务，并激活该任务
        # 检查已完成的任务,取消激活状态
        for task in self.task_list:
            if task[1]:
                task[2] = False
            else:
                self.task_name = task[0]
                task[2] = True
                break
       
        
    def camfed_callback(self,fed_msg):
        task_name = fed_msg.task_name
        task_status = fed_msg.task_status
        # 相机任务反馈
        for task in self.task_list:
            if (task_name == self.task_name 
                and task_name == task[0]):
                task[1] = task_status
                break
            
        

    def odom_callback(self,odom_msg):
        self.now_pose = odom_msg.pose.pose
        for task in self.task_list:
            if task[2]:
                task[1] = task[3]()
        
def main():
    rclpy.init()
    carbot_plan = Carbot_Plan("carbot_plan")
    rclpy.spin(carbot_plan)
    carbot_plan.destroy_node()
    rclpy.shutdown()