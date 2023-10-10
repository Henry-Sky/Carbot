#!/usr/bin/env python
# encoding: utf-8

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose

class Carbot_Plan(Node):
    def __init__(self,name):
        super().__init__(name)
        self.pose_pub = self.create_publisher(Pose,"navi_pose",10)
        self.navi_req = Navipose.Request()
        
        
    def publish_pose(self,pose):
        self.pose_pub.publish(pose)
        
def main():
    rclpy.init()
    
    carbot_plan = Carbot_Plan("carbot_plan")

    the_pose = Pose()
    the_pose.position.x = 3.0
    the_pose.position.y = 3.0
    carbot_plan.publish_pose(the_pose)
    print("已经发送位置")
    
    carbot_plan.destroy_node()
    rclpy.shutdown()

main()