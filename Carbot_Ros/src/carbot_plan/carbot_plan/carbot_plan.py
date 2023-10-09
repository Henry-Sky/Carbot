#!/usr/bin/env python
# encoding: utf-8

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose

class Carbot_Plan(Node):
    def __init__(self,name):
        super().__init__(name)
        self.pose_pub = self.create_publisher(Pose,"set_pose",2)
        
    def publish_pose(self,pose):
        self.pose_pub.publish(pose)
        
def main():
    rclpy.init()
    
    carbot_plan = Carbot_Plan("carbot_plan")
    pose = Pose()
    pose.position.x = 2.0
    carbot_plan.publish_pose(pose)
    
    carbot_plan.destroy_node()
    rclpy.shutdown()

main()