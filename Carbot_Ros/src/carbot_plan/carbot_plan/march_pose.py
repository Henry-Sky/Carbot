#!/usr/bin/env python
# encoding: utf-8

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry
import tf_transformations


class March_Pose(Node):
    def __init__(self,name):
        super().__init__(name)
        # 订阅/发布
        self.pose_sub = self.create_subscription(Pose,"set_pose",self.march_callback,1)
        self.odom_sub = self.create_subscription(Odometry,"odom_data",self.odom_callback,1)
        self.twist_pub = self.create_publisher(Twist,"twist_cmd",2)
        # 参数
        self.declare_parameter("reducer_buffer", 0.2)
        self.reducer_buffer = self.get_parameter("reducer_buffer").get_parameter_value().double_value
        self.declare_parameter("stop_buffer", 0.05)
        self.stop_buffer = self.get_parameter("stop_buffer").get_parameter_value().double_value
        self.declare_parameter("turn_buffer", 0.1)
        self.turn_buffer = self.get_parameter("turn_buffer").get_parameter_value().double_value
        self.declare_parameter("cruising_speed", 0.2)
        self.cruising_speed = self.get_parameter("cruising_speed").get_parameter_value().double_value
        self.declare_parameter("reducer_speed", 0.1)
        self.reducer_speed = self.get_parameter("reducer_speed").get_parameter_value().double_value
        self.declare_parameter("turn_speed", 0.05)
        self.turn_speed = self.get_parameter("turn_speed").get_parameter_value().double_value
        
        # 位置初始化
        self.now_pose = Pose()
        self.goal_pose = Pose()
        # 标志
        self.twist_start = True
        self.navigation_start = False
        # 初始化
        self.heading_yaw = 0.0 # 弧度 (-3.14, +3.14]
        
    def twist_init(self):
        if self.twist_start:
            self.twist_pub.publish(Twist())
            self.twist_start = False
            
    def go_navigation(self, goal_pose, now_pose):
        # 坐标获取
        goal_x = goal_pose.position.x
        goal_y = goal_pose.position.y
        now_x = now_pose.position.x
        now_y = now_pose.position.y
        # 方向定义
        north = 0.0
        south = 3.14
        west = 1.57
        east = -1.57
        # 消息初始化
        nav_twist = Twist()
        
        # 导航决策 (x轴坐标优先，转向，y轴坐标)
        # x轴坐标满足
        if (goal_x - self.stop_buffer < now_x
            and goal_x + self.stop_buffer > now_x):
            # y轴坐标满足
            if (goal_y -self.stop_buffer < now_y
                and goal_y +self.stop_buffer > now_y):
                # 停车，导航结束
                nav_twist.linear.x = 0.0
                nav_twist.linear.y = 0.0
                nav_twist.angular.z = 0.0
                self.navigation_start = False
            # 目标在西方(y+)
            elif goal_y + self.stop_buffer < now_y:
                # 面向西方
                if (self.heading_yaw > west - self.turn_buffer 
                    and self.heading_yaw < west + self.turn_buffer):
                    # 距离过远
                    if (goal_y + self.reducer_buffer < now_y):
                        nav_twist.linear.x = self.cruising_speed
                    # 接近目标
                    else:
                        nav_twist.linear.x = self.reducer_speed
                # 需要左转
                elif (self.heading_yaw >= east
                      and self.heading_yaw < west):
                    nav_twist.angular.z = self.turn_speed
                # 需要右转
                else:
                    nav_twist.angular.z = -self.turn_speed
            # 目标在东方(y-)
            else:
                # 面向东方
                if (self.heading_yaw > east - self.turn_buffer 
                    and self.heading_yaw < east + self.turn_buffer):
                    # 距离过远
                    if (now_y < goal_y - self.reducer_buffer):
                        nav_twist.linear.x = self.cruising_speed
                    # 接近目标
                    else:
                        nav_twist.linear.x = self.reducer_speed
                # 需要右转
                elif (self.heading_yaw > east
                      and self.heading_yaw < west):
                    nav_twist.angular.z = -self.turn_speed
                # 需要左转
                else:
                    nav_twist.angular.z = self.turn_speed
        # 目标在北方(x+)
        elif (goal_x - self.stop_buffer > now_x):
            # 面向北方
            if (self.heading_yaw > north - self.turn_buffer 
                and self.heading_yaw < north + self.turn_buffer):
                # 距离过远
                if (goal_x - self.reducer_buffer > now_x):
                    nav_twist.linear.x = self.cruising_speed
                # 接近目标
                else:
                    nav_twist.linear.x = self.reducer_speed
            # 需要右转
            elif (self.heading_yaw > north
                  and self.heading_yaw < south):
                nav_twist.angular.z = -self.turn_speed
            # 需要左转
            else:
                nav_twist.angular.z = self.turn_speed
        # 目标在南方(x-)
        else:
            # 面向南方
            if (self.heading_yaw > south - self.turn_buffer 
                and self.heading_yaw < south + self.turn_buffer):
                # 距离过远
                if (goal_x + self.stop_buffer < now_x):
                    nav_twist.linear.x = self.cruising_speed
                # 接近目标
                else:
                    nav_twist.linear.x = self.reducer_speed
            # 需要左转
            elif(self.heading_yaw > north 
                 and self.heading_yaw < south):
                nav_twist.angular.z = self.turn_speed
            # 需要右转
            else:
                nav_twist.angular.z = -self.turn_speed
        self.twist_pub.publish(nav_twist)                
    
    def heading_update(self,now_pose):
        q = nowpose.orientation
        roll, pitch, yaw = tf_transformations.euler_from_quaternion([q.x,q.y,q.z,q.w])
        if yaw >= 0:
            yaw = yaw % 6.28
        else:
            yaw = yaw % -6.28
        # (-6.28,6.28)
        if yaw > 3.14:
            yaw -= 6.28
        elif yaw <= -3.14:
            yaw += 6.28
        else:
            self.get_logger().info("Error in heading_update")
            
        self.heading_yaw = yaw

    def odom_callback(self,odom_msg):
        self.now_pose = odom_msg.pose.pose
        self.heading_update(self.now_pose)
        # 由于要监测odom，故在odom回调中导航
        if self.navigation_start:
            self.go_navigation(self.goal_pose, self.now_pose)
        else:
            pass
        
    def march_callback(self,pose_msg):
        self.goal_pose = pose_msg
        self.navigation_start = True
        
def main():
    rclpy.init()
    march_pose = March_Pose("march_pose")
    rclpy.spin(march_pose)
    march_pose.destroy_node()
    rclpy.shutdown()    