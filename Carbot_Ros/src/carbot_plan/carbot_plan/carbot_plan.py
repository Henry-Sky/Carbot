#!/usr/bin/env python
# encoding: utf-8

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry
from rclpy.callback_groups import ReentrantCallbackGroup
from carbot_interfaces.msg import Camreq
from carbot_interfaces.msg import Camfed
import tf_transformations

class Carbot_Plan(Node):
    def __init__(self,name):
        super().__init__(name)

        self.twist_pub = self.create_publisher(Twist,"twist_cmd",2)
        
        self.now_pose = Pose()
        self.heading = 0.0
        # 位置反馈
        self.odom_sub = self.create_subscription(Odometry,"odom_data",
                                                 self.odom_callback,2,callback_group=ReentrantCallbackGroup())
        
        self.stop_buffer = 0.02
        self.turn_buffer = 0.001
        self.reducer_buffer = 0.2
        self.cruising_speed = 0.2
        self.reducer_speed = 0.1
        self.turn_speed = 0.06
            
        # 摄像头调取
        self.cam_pub = self.create_publisher(Camreq,"cam_req",5)
        self.cam_sub = self.create_subscription(Camfed,"cam_fed",
                                                self.camfed_callback,1)
        # self.cam_status = Camfed()
        
        # (name,status,active,func)
        self.task_list = [
            ["move_out",False,False,self.task_moveout],
            ["move_qr",False,False,self.task_moveqr],
            ["qrcode_scan",False,False,self.task_qrscan],
            ["move_plt",False,False,self.task_moveplt],
            
        ]

        # ["object_pick",False,False,self.task_objpick],
        
        # 任务线程
        self.task_name = " "
        self.task_proc = self.create_timer(0.01,
                                           self.task_callback,callback_group=ReentrantCallbackGroup())  
        self.activate = True
        # self.activate_sub = self.create_subscription(Bool,"activate",self.activate_callback,5)

    def task_movergh(self):
        pose = Pose()
        pose.position.x = 1.8
        pose.position.y = 1.0
        return self.go_navigation(pose,self.heading)

    def task_objpick(self):
        camreq = Camreq()
        camreq.task_name = self.task_name
        self.cam_pub.publish(camreq)
        return False       

    def task_moveplt(self):
        pose = Pose()
        pose.position.x = 1.5
        pose.position.y = 0.20
        return self.go_navigation(pose,self.heading)
    
    def task_qrscan(self):
        camreq = Camreq()
        camreq.task_name = self.task_name
        self.cam_pub.publish(camreq)
        return False
        
    def task_moveqr(self):
        pose = Pose()
        pose.position.x = 0.6
        pose.position.y = 0.20
        return self.go_navigation(pose,self.heading)
    
    def task_moveout(self):
        now_y = self.now_pose.position.y
        twist = Twist()
        if (now_y > 0.20 - self.stop_buffer
            and now_y < 0.20 + self.stop_buffer):
            twist.linear.y = 0.0
            self.twist_pub.publish(twist)
            return True
        elif now_y < 0.22 - self.stop_buffer:
            twist.linear.y = self.reducer_speed
        else:
            twist.linear.y = -self.reducer_speed
        self.twist_pub.publish(twist)
        return False   

    def activate_callback(self,act_msg):
        self.activate = act_msg.data
                
    def task_callback(self):
        if self.activate:
            # 检查已完成的任务,取消激活状态
            # 获取一个未完成的任务，并激活该任务
            for task in self.task_list:
                if task[1]:
                    task[2] = False
                else:
                    self.task_name = task[0]
                    if not task[2]:
                        task[2] = True
                        self.get_logger().info("激活任务:"+str(task[0]))
                        self.twist_pub.publish(Twist())
                    break
        if self.now_pose == Pose():
            self.twist_pub.publish(Twist())
        else:
            pass
    
    def go_navigation(self, goal_pose, goal_heading):
        # 坐标获取
        goal_x = goal_pose.position.x
        goal_y = goal_pose.position.y
        now_x = self.now_pose.position.x
        now_y = self.now_pose.position.y
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
            if (goal_y - self.stop_buffer < now_y
                and goal_y + self.stop_buffer > now_y):
                # 停车，导航结束
                nav_twist.linear.x = 0.0
                nav_twist.linear.y = 0.0
                nav_twist.angular.z = 0.0
                self.twist_pub.publish(nav_twist)
                return True
            # 目标在西方(y+)
            elif goal_y - self.stop_buffer > now_y:
                # 面向西方
                if (self.heading > west - self.turn_buffer 
                    and self.heading < west + self.turn_buffer):
                    # 距离过远
                    if (goal_y + self.reducer_buffer < now_y):
                        nav_twist.linear.x = self.cruising_speed
                    # 接近目标
                    else:
                        nav_twist.linear.x = self.reducer_speed
                # 需要左转
                elif (self.heading >= east
                      and self.heading < west):
                    nav_twist.angular.z = self.turn_speed
                # 需要右转
                else:
                    nav_twist.angular.z = -self.turn_speed
            # 目标在东方(y-)
            else:
                # 面向东方
                if (self.heading > east - self.turn_buffer 
                    and self.heading < east + self.turn_buffer):
                    # 距离过远
                    if (now_y < goal_y - self.reducer_buffer):
                        nav_twist.linear.x = self.cruising_speed
                    # 接近目标
                    else:
                        nav_twist.linear.x = self.reducer_speed
                # 需要右转
                elif (self.heading > east
                      and self.heading < west):
                    nav_twist.angular.z = -self.turn_speed
                # 需要左转
                else:
                    nav_twist.angular.z = self.turn_speed
        # 目标在北方(x+)
        elif (goal_x - self.stop_buffer > now_x):
            # 面向北方
            if (self.heading > north - self.turn_buffer 
                and self.heading < north + self.turn_buffer):
                # 距离过远
                if (goal_x - self.reducer_buffer > now_x):
                    nav_twist.linear.x = self.cruising_speed
                # 接近目标
                else:
                    nav_twist.linear.x = self.reducer_speed
                    
            # 需要右转
            elif (self.heading > north
                  and self.heading < south):
                nav_twist.angular.z = -self.turn_speed
            # 需要左转
            else:
                nav_twist.angular.z = self.turn_speed
        # 目标在南方(x-)
        else:
            # 面向南方
            if (self.heading > south - self.turn_buffer 
                and self.heading < south + self.turn_buffer):
                # 距离过远
                if (goal_x + self.stop_buffer < now_x):
                    nav_twist.linear.x = self.cruising_speed
                # 接近目标
                else:
                    nav_twist.linear.x = self.reducer_speed
            # 需要左转
            elif(self.heading > north 
                 and self.heading < south):
                nav_twist.angular.z = self.turn_speed
            # 需要右转
            else:
                nav_twist.angular.z = -self.turn_speed
        self.twist_pub.publish(nav_twist)
        return False
       
    def camfed_callback(self,fed_msg):
        task_name = fed_msg.task_name
        task_status = fed_msg.task_status
        # 相机任务反馈
        for task in self.task_list:
            if (task_name == self.task_name 
                and task_name == task[0]):
                task[1] = task_status
                if task[1]:
                    task[2] = False
                break
            
    def odom_callback(self,odom_msg):
        self.now_pose = odom_msg.pose.pose
        self.heading = self.heading_update(self.now_pose)
        for task in self.task_list:
            if task[2]:
                task[1] = task[3]()
                if task[1]:
                    task[2] = False
                    self.get_logger().info(task[0] + "完成!"+
                                           "("+str(self.now_pose.position.x)+","+str(self.now_pose.position.y)+")")
                
    def heading_update(self,now_pose):
        quaternion = now_pose.orientation
        roll, pitch, yaw = tf_transformations.euler_from_quaternion(
            [quaternion.x,quaternion.y,quaternion.z,quaternion.w])
        if yaw >= 0:
            yaw %= 6.28
        else:
            yaw %= -6.28
        # (-6.28, +6.28)
        if yaw > 3.14:
            yaw -= 6.28
        elif yaw <= -3.14:
            yaw += 6.28
        else:
            pass
        # (-3.14, +3.14]
        return yaw
        
def main():
    rclpy.init()
    carbot_plan = Carbot_Plan("carbot_plan")
    rclpy.spin(carbot_plan)
    carbot_plan.destroy_node()
    rclpy.shutdown()