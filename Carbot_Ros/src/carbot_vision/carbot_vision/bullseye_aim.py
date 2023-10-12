import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from carbot_interfaces.msg import Aimtask
from carbot_interfaces.msg import Eyestask


# HSV格式色表
color_dist = {'red': {'Lower': np.array([0, 60, 60]), 'Upper': np.array([6, 255, 255])},
              'gray': {'Lower': np.array([0, 0, 46]), 'Upper': np.array([180, 43, 220])},
              'yellow': {'Lower': np.array([26, 43, 46]), 'Upper': np.array([34, 255, 255])},          
              'blue': {'Lower': np.array([100, 80, 46]), 'Upper': np.array([124, 255, 255])},
              'green': {'Lower': np.array([35, 43, 35]), 'Upper': np.array([90, 255, 255])},}

class Bullseye_Aim(Node):
    def __init__(self,name):
        super().__init__(name)
        # 参数声明
        self.declare_parameter("aim_x",320)
        self.declare_parameter("aim_y",240)
        self.declare_parameter("buffer",8)
        
        self.aim_x = self.get_parameter("aim_x").get_parameter_value().integer_value
        self.aim_y = self.get_parameter("aim_y").get_parameter_value().integer_value
        self.buffer = self.get_parameter("buffer").get_parameter_value().integer_value
        
        # 消息创建
        self.image_sub = self.create_subscription(Image,"image_raw",self.image_callback,1)
        self.aim_pub = self.create_publisher(Aimtask,"aim_task")
        self.eyes_sub = self.create_subscription(Eyestask,"eyes_task",self.eyes_callback,1)
        # 初始化
        self.aim_name = " "
        self.aim_info = " "
        
    def eyes_callback(self,eyes_msg):
        self.aim_name = eyes_msg.task_name
        self.aim_info = eyes_msg.task_info
        
        
    def image_callback(self,img_msg):
        img = CvBridge().imgmsg_to_cv2(img_msg)
        aimtask = Aimtask()
        aimtask = self.aim_name
        # 更新任务目标
        if self.aim_name == "bullseye":
            if self.get_circle(img) is not None:
                coord = self.get_circle
                aimtask.aim_x = coord[0]
                aimtask.aim_y = coord[1]
                self.aim_pub.publish(aimtask)
            else:
                pass
        elif self.aim_name == "object":
            if self.get_object(img) is not None:
                coord = self.get_circle
                aimtask.aim_x = coord[0]
                aimtask.aim_y = coord[1]
                self.aim_pub.publish(aimtask)
            else:
                pass
        else:
            pass
            
            
    def get_object(self,img):
        # 识别物体，返回物体坐标，或None
        hsv_img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        inRange_hsv = cv2.inRange(hsv_img,
                                  color_dist[self.aim_info]["Lower"], 
                                  color_dist[self.aim_info]["Upper"]
                                  )
        inRange_bgr = cv2.cvtColor(inRange_hsv,cv2.COLOR_HSV2BGR)
        inRange_gray = cv2.cvtColor(inRange_bgr,cv2.COLOR_BGR2GRAY)
        img_edges = cv2.Canny(inRange_gray,50,150,apertureSize = 3)
        circles = cv2.HoughCircles(img_edges, cv2.HOUGH_GRADIENT, 1, 50,
                         param1=100, param2=80, minRadius=50, maxRadius=200)
        if circles is not None:
            aim_coord = circles[0][0]
            return aim_coord
        else:
            return None
            
            
    def get_circle(self,img):
        # 识别靶心，返回靶心坐标，或None
        hsv_img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        inRange_hsv = cv2.inRange(hsv_img,
                                  color_dist[self.aim_info]["Lower"], 
                                  color_dist[self.aim_info]["Upper"]
                                  )
        circles = cv2.HoughCircles(inRange_hsv, cv2.HOUGH_GRADIENT, 1, 50,
                         param1=100, param2=80, minRadius=60, maxRadius=200)
        if circles is not None:
            aim_coord = circles[0][0]
            return aim_coord
        else:
            return None