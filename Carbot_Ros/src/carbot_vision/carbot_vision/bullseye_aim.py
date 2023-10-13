import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from std_msgs.msg import Int64MultiArray
from carbot_interfaces.msg import Camreq
from carbot_interfaces.msg import Camfed


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
        self.twist_pub = self.create_publisher(Twist,"twist_cmd",2)
        self.arm_pub = self.create_publisher(Int64MultiArray,"arm_cmd",2)
        self.aim_sub = self.create_subscription(Camreq,"cam_req",self.aim_callback,1)
        self.aim_pub = self.create_publisher(Camfed,"cam_fed",2)
        self.task_name = None
        self.start_aim_task = False
        self.codeinfo = " "
        # 任务列表
        # task1 : 扫描二维码
        # task2 : 抓取转盘物块
        # task3 : 识别靶心放置物块
        # (name,status,active,func)
        self.task_list = [
            ["qrcode_scan",False,False,self.task_qrscan],
            ["object_pick",False,False,self.task_objpick],
        ]
        
    def aim_callback(self,task_msg):
        # 检索请求
        self.task_name = task_msg.task_name
        for task in self.task_list:
            # 对于请求任务
            if task[0] == self.task_name:
                if task[1] == False:
                    # 激活任务
                    task[2] = True
                cam_fed = Camfed()
                cam_fed.task_name = self.task_name
                cam_fed.task_status = task[1]
                self.aim_pub.publish(cam_fed)
                break
            # 对于非请求任务
            else:
                if task[1]:
                    task[2] = False
                else:
                    pass
        
    def image_callback(self,img_msg):
        img = CvBridge().imgmsg_to_cv2(img_msg)
        # 更新任务目标
        for task in self.task_list:
            if (task[0] == self.task_name 
                and task[2] == True):
                # 执行任务
                task[1] = task[3](img)
                task[2] = not task[1]
        
    # task1 : 扫描二维码    
    def task_qrscan(self,img):
        arm_cmd = Int64MultiArray()
        # 设置机械臂
        # self.arm_pub.publish(arm_cmd)
        codeinfo = self.get_code(img)
        if len(codeinfo) != 0:
            self.codeinfo = codeinfo
            self.get_logger().info("获取二维码:"+str(self.codeinfo))
            return True
        else:
            return False
        
    def task_objpick(self,img):
        return False
        
    def get_control(self,coord):
        now_x = coord[0]
        now_y = coord[1]
        twist = Twist()
        if (now_x > self.aim_x - self.buffer
            and now_x < self.aim_x + self.buffer):
            if (now_x > self.aim_y - self.buffer
                and now_y < self.aim_y + self.buffer):
                    twist = Twist()
                    self.twist_pub.publish(twist)
                    return True
        else:
            return False       
            
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
        
    def get_code(self,img):
        # 识别二维码，返回二维码信息
        codeinfo, points, straight_qrcode = cv2.QRCodeDetector().detectAndDecode(img)
        if codeinfo is not None:
            return codeinfo
        else:
            return None
        
def main():
    rclpy.init()
    bullseye_aim = Bullseye_Aim("bullseye_aim")
    rclpy.spin(bullseye_aim)
    bullseye_aim.destroy_node()
    rclpy.shutdown()