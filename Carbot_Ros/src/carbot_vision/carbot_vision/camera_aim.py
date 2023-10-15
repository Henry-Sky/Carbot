import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from rclpy.callback_groups import ReentrantCallbackGroup
from carbot_interfaces.msg import Camreq
from carbot_interfaces.msg import Camfed
from std_msgs.msg import String
from carbot_lib import Carbot
import time

# 摄像头内参
mtx = np.array([[1.35635605e+03, 0.00000000e+00, 6.46212314e+02],
       [0.00000000e+00, 1.35578874e+03, 3.78009469e+02],
       [0.00000000e+00, 0.00000000e+00, 1.00000000e+00]])

dist = np.array([[-2.52464877e-01, 1.92828476e-01,
                  -6.55508668e-04, 2.48903668e-03,
                  -7.45335496e-01]])


# HSV格式色表
color_dist = {'red': {'Lower': np.array([0, 60, 60]), 'Upper': np.array([6, 255, 255])},
              'gray': {'Lower': np.array([0, 0, 46]), 'Upper': np.array([180, 43, 220])},
              'yellow': {'Lower': np.array([26, 43, 46]), 'Upper': np.array([34, 255, 255])},          
              'blue': {'Lower': np.array([100, 80, 46]), 'Upper': np.array([124, 255, 255])},
              'green': {'Lower': np.array([35, 43, 35]), 'Upper': np.array([90, 255, 255])},
              'none': {'Lower': np.array([0, 0, 0]), 'Upper': np.array([255, 255, 255])},}

class Camera_Aim(Node):
    def __init__(self,name):
        super().__init__(name)
        # 参数声明
        self.cam_id = 0
        self.img = None
        
        # 消息创建
        self.twist_pub = self.create_publisher(Twist,"twist_cmd",2)
        
        self.cam_sub = self.create_subscription(Camreq,"cam_req",self.cam_callback,1)
        self.cam_pub = self.create_publisher(Camfed,"cam_fed",5)
        
        self.code_pub = self.create_publisher(String,"code_info",10)
        
        self.task_name = None
        self.codeinfo = "等待任务获取"
        self.color_coord_list = ["none"]
        self.task_proc = self.create_timer(0.01,
                                           self.task_callback,
                                           callback_group=ReentrantCallbackGroup()) 
        self.car = Carbot()
        
        # 任务列表
        # (name,status,active,func)
        self.task_list = [
            ["qrcode_scan",False,False,self.task_qrscan],
            ["object_pick",False,False,self.task_objpick],
        ]
        self.car = Carbot()
        
    # task1 : 扫描二维码    
    def task_qrscan(self,img):
        # 设置机械臂
        #self.car.set_uart_servo_angle_array([])
        codeinfo = self.get_code(img)
        
        if len(codeinfo) != 0:
            self.codeinfo = codeinfo
            code_pub = String()
            code_pub.data = codeinfo
            self.code_pub.publish(code_pub)
            self.get_logger().info("获取二维码:" + str(self.codeinfo))
            return True
        else:
            return False
    
    # task2 : 识别并抓取物块
    def task_objpick(self,img):
        # 识别一次颜色切换
        now_color = None
        now_coord = None
        for color in ["red","green","blue"]:
            obj_coord = self.get_object(img)
            if obj_coord is not None:
                now_color = color
                now_coord = obj_coord
                break
        last_coord =  self.color_coord_list[len(self.color_coord_list)-1]
        self.color_coord_list.append(now_coord)
        if now_coord == last_coord:
            pass
        # 校准机械臂位置
        # 按照任务信息操作
            # 再次识别颜色切换(符合当前颜色)
            # 机械臂抓取
        return False
        
    def cam_callback(self,task_msg):
        # 检索请求
        self.task_name = task_msg.task_name
        for task in self.task_list:
            # 对于请求任务
            if (task[0] == self.task_name 
                and task[1] == False
                and task[2] == False):
                task[2] == True
                break
        
    def task_callback(self):
        # 更新任务目标
        for task in self.task_list:
            if (task[0] == self.task_name 
                and task[2] == True):
                # 执行任务
                img = self.get_img()
                task[1] = task[3](img)
                task[2] = not task[1]
                cam_fed = Camfed()
                cam_fed.task_name = self.task_name
                cam_fed.task_status = task[1]
                self.cam_pub(cam_fed)
                break
                
    def get_img(self):
        ret, frame = self.cap.read(self.cam_id)
        if ret:
            dst = cv2.undistort(frame, mtx, dist, None, mtx)
            return dst
        else:
            self.get_logger().info("camera error!")           
            
    def get_object(self,img,color = "none"):
        # 识别物体，返回物体坐标，或None
        hsv_img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        inRange_hsv = cv2.inRange(hsv_img,
                            color_dist[color]["Lower"], 
                            color_dist[color]["Upper"]
                            )
        contours, hierarchy = cv2.findContours(inRange_hsv, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
        if len(contours) > 0:
            obj_contour = []
            for contour in contours:
                if len(contour) > len(obj_contour):
                    obj_contour = contour
            if len(obj_contour) > 800:
                x, y, w, h = cv2.boundingRect(obj_contour)
                obj_coord = [x,y]
                return obj_coord
            else:
                return None
        else:
            return None
                        
    def get_circle(self,img,color = "none"):
        # 识别靶心，返回靶心坐标，或None
        hsv_img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        inRange_hsv = cv2.inRange(hsv_img,
                                  color_dist[color]["Lower"], 
                                  color_dist[color]["Upper"]
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
    camera_aim = Camera_Aim("camera_aim")
    rclpy.spin(camera_aim)
    camera_aim.destroy_node()
    rclpy.shutdown()
