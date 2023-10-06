import rclpy
import cv2
import numpy as np
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from carbot_camera.srv import Scan

mtx = np.array([[1.35635605e+03, 0.00000000e+00, 6.46212314e+02],
       [0.00000000e+00, 1.35578874e+03, 3.78009469e+02],
       [0.00000000e+00, 0.00000000e+00, 1.00000000e+00]])

dist = np.array([[-2.52464877e-01, 1.92828476e-01, -6.55508668e-04, 2.48903668e-03, -7.45335496e-01]])

class Scan_Server(Node):
    def __init__(self,name):
        super().__init__(name)
        # 参数配置
        self.declare_parameter("cap_id",0)
        self.cap_id = self.get_parameter("cap_id").get_parameter_value().integer_value
        self.declare_parameter("pub_img",False)
        self.pub_img = self.get_parameter("pub_img").get_parameter_value().bool_value
        # 摄像头获取
        self.cap = cv2.VideoCapture(self.cap_id)
        self.flag, self.frame = self.cap.read()
        self.dst = cv2.undistort(self.frame, mtx, dist, None, mtx)
        # 创建转换桥(Ros与Opencv)
        self.bridge = CvBridge()
        # 创建服务节点
        self.scan_srv = self.create_service(Scan, 'scan_task', self.reply_callback)
        self.img_pub = self.create_publisher(Image, "/image_raw", 2)
        # 回调函数更新图片
        self.timer = self.create_timer(0.001, self.timer_callback)

    def task_qr(self):
        codeinfo, points, straight_qrcode = cv2.QRCodeDetector().detectAndDecode(self.dst)
        return False, "none"
    
    def task_aim(self):
        return False, "none"

        
    def reply_callback(self,request,response):
        self.get_logger().info('scaning...')
        self.task = request.task
        if self.task == "task_qr":
            response.flag, response.info = self.task_qr()
        elif self.task == "task_aim":
            response.flag, response.info = self.task_aim()
        else:
            response.flag, response.info = False, "none"
        return response 
    
    def timer_callback(self):
        self.flag, self.frame = self.cap.read()
        self.dst = cv2.undistort(self.frame, mtx, dist, None, mtx)
        if self.flag and self.pub_img:
            self.msg = self.bridge.cv2_to_imgmsg(self.frame, encoding='passthrough')
            self.img_pub.publish(self.msg)

def main():
    rclpy.init()
    scan_server = Scan_Server()
    rclpy.spin(scan_server)
    rclpy.shutdown()