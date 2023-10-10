import rclpy
import cv2
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import String

class Scan_Qrcode(Node):
    def __init__(self,name):
        super().__init__(name)

        # 创建转换桥(Ros与Opencv)
        self.bridge = CvBridge()
        # 创建服务节点
        self.image_sub = self.create_subscription(Image, 'image_raw', self.image_callback, 1)
        self.task_pub = self.create_publisher(String, 'task_info', 10)

    def image_callback(self,image):
        img = CvBridge().imgmsg_to_cv2(image)
        codeinfo, points, straight_qrcode = cv2.QRCodeDetector().detectAndDecode(img)
        task = String()
        if len(codeinfo) != 0:
            task.data = codeinfo
            self.task_pub.publish(task)
        else:
            task.data = "none"
            self.task_pub.publish(task)
        
def main():
    rclpy.init()
    scan_qrcode = Scan_Qrcode("scan_qrcode")
    rclpy.spin(scan_qrcode)
    rclpy.shutdown()