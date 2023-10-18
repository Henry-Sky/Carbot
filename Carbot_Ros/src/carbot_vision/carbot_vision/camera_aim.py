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
              'blue': {'Lower': np.array([108, 88, 58]), 'Upper': np.array([118, 255, 255])},
              'green': {'Lower': np.array([90, 50, 45]), 'Upper': np.array([100, 255, 255])},
              'none': {'Lower': np.array([0, 0, 0]), 'Upper': np.array([255, 255, 255])},}

class Camera_Aim(Node):
    def __init__(self,name):
        super().__init__(name)
        # 参数声明
        self.cam_id = 0
        self.img = None

        self.car = Carbot()
        
        # 消息创建
        self.twist_pub = self.create_publisher(Twist,"twist_cmd",2)
        self.img_pub = self.create_publisher(Image, "image_raw", 10)
        
        self.cam_sub = self.create_subscription(Camreq,"cam_req",self.cam_callback,1)
        self.cam_pub = self.create_publisher(Camfed,"cam_fed",5)
        
        self.code_pub = self.create_publisher(String,"code_info",10)
        
        # 二维码任务(1:红 2:绿 3:蓝)
        self.waittimes = 0
        self.codeinfo = "等待任务获取"
        # 抓取任务
        self.color_list = []
        self.obj_code = 0

        # 任务调度
        self.task_proc = self.create_timer(0.001,
                                           self.task_callback,
                                           callback_group=ReentrantCallbackGroup()) 
        self.task_name = None
        # (name,status,active,func)
        self.task_list = [
            ["qrcode_scan",False,False,self.task_qrscan],
            ["object_pick",False,False,self.task_objpick],
            ["object_place", False, False, self.task_objplace]
        ]

    # task3 : 识别靶心并放置物块
    def task_objplace(self, img):
        pass

        
    # task2 : 识别并抓取物块
    def task_objpick(self, img):
        arm_obj_aim = [133, 108, 98]
        pwm_circle = [8,88,168]

        self.car.set_uart_servo_angle_array(arm_obj_aim)
        #识别颜色
        now_color = self.get_color(img)
        self.get_logger().info("目前颜色:" + str(now_color))
        if len(self.color_list) == 0:
            self.color_list.append(now_color)
            return False
        else:
            last_color = self.color_list.pop()
            self.color_list.append(now_color)
            rgb = ["red", "green", "blue"]
            aim_color = rgb[int(self.codeinfo[self.obj_code])-1]
            if (last_color != now_color 
            and now_color == aim_color):
                time.sleep(1)
                self.pick_from_plt(pwm_circle[self.obj_code])
                self.obj_code += 1
            if self.obj_code == 3:
                return True
        return False

    # task1 : 扫描二维码    
    def task_qrscan(self,img):
        # 设置机械臂
        self.car.set_pwm_servo(1, 120)
        arm_scan = [226, 126, 35]
        self.car.set_uart_servo_angle_array(arm_scan)
        codeinfo = self.get_code(img)
        twist = Twist()
        self.twist_pub.publish(twist)
        if len(codeinfo) != 0:
            self.codeinfo = codeinfo
            code_pub = String()
            code_pub.data = codeinfo
            self.code_pub.publish(code_pub)
            self.get_logger().info("获取二维码:" + str(self.codeinfo))
            return True
        else:
            self.waittimes += 1
            if self.waittimes % 5 == 0:
                twist.linear.x = 0.1
            elif self.waittimes % 9 == 0:
                twist.linear.x = -0.1
            else:
                twist.linear.x = 0.0
            self.twist_pub.publish(twist)
            return False


    def pick_from_plt(self, pwm_circle):
        # 死参数
        arm_obj_pick_pre = [176, 7, 170]
        arm_obj_pick =   [130, 7, 170]
        arm_obj_pick_back = [99, 269, 132]
        pwm_pick = 110
        pwm_free = 75

        self.car.set_pwm_servo(2, pwm_circle)
        self.car.set_pwm_servo(1, pwm_free)
        self.car.set_uart_servo_angle_array(arm_obj_pick_pre,500)
        time.sleep(1)
        self.car.set_uart_servo_angle_array(arm_obj_pick,1000)
        time.sleep(2)
        self.car.set_pwm_servo(1, pwm_pick)
        time.sleep(1)
        self.car.set_uart_servo_angle_array(arm_obj_pick_back,1000)
        time.sleep(2)
        self.car.set_pwm_servo(1, pwm_free)
        time.sleep(1)
        
    def cam_callback(self,task_msg):
        # 检索请求
        self.task_name = task_msg.task_name
        for task in self.task_list:
            # 对于请求任务
            if (task[0] == self.task_name 
                and task[1] == False
                and task[2] == False):
                task[2] = True
                break
        
    def task_callback(self):
        # 更新任务目标
        for task in self.task_list:
            if (task[0] == self.task_name 
                and task[2] == True):
                # 执行任务
                img = self.get_img()
                if img is not None:
                    task[1] = task[3](img)
                    task[2] = not task[1]
                cam_fed = Camfed()
                cam_fed.task_name = self.task_name
                cam_fed.task_status = task[1]
                self.cam_pub.publish(cam_fed)
                break        

    def get_color(self,img):
        tmp = 0
        hsv_img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        prb_color = "none"
        for color in ["red", "green", "blue"]:
            inRange_hsv = cv2.inRange(hsv_img,
                                    color_dist[color]["Lower"], 
                                    color_dist[color]["Upper"])
            circle = cv2.HoughCircles(inRange_hsv, cv2.HOUGH_GRADIENT, 3, 60,
                                param1=100, param2=75, minRadius=220, maxRadius=250)
            if circle is not None:
                return color
            else:
                if np.mean(inRange_hsv) > tmp:
                    tmp = np.mean(inRange_hsv)
                    print(tmp)
                    prb_color = color
        return prb_color
        
        image = Image()
        image = CvBridge().cv2_to_imgmsg(res_img)
        self.img_pub.publish(image)

        self.get_logger().info("目前坐标:"+str(now_x)+","+str(now_y))

        twist = Twist()
        aim_x = bia_coord[0]
        aim_y = bia_coord[1]
        # --begin--
        if (now_x > aim_x - buffer 
            and now_x < aim_x + buffer):
            if (now_y > aim_y - buffer
                and now_y < aim_y + buffer):
                self.twist_pub.publish(Twist())
                return True
            elif (now_y < aim_y - buffer):
                twist.linear.y = -speed
            else:
                twist.linear.y = speed
        elif (now_x < aim_x - buffer):
            twist.linear.x = -speed
        else:
            twist.linear.x = speed
        self.twist_pub.publish(twist)
        return False
        
    def get_img(self):
        cap = cv2.VideoCapture(self.cam_id)
        ret, frame = cap.read()
        if ret:
            dst = cv2.undistort(frame, mtx, dist, None, mtx)
            return dst
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
