import rclpy
from rclpy.node import Node
import cv2
import numpy as np
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


# 靶心HSV色表
bull_color_dist = {'red': {'Lower': np.array([0, 35, 32]), 'Upper': np.array([17, 255, 255])},          
              'blue': {'Lower': np.array([95, 28, 35]), 'Upper': np.array([115, 255, 255])},
              'green': {'Lower': np.array([42, 35, 38]), 'Upper': np.array([80, 255, 255])},
              'none': {'Lower': np.array([0, 0, 0]), 'Upper': np.array([255, 255, 255])},}

# 物块HSV色表
obj_color_dist = {'red': {'Lower': np.array([0, 60, 60]), 'Upper': np.array([6, 255, 255])},          
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
        # 瞄准任务
        self.time_list = []
        self.coord_list = []
        self.aim_delay_flag = False
        # 抓取任务
        self.color_list = []
        self.obj_code = 0
        self.obj_pwm_list = []
        self.pick_delay_flag = False

        # 任务调度
        self.task_proc = self.create_timer(0.001,
                                           self.task_callback,
                                           callback_group=ReentrantCallbackGroup()) 
        self.task_name = None
        # (name,status,active,func)
        self.task_list = [
            ["qrcode_scan",False,False,self.task_qrscan],
            ["obj_aim", False, False,self.task_objaim],
            ["object_pick",False,False,self.task_objpick],
            ["bull_place", False, False, self.task_bullplace],
            ["bull_get", False, False, self.task_bullget]
        ]

    # task5 : 识别物体，回收
    def task_bullget(self):

        aim_coord = (150, 260)  # 靶心相对摄像头的偏移坐标
        pwm_circle = [8,88,168]

        color_list = []
        for i in range(3):
            index = self.codeinfo[i]
            if index == 1:
                color_list.append("red")
            elif index == 2:
                color_list.append("green")
            elif index == 3:
                color_list.append("blue")
            else:
                pass
        bull_index = 0
        while bull_index != 3:
            img = self.get_img()
            color = color_list[bull_index]
            res = self.get_obj_aim(img, color)
            if res is not None:
                flag = self.move_aim(res, aim_coord)
            if flag:
                self.pick_from_bull(pwm_circle[bull_index])
                bull_index += 1
        return True

    def pick_from_bull(self, pwm_circle):
        # 死参数
        arm_obj_pick_pre = [176, 7, 170]    # 预抓取机械臂 (待测)
        arm_obj_pick = [130, 7, 170]  # 抓取机械臂 (待测)
        arm_obj_pick_back = [99, 269, 132]  # 放回机械臂 (待测)
        pwm_pick = 110  # 机械臂抓取
        pwm_free = 50   # 机械臂释放

        self.car.set_pwm_servo(2, pwm_circle)
        self.car.set_pwm_servo(1, pwm_free)
        # 预抓取
        self.car.set_uart_servo_angle_array(arm_obj_pick_pre,500)
        time.sleep(1)
        # 抓取
        self.car.set_uart_servo_angle_array(arm_obj_pick,500)
        time.sleep(2)
        self.car.set_pwm_servo(1, pwm_pick)
        time.sleep(1)
        # 放回
        self.car.set_uart_servo_angle_array(arm_obj_pick_back,800)
        time.sleep(2)
        self.car.set_pwm_servo(1, pwm_free)
        time.sleep(1)


    # task4 : 识别靶心, 并放置物块
    def task_bullplace(self, img):
        self.car.set_uart_servo_angle_array(arm_obj_aim)
        time.sleep(1)
        arm_obj_aim = [133, 108, 98]    # 机械臂识别靶心的角度
        pwm_circle = [8,88,168]
        color_list = []
        aim_coord = (150, 260)  # 靶心相对摄像头的偏移坐标
        bull_index = 0
        for i in range(3):
            index = self.codeinfo[i]
            if index == 1:
                color_list.append("red")
            elif index == 2:
                color_list.append("green")
            elif index == 3:
                color_list.append("blue")
            else:
                pass
        while bull_index != 3:
            img = self.get_img()
            color = color_list[bull_index]
            res = self.get_bull_aim(img, color)
            if res is not None:
                flag = self.move_aim(res, aim_coord)
            if flag:
                self.place_to_bull(pwm_circle[bull_index])
                bull_index += 1
        return True

    def get_bull_aim(self, img, color):  
        hsv_img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        inRange_hsv = cv2.inRange(hsv_img,
                                    bull_color_dist[color]["Lower"], 
                                    bull_color_dist[color]["Upper"])
        circles = cv2.HoughCircles(inRange_hsv, cv2.HOUGH_GRADIENT, 3, 60,
                            param1=100, param2=75, minRadius=150, maxRadius=250)
        if circles is None:
            return None
        cir_x = []
        cir_y = []
        num = 0
        for circle in circles[0]:
            num += 1
            cir_x.append(circle[0])
            cir_y.append(circle[1])
        cir_x = sorted(cir_x)
        cir_y = sorted(cir_y)
        bull_x = int(np.mean(cir_x[int(num/4):int(num/4*3)]))
        bull_y = int(np.mean(cir_y[int(num/4):int(num/4*3)]))
        return (bull_x, bull_y)
    
    def place_to_bull(self, pwm_circle):
        pwm_pick = 110  # 机械臂抓取
        pwm_free = 50   # 机械臂释放
        arm_obj_get_pre = [66, 269, 132]    # 预拿取物块
        arm_obj_get = [99, 269, 132]    # 拿取物块
        arm_obj_place = [133, 108, 98]  # 放置物块 (待测)
        # 托盘转到指定颜色
        self.car.set_pwm_servo(2, pwm_circle)
        # 预拿取
        self.car.set_uart_servo_angle_array(arm_obj_get_pre, 1000)
        # 拿取物块
        self.car.set_uart_servo_angle_array(arm_obj_get, 800)
        time.sleep(1)
        self.car.set_pwm_servo(1, pwm_pick)
        time.sleep(1)
        # 放置物块
        self.car.set_uart_servo_angle_array(arm_obj_place, 1200)
        time.sleep(1)
        self.car.set_pwm_servo(1, pwm_free)
        return True
        
    # task3 : 抓取物块
    def task_objpick(self, img):
        arm_obj_aim = [133, 108, 98]    # 识别物块颜色的角度
        pwm_circle = [8,88,168]
        self.car.set_uart_servo_angle_array(arm_obj_aim)
        time.sleep(1)
        #识别颜色
        now_color = self.get_color(img, obj_color_dist)
        self.get_logger().info("现在颜色:"+str(now_color))
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
                time.sleep(2)
                self.pick_from_plt(pwm_circle[self.obj_code])

                self.obj_code += 1
                self.color_list = []
            if self.obj_code == 3:
                return True
        return False
    
    def pick_from_plt(self, pwm_circle):
        # 死参数
        arm_obj_pick_pre = [176, 7, 170]    # 预抓取机械臂
        arm_obj_pick = [130, 7, 170]  # 抓取机械臂
        arm_obj_pick_back = [99, 269, 132]  # 放回机械臂
        pwm_pick = 110  # 机械臂抓取
        pwm_free = 50   # 机械臂释放

        self.car.set_pwm_servo(2, pwm_circle)
        self.car.set_pwm_servo(1, pwm_free)
        # 预抓取
        self.car.set_uart_servo_angle_array(arm_obj_pick_pre,500)
        time.sleep(1)
        # 抓取
        self.car.set_uart_servo_angle_array(arm_obj_pick,500)
        time.sleep(2)
        self.car.set_pwm_servo(1, pwm_pick)
        time.sleep(1)
        # 放回
        self.car.set_uart_servo_angle_array(arm_obj_pick_back,800)
        time.sleep(2)
        self.car.set_pwm_servo(1, pwm_free)
        time.sleep(1)

    # task2 : 物块瞄准
    def task_objaim(self, img):
        arm_obj_aim = [133, 108, 98]
        self.car.set_uart_servo_angle_array(arm_obj_aim)
        if not self.aim_delay_flag:
            time.sleep(2)
            self.aim_delay_flag = True
        aim_coord = (150, 260)
        flag = False
        while not flag:
            img = self.get_img()
            color = self.get_color(img, obj_color_dist)
            res = self.get_obj_aim(img, color)
            if res is not None:
                flag = self.move_aim(res, aim_coord)
            else:
                continue
        return True

    def get_obj_aim(self, img, color):
        hsv_img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        inRange_hsv = cv2.inRange(hsv_img,
                                    obj_color_dist[color]["Lower"], 
                                    obj_color_dist[color]["Upper"])
        contours, hierarchy = cv2.findContours(inRange_hsv, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        res_con = []
        for contour in contours:
            if len(contour) > len(res_con):
                res_con = contour
        if len(res_con) != 0:
            x, y, w, h = cv2.boundingRect(res_con)
            x = x + w/2
            y = y + h/2
            self.get_logger().info("捕获物体坐标:"+str(x)+","+str(y))
            return (x, y)
        else:
            return None

    # task1 : 扫描二维码    
    def task_qrscan(self, img):
        # 设置机械臂
        self.car.set_pwm_servo(1, 110)
        arm_scan = [226, 126, 35]   # 二维码机械臂
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

    def move_aim(self,now_coord,aim_coord):

        # # --------------------
        # now_time = self.get_clock().now()
        # if len(self.time_list) == 0:
        #     self.time_list.append(now_time)
        #     return False
        # else:
        #     last_time = self.time_list.pop()
        # diff_time = now_time - last_time
        # print("瞄准时间间隔:" + str(diff_time))
        # # --------------------

        now_x = now_coord[0]
        now_y = now_coord[1]
        aim_x = aim_coord[0]
        aim_y = aim_coord[1]

        # # --------------------
        # if len(self.coord_list) == 0:
        #     self.coord_list.append((now_x, now_y))
        # else:
        #     coord_length = len(self.coord_list)
        #     last_coord = self.coord_list[coord_length - 1]
        # self.get_logger().info("瞄准时间间隔:" + str(diff_time)+"坐标:"+str(now_x)+","+str(now_y))
        # # --------------------

        buffer = 30
        speed = 0.01
        twist = Twist()
        if abs(now_x - aim_x) < buffer:
            if abs(now_y - aim_y) < buffer:
                self.twist_pub.publish(Twist())
                self.get_logger().info("已校准!")
                return True
            elif now_y > aim_y + buffer:
                twist.linear.y = speed
            elif now_y < aim_y - buffer:
                twist.linear.y = -speed
            else:
                self.get_logger().info("aim error!")
        elif now_x > aim_x + buffer:
            twist.linear.x = -speed
        elif now_x < aim_x - buffer:
            twist.linear.x = speed
        else:
            self.get_logger().info("aim error!")

        self.twist_pub.publish(twist)
        return False

    def get_color(self, img, color_dist = obj_color_dist):
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
                    prb_color = color
        return prb_color
        
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
