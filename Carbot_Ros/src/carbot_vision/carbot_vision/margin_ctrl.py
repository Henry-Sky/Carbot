import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
import cv2
import numpy as np
import math
from cv_bridge import CvBridge
from carbot_interfaces.srv import Margin
from queue import PriorityQueue


color_dist = {'red': {'Lower': np.array([0, 60, 60]), 'Upper': np.array([6, 255, 255])},
              'gray': {'Lower': np.array([0, 0, 46]), 'Upper': np.array([180, 43, 220])},
              'yellow': {'Lower': np.array([26, 43, 46]), 'Upper': np.array([34, 255, 255])},          
              'blue': {'Lower': np.array([100, 80, 46]), 'Upper': np.array([124, 255, 255])},
              'green': {'Lower': np.array([35, 43, 35]), 'Upper': np.array([90, 255, 255])},
              'none': {'Lower': np.array([0, 0, 0]), 'Upper': np.array([255, 255, 255])},}


class Margin_Ctrl(Node):
    def __init__(self, name):
        super().__init__(name)
        # 订阅/发布
        self.twist_pub = self.create_publisher(Twist,"twist_cmd",2)
        self.image_sub = self.create_subscription(Image,"image_raw",self.margin_callback,1)
        # 服务
        self.margin_srv = self.create_service(Margin,"margin_server",self.margin_srv_callback)
        # 参数
        # 初始化
        self.max_window = [640,480]
        self.margin_start = False
        self.upcolor = "gray"
        self.downcolor = "blue"
        self.aim_dis =  240
        
        
    def margin_srv_callback(self,request,response):
        self.upcolor = request.upcolor
        self.downcolor = request.downcolor
        self.aim_dis = request.aim_dis
        self.margin_start = request.send_flag
        response.get_flag = (self.margin_start == request.send_flag)
        return response
        
    def margin_callback(self,img_msg):
        img = CvBridge().imgmsg_to_cv2(img_msg)
        twist = Twist()
        if (self.margin_start 
            and self.cruising_boundary(img,self.upcolor,self.downcolor) != None):
            line_coord, line_analy = self.cruising_boundary(img,self.upcolor,self.downcolor)
            pix_dis = line_coord[1]
            line_k = line_analy[0]
            self.get_logger().info("识别到边界:"+str(pix_dis))
            # 方向水平
            if (line_k < 0.05 and line_k > -0.05):
                twist.angular.z = 0.0
                # 满足最大间距
                if (pix_dis < self.aim_dis):
                    twist.linear.y = -0.02
                elif (pix_dis > self.aim_dis + 20):
                    twist.linear.y = 0.02
                else:
                    twist.linear.y = 0.0
            elif (line_k >= 0.05):
                twist.angular.z = -0.01
            else:
                twist.angular.z = 0.01
            self.twist_pub.publish(twist)
        else:
            # twist.linear.y = 0.0
            # twist.angular.z = 0.0
            # self.twist_pub.publish(twist)
            pass

        
    def analy_line(self,x1,y1,x2,y2):
        assert(x1 != x2)
        x1 *= 1.0
        y1 *= 1.0
        x2 *= 1.0
        y2 *= 1.0
        k = (y2 - y1)/(x2 - x1)
        b = y1 - (k * x1)
        return k, b

    def vertical_distance_point(self,analy,coord,dis):
        # 最大窗口限制
        max_x = self.max_window[0]
        min_x = 0
        max_y = self.max_window[1]
        min_y = 0
        if analy[0] != 0.0:
        # dis为正,返回在analy(x)右方距离点
            vert_k = -(1.0/analy[0])
            vert_b = coord[1] - (vert_k * coord[0])
            res_coord = []
            res_x_1 = coord[0] + (dis * 1.0) / (math.sqrt(1.0 + vert_k * vert_k))
            res_y_1 = vert_k * res_x_1 + vert_b
            res_coord.append([res_x_1,res_y_1])
            res_x_2 = coord[0] - (dis * 1.0) / (math.sqrt(1.0 + vert_k * vert_k))
            res_y_2 = vert_k * res_x_2 + vert_b
            res_coord.append([res_x_2,res_y_2])
            for the_coord in res_coord:
                res_x = the_coord[0]
                rex_y = the_coord[1]
                if res_x >= max_x:
                    res_x = max_x - 1
                elif res_x <= min_x:
                    res_x = min_x + 1
                res_y = vert_k * res_x + vert_b
                if res_y >= max_y:
                    res_y = max_y - 1
                elif res_y <= min_y:
                    res_y = min_y + 1
                res_x = (res_y - vert_b) * 1.0 / vert_k
                the_coord[0] = int(res_x)
                the_coord[1] = int(res_y)
            if coord[0][1] > coord[1][1]:
                tmp_y = coord[1][1]
                tmp_x = coord[1][0]
                coord[1] = coord[0]
                coord[0] = [tmp_x,tmp_y]
            return coord[0],coord[1]
        else:
        # dis为正,返回在analy(x)下方距离点
            res_coord = []
            res_x = coord[0]
            res_y = coord[0] - dis
            if res_y <= min_y:
                res_y = min_y + 1
            res_coord.append([res_x, res_y])
            res_y = coord[0] + dis
            if res_y >= max_y:
                res_y = max_y - 1
            res_coord.append([res_x, res_y])
            return res_coord[0],res_coord[1]

    def fit_hsv_color(self,img_hsv, coord_1, coord_2, color, sampling_num = 10, assert_num = 0.6):
        # sampling_num: 两点间采样数 ； assert_num: 判断阈值，0.6表示有，60%的采样点符合即返回True
        if coord_1[0] > coord_2[0]:
            coord_tmp = coord_1
            coord_1 = coord_2
            coord_2 = coord_tmp
        sample_interval = (coord_2[0] - coord_1[0]) // sampling_num
        k,b = self.analy_line(coord_1[0], coord_1[1], coord_2[0], coord_2[1])
        true_num = 0
        for i in range(sampling_num + 1):
            coord = []
            coord.append(coord_1[0] + i * sample_interval)
            coord.append(k * coord[0] + b)
            
            pix = img_hsv[int(coord[1]),int(coord[0])]
            color_flag = True
            for i in range(3):
                if (pix[i] < color_dist[color]["Lower"][i] 
                    or pix[i] > color_dist[color]["Upper"][i]):
                    color_flag = False
            true_num += int(color_flag)
        if (true_num * 1.0)/((sampling_num + 1) * 1.0) >= assert_num:
            return True
        else:
            return False

    def line_len(self,x1,y1,x2,y2):
        x_dis = abs(x2 - x1)
        x_dis *= 1.0
        y_dis = abs(y2 - y1)
        y_dis *= 1.0
        lenth = math.sqrt(math.pow(x_dis,2)+math.pow(y_dis,2))
        return lenth
        
    # 识别巡航边界
    def cruising_boundary(self,img,upcolor,downcolor):
        img_gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
        img_edges = cv2.Canny(img_gray,50,150,apertureSize = 3)
        img_hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        # 获取线条
        lines = cv2.HoughLinesP(img_edges,1,np.pi/180,100,minLineLength=100,maxLineGap=10)
        # self.get_logger().info("线条数量:"+str(len(lines)))
        sort_lines = []
        if lines is None:
            return None
        for line in lines:
            x1, y1, x2, y2 = line[0]
            # 计算直线解析式 (y = k*x + b)
            if x1 != x2:
                k, b = self.analy_line(x1, y1, x2, y2)
            else:
                continue
            # 根据解析式筛除(+-30度)
            k_abs = 1.0 / math.sqrt(3)
            if (k < -k_abs or k > k_abs):
                continue
            else:
                pass
            # 根据颜色筛除
            sampling_distance = 10
            
            up_coord_1, down_coord_1 = self.vertical_distance_point((k,b), (x1,y1), sampling_distance)
            up_coord_2, down_coord_2 = self.vertical_distance_point((k,b), (x2,y2), sampling_distance)

            if (self.fit_hsv_color(img_hsv, up_coord_1, up_coord_2, upcolor) 
                and self.fit_hsv_color(img_hsv, down_coord_1, down_coord_2, downcolor)):
                pass
            else:
                continue
            
            sort_lines.append(line)
        if len(sort_lines) != 0:
            # 扩展线条(k,b 值相近(< buffer)的线条合并成一条线)
            # for line in sort_lines:
            #     # (todo)采用优先队列排序
            #     pass
            
            # 筛选线条(选出最长的线条,即最接近目标的线条)
            length_line_queue = PriorityQueue()
            for line in sort_lines:
                x1, y1, x2, y2 = line[0]
                k, b = self.analy_line(x1, y1, x2, y2)
                length_line_queue.put((-self.line_len(x1, y1, x2, y2),((x1+x2)//2,(y1+y2)//2),(k,b)))
            res_line = length_line_queue.get()
            res_coord = res_line[1]
            res_analy = res_line[2]
            # 返回边界线(x,y),(k,b)
            return res_coord,res_analy
        else:
            return None

def main():
    rclpy.init()
    margin_ctrl = Margin_Ctrl("margin_ctrl")
    rclpy.spin(margin_ctrl)
    margin_ctrl.destroy_node()
    rclpy.shutdown()