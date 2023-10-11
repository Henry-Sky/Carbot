import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
import cv2
import numpy as np
from cv_bridge import CvBridge
from queue import PriorityQueue

class Margin_Ctrl(Node):
    def __init__(self, name):
        super().__init__(name)
        # 订阅/发布
        self.twist_pub = self.create_publisher(Twist,"twist_cmd",2)
        self.image_sub = self.create_subscription(Image,"image_raw",self.margin_callback,1)
        # 参数
        self.declare_parameter
        
    def margin_callback(self,img_msg):
        img = CvBridge().imgmsg_to_cv2(img_msg)
        
        
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
        max_x = 640
        min_x = 0
        max_y = 480
        min_y = 0
        if analy[0] != 0.0:
        # dis为正,返回在analy(x)右方距离点
            vert_k = -(1.0/analy[0])
            vert_b = coord[1] - (vert_k * coord[0])
            res_x = coord[0] + (dis * 1.0) / (math.sqrt(1.0 + vert_k * vert_k))
            if res_x > max_x:
                res_x = max_x - 1
            elif res_x < min_x:
                res_x = min_x + 1
            res_y = vert_k * res_x + vert_b
            if res_y > max_y:
                res_y = max_y - 1
                res_x = (res_y - vert_b) * 1.0 / vert_k
            elif res_y < min_y:
                res_y = min_y + 1
                res_x = (res_y - vert_b) * 1.0 / vert_k
            return (int(res_x), int(res_y))
        else:
        # dis为正,返回在analy(x)下方距离点
            res_x = coord[0]
            res_y = coord[0] + dis
            if res_y > max_y:
                res_y = max_y - 1
            elif res_y < min_y:
                res_y = min_y + 1
            return (int(res_x), int(res_y))

    def fit_hsv_color(self,img_hsv, coord_1, coord_2, color, sampling_num = 10, assert_num = 0.6):
        # sampling_num: 两点间采样数 ； assert_num: 判断阈值，0.6表示有，60%的采样点符合即返回True
        if coord_1[0] > coord_2[0]:
            swap(coord_1,coord_2)
        sample_interval = (coord_2[0] - coord_1[0]) // sampling_num
        k,b = self.analy_line(coord_1[0], coord_1[1], coord_2[0], coord_2[1])
        true_num = 0
        for i in range(sampling_num):
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
        if (true_num * 1.0)/(sampling_num * 1.0) >= assert_num:
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
        sort_lines = []
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
            sampling_distance = 8
            
            up_coord_1 = self.vertical_distance_point((k,b), (x1,y1), sampling_distance)
            up_coord_2 = self.vertical_distance_point((k,b), (x2,y2), sampling_distance)

            if self.fit_hsv_color(img_hsv, up_coord_1, up_coord_2, upcolor):
                pass
            else:
                continue
            
            down_coord_1 = self.vertical_distance_point((k,b), (x1,y1), -sampling_distance)
            down_coord_2 = self.vertical_distance_point((k,b), (x2,y2), -sampling_distance)
            
            if self.fit_hsv_color(img_hsv, down_coord_1, down_coord_2, downcolor):
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
                length_line_queue.put((-line_len(x1, y1, x2, y2),((x1+x2)//2,(y1+y2)//2),(k,b)))
            res_line = length_line_queue.get()
            res_coord = res_line[1]
            res_analy = res_line[2]
            # 返回边界线(x,y),(k,b)
            return res_coord,res_analy
        else:
            return None