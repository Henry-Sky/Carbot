import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry


class March_Pose(Node):
    def __init__(self,name):
        super().__init__(name)
        self.pose_sub = self.create_subscription(Pose,"set_pose",march_callback,1)
        self.odom_sub = self.create_subscription(Odometry,"odom_raw",odom_callback,1)
        self.twist_pub = self.create_publisher(Twist,"twist_cmd",10)
        
        # 位置初始化
        self.now_pose = Pose()
        self.goal_pose = Pose()
        
    def odom_callback(self,odom_msg):
        self.now_pose = odom_msg.pose
        
    def march_callback(self,pose_msg):
        self.goal_pose = pose_msg
        
        