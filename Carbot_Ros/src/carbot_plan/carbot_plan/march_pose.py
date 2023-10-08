import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry


class March_Pose(Node):
    def __init__(self,name):
        super().__init__(name)
        self.pose_sub = self.create_subscription(Pose,"set_pose",self.march_callback,1)
        self.odom_sub = self.create_subscription(Odometry,"odom_data",self.odom_callback,1)
        self.twist_pub = self.create_publisher(Twist,"twist_cmd",2)
        # 位置初始化
        self.now_pose = Pose()
        self.goal_pose = Pose()
        
    def odom_callback(self,odom_msg):
        self.now_pose = odom_msg.pose.pose
        march_twist = Twist()
        
        if self.goal_pose.position.x > self.now_pose.position.x:
            march_twist.linear.x = 0.2
        elif self.goal_pose.position.x < self.now_pose.position.x:
            march_twist.linear.x = -0.2
        else:
            if self.goal_pose.position.y > self.now_pose.position.y:
                march_twist.linear.y = 0.2
            elif self.goal_pose.position.y < self.now_pose.position.y:
                march_twist.linear.y = -0.2
            else:
                march_twist.linear.x = 0.0
                march_twist.linear.y = 0.0
        self.twist_pub.publish(march_twist)
        
    def march_callback(self,pose_msg):
        self.goal_pose = pose_msg
        
def main():
    rclpy.init()
    march_pose = March_Pose("march_pose")
    rclpy.spin(march_pose)
    march_pose.destroy_node()
    rclpy.shutdown()    

main()