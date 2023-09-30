import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from carbot_lib import Carbot

class Carbot_Imu_Data(Node):
    def __init__(self,name):
        super().__init__(name)
        # 实例化小车
        self.car = Carbot()
        # 打开数据接收线程
        self.car.create_receive_threading()
        # 数据初始化
        self.ACC_X:float = 0.0                   # X轴加速度
        self.ACC_Y:float = 0.0                   # Y轴加速度
        self.ACC_Z:float  =  0.0                 # Z轴加速度
        self.GYRO_X :float = 0.0                 # X轴陀螺仪
        self.GYRO_Y :float = 0.0                 # Y轴陀螺仪
        self.GYRO_Z :float = 0.0                 # Z轴陀螺仪
        self.roll :float = 0.0                   # 横滚角    
        self.pitch :float = 0.0                  # 俯仰角
        self.yaw :float = 0.0                    # 航向角
        self.MAG_X :float = 0.0                  # 磁场X轴
        self.MAG_Y :float = 0.0                  # 磁场Y轴
        self.MAG_Z :float = 0.0                  # 磁场Z轴
        self.Q0 :float = 0.0                     # 四元数Q0
        self.Q1 :float = 0.0                     # 四元数Q1
        self.Q2 :float = 0.0                     # 四元数Q2
        self.Q3 :float = 0.0                     # 四元数Q3
        # 创建发布者
        self.publisher_ = self.create_publisher(Imu, 'imu_data', 1)

        # 回调函数返回周期
        time_period = 0.001         
        self.timer = self.create_timer(time_period, self.timer_callback)

    def timer_callback(self):
        # 更新数据
        self.ACC_X, self.ACC_Y, self.ACC_Z = self.car.get_accelerometer_data()
        self.GYRO_X, self.GYRO_Y, self.GYRO_Z = self.car.get_gyroscope_data()
        self.roll, self.pitch, self.yaw = self.car.get_imu_attitude_data()
        self.MAG_X, self.MAG_Y, self.MAG_Z = self.car.get_magnetometer_data()

        # 创建imu消息
        imu_data = Imu()
        imu_data.header.frame_id = "map"
        imu_data.header.stamp = self.get_clock().now().to_msg()
        imu_data.linear_acceleration.x = self.ACC_X
        imu_data.linear_acceleration.y = self.ACC_Y
        imu_data.linear_acceleration.z = self.ACC_Z
        imu_data.angular_velocity.x = self.GYRO_X
        imu_data.angular_velocity.y = self.GYRO_Y 
        imu_data.angular_velocity.z = self.GYRO_Z 
        # 发布sensor_msgs/Imu 数据类型
        self.publisher_.publish(imu_data)

def main(args=None):
    rclpy.init(args=args)
    carbot_imu_data = Carbot_Imu_Data()
    rclpy.spin(carbot_imu_data)
    rclpy.shutdown()