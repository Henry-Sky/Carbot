import rclpy
import threading
from rclpy.node import Node
from sensor_msgs.msg import Imu
from carbot_lib import Carbot
from scipy.spatial.transform import Rotation


class Carbot_Imu_Data(Node):
    def __init__(self,name):
        super().__init__(name)
        # 实例化小车
        self.car = Carbot()
        # 打开数据接收线程
        self.car.create_receive_threading()
        # 数据初始化
        self.ACC_X = 0.0                  # X轴加速度
        self.ACC_Y = 0.0                  # Y轴加速度
        self.ACC_Z = 0.0                  # Z轴加速度
        self.GYRO_X = 0.0                 # X轴陀螺仪
        self.GYRO_Y = 0.0                 # Y轴陀螺仪
        self.GYRO_Z = 0.0                 # Z轴陀螺仪
        self.MAG_X = 0.0                  # 磁场X轴
        self.MAG_Y = 0.0                  # 磁场Y轴
        self.MAG_Z = 0.0                  # 磁场Z轴
        self.yaw = 0.0
        self.roll = 0.0
        self.pitch = 0.0

        # 创建发布者
        self.publisher_ = self.create_publisher(Imu, 'imu_data', 1)

        # 回调函数返回周期
        time_period = 0.001         
        self.timer = self.create_timer(time_period, self.timer_callback)

    def euler_to_quaternion(self, yaw, roll, pitch):
        euler = [yaw, roll, pitch]
        r = Rotation.from_euler('xyz', euler, degrees=True)
        quaternion = r.as_quat()
        return quaternion


    def timer_callback(self):
        # 更新数据
        self.ACC_X, self.ACC_Y, self.ACC_Z = self.car.get_accelerometer_data()
        self.GYRO_X, self.GYRO_Y, self.GYRO_Z = self.car.get_gyroscope_data()
        self.MAG_X, self.MAG_Y, self.MAG_Z = self.car.get_magnetometer_data()
        self.yaw, self.roll, self.pitch = self.car.get_imu_attitude_data()
        self.q = self.euler_to_quaternion(self.yaw, self.roll, self.pitch)

        # 创建imu消息
        imu_data = Imu()
        imu_data.header.frame_id = "map"
        imu_data.header.stamp = self.get_clock().now().to_msg()
        imu_data.linear_acceleration.x = self.ACC_X * 1.0
        imu_data.linear_acceleration.y = self.ACC_Y * 1.0
        imu_data.linear_acceleration.z = self.ACC_Z * 1.0
        imu_data.angular_velocity.x = self.GYRO_X * 1.0
        imu_data.angular_velocity.y = self.GYRO_Y * 1.0
        imu_data.angular_velocity.z = self.GYRO_Z * 1.0
        imu_data.orientation.x = self.q[0]
        imu_data.orientation.y = self.q[1]
        imu_data.orientation.z = self.q[2]
        imu_data.orientation.w = self.q[3]
        # 发布sensor_msgs/Imu 数据类型
        self.publisher_.publish(imu_data)

def main():
    rclpy.init()
    carbot_imu_data = Carbot_Imu_Data("Carbot_Imu_Data")
    rclpy.spin(carbot_imu_data)
    rclpy.shutdown()