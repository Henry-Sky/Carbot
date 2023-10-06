import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    car_driver_config = os.path.join(
        get_package_share_directory('carbot_launch'),
        'config',
        'car_driver.yaml' 
    )
    
    ekf_node_config = os.path.join(
        get_package_share_directory('carbot_launch'),
        'config',
        'ekf_node.yaml'
    )
    
    return LaunchDescription([
        Node(
            package='carbot_driver',
            executable='car_driver',
            parameters=[car_driver_config]
        ),
        Node(
            package='carbot_imu',
            executable='carbot_imu_data',
        ),
        Node(
            package='carbot_odom',
            executable='carbot_odom',
        ),
        Node(
            package='robot_localization',
            executable='ekf_node',
            parameters=[ekf_node_config]
        )
    ])
