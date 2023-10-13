#!/usr/bin/env python
# encoding: utf-8

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

    odom_data_config = os.path.join(
        get_package_share_directory('carbot_launch'),
        'config',
        'odom_data.yaml'
    )
    
    return LaunchDescription([
        # Node(
        #     package='carbot_driver',
        #     executable='car_driver',
        #     parameters=[car_driver_config]
        # ),
        Node(
            package='carbot_location',
            executable='odom_data',
            parameters=[odom_data_config]
        ),
        Node(
            package='carbot_plan',
            executable='carbot_plan',
        )
    ])