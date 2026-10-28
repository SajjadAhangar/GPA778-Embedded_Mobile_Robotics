#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='v4l2_camera',
            executable='v4l2_camera_node',
            name='v4l2_camera_node',
            parameters=[{
                'image_raw.enable_pub_plugins': ['image_transport/raw', 'image_transport/compressed']
            }]
        ),
        Node(
            package='jerro_perception',
            executable='lidar_processor',
            name='lidar_processor_node',
        ),
        Node(
            package='jerro_perception',
            executable='camera_processor',
            name='camera_processor_node',
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('sllidar_ros2'),
                    'launch',
                    'sllidar_s2_launch.py'
                )
            ),
            launch_arguments={
                'frame_id': 'lidar',
            }.items()
        )
    ])