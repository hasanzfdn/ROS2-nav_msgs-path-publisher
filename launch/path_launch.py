#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    rviz_path='/home/hasan/LEO_DRIVE/dev_ws/src/cpp_pubsub/config/vehicle.rviz'
    return LaunchDescription(
        [   
            Node(
                package='path_tracker',
                executable='tracker',
                name='tracker',
                parameters=[
                    os.path.join(
                        get_package_share_directory('path_tracker'),
                        'param',
                        'default.param.yaml')
                ],
                output='screen',
            ),
	        Node(
                package="cpp_pubsub",
                executable="path",
                name="path_producer",
                output="screen",
            ),
            Node(package='rviz2',
                 executable='rviz2',
                 arguments=['-d'+str(rviz_path)])
        ])

