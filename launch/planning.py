#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="cpp_pubsub",
                executable="path",
                name="path_producer",
                parameters=[
                    os.path.join(
                        get_package_share_directory("cpp_pubsub"),
                        "config",
                        
                    )
                ],
                output="screen",
            ),
            
            Node(
                package="path_tracker",
                executable="tracker",
                name="vehicle",
                parameters=[
                    os.path.join(
                        get_package_share_directory("path_tracker"),
                        "config",
                        "params.yaml",
                    )
                ],
                output="screen",
            ),
        ]
    )
