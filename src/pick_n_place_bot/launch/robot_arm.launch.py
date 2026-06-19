# Copyright 2026 goober
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node


def generate_launch_description():
    # Get package directory
    package_dir = get_package_share_directory('pick_n_place_bot')
    config_file = os.path.join(package_dir, 'config', 'params.yaml')

    # Declare launch arguments
    use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )

    # Camera Node
    camera_node = Node(
        package='pick_n_place_bot',
        executable='camera_node',
        name='camera_node',
        output='screen',
        parameters=[config_file],
    )

    # Detector Node — subscribes to camera/image, publishes std_msgs/Int32 on detector/aruco_id
    detector_node = Node(
        package='pick_n_place_bot',
        executable='detector_node',
        name='detector_node',
        output='screen',
        parameters=[config_file],
        remappings=[
            ('camera/image', '/camera/image'),
            ('detector/aruco_id', '/detector/aruco_id'),
        ],
    )

    # Manipulator Node — subscribes to std_msgs/Int32 on detector/aruco_id
    manipulator_node = Node(
        package='pick_n_place_bot',
        executable='manipulator_node',
        name='manipulator_node',
        output='screen',
        parameters=[config_file],
        remappings=[
            ('detector/aruco_id', '/detector/aruco_id'),
        ]
    )

    return LaunchDescription([
        use_sim_time,
        camera_node,
        detector_node,
        manipulator_node,
    ])
