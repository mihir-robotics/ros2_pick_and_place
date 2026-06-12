import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


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
