import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    image_params = LaunchConfiguration(
    'image_params',
    default = os.path.join(
        get_package_share_directory('yolo_tracer'),
        'param',
        'image_config.yaml'))

    return LaunchDescription([
        DeclareLaunchArgument(
            'image_params',
            default_value = image_params,
            description = 'Full path of parameter file'),

        Node(
            package = 'yolo_tracer',
            executable = 'img_pub',
            name = 'img_pub',
            parameters = [image_params],
            output='screen'),

        Node(
            package = 'yolo_tracer',
            executable = 'img_sub',
            name = 'img_sub',
            parameters = [image_params],
            output = 'screen'),
        ])
