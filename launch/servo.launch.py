import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    servo_params = LaunchConfiguration(
    'servo_params',
    default = os.path.join(
        get_package_share_directory('yolo_tracking'),
        'param',
        'servo_config.yaml'))

#    pos_params = LaunchConfiguration(
#    'pos_params',
#    default = os.path.join(
#        get_package_share_directory('yolo_tracking'),
#        'param',
#        'pos_info.yaml'))

    return LaunchDescription([
        DeclareLaunchArgument(
            'servo_params',
            default_value = servo_params,
            description = 'Full path of parameter file'),

#        DeclareLaunchArgument(
#            'pos_params',
#            default_value = pos_params,
#            description = 'Full path of parameter file'),

        Node(
            package = 'yolo_tracking',
            executable = 'servo',
            name = 'servo',
            parameters = [servo_params], # pos_params],
            output = 'screen'),

        Node(
            package = 'yolo_tracking',
            executable = 'control',
            name = 'control',
            parameters = [servo_params], # pos_params],
            output = 'screen'),
    ])
