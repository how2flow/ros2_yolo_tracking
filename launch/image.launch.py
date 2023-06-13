import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    cap_yaml_path = os.path.join(get_package_share_directory('yolo_tracking'),
                                 'param',
                                 'cap_config.yaml')

    cap_arg = DeclareLaunchArgument('cap_conf', default_value = cap_yaml_path,
                                    description = 'Full path of param yaml file')

    yt_img_pub_node = Node(
        package = 'yolo_tracking',
        executable = 'img_pub',
        name = 'img_pub',
        parameters = [{'cap_param': LaunchConfiguration('cap_conf')}],
        output = 'screen')

    yt_img_sub_node = Node(
        package = 'yolo_tracking',
        executable = 'img_sub',
        name = 'img_sub',
        parameters = [{'cap_param': LaunchConfiguration('cap_conf')}],
        output = 'screen')

    return LaunchDescription([
        cap_arg,
        yt_img_pub_node,
        yt_img_sub_node
    ])
