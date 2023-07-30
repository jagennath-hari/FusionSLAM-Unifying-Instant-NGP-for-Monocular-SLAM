import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('image_path',           default_value='images',       description='directory of image path'),
        DeclareLaunchArgument('transform_path',           default_value='tranforms.json',       description='directory of transforms.json'),
        Node(
            package = 'ngp_ros2', executable = 'data_loader', output = 'screen',
            parameters=[
                {"image_path": LaunchConfiguration('image_path')},
                {"transform_path": LaunchConfiguration('transform_path')}
            ]
        )
    ])