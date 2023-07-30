import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    zed_wrapper = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
        get_package_share_directory('zed_wrapper'), 'launch'), 
        '/zed2i.launch.py'])
    )

    rtabmap = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
        get_package_share_directory('ngp_ros2'), 'launch'),
        '/rtabmap.launch.py'])
    )

    dataloader = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
        get_package_share_directory('ngp_ros2'), 'launch'),
        '/dataloader.launch.py'])
    )

    return LaunchDescription([
        zed_wrapper,
        rtabmap,
        dataloader
    ])