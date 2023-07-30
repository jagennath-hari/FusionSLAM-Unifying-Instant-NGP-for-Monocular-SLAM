import os

from launch import LaunchDescription, Substitution, LaunchContext
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration, ThisLaunchFileDir, PythonExpression, TextSubstitution
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node
from launch_ros.actions import SetParameter
from typing import Text
from ament_index_python.packages import get_package_share_directory
            
def launch_setup(context, *args, **kwargs):      

    arugments = ["--delete_db_on_start", "--Rtabmap/DetectionRate", LaunchConfiguration('detection_rate')]      

    return [
        # RGB-D Sync Node
        Node(
            package = 'rtabmap_sync', executable = 'rgbd_sync', output = "log",
            parameters = [{
                "approx_sync": False,
                "queue_size": 10,
                "qos": 1,
                "qos_camera_info": 1,
                "depth_scale": 1.0}],
            remappings = [
                ("rgb/image", LaunchConfiguration('rgb_topic')),
                ("depth/image", LaunchConfiguration('depth_topic')),
                ("rgb/camera_info", LaunchConfiguration('camera_info_topic'))],
            namespace = 'rtabmap'),

        # RTABMAP Node
        Node(
            package = 'rtabmap_slam', executable = 'rtabmap', output = "screen",
            parameters = [{
                "subscribe_depth": False,
                "subscribe_rgbd": True,
                "subscribe_rgb": False,
                "subscribe_stereo": False,
                "subscribe_scan": False,
                "subscribe_scan_cloud": False,
                "subscribe_user_data": False,
                "subscribe_odom_info": False,
                "frame_id": "base_link",
                "map_frame_id": "map",
                "odom_frame_id": "",
                "publish_tf": True,
                "initial_pose": "",
                "ground_truth_frame_id": "",
                "ground_truth_base_frame_id": "",
                "odom_tf_angular_variance": 0.01,
                "odom_tf_linear_variance": 0.001,
                "odom_sensor_sync": False,
                "wait_for_transform": 0.2,
                "database_path": "~/.ros/rtabmap.db",
                "approx_sync": False,
                "config_path": "",
                "queue_size": 10,
                "qos_image": 1,
                "qos_scan": 1,
                "qos_odom": 1,
                "qos_camera_info": 1,
                "qos_imu": 1,
                "qos_gps": 1,
                "qos_user_data": 1,
                "scan_normal_k": 0,
                "landmark_linear_variance": 0.0001,
                "landmark_angular_variance": 9999.0,
                "Mem/IncrementalMemory": "true",
                "Mem/InitWMWithAllNodes": "true",
                "SuperPoint/ModelPath" : LaunchConfiguration('superpoint_model_path'),
                "PyDetector/Path" : LaunchConfiguration('pydetector_path'),
                "PyMatcher/Path" : LaunchConfiguration('pymatcher_path'),
                "SuperPoint/Threshold" : "0.08",
                "SuperPoint/NMS" : "true",
                "SuperPoint/NMSRadius" : "5",
                "SuperPoint/Cuda" : "true",
                "ORB/Gpu" : "true",
                "Reg/Force3DoF" : "true",
                "SURF/GpuVersion" : "true",
                "Mem/UseOdomFeatures" : "true",
                "Kp/DetectorStrategy" : "11",
                "Kp/MaxFeatures" : "0",
                "Kp/NNStrategy" : "4",
                "Optimizer/Strategy" : "1",
                "Optimizer/Robust" : "true",
                "RGBD/OptimizeMaxError" : "0",
                "Vis/CorNNType" : "6",
                "PyMatcher/Iterations" : "30",
                "PyMatcher/Threshold" : "0.2",
                "PyMatcher/Cuda" : "true",
                "PyMatcher/Model" : "indoor",
                "Vis/FeatureType" : "11",
                "Vis/MaxFeatures" : "0",
                "Optimizer/Iterations" : "40",
                "Optimizer/VarianceIgnored" : "true"
            }],
            remappings=[
                ("scan_cloud", LaunchConfiguration('scan_cloud_topic')),
                ("odom", LaunchConfiguration('odom_topic')),
                ("imu", LaunchConfiguration('imu_topic'))],
            arguments = arugments,
            namespace = 'rtabmap'),

        # RTABMAP Viz Node
        Node(
            package = 'rtabmap_viz', executable = 'rtabmap_viz', output = 'screen',
            parameters = [{
                "subscribe_depth": False,
                "subscribe_rgbd": True,
                "subscribe_rgb": False,
                "subscribe_stereo": False,
                "subscribe_scan": False,
                "subscribe_scan_cloud": False,
                "subscribe_user_data": False,
                "subscribe_odom_info": False,
                "frame_id": "base_link",
                "odom_frame_id": "",
                "wait_for_transform": 0.2,
                "approx_sync": False,
                "queue_size": 10,
                "qos_image": 1,
                "qos_scan": 1,
                "qos_odom": 1,
                "qos_camera_info": 1,
                "qos_user_data": 1
            }],
            remappings=[
                ("scan_cloud", LaunchConfiguration('scan_cloud_topic')),
                ("odom", LaunchConfiguration('odom_topic'))],
            namespace = 'rtabmap')
        ]

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('rgb_topic',           default_value='/camera/rgb/image_rect_color',       description='camera rgb topic'),
        DeclareLaunchArgument('depth_topic',         default_value='/camera/depth_registered/image_raw', description='camera depth topic'),
        DeclareLaunchArgument('camera_info_topic',   default_value='/camera/rgb/camera_info',            description='camera info topic'),
        DeclareLaunchArgument('odom_topic',   default_value='/odom',            description='odom topic'),
        DeclareLaunchArgument('imu_topic',   default_value='/imu',            description='imu topic'),
        DeclareLaunchArgument('scan_cloud_topic',   default_value='/scan_cloud',            description='scan cloud topic'),
        DeclareLaunchArgument('superpoint_model_path',   default_value='superpoint_v1.pt',            description='superpoint model path'),
        DeclareLaunchArgument('pydetector_path',   default_value='rtabmap_superpoint.py',            description='py detector path'),
        DeclareLaunchArgument('pymatcher_path',   default_value='rtabmap_superglue.py',            description='py matcher path'),
        DeclareLaunchArgument('detection_rate',   default_value='0',            description='rate at which map gets update 0 means process as fast as possible'),
        OpaqueFunction(function = launch_setup)
    ])