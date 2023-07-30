# FusionSLAM-Unifying-Instant-NGP-for-Monocular-SLAM
Dive into cutting-edge FusionSLAM, where SuperPoint, SuperGlue, Neural Depth Estimation, and Instant-NGP converge, elevating Monocular SLAM to unparalleled precision and performance. Redefining mapping, localization, and reconstruction in a single camera setup.


## 🏁 Dependencies
1) NVIDIA Driver ([Official Download Link](https://www.nvidia.com/download/index.aspx))
2) CUDA Toolkit ([Official Link](https://developer.nvidia.com/cuda-downloads))
3) ZED SDK ([Official Guide](https://www.stereolabs.com/developers/release/))
4) OpenCV CUDA ([Github Guide](https://gist.github.com/raulqf/f42c718a658cddc16f9df07ecc627be7))
5) ROS 2 Humble ([Official Link](https://docs.ros.org/en/humble/Installation.html))
6) Miniconda ([Official Link](https://docs.conda.io/en/main/miniconda.html))
7) ZED ROS 2 Wrapper ([Official Github Link](https://github.com/stereolabs/zed-ros2-wrapper))
8) RTAB-Map ([Official Github Link](https://github.com/introlab/rtabmap))
9) RTAB-Map ROS 2 ([Official Github Link](https://github.com/introlab/rtabmap_ros/tree/ros2#rtabmap_ros))
10) PyTorch ([Official Link](https://pytorch.org/))
11) Instant-ngp ([Official Github Link](https://github.com/NVlabs/instant-ngp))
12) SuperPoint ([Official Github Link](https://github.com/magicleap/SuperPointPretrainedNetwork))
13) SuperGlue ([Official Github Link](https://github.com/magicleap/SuperGluePretrainedNetwork))

## ⚙️ Install
1) Install all non ROS 2 libraries
2) Clone all ROS 2 packages into workspace
3) Clone reporsitory into ROS 2 workspace
4) `colcon build --symlink-install --cmake-args -DRTABMAP_SYNC_MULTI_RGBD=ON -DRTABMAP_SYNC_USER_DATA=ON -DPYTHON_EXECUTABLE=/usr/bin/python3 -DCMAKE_BUILD_TYPE=Release --parallel-workers $(nproc) --executor sequential`
5) `source ~/.bashrc` or source ROS 2 workspace
6) Run `python trace.py` and change path of SuperPoint weights, this will generate a model compatible with your version of PyTorch
7) Add libtorch path `export LD_LIBRARY_PATH=LD_LIBRARY_PATH:../miniconda3/envs/rtabmap/lib/python3.10/site-packages/torch/lib${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}` ensure the path is correct else RTAB-Map will not work

## ⌛️ SLAM
1) Run SLAM to generate dataset
2) `ros2 launch ngp_ros2 slam.launch.py rgb_topic:=/zed2i/zed_node/rgb/image_rect_color depth_topic:=/zed2i/zed_node/depth/depth_registered camera_info_topic:=/zed2i/zed_node/rgb/camera_info odom_topic:=/zed2i/zed_node/odom imu_topic:=/zed2i/zed_node/imu/data scan_cloud_topic:=/zed2i/zed_node/point_cloud/cloud_registered superpoint_model_path:=../SuperPointPretrainedNetwork/superpoint_v1.pt pydetector_path:=../rtabmap_superpoint.py pymatcher_path:=../rtabmap_superglue.py detection_rate:=1 image_path:=../images/ transform_path:=../transforms.json`
