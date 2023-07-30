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

## ⚙️ Install
1) Clone all ROS 2 packages into workspace
2) Clone reporsitory into ROS 2 workspace
3) `colcon build --symlink-install --cmake-args -DRTABMAP_SYNC_MULTI_RGBD=ON -DRTABMAP_SYNC_USER_DATA=ON -DPYTHON_EXECUTABLE=/usr/bin/python3 -DCMAKE_BUILD_TYPE=Release --parallel-workers $(nproc) --executor sequential`
4) `source ~/.bashrc` or source ROS 2 workspace
