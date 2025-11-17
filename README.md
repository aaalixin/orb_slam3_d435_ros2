ORB_SLAM3 D435 ROS2 项目
项目简介
本项目基于 ORB_SLAM3 进行开发，主要完成了 ROS2 的适配工作，并针对 Intel RealSense D435 相机进行了优化。项目对原始源码进行了修改，发布了 SLAM 的定位和位姿话题，使其能够更好地在 ROS2 环境中运行。

系统依赖
在开始使用前，请确保以下依赖已正确安装和配置：

Pangolin

OpenGL

构建指南
1. 构建第三方库
按照以下步骤构建所需的第三方库：

bash
# 构建 DBoW2
echo "Configuring and building Thirdparty/DBoW2 ..."
cd Thirdparty/DBoW2
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j

# 构建 g2o
cd ../../g2o
echo "Configuring and building Thirdparty/g2o ..."
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j

# 构建 Sophus
cd ../../Sophus
echo "Configuring and building Thirdparty/Sophus ..."
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j
2. 解压词典文件
bash
cd ../../Vocabulary
tar -xf ORBvoc.txt.tar.gz
cd ..
3. 构建 ROS2 包
bash
colcon build
运行步骤
1. 启动相机节点
bash
cd python
python3 d435_put.py
此脚本将发布彩色图和深度图。

2. 配置环境
在新终端中，进入项目根目录并执行：

bash
source install/setup.bash

# 设置库路径（请根据您的实际路径修改）
export LD_LIBRARY_PATH=/home/lx/catkin_ws/d435_slam_ros2/Thirdparty/DBoW2/lib:$LD_LIBRARY_PATH
export LD_LIBRARY_PATH=/home/lx/catkin_ws/d435_slam_ros2/Thirdparty/g2o/lib:$LD_LIBRARY_PATH
export LD_LIBRARY_PATH=/home/lx/catkin_ws/d435_slam_ros2/lib:$LD_LIBRARY_PATH
3. 启动 SLAM 节点
bash
ros2 run d435_slam_ros2 d435_slam_node Vocabulary/ORBvoc.txt config/d435.yaml
配置说明
相机参数配置
config/d435.yaml 文件中包含了 D435 相机的内参数据。您可以根据自己的相机情况进行调整。

获取相机内参的方法：

取消注释 python/d435_put.py 中的相关代码

运行 python3 d435_put.py

订阅 /d435/info 话题获取相机内参

记录数据后重新注释相关代码以节省计算资源

可用话题
运行 ros2 topic list 可以查看以下 SLAM 相关话题：

/d435/camera_pose - 相机位姿数据

/d435/trajectory - 轨迹路径信息

/d435/map_points - 地图点云数据

/d435/current_points - 当前帧点云数据

可视化
使用 rviz2 工具可以可视化 SLAM 效果：

https://github.com/user-attachments/assets/825dbc9f-298b-480c-8f3c-d65fc8d7af3c

右上角显示的是 /d435/camera_pose 话题的数据，包含完整的坐标和位姿信息。

注意事项
请根据您的实际安装路径修改环境变量中的路径

相机内参配置对 SLAM 精度至关重要，请确保准确配置

建议在性能较好的硬件上运行以获得更好的实时性
