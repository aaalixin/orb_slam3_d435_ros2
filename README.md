"# orb_slam3_d435_ros2" 
这个工程是修改的ORB_SLAM3的教程的，源码地址来自https://github.com/UZ-SLAMLab/ORB_SLAM3

本人主要进行的ros2适配，并且使用的d435相机

想要使用这个工程先确保部分依赖没有问题， Pangolin和OpenGL要提前配置好
然后和build.sh教程一样执行以下命令

echo "Configuring and building Thirdparty/DBoW2 ..."

cd Thirdparty/DBoW2

mkdir build

cd build

cmake .. -DCMAKE_BUILD_TYPE=Release

make -j


cd ../../g2o

echo "Configuring and building Thirdparty/g2o ..."

mkdir build

cd build

cmake .. -DCMAKE_BUILD_TYPE=Release

make -j


cd ../../Sophus

echo "Configuring and building Thirdparty/Sophus ..."

mkdir build

cd build

cmake .. -DCMAKE_BUILD_TYPE=Release

make -j


cd ../../../

echo "Uncompress vocabulary ..."

cd Vocabulary

tar -xf ORBvoc.txt.tar.gz

结束后执行cd ..回到工程包最外层路径执行
colcon build
编译成功后，要运行这个包先在python目录下python3 d435_put.py来发布彩色图和深度图

然后新开终端在包最外层路径执行sourse install/setup.bash
# 设置库路径

export LD_LIBRARY_PATH=/home/lx/catkin_ws/d435_slam_ros2/Thirdparty/DBoW2/lib:$LD_LIBRARY_PATH

export LD_LIBRARY_PATH=/home/lx/catkin_ws/d435_slam_ros2/Thirdparty/g2o/lib:$LD_LIBRARY_PATH

export LD_LIBRARY_PATH=/home/lx/catkin_ws/d435_slam_ros2/lib:$LD_LIBRARY_PATH

其中/home/lx/catkin_ws/d435_slam_ros2根据自己的情况进行路径修改

# 然后运行
ros2 run d435_slam_ros2 d435_slam_node Vocabulary/ORBvoc.txt config/d435.yaml

这样就可以运行ros2包了

ros2 topic list

可以看到

/d435/camera_pose - 相机位姿

/d435/trajectory - 轨迹路径

/d435/map_points - 地图点云

/d435/current_points - 当前帧点云

这几个slam的重要话题可以使用rviz2工具看效果

下面是图片效果

![755092d38a8becf83a3853bc43ed918e](https://github.com/user-attachments/assets/825dbc9f-298b-480c-8f3c-d65fc8d7af3c)

