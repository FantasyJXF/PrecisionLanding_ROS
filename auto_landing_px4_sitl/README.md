## 操作步骤

### ROS环境配置
 
```
cd src/Firmware
make posix_sitl_default
source ~/catkin_ws/devel/setup.bash    # (只有mavros是用源码编译的才需要)
source Tools/setup_gazebo.bash $(pwd) $(pwd)/build_posix_sitl_default 
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd) # 将 px4包添加到ROS路径
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)/Tools/sitl_gazebo
```

### 启动ROS程序

* 启动PX4 + gazebo的仿真环境
 ```
 roslaunch px4 mavros_posix_sitl_apriltag.launch
 ```


* 控制飞机到达指定位置，tag上方
 ```
 rosrun position position_node（到达指定位置后需要关闭此节点）
 ```

* 进行目标识别
 ``` 
 roslaunch apriltag_ros example.launch
 ```

* 自动着陆
 ```
 rosrun auto_landing_px4_sitl auto_landing_px4_sitl_node
 ```

### 结果查看

* 查看相机的画面
 ```
 rosrun image_view image_view image:=/camera/image_raw(通过rostopic type topic_name && rosmsg show msg_name查看camera的主题消息类型)
 ```


