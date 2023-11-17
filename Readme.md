# 简介

这个仓库基于`Autolabor`的[ROS入门与实践教程](https://www.bilibili.com/video/BV1Ci4y1L7ZZ/?share_source=copy_web&vd_source=51c71b6bfa7dec49dc80c06885da3aa7)，包含了教程中的代码与个人记录笔记。

# 代码说明

1. 环境
本仓库代码基于**Windows11/WSL2/Ubuntu2004**版本

2. 使用

>a. 详情参考[ROS Wiki/CN ](https://wiki.ros.org/cn/noetic/Installation/Ubuntu)，以下摘出重要部分：

```bash
sudo sh -c '. /etc/lsb-release && echo "deb http://mirrors.tuna.tsinghua.edu.cn/ros/ubuntu/ `lsb_release -cs` main" > /etc/apt/sources.list.d/ros-latest.list'	

sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

sudo apt update

sudo apt install ros-noetic-desktop-full

echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

 >b. 在对应文件夹下使用 `catkin_make` 进行编译即可

 例如：
```bash
# 当前目录是 ROS/helloros
ROS/helloros$ catkin_make
```

等待编译完成即可

# 代码与笔记的对应关系

```
ros_introduction: 1-ros概述与简单应用
	helloros -> 简单的ros程序
	
ros_communication: 2-ros通信/3-ros通信进阶
	topics_communication -> 标准消息通信
	topics_custom_msg -> 自定义消息通信
	services_communication -> 自定义服务通信
	param_server -> 参数服务器
	practice_turtle_module -> 针对turtlesim模块应用通信
	metapack -> 元功能包

ros_tf: 5-ROS常用组件
	static_tf -> 静态坐标变换及多坐标变换
	dynamic_tf -> 动态坐标变换
	rosbag_use -> 使用rosbag写入和读入数据
	practice_turtle_trace -> 针对turtlesim模型应用坐标变换

ros_sim: 6-ROS系统仿真/7-ROS导航仿真
	urdf_xacro_gazebo -> ros的urdf, xacro, gazebo的使用
	navi -> 导航的使用
	
ros_adv: 8-ROS进阶
	action_use -> action的使用
	dynamic_reconfigure_use -> 动态参数的使用
	pluginlib_use -> 插件库的使用
	nodelet_use -> nodelet的使用
```

# 参考资料

[ROS Wiki中文参考](https://wiki.ros.org/cn)

[Autolabor教程手册](http://www.autolabor.com.cn/book/ROSTutorials/)

[ROS入门与实践教程](https://www.bilibili.com/video/BV1Ci4y1L7ZZ/?share_source=copy_web&vd_source=51c71b6bfa7dec49dc80c06885da3aa7)
