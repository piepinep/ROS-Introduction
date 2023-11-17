# 安装

参考网址：
[ROS wiki中文](https://wiki.ros.org/cn)

[在Ubuntu上安装ROS Noetic](https://wiki.ros.org/cn/noetic/Installation/Ubuntu)

# 测试

启动三个终端，并在三个终端中分别输入如下三条命令

```bash
roscore
rosrun turtlesim turtlesim_node
rosrun turtlesim turtle_teleop_key 
```


* 终端1：

```bash
roscore

# Checking log directory for disk usage. This may take a while.
# Press Ctrl-C to interrupt
# Done checking log file disk usage. Usage is <1GB.

# started roslaunch server http://youruserhost:42129/
# ros_comm version 1.16.0


# SUMMARY
# ========

# PARAMETERS
#  * /rosdistro: noetic
#  * /rosversion: 1.16.0

# NODES

# auto-starting new master
# process[master]: started with pid [3503]
# ROS_MASTER_URI=http://youruserhost:11311/

# setting /run_id to f19aa236-7c38-11ee-a805-351a448bf6af
# process[rosout-1]: started with pid [3520]
# started core service [/rosout]
```

* 终端2
```bash
rosrun turtlesim turtlesim_node

# [ INFO] [1699229583.704368466]: Starting turtlesim with node name /turtlesim
# [ INFO] [1699229583.709179872]: Spawning turtle [turtle1] at x=[5.544445], y=[5.544445], theta=[0.000000]
```

* 终端3
```bash
rosrun turtlesim turtle_teleop_key 

# Reading from keyboard
# ---------------------------
# Use arrow keys to move the turtle. 'q' to quit.
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