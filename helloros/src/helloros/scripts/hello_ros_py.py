#! /usr/bin/env python3

# 包含所需的库
import rospy
import sys

# python程序中没有也不需要 NodeHandle
"""
当前目录:ros_introduction/helloros
运行命令:
$ roscore
$ source ./devel/setup.bash
$ rosrun helloros hello_ros_py.py
"""
if __name__ == "__main__":
    # 初始化节点, hello_ros_py是节点名称, 在命令行可用 __name:=节点名称指定
    rospy.init_node("hello_ros_py", sys.argv)
    # 控制台日志输出
    rospy.loginfo("hello ros! this is a python promgram")
    
    