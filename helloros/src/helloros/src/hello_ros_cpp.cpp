// 包含头文件
#include "ros/ros.h"

/*
当前目录:ros_introduction/helloros
运行命令:
$ roscore
$ source ./devel/setup.bash
$ rosrun helloros hello_ros_cpp
*/

// main函数需要携带参数
int main(int argc, char *argv[])
{
    // 设置定位,允许中文输出
    setlocale(LC_ALL, "");
    // hello_ros_cpp是节点名称,在命令行可以通过 __name:=节点名称 指定
    ros::init(argc, argv, "hello_ros_cpp");
    // 创建节点句柄,节点句柄用于ros程序的后续使用
    ros::NodeHandle nh;
    // ROS_INFO在终端输出日志信息
    ROS_INFO("hello ros! this is a cpp program");
    return 0;
}
