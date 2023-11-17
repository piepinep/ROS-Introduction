#include "ros/ros.h"
#include "turtlesim/Spawn.h"

/**
 * 先启动 turtlesim 节点,再运行此文件
 * rosrun turtlesim turtlesim_node
 */

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "print_turtle");
    ros::NodeHandle nh;

    ros::ServiceClient sc = nh.serviceClient<turtlesim::Spawn>("spawn");

    turtlesim::Spawn sp;
    sp.request.name = argv[argc - 1];
    sp.request.x = 10;
    sp.request.y = 10;
    sp.request.theta = 0;

    if (sc.call(sp))
    {
        ROS_INFO("ok");
    }
    else
    {
        ROS_INFO("failed");
    }
    return 0;
}