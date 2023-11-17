#include "ros/ros.h"
#include "turtlesim/Pose.h"

/**
 * 先启动 turtlesim 节点,再运行此文件
 * rosrun turtlesim turtlesim_node
 */

void locate_info(const turtlesim::Pose::ConstPtr &cptr)
{
    ROS_INFO("x = %.2f\n"
             "y = %.2f\n"
             "theta = %.2f\n"
             "linear_velocity=%.2f\n"
             "angle_velocity=%.2f",
             cptr->x, cptr->y, cptr->theta, cptr->linear_velocity, cptr->angular_velocity);
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "print_tutle");
    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe("turtle1/pose", 10, locate_info);

    ros::spin();
}