#include "ros/ros.h"
#include "geometry_msgs/Twist.h"

/**
 * 先启动 turtlesim 节点,再运行此文件
 * rosrun turtlesim turtlesim_node
 */

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "turtlemove_c");
    ros::NodeHandle nh;

    ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("turtle1/cmd_vel", 10);
    geometry_msgs::Twist tw;

    tw.linear.x = 1;
    tw.angular.z = 1;

    while (ros::ok())
    {
        pub.publish(tw);
        ros::spinOnce();
    }

    return 0;
}
