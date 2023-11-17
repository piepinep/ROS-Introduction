#include "ros/ros.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "geometry_msgs/Twist.h"
#include "cmath"

/**
 * 1. 换算出 turtle1 相较于 turtle2 的消息
 * 使用多坐标变换
 * 2. 计算角速度和线速度, 并发布
 */

int main(int argc, char *argv[])
{
    setlocale(LC_ALL, "");
    ros::init(argc, argv, "turtle_trace");
    ros::NodeHandle nh;

    tf2_ros::Buffer buff;
    tf2_ros::TransformListener listener(buff);

    ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("/turtle2/cmd_vel", 100);

    ros::Rate r(10);

    while (ros::ok())
    {
        try
        {
            geometry_msgs::TransformStamped tfs = buff.lookupTransform("turtle2", "turtle1", ros::Time(0));
            geometry_msgs::Twist tw;

            tw.linear.x = 0.5 * sqrt(pow(tfs.transform.translation.x, 2) + pow(tfs.transform.translation.y, 2));
            tw.angular.z = 4 * atan2(tfs.transform.translation.y, tfs.transform.translation.x);

            pub.publish(tw);
        }
        catch (const std::exception &e)
        {
            ROS_INFO("%s", e.what());
        }

        r.sleep();
        ros::spinOnce();
    }
    return 0;
}
