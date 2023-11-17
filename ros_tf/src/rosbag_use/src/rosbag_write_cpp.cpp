#include "ros/ros.h"
#include "rosbag/bag.h"
#include "std_msgs/String.h"

/**
 * 使用rosbag向磁盘写出数据
 */

int main(int argc, char *argv[])
{
    setlocale(LC_ALL, "");

    ros::init(argc, argv, "rosbag_write_cpp");
    ros::NodeHandle nh;

    // 新建bag对象
    rosbag::Bag bag;

    // 打开bag文件
    bag.open("begs/hello.bag", rosbag::BagMode::Write);

    std_msgs::String msg;
    msg.data = "hello world";

    // 写入消息
    // "topic", timestamp, msg
    bag.write("/chatter", ros::Time::now(), msg);
    bag.write("/chatter", ros::Time::now(), msg);
    bag.write("/chatter", ros::Time::now(), msg);
    bag.write("/chatter", ros::Time::now(), msg);

    // 关闭 bag 文件
    bag.close();
    return 0;
}
