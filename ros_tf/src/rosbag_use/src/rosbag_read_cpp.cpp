#include "ros/ros.h"
#include "rosbag/bag.h"
#include "std_msgs/String.h"
#include "rosbag/view.h"

/**
 * 使用rosbag从磁盘读数据
 */

int main(int argc, char *argv[])
{
    setlocale(LC_ALL, "");

    ros::init(argc, argv, "rosbag_read_cpp");
    ros::NodeHandle nh;

    // 新建bag对象
    rosbag::Bag bag;

    // 打开bag文件
    bag.open("begs/hello.bag", rosbag::BagMode::Read);

    std_msgs::String msg;
    msg.data = "hello world";

    // 取出话题，时间戳和消息
    rosbag::View v(bag);

    for (auto &m : v)
    {
        std::string topic = m.getTopic();
        ros::Time timestamp = m.getTime();
        std_msgs::StringPtr p = m.instantiate<std_msgs::String>();
        ROS_INFO("\n解析的内容:\n"
                 "topic:%s\n"
                 "timestamp:%.2f\n"
                 "msg:%s",
                 topic.c_str(), timestamp.toSec(), p->data.c_str());
    }

    // 关闭 bag 文件
    bag.close();
    return 0;
}
