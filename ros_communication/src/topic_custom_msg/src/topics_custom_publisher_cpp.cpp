#include "ros/ros.h"
#include "topic_custom_msg/Person.h"

/*
发布方实现：
1. 包含头文件
    ros/ros.h -> 提供ros的基本功能
    pubsub/Person.h -> 通信的数据载体
2. 初始化
    ros节点 使用 ros::init
3. 创建节点句柄
    节点句柄用于创建通信对象 ros::NodeHandle nh;
4. 创建发布者对象
    发布者对象由节点句柄创建
    nh.advertise<T>, T是数据载体（消息类型）
5. 编写逻辑并发布数据
    ros::ok 是确认当前节点是否正常
*/

int main(int argc, char *argv[])
{
    // 初始化ros节点
    ros::init(argc, argv, "topics_custom_publisher_cpp");

    // 创建节点句柄
    ros::NodeHandle nh;

    // 根据句柄创建发布者对象
    ros::Publisher publisher = nh.advertise<topic_custom_msg::Person>("topic_custom", 10);

    // 创建数据格式
    topic_custom_msg::Person msg;
    msg.name = "liming";

    // 设置频率
    ros::Rate rate(1);

    // 逻辑结构
    int i = 0;
    while (ros::ok())
    {
        msg.age = i;
        msg.height = i * 10;
        i++;
        publisher.publish(msg);
        rate.sleep();
        ros::spinOnce();
    }

    return 0;
}