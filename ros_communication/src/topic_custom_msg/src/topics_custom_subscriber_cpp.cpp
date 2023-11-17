#include "ros/ros.h"
#include "topic_custom_msg/Person.h"

/*

订阅方实现：
1. 包含头文件
    ros/ros.h -> 提供ros的基本功能
    std_msgs/String.h -> 通信的数据载体
2. 初始化
    ros节点 使用 ros::init
3. 创建节点句柄
    节点句柄用于创建通信对象 ros::NodeHandle nh;
4. 创建订阅者者对象
    发布者对象由节点句柄创建
    nh.subscribe<T>

5. 创建回调函数，处理数据
    void (*fp) (const shared_ptr<const T> &cptr)
6. 编写主程序逻辑
    包含spin函数
*/

void callback(const topic_custom_msg::Person::ConstPtr &ptr)
{
    ROS_INFO("name = %s, age = %u, height = %.2f", ptr->name.c_str(), ptr->age, ptr->height);
}

int main(int argc, char *argv[])
{
    // 初始化ros节点
    ros::init(argc, argv, "topics_custom_subscriber_cpp");

    // 创建节点句柄
    ros::NodeHandle nh;

    // 根据句柄创建订阅者对象
    ros::Subscriber subscribe = nh.subscribe("topic_custom", 10, callback);
    // 自旋
    ros::spin();

    return 0;
}