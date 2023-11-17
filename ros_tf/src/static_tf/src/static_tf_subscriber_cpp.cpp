#include "ros/ros.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "geometry_msgs/PointStamped.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

/**
 * 订阅发布方的坐标系相对关系，传入一个坐标系，使用tf变换
 * 1. 包含头文件
 * 2. 设置编码，节点初始化
 * 3. 创建订阅者对象
 * 4. 组织坐标系数据点
 * 5. 转换算法
 * 6. 输出
 */

int main(int argc, char *argv[])
{
    setlocale(LC_ALL, "");
    ros::init(argc, argv, "static_tf_subscriber_cpp");
    ros::NodeHandle nh;

    // 创建 buffer 对象，数据会存储在缓存中
    tf2_ros::Buffer buff;
    // 自动订阅相关数据，不需要手动编写
    // 订阅到的数据是变换信息，订阅者是利用变换信息进行变换
    tf2_ros::TransformListener sub(buff);

    // 待变换的点
    geometry_msgs::PointStamped ps;
    // 该点的参考坐标系
    ps.header.frame_id = "laser";
    // 时间戳
    ps.header.stamp = ros::Time::now();

    // 该点坐标
    ps.point.x = 2.0;
    ps.point.x = 3.0;
    ps.point.x = 5.0;

    ros::Rate r(2);

    while (ros::ok())
    {
        try
        {
            geometry_msgs::PointStamped point_base;
            point_base = buff.transform(ps, "base_link"); // 调用时必须包含 "tf2_geometry_msgs/tf2_geometry_msgs.h"
            ROS_INFO("转换后的数据:(%.2f,%.2f,%.2f),参考的坐标系是:%s",
                     point_base.point.x,
                     point_base.point.y,
                     point_base.point.z,
                     point_base.header.frame_id.c_str());
        }
        catch (const std::exception &e)
        {
            // 首次调用buffer可能为空
            ROS_INFO("程序异常.....");
        }
        r.sleep();
        ros::spinOnce();
    }
    return 0;
}
