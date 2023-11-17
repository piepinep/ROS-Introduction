#include "ros/ros.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

// 先通过publish_tf.launch发布坐标系,后运行此文件查看变换结果

/**
 * 订阅方实现
 * 1. 计算两个静态坐标系的相对关系
 * 2. 计算某个坐标系中的点在另一个坐标系中的坐标值
 *
 * 流程：
 * 1. 包含头文件
 * 2. 编码，初始化
 * 3. 创建订阅对象
 * 4. 编写解析逻辑
 * 5. spinOnce
 */

int main(int argc, char *argv[])
{
    setlocale(LC_ALL, "");
    ros::init(argc, argv, "static_multi_tf_cpp");
    ros::NodeHandle nh;

    tf2_ros::Buffer buff;
    tf2_ros::TransformListener listener(buff);

    ros::Rate r(2);

    geometry_msgs::PointStamped ps1;

    ps1.header.frame_id = "son1";
    ps1.header.stamp = ros::Time::now();

    ps1.point.x = 1;
    ps1.point.y = 2;
    ps1.point.z = 3;

    while (ros::ok())
    {
        try
        {
            // 计算 son1 和 son2的相对关系
            geometry_msgs::TransformStamped tfs = buff.lookupTransform("son2", "son1", ros::Time(0));
            ROS_INFO("\n坐标系相对关系:\n"
                     "目标坐标系：%s, 基础坐标系: %s\n"
                     "偏移量:(%.2f, %.2f, %.2f)\n"
                     "四元数:(%.2f, %.2f, %.2f, %.2f)\n",
                     tfs.header.frame_id.c_str(), tfs.child_frame_id.c_str(),
                     tfs.transform.translation.x, tfs.transform.translation.y, tfs.transform.translation.z,
                     tfs.transform.rotation.x, tfs.transform.rotation.y, tfs.transform.rotation.z, tfs.transform.rotation.w);

            // son1 与 son2 点的转换，与两个坐标系转换相同
            geometry_msgs::PointStamped nps = buff.transform(ps1, "son2");
            ROS_INFO("转换后的坐标为(%.2f, %.2f, %.2f), 基准坐标系是:%s\n",
                     nps.point.x, nps.point.y, nps.point.z, nps.header.frame_id.c_str());
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
