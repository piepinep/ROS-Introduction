#include "ros/ros.h"
#include "tf2_ros/static_transform_broadcaster.h"
#include "geometry_msgs/TransformStamped.h"
#include "tf2/LinearMath/Quaternion.h"

/**
 * 发布两个坐标系的相对关系
 * 1. 包含头文件
 * 2. 设置编码，节点初始化
 * 3. 创建发布者对象
 * 4. 组织被发布的消息
 * 5. 发布数据
 * 6. spin
 */

int main(int argc, char *argv[])
{
    setlocale(LC_ALL, "");
    ros::init(argc, argv, "static_tf_publisher_cpp");
    ros::NodeHandle nh;

    tf2_ros::StaticTransformBroadcaster pub;

    geometry_msgs::TransformStamped tf;
    tf.header.stamp = ros::Time::now();
    tf.header.frame_id = "base_link";
    tf.child_frame_id = "laser";

    // 设置偏移量 x，y，z
    tf.transform.translation.x = 0.2;
    tf.transform.translation.y = 0.0;
    tf.transform.translation.z = 0.5;

    // 设置四元数 x，y，z，w
    // 四元数设置需要考虑欧拉角
    tf2::Quaternion quat;
    quat.setRPY(0, 0, 0);

    tf.transform.rotation.x = quat.getX();
    tf.transform.rotation.y = quat.getY();
    tf.transform.rotation.z = quat.getZ();
    tf.transform.rotation.w = quat.getW();

    // 发布数据
    pub.sendTransform(tf);

    ros::spin();
    return 0;
}
