#include "ros/ros.h"
#include "turtlesim/Pose.h"
#include "tf2/LinearMath/Quaternion.h"
#include "geometry_msgs/TransformStamped.h"
#include "tf2_ros/transform_broadcaster.h"

/**
 * 需要先启动 turtlesim gui 节点
 * rosrun turtlesim turtlesim_node
 * 
 * 既需要订阅乌龟的位姿信息，也需要发布坐标系的变化信息
 * 订阅乌龟信息
 *     话题：/turtle1/pose
 *     消息：/turtlesim/Pose
 * 流程：
 * 1. 包含头文件
 * 2. 初始化
 * 3. 创建订阅对象
 * 4. 回调函数处理消息，并进行转换后发布
 * 5. spin
 *
 */

void sub2pub(const turtlesim::PoseConstPtr &cptr)
{
    geometry_msgs::TransformStamped tfs;
    static tf2_ros::TransformBroadcaster tb;

    tfs.header.frame_id = "world";
    tfs.header.stamp = ros::Time::now();

    tfs.child_frame_id = "turtle1";
    tfs.transform.translation.x = cptr->x;
    tfs.transform.translation.y = cptr->y;
    tfs.transform.translation.z = 0;

    tf2::Quaternion quat;
    quat.setRPY(0, 0, cptr->theta);
    tfs.transform.rotation.x = quat.getX();
    tfs.transform.rotation.y = quat.getY();
    tfs.transform.rotation.z = quat.getZ();
    tfs.transform.rotation.w = quat.getW();

    tb.sendTransform(tfs);
}

int main(int argc, char *argv[])
{
    setlocale(LC_ALL, "");

    ros::init(argc, argv, "dynamic_tf_publisher_cpp");
    ros::NodeHandle nh;

    ros::Subscriber subt = nh.subscribe("/turtle1/pose", 10, sub2pub);

    ros::spin();
    return 0;
}
