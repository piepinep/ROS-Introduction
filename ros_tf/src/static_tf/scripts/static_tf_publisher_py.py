#! /usr/bin/env python

import rospy
from tf2_ros import StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped
import tf_conversions as tfc


if __name__ == "__main__":
    
    rospy.init_node("static_tf_publisher_py")
    
    # 创建发布者对象
    pub = StaticTransformBroadcaster()
    
    # 创建转换坐标系
    tfs = TransformStamped()
    
    # 设置基础坐标系信息
    tfs.header.frame_id = "base_link"
    tfs.header.stamp = rospy.Time.now()
    
    # 设置子坐标系信息
    tfs.child_frame_id = "laser"
    tfs.transform.translation.x = 0.2
    tfs.transform.translation.y = 0.0
    tfs.transform.translation.z = 0.5

    # 进行Euler和四元数转换
    quat = tfc.transformations.quaternion_from_euler(0, 0, 0)
    tfs.transform.rotation.x = quat[0]
    tfs.transform.rotation.y = quat[1]
    tfs.transform.rotation.z = quat[2]
    tfs.transform.rotation.w = quat[3]
    
    # 发布转换坐标系
    pub.sendTransform(tfs)
    
    rospy.spin()
    
    