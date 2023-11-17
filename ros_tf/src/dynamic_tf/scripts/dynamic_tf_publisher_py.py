#! /usr/bin/env python3

# 需要先启动 turtlesim gui 节点
# rosrun turtlesim turtlesim_node

import rospy
from turtlesim.msg import Pose
import tf_conversions as tfc
import tf2_ros
from geometry_msgs.msg import TransformStamped


tb = tf2_ros.TransformBroadcaster()
def sub2pub(pose: Pose):
    
    tfs = TransformStamped()
    
    tfs.header.frame_id = "world"
    tfs.header.stamp = rospy.Time.now()
    
    tfs.child_frame_id = "turtle1"
    
    tfs.transform.translation.x = pose.x
    tfs.transform.translation.y = pose.y
    tfs.transform.translation.z = 0
    
    quat = tfc.transformations.quaternion_from_euler(0, 0, pose.theta)
    tfs.transform.rotation.x = quat[0]
    tfs.transform.rotation.y = quat[1]
    tfs.transform.rotation.z = quat[2]
    tfs.transform.rotation.w = quat[3]

    tb.sendTransform(tfs)


if __name__ == "__main__":
    rospy.init_node("dynamic_tf_publiser_py")
    sub = rospy.Subscriber("/turtle1/pose", Pose, sub2pub, queue_size=10)
    rospy.spin()
    