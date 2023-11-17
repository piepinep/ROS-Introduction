#! /usr/bin/env python

#  先通过publish_tf.launch发布坐标系,后运行此文件查看变换结果


import rospy
from tf2_ros import Buffer, TransformListener
from geometry_msgs.msg import PointStamped, TransformStamped
from tf2_geometry_msgs import tf2_geometry_msgs

if __name__ == "__main__":
    
    rospy.init_node("dynamic_subp")
    
    buff = Buffer()
    sub = TransformListener(buff)
    
    ps = tf2_geometry_msgs.PointStamped()
    
    ps.header.frame_id = "son1"
    ps.header.stamp = rospy.Time.now()
    
    ps.point.x = 1
    ps.point.y = 2
    ps.point.z = 3
    
    r = rospy.Rate(2)
    while not rospy.is_shutdown():
        
        try:
            ntfs: TransformStamped = buff.lookup_transform("son2", "son1", rospy.Time(0))
            rospy.loginfo("\n1. %s\n2. %s\n(%.2f, %.2f, %.2f)",
                          ntfs.header.frame_id, ntfs.child_frame_id, 
                          ntfs.transform.translation.x,ntfs.transform.translation.y,ntfs.transform.translation.z
                          )
            
            nps: PointStamped = buff.transform(ps, "son2")
            rospy.loginfo("变换后坐标: (%.2f, %.2f, %.2f),参考的坐标系是:%s",
                          nps.point.x,
                          nps.point.y,
                          nps.point.z,
                          nps.header.frame_id
                          )
        except Exception as e:
            rospy.loginfo("%s", str(e))
        
        r.sleep()
        
        