#! /usr/bin/env python

import rospy
from tf2_ros import Buffer, TransformListener
from geometry_msgs.msg import PointStamped
from tf2_geometry_msgs import tf2_geometry_msgs

if __name__ == "__main__":
    
    rospy.init_node("static_tf_subscriber_py")
    
    buff = Buffer()
    sub = TransformListener(buff)
    
    ps = tf2_geometry_msgs.PointStamped()
    
    ps.header.frame_id = "laser"
    ps.header.stamp = rospy.Time.now()
    
    ps.point.x = 2
    ps.point.y = 3
    ps.point.z = 5
    
    r = rospy.Rate(2)
    while not rospy.is_shutdown():
        
        try:
            nps: tf2_geometry_msgs.PointStamped = buff.transform(ps, "base_link")
            rospy.loginfo("变换后坐标: (%.2f, %.2f, %.2f),参考的坐标系是:%s",
                          nps.point.x,
                          nps.point.y,
                          nps.point.z,
                          nps.header.frame_id
                          )
        except Exception as e:
            rospy.loginfo("%s", str(e))
        
        r.sleep()
        
        