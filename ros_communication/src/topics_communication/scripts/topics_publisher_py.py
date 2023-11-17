#! /usr/bin/env python

import rospy
from std_msgs.msg import String

"""
1. 导包
2. 初始化ros节点
    rospy.init
3. 创建发布者对象
    rospy.Publisher()
4. 编写发送逻辑并发送数据
"""


if __name__ == "__main__":
    
    rospy.init_node("topics_publisher_py")
    
    publisher = rospy.Publisher("topics", String, queue_size=10)
    
    msg = String()
    rate = rospy.Rate(0.5)
    i = 0
    while not rospy.is_shutdown():
        
        msg.data = "this is the " + str(i) + " times"
        i += 1
        publisher.publish(msg)
        
        rate.sleep()
    
    