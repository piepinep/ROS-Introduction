#! /usr/bin/env python3

import rospy
from topic_custom_msg.msg import Person

"""
1. 导包
2. 初始化ros节点
    rospy.init
3. 创建发布者对象
    rospy.Publisher()
4. 编写发送逻辑并发送数据
"""


if __name__ == "__main__":
    
    rospy.init_node("topics_custom_publisher_py")
    
    publisher = rospy.Publisher("topic_custom", Person, queue_size=10)
    
    msg = Person()
    msg.name = "lihua"
    rate = rospy.Rate(1)
    i = 0
    while not rospy.is_shutdown():
        
        msg.age = i
        msg.height = i * 10
        i += 1
        publisher.publish(msg)
        rate.sleep()
    
    