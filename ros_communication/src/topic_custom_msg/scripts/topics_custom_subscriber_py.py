#! /usr/bin/env python3

import rospy
from topic_custom_msg.msg import Person

"""
1. 导包
2. 初始化ros节点
    rospy.init
3. 创建订阅者对象
    rospy.Subscriber()
4. 编写回调函数
    def callback(msg)
5. 进行自旋
    spin
"""

def callback(msg:Person):
    rospy.loginfo("name = %s, age = %u, height = %.2f", msg.name, msg.age, msg.height)

if __name__ == "__main__":
    
    rospy.init_node("topics_custom_subscriber_py")
    
    subscribe = rospy.Subscriber("topic_custom", Person, callback=callback)
    
    rospy.spin()
    
    