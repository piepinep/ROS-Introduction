#! /usr/bin/env python

import rospy
from std_msgs.msg import String

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

def callback(msg:String):
    rospy.loginfo(msg.data)

if __name__ == "__main__":
    
    rospy.init_node("topics_subscriber_py")
    
    subscribe = rospy.Subscriber("topics", String, callback=callback)
    
    rospy.spin()
    
    