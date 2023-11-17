#! /usr/bin/env python

import rospy
from actionlib import SimpleActionServer
from action_use.msg import *

"""
    1. 导包
    2. 初始化 ros 节点
    3. 单独封装类
    4. 类中创建action服务端对象
    5. 处理数据
    6. spin
"""

class MyAction:
    
    def __init__(self, topic) -> None:
        self.server = SimpleActionServer(topic, TwoIntSumAction, self.cb, False)
        self.server.start()
        
    def cb(self, goal: TwoIntSumGoal):
        a = goal.a
        b = goal.b
        result = 0

        r = rospy.Rate(1)
        for i in range(a, b + 1):
            result += i
            fb = TwoIntSumFeedback()
            fb.percent = (i - a + 1) / (b - a + 1)
            self.server.publish_feedback(fb)
            r.sleep()
        
        res = TwoIntSumResult()
        res.result = result
        self.server.set_succeeded(res)
        
        
if __name__ == "__main__":
    rospy.init_node("action_server_py")
    
    ma = MyAction("actions")
    
    rospy.spin()
