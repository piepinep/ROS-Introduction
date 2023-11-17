#! /usr/bin/env python3

import rospy
from services_communication.srv import *

"""
    服务端:解析客户端请求,产生响应
    1. 导包
    2. 初始化ros节点
    3. 创建服务端对象
    4. 处理请求(回调函数)
    5. spin()
"""

def callback(req: TwoIntSumRequest):
    a, b = req.int1, req.int2
    res = TwoIntSumResponse()
    res.sum = a + b
    rospy.loginfo("input a = %d, b = %d, sum = %d", a, b, res.sum)
    return res

if __name__ == "__main__":
    
    rospy.init_node("services_server_py")
    
    server = rospy.Service("services", TwoIntSum, callback)
    
    rospy.spin()
    