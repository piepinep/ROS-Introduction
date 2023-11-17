#! /usr/bib/env python

import rospy
from services_communication.srv import *
import sys

"""
    客户端:发送请求并处理响应
    1. 导包
    2. 初始化ros节点
    3. 创建客户端对象
    4. 发送请求
    5. 获取响应并处理
"""

if __name__ == "__main__":
    
    if(len(sys.argv) != 3):
        exit(1)
    
    rospy.init_node("services_client_py")
    sp = rospy.ServiceProxy("services", TwoIntSum)
    
    sp.wait_for_service()
    req = TwoIntSumRequest()
    
    req.int1 = int(sys.argv[-2])
    req.int2 = int(sys.argv[-1])
    
    res = sp.call(req)
    rospy.loginfo("response = %d", res.sum)
    