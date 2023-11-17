#! /usr/bin/env python3

"""
    1. 导包
    2. 初始化节点
    3. 创建服务器对象
    4. 回调函数解析
    5. spin
"""

import rospy
from dynamic_reconfigure.server import Server
from dynamic_reconfigure_use.cfg import dynamic_reconfigure_clientConfig

def cb(dyc:dynamic_reconfigure_clientConfig, level: int):
    rospy.loginfo("修改后的参数为: %d", dyc.int_param)
    return dyc

if __name__ == "__main__":
    
    rospy.init_node("dynamic_reconfigure_server_py")
    
    server = Server(dynamic_reconfigure_clientConfig, cb)
    
    rospy.spin()
