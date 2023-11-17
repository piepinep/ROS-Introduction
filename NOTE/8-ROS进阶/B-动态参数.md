参数服务器进行修改时,如果节点不重新访问,就不能获得修改后的数据.如果需要能够即时更新,那么就可以不重新启动节点也可以获取新的数据

ros中针对这种场景给出了解决方案: dynamic reconfigure

动态参数可以动态更新,其原因在于采用了C/S架构(客户端/服务器架构),每次获取数据都是一次请求

# 自定义cfg文件

1. 新建功能包,包含 `roscpp rospy std_msgs dynamic_reconfigure` 依赖

2. 创建cfg目录

3. 在cfg目录下创建xxx.cfg文件(实际上是一个python文件)

```cfg
#! /usr/bin/env python

"""
    1. 导包
    2. 创建一个参数生成器
    3. 向生成器添加参数
    4. 配置节点并退出
"""

from dynamic_reconfigure.parameter_generator_catkin import *

gen = ParameterGenerator()
# add(  name: Any,  参数名
#       paramtype: Any, 参数类型
#       level: Any, 掩码，标记是否修改过，一般传0
#       description: Any, 参数描述信息
#       default: Any | None = None, 默认值
#       min: Any | None = None, 最小值
#       max: Any | None = None, 最大值
#       edit_method: str = "" 修改方法
# ) -> None
gen.add("int_param", int_t, 0, "整型参数", 0, 0, 100)
# generate(
#     pkgname: Any, 包名
#     nodename: Any, 节点名
#     name: Any, 文件名
# ) -> None
exit(gen.generate("dycfg", "dycfg_client", "dy"))
```

4. 对cfg文件添加可执行权限

5. 修改cmake文件

找到 `generate_dynamic_reconfigure_options` 语句,并在相关内容中添加自定义的cfg文件

```cmake
generate_dynamic_reconfigure_options(
  cfg/dy.cfg
)
```

6. 编译项目

# 动态参数服务端

## CPP

```cpp
#include "ros/ros.h"
#include "dynamic_reconfigure/server.h"
#include "dycfg/dyConfig.h"

/**
 * 1. 包含头文件
 * 2. 初始化节点
 * 3. 创建服务器对象
 * 4. 回调对象解析
 * 5. spin
*/

// const boost::function<void (dycfg::dyConfig &, uint32_t level)> &callback
// 需要使用boost::bind进行绑定/测试后,不绑定也可以使用
void cb(dycfg::dyConfig &dyc, uint32_t level)
{
    ROS_INFO("修改后的参数是 %d",dyc.int_param);
}

int main(int argc, char *argv[])
{
    setlocale(LC_ALL, "");
    ros::init(argc, argv, "dy_cfg_cpp");

    dynamic_reconfigure::Server<dycfg::dyConfig> server;

    // server.setCallback(boost::bind(&cb, _1, _2));
    server.setCallback(cb);

    ros::spin();
    return 0;
}

```

## Python

```python
#! /usr/bin/env python

"""
    1. 导包
    2. 初始化节点
    3. 创建服务器对象
    4. 回调函数解析
    5. spin
"""

import rospy
from dynamic_reconfigure.server import Server
from dycfg.cfg import dyConfig

def cb(dyc:dyConfig, level: int):
    rospy.loginfo("修改后的参数为: %d", dyc.int_param)
    return dyc

if __name__ == "__main__":
    
    rospy.init_node("dy_server_py")
    
    server = Server(dyConfig, cb)
    
    rospy.spin()

```

其余类型的动态参数不再编写,可以参考[动态参数](http://www.autolabor.com.cn/book/ROSTutorials/di-7-zhang-ji-qi-ren-dao-822a28-fang-771f29/1002.html)

