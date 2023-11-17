服务通信是ROS中一种极其常用的通信模式，服务通信是基于**请求响应**模式的，是一种应答机制。也即一个节点A向另一个节点B发送请求，B接收处理请求并产生响应结构返回给A

服务通信适用于对实时性有要求，具有一定逻辑处理的应用场景。

# 服务通信的理论模型

![](C-服务通信-理论模型.png)

角色：
* master：管理者，用于保存客户端和服务器的注册信息，并匹配话题相同的服务器和客户端
* Server：服务器，响应信息
* Client：客户端，请求信息

过程：
0. server注册，server会通过RPC向master发送注册信息，包含服务的名称，master会将这些信息写入到注册表中
1. client注册，client会通过RPC向master发送注册信息，包含要请求的服务名称，master也会将信息写入注册表
2. master进行信息匹配，使用RPC向client发送server的tcp地址
3. client通过tcp与server建立连接，并请求数据
4. server响应连接，并返回结果

>注意：
>1. 客户端请求处理时，服务器需要启动
>2. 客户端和服务器都可以存在多个

# 自定义服务消息 srv

srv文件中可利用类型与msg文件一致，且srv实现流程和msg实现类似

## 定义srv文件

srv文件包含两部分，两部分通过三个减号(`---`)分割, 减号上方为客户端发送请求的数据格式,下方为服务器响应的数据格式

例如如下格式:
```txt
int32 num1
int32 num2
---
int32 num
```

## 配置文件

### 修改package.xml

添加两个标签: 

* `<build_depend>message_generation</build_depend>`
* `<exec_depend>message_runtime</exec_depend>`

```xml
  <buildtool_depend>catkin</buildtool_depend>
  <build_depend>roscpp</build_depend>
    ...
  <build_depend>message_generation</build_depend>
    ...
  <exec_depend>roscpp</exec_depend>
    ...
  <exec_depend>message_runtime</exec_depend>
```

### 配置cmakelists.txt

1. 修改 `find_package`

```cmake
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation # 新加参数
)
```

2. 修改 `add_service_files`

```cmake
## Generate services in the 'srv' folder
## add_service_files(
##   FILES
##   Service1.srv
##   Service2.srv
## )
add_service_files(
  FILES
  Addints.srv
)
```

3. 修改 `generate_messages`

```cmake
## Generate added messages and services with any dependencies listed here
## generate_messages(
##   DEPENDENCIES
##   std_msgs
## )
generate_messages(
  DEPENDENCIES
  std_msgs
)
```

4. 修改 `catkin_package`

```cmake
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES sercli
 CATKIN_DEPENDS roscpp rospy std_msgs message_runtime
#  DEPENDS system_lib
)
```

# 服务通信的实现

## VSCODE配置

可参考 [#vscode配置](/ROS/2-ROS通信/B-话题通信.md) 中的`VSCODE配置`

## CPP实现

### 服务器实现

```cpp
#include "ros/ros.h"
#include "sercli/Addints.h"

/**
 * 服务端实现,解析客户端的数据并响应
 * 1. 包含相关头文件
 * 2. 初始化ros节点
 * 3. 创建节点句柄
 * 4. 创建服务器对象
 * 5. 处理请求并产生响应
 * 6. spin()
*/

bool callback(sercli::Addints::Request &req, sercli::Addints::Response &res)
{
    // 处理请求
    int num1 = req.num1, num2 = req.num2;

    // 组织响应
    res.sum = num1 + num2;
    
    ROS_INFO("num1 = %d, num2 = %d, sum = %d", num1, num2, res.sum);
    return true;
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "server_cpp");

    ros::NodeHandle nh;

    ros::ServiceServer ss = nh.advertiseService("sercli_cpp", callback);

    ros::spin();
    return 0;
}
```

### 客户端实现

```cpp
#include "ros/ros.h"
#include "sercli/Addints.h"

/**
 * 服务端实现,解析客户端的数据并响应
 * 1. 包含相关头文件
 * 2. 初始化ros节点
 * 3. 创建节点句柄
 * 4. 创建客户端对象
 * 5. 提交请求
 * 6. 处理响应
 */

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "client_cpp");

    ros::NodeHandle nh;

    ros::ServiceClient sc = nh.serviceClient<sercli::Addints>("sercli_cpp");

    sercli::Addints ai;
    ai.request.num1 = atoi(argv[argc - 2]);
    ai.request.num2 = atoi(argv[argc - 1]);

    if (sc.call(ai))
    {
        ROS_INFO("response = %d", ai.response.sum);
    }
    else
    {
        ROS_INFO("failed");
    }

    return 0;
}
```

### cmake配置

修改 `add_executable` 和 `target_link_libraries`,此外
`add_dependencies`第二个参数可以使用默认值,也可以使用 `${PROJECT_NAME}_gencpp`

### 客户端先启动处理

如果先启动客户端,就会产生异常,更希望客户端检测是否启动服务器,如果没有就挂起并等待服务器

解决方法:
1. 在发送请求前,调用判断服务器状态函数

```cpp
// ...
sc.waitForExistence();
sc.call(ai)
// ...
```

2. 在发送请求前,调用请求服务器函数

```cpp
// ...
ros::service::waitForService("sercli_cpp"); // 这里是话题
sc.call(ai)
// ...
```

## PYTHON实现

### 客户端实现

```python
#! /usr/bin/env python

import rospy
from sercli.srv import Addints, AddintsRequest, AddintsResponse

"""
    服务端:解析客户端请求,产生响应
    1. 导包
    2. 初始化ros节点
    3. 创建服务端对象
    4. 处理请求(回调函数)
    5. spin()
"""

def callback(req: AddintsRequest):
    num1, num2 = req.num1, req.num2
    res = AddintsResponse()
    res.sum = num1 + num2
    return res

if __name__ == "__main__":
    
    rospy.init_node("server_py")
    
    server = rospy.Service("sercli_py", Addints, callback)
    
    rospy.spin()
```

### 服务器实现

```python
#! /usr/bib/env python

import rospy
from sercli.srv import Addints, AddintsRequest
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
    
    rospy.init_node("client_py")
    sp = rospy.ServiceProxy("sercli_py", Addints)
    
    sp.wait_for_service()

    req = AddintsRequest()
    
    req.num1 = int(sys.argv[-2])
    req.num2 = int(sys.argv[-1])
    
    res = sp.call(req)
    
    rospy.loginfo("response = %d", res.sum)
    
```

### cmake配置

与之前的配置方法一致

### 客户端先启动

```python
...
sp.wait_for_service()
sp.call(*argc)
...
```

```python
...
rospy.wait_for_service("sercli_py")
sp.call(*argc)
...
```

