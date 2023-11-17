# 话题通信

话题通信是ROS中使用频率最高的一种通信方式,话题同学是基于**发布订阅模式**的,也即一个节点发布消息,另一个 节点订阅该消息.

话题通信常用于不断更新的,少逻辑处理的数据通信场景

# 话题通信理论模型

![](B-话题通信-理论模型.png)

话题通信分为三个角色：
* master：管理者，master可以根据话题建立发布者和订阅者的连接
* talker：发布者，发布者发布信息
* listener：订阅者，订阅者根据订阅获得信息

基本流程：
0. Talker向Master提交话题和RPC地址进行注册
1. listener向master提交话题，进行注册。listener只需要提交自己需要订阅的信息即可。
2. master根据注册表中的信息进行匹配，并通过RPC向listener发送talker的RPC地址信息
3. listener通过RCP向talker发送请求信息，获取talker的TCP地址
4. talker根据接收到的RPC地址，通过RPC向listener发送确认信息，并发送自身的TCP地址

5. listener发起向talker建立tcp连接
6. talker与listener进行数据通信

* 0-4使用**RPC**通信，5-6使用**TCP**通信
* talker和listener无先后要求
* talker和listener无数量限制
* tcp连接与master独立，可以不依赖于master
* 大部分已经被封装只需要关注以下几条：
	1. 话题设置
	2. 发布者如何实现
	3. 订阅者如何实现
	4. 消息载体是什么样的


# 实现一个简单的话题通信

## CPP实现

建立工作空间与包 [编写一个简单的ROS程序](/ROS/1-ROS概述与简单应用/A-编写一个简单的ROS程序.md)

```cpp
ros::Duration(double d); //持续时间
ros::Rate(double hz); // 频率
ros::spin(); // 自旋
ros::spinOnce(); //自旋一次
```
### 发布方实现

```cpp
#include "ros/ros.h"
#include "std_msgs/String.h"

/*

发布方实现：
1. 包含头文件
    ros/ros.h -> 提供ros的基本功能
    std_msgs/String.h -> 通信的数据载体
2. 初始化
    ros节点 使用 ros::init
3. 创建节点句柄
    节点句柄用于创建通信对象 ros::NodeHandle nh;
4. 创建发布者对象
    发布者对象由节点句柄创建
    nh.advertise<T>, T是数据载体（消息类型）
5. 编写逻辑并发布数据
    ros::ok 是确认当前节点是否正常
*/

int main(int argc, char *argv[])
{
    // 初始化ros节点
    ros::init(argc, argv, "public_cpp");

    // 创建节点句柄
    ros::NodeHandle nh;

    // 根据句柄创建发布者对象
    ros::Publisher publisher = nh.advertise<std_msgs::String>("cpp", 10);

    // 创建数据格式
    std_msgs::String msgs;

    // 设置频率
    ros::Rate rate(2);

    // 逻辑结构
    int i = 0;
    while (ros::ok())
    {
        msgs.data = "i am a cpp publisher, this is the " + std::to_string(i++) + "times";
        publisher.publish(msgs);
        rate.sleep();
		ros::spinOnce();
    }

    return 0;
}
```

### 订阅方实现

```cpp
#include "ros/ros.h"
#include "std_msgs/String.h"

/*

订阅方实现：
1. 包含头文件
    ros/ros.h -> 提供ros的基本功能
    std_msgs/String.h -> 通信的数据载体
2. 初始化
    ros节点 使用 ros::init
3. 创建节点句柄
    节点句柄用于创建通信对象 ros::NodeHandle nh;
4. 创建订阅者者对象
    发布者对象由节点句柄创建
    nh.subscribe<T>
5. 创建回调函数，处理数据
    void (*fp) (const shared_ptr<const T> &cptr)
6. 编写主程序逻辑
    包含spin函数
*/

void callback(const std_msgs::String::ConstPtr &ptr)
{
    ROS_INFO("%s\n", ptr->data.c_str());
}

int main(int argc, char *argv[])
{
    // 初始化ros节点
    ros::init(argc, argv, "subscribe_cpp");

    // 创建节点句柄
    ros::NodeHandle nh;

    // 根据句柄创建订阅者对象
    ros::Subscriber subscribe = nh.subscribe("cpp", 10, callback);
    // 自旋
    ros::spin();

    return 0;
}
```

### 计算图

![](B-话题通信-cpp计算图.png)

## PYTHON实现

### 发布方实现

```python
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
    
    rospy.init_node("publish_py")
    
    publisher = rospy.Publisher("py", String, queue_size=10)
    
    msg = String()
    rate = rospy.Rate(0.5)
    i = 0
    while not rospy.is_shutdown():
        
        msg.data = "this is the " + str(i) + " times"
        i += 1
        publisher.publish(msg)
        
        rate.sleep()
    
    
```

### 订阅方实现

```python
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
    
    rospy.init_node("subscribe_py")
    
    subscribe = rospy.Subscriber("py", String, callback=callback)
    
    rospy.spin()
    
```

### 计算图

![](B-话题通信-python计算图.png)

# 不同的语言也可以进行通信!

# 自定义msg

在ROS中,数据载体是非常重要的一部分,ROS通过std_msgs封装了一些原生的数据类型,比如String, Int32, Int64, Char, Bool,..(首字母大写). 但是这些消息类型数据太简单,不能传输很复杂的数据,因此需要自定义数据类型

## 自定义消息文件  .msg 文本文件

.msg文件可以包含很多字段
* int8, int16, int32, int64, uint...
* float32, float64
* string
* time, duration
* other msg files
* variable-length arr[] 或者 fixed-length arr[C]
* Header 包含时间戳等信息

## 定义 .msg 文件

1. 在功能包中新建目录 msg
2. 在msg目录下创建 .msg 文件

![](B-话题通信-Person.msg.png)

## 修改package.xml

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

## 配置cmake

1. 在find_package()中添加 message_generation

```cmake
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation # 新添加的
)
```

2. 找到add_message_files并配置

```cmake
## Generate messages in the 'msg' folder
## add_message_files(
##   FILES
##   Message1.msg
##   Message2.msg
## )

add_message_files(
  FILES
  Person.msg
)
```

3. 找到generate_messages并配置

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

4. 找到catkin_package并配置

```cmake
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES pubsub
  CATKIN_DEPENDS roscpp rospy std_msgs message_runtime
#  DEPENDS system_lib
)
```

## 编译

`ctrl+shift+b`进行编译

* cpp头文件生成位置在 devel/include/pack_name
* python文件生成位置在 devel/lib/python3/dist-packages/msg

# 使用自定义msg

## VSCODE 配置

配置 .vscode目录下的setting.json和c_cpp_properties.json
添加include path

## CPP实现

### 发送方实现

```cpp
#include "ros/ros.h"
#include "pubsub/Person.h"

/*
发布方实现：
1. 包含头文件
    ros/ros.h -> 提供ros的基本功能
    pubsub/Person.h -> 通信的数据载体
2. 初始化
    ros节点 使用 ros::init
3. 创建节点句柄
    节点句柄用于创建通信对象 ros::NodeHandle nh;
4. 创建发布者对象
    发布者对象由节点句柄创建
    nh.advertise<T>, T是数据载体（消息类型）
5. 编写逻辑并发布数据
    ros::ok 是确认当前节点是否正常
*/

int main(int argc, char *argv[])
{
    // 初始化ros节点
    ros::init(argc, argv, "public_cpp");

    // 创建节点句柄
    ros::NodeHandle nh;

    // 根据句柄创建发布者对象
    ros::Publisher publisher = nh.advertise<pubsub::Person>("cpp", 10);

    // 创建数据格式
    pubsub::Person msg;

    // 设置频率
    ros::Rate rate(2);

    // 逻辑结构
    int i = 0;
    while (ros::ok())
    {
        msg.age = i;
        msg.name = "liming";
        msg.height = i * 10;
        publisher.publish(msg);
        rate.sleep();
        ros::spinOnce();
    }

    return 0;
}
```

### 订阅方实现

```cpp
#include "ros/ros.h"
#include "pubsub/Person.h"

/*

订阅方实现：
1. 包含头文件
    ros/ros.h -> 提供ros的基本功能
    std_msgs/String.h -> 通信的数据载体
2. 初始化
    ros节点 使用 ros::init
3. 创建节点句柄
    节点句柄用于创建通信对象 ros::NodeHandle nh;
4. 创建订阅者者对象
    发布者对象由节点句柄创建
    nh.subscribe<T>

5. 创建回调函数，处理数据
    void (*fp) (const shared_ptr<const T> &cptr)
6. 编写主程序逻辑
    包含spin函数
*/

void callback(const pubsub::Person::ConstPtr &ptr)
{
    ROS_INFO("name = %s, age = %ud, height = %.2f", ptr->name.c_str(), ptr->age, ptr->height);
}

int main(int argc, char *argv[])
{
    // 初始化ros节点
    ros::init(argc, argv, "subscribe_cpp");

    // 创建节点句柄
    ros::NodeHandle nh;

    // 根据句柄创建订阅者对象
    ros::Subscriber subscribe = nh.subscribe("cpp", 10, callback);
    // 自旋
    ros::spin();

    return 0;
}
```

### 修改cmakelists文件

除了add_executable和target_link_libraries外,还需要修改add_dependence

```cmake
add_dependencies(${PROJECT_NAME}_pubcmsg ${PROJECT_NAME}_generate_messages_cpp)
add_dependencies(${PROJECT_NAME}_subcmsg ${PROJECT_NAME}_generate_messages_cpp)
```

## PYTHON实现

### 发布方实现

```python
#! /usr/bin/env python

import rospy
from pubsub.msg import Person

"""
1. 导包
2. 初始化ros节点
    rospy.init
3. 创建发布者对象
    rospy.Publisher()
4. 编写发送逻辑并发送数据
"""


if __name__ == "__main__":
    
    rospy.init_node("publish_py")
    
    publisher = rospy.Publisher("py", Person, queue_size=10)
    
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
    
    
```

### 订阅方实现

```python
#! /usr/bin/env python

import rospy
from pubsub.msg import Person

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
    rospy.loginfo("name = %s, age = %u, height = %.2f\n", msg.name, msg.age, msg.height)

if __name__ == "__main__":
    
    rospy.init_node("subscribe_py")
    
    subscribe = rospy.Subscriber("py", Person, callback=callback)
    
    rospy.spin()
    
    
```

### 修改cmakelists文件

catkin_install_python处添加两个脚本文件即可


