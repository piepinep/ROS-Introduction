# 概念
tf: Transform Frame. 变换框架
坐标系: ROS是通过右手坐标系进行标定的
rviz：图形化工具查看坐标变换
# 作用
在ROS中实现不同坐标系之间的点或者向量之间的转换

# 功能包
ROS中坐标变换封装在 `tf` 功能包中,不过后续退出了更加简洁高效的 `tf2` 工具包, tf2工具包常用的有
* tf2_geometry_msgs:可以将ROS消息转换为tf2消息
* tf2:封装了坐标变换的常用消息
* tf2_ros:为tf2提供了roscpp和rospy绑定,封装了常用的api

# 坐标msg消息

常用消息有两个
1. `geometry_msgs/TransformStamped` :传输做消息相关位置信息
2. `geometry_msgs/PointStamped` :传输坐标系坐标点的消息

```bash
$ rosmsg info geometry_msgs/TransformStamped

# std_msgs/Header header
#   uint32 seq
#   time stamp
#   string frame_id 被参考的坐标系
# string child_frame_id 子坐标系
# geometry_msgs/Transform transform
#   geometry_msgs/Vector3 translation 子坐标系在被参考坐标系下的偏移量信息
#     float64 x
#     float64 y
#     float64 z
#   geometry_msgs/Quaternion rotation 四元数
#     float64 x
#     float64 y
#     float64 z
#     float64 w

```

```bash
$ rosmsg info geometry_msgs/PointStamped

# std_msgs/Header header
#   uint32 seq 序列号
#   time stamp 时间戳
#   string frame_id 参考物
# geometry_msgs/Point point 点的x,y,z坐标
#   float64 x 
#   float64 y
#   float64 z

```
# 静态坐标变换

静态坐标变换是指两个坐标系之间的相对位置是固定的

## CPP实现

1. 创建功能包
该功能包依赖于 tf2, tf2_ros, tf2_geometry_msgs, roscpp, rospy. std_msgs, geometry_msgs

2. 编写代码

* 发布方

```cpp
#include "ros/ros.h"
#include "tf2_ros/static_transform_broadcaster.h"
#include "geometry_msgs/TransformStamped.h"
#include "tf2/LinearMath/Quaternion.h"

/**
 * 发布两个坐标系的相对关系
 * 1. 包含头文件
 * 2. 设置编码，节点初始化
 * 3. 创建发布者对象
 * 4. 组织被发布的消息
 * 5. 发布数据
 * 6. spin
*/

int main(int argc, char *argv[])
{
    setlocale(LC_ALL, "");
    ros::init(argc, argv, "static_pub");
    ros::NodeHandle nh;

	// 创建发布者对象
    tf2_ros::StaticTransformBroadcaster pub;

	// 设置消息
    geometry_msgs::TransformStamped tf;
    tf.header.stamp = ros::Time::now();
    tf.header.frame_id = "base_link";
    tf.child_frame_id = "laser";

    // 设置偏移量 x，y，z
    tf.transform.translation.x = 0.2;
    tf.transform.translation.y = 0.0;
    tf.transform.translation.z = 0.5;

    // 设置四元数 x，y，z，w
    // 四元数设置需要考虑RPY 
    // RPY是对世界坐标系变换, 欧拉角是对自身坐标系变换
    // RPY和欧拉角变换过程不同,结果相同
    tf2::Quaternion quat;
    quat.setRPY(0, 0, 0);

    tf.transform.rotation.x = quat.getX();
    tf.transform.rotation.y = quat.getY();
    tf.transform.rotation.z = quat.getZ();
    tf.transform.rotation.w = quat.getW();

    // 发布数据
    pub.sendTransform(tf);


    ros::spin();
    return 0;
}

```

* 订阅方实现

```cpp
#include "ros/ros.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "geometry_msgs/PointStamped.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

/**
 * 订阅发布方的坐标系相对关系，传入一个坐标系，使用tf变换
 * 1. 包含头文件
 * 2. 设置编码，节点初始化
 * 3. 创建订阅者对象
 * 4. 组织坐标系数据点
 * 5. 转换算法
 * 6. 输出
 */

int main(int argc, char *argv[])
{
    setlocale(LC_ALL, "");
    ros::init(argc, argv, "static_sub");
    ros::NodeHandle nh;

    // 创建 buffer 对象，数据会存储在缓存中
    tf2_ros::Buffer buff;
    // 自动订阅相关数据，不需要手动编写
    // 订阅到的数据是变换信息，订阅者是利用变换信息进行变换
    tf2_ros::TransformListener sub(buff);

    // 待变换的点
    geometry_msgs::PointStamped ps;
    // 该点的参考坐标系
    ps.header.frame_id = "laser";
    // 时间戳
    ps.header.stamp = ros::Time::now();

    // 该点坐标
    ps.point.x = 2.0;
    ps.point.x = 3.0;
    ps.point.x = 5.0;

    ros::Rate r(2);

    while (ros::ok())
    {
        try
        {
            geometry_msgs::PointStamped point_base;
            point_base = buff.transform(ps, "base_link"); // 调用时必须包含 "tf2_geometry_msgs/tf2_geometry_msgs.h"
            ROS_INFO("转换后的数据:(%.2f,%.2f,%.2f),参考的坐标系是:%s",
                     point_base.point.x,
                     point_base.point.y,
                     point_base.point.z,
                     point_base.header.frame_id.c_str());
        }
        catch (const std::exception &e)
        {
            // 首次调用buffer可能为空
            ROS_INFO("程序异常.....");
        }
        r.sleep();
        ros::spinOnce();
    }
    return 0;
}

```

## Python实现

* 发布方实现

```python
#! /usr/bin/env python

import rospy
from tf2_ros import StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped
import tf_conversions as tfc


if __name__ == "__main__":
    
    rospy.init_node("static_pubp")
    
    # 创建发布者对象,注意是静态的
    pub = StaticTransformBroadcaster()
    
    # 创建转换坐标系
    tfs = TransformStamped()
    
    # 设置基础坐标系信息
    tfs.header.frame_id = "base_link"
    tfs.header.stamp = rospy.Time.now()
    
    # 设置子坐标系信息
    tfs.child_frame_id = "laser"
    tfs.transform.translation.x = 0.2
    tfs.transform.translation.y = 0.0
    tfs.transform.translation.z = 0.5

    # 进行Euler和四元数转换
    quat = tfc.transformations.quaternion_from_euler(0, 0, 0)
    tfs.transform.rotation.x = quat[0]
    tfs.transform.rotation.y = quat[1]
    tfs.transform.rotation.z = quat[2]
    tfs.transform.rotation.w = quat[3]
    
    # 发布转换坐标系
    pub.sendTransform(tfs)
    
    rospy.spin()
    
```

* 订阅方实现

```python
#! /usr/bin/env python

import rospy
from tf2_ros import Buffer, TransformListener
from tf2_geometry_msgs import tf2_geometry_msgs

if __name__ == "__main__":
    
    rospy.init_node("static_subp")
    
    buff = Buffer()
    sub = TransformListener(buff)
    
    ps = tf2_geometry_msgs.PointStamped()
    
    ps.header.frame_id = "laser"
    ps.header.stamp = rospy.Time.now()
    
    ps.point.x = 2
    ps.point.y = 3
    ps.point.z = 5
    
    r = rospy.Rate(2)
    while not rospy.is_shutdown():
        
        try:
            nps: PointStamped = buff.transform(ps, "base_link")
            rospy.loginfo("变换后坐标: (%.2f, %.2f, %.2f),参考的坐标系是:%s",
                          nps.point.x,
                          nps.point.y,
                          nps.point.z,
                          nps.header.frame_id
                          )
        except Exception as e:
            rospy.loginfo("%s", str(e))
        
        r.sleep()
```

## 补充

当坐标系之间的相对位置固定时，那么所需参数也是固定的: 父系坐标名称、子级坐标系名称、x偏移量、y偏移量、z偏移量、x 翻滚角度、y俯仰角度、z偏航角度，实现逻辑相同，参数不同，那么 ROS 系统就已经封装好了专门的节点，使用方式如下:

`rosrun tf2_ros static_transform_publisher x偏移量 y偏移量 z偏移量 z偏航角度 y俯仰角度 x翻滚角度 父级坐标系 子级坐标系`

示例:

`rosrun tf2_ros static_transform_publisher 0.2 0 0.5 0 0 0 /baselink /laser`

也建议使用该种方式直接实现静态坐标系相对信息发布


# 动态坐标变换

## cpp实现

* 发布方实现

```cpp
#include "ros/ros.h"
#include "turtlesim/Pose.h"
#include "tf2/LinearMath/Quaternion.h"
#include "geometry_msgs/TransformStamped.h"
#include "tf2_ros/transform_broadcaster.h"

/**
 * 既需要订阅乌龟的位姿信息，也需要发布坐标系的变化信息
 * 订阅乌龟信息
 *     话题：/turtle1/pose
 *     消息：/turtlesim/Pose
 * 流程：
 * 1. 包含头文件
 * 2. 初始化
 * 3. 创建订阅对象
 * 4. 回调函数处理消息，并进行转换后发布
 * 5. spin
 * 
*/

void sub2pub(const turtlesim::PoseConstPtr &cptr)
{
    geometry_msgs::TransformStamped tfs;
    static tf2_ros::TransformBroadcaster tb;

    tfs.header.frame_id = "base_link";
    tfs.header.stamp = ros::Time::now();

    tfs.child_frame_id = "turtle1";
    tfs.transform.translation.x = cptr->x;
    tfs.transform.translation.y = cptr->y;
    tfs.transform.translation.z = 0;

    tf2::Quaternion quat;
    quat.setRPY(0, 0, cptr->theta);
    tfs.transform.rotation.x = quat.getX();
    tfs.transform.rotation.y = quat.getY();
    tfs.transform.rotation.z = quat.getZ();
    tfs.transform.rotation.w = quat.getW();

    tb.sendTransform(tfs);
}

int main(int argc, char *argv[])
{
    setlocale(LC_ALL, "");

    ros::init(argc, argv, "dynamic_pubc");
    ros::NodeHandle nh;

    ros::Subscriber subt = nh.subscribe("/turtle1/pose", 10, sub2pub);

    ros::spin();
    return 0;
}

```

* 订阅方实现

```cpp
#include "ros/ros.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "geometry_msgs/PointStamped.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

/**
 * 订阅发布方的坐标系相对关系，传入一个坐标系，使用tf变换
 * 1. 包含头文件
 * 2. 设置编码，节点初始化
 * 3. 创建订阅者对象
 * 4. 组织坐标系数据点
 * 5. 转换算法
 * 6. 输出
 */

int main(int argc, char *argv[])
{
    setlocale(LC_ALL, "");
    ros::init(argc, argv, "dynamic_sub");
    ros::NodeHandle nh;

    // 创建 buffer 对象，数据会存储在缓存中
    tf2_ros::Buffer buff;
    // 自动订阅相关数据，不需要手动编写
    // 订阅到的数据是变换信息，订阅者是利用变换信息进行变换
    tf2_ros::TransformListener sub(buff);

    // 待变换的点
    geometry_msgs::PointStamped ps;
    // 该点的参考坐标系
    ps.header.frame_id = "turtle1";
    // 时间戳,需要设置为0
    ps.header.stamp = ros::Time();

    // 该点坐标
    ps.point.x = .0;
    ps.point.x = .0;
    ps.point.x = .0;

    ros::Rate r(2);

    while (ros::ok())
    {
        try
        {
            geometry_msgs::PointStamped point_base;
            point_base = buff.transform(ps, "world"); // 调用时必须包含 "tf2_geometry_msgs/tf2_geometry_msgs.h"
            ROS_INFO("转换后的数据:(%.2f,%.2f,%.2f),参考的坐标系是:%s",
                     point_base.point.x,
                     point_base.point.y,
                     point_base.point.z,
                     point_base.header.frame_id.c_str());
        }
        catch (const std::exception &e)
        {
            // 首次调用buffer可能为空
            ROS_INFO("程序异常.....");
        }
        r.sleep();
        ros::spinOnce();
    }
    return 0;
}

```


## Python实现

* 发布方实现

```python
#! /usr/bin/env python

import rospy
from turtlesim.msg import Pose
import tf_conversions as tfc
import tf2_ros
from geometry_msgs.msg import TransformStamped


tb = tf2_ros.TransformBroadcaster()
def sub2pub(pose: Pose):
    
    tfs = TransformStamped()
    
    tfs.header.frame_id = "world"
    tfs.header.stamp = rospy.Time.now()
    
    tfs.child_frame_id = "turtle1"
    
    tfs.transform.translation.x = pose.x
    tfs.transform.translation.y = pose.y
    tfs.transform.translation.z = 0
    
    quat = tfc.transformations.quaternion_from_euler(0, 0, pose.theta)
    tfs.transform.rotation.x = quat[0]
    tfs.transform.rotation.y = quat[1]
    tfs.transform.rotation.z = quat[2]
    tfs.transform.rotation.w = quat[3]

    tb.sendTransform(tfs)


if __name__ == "__main__":
    rospy.init_node("dynamic_pubp")
    sub = rospy.Subscriber("/turtle1/pose", Pose, sub2pub, queue_size=10)
    rospy.spin()
    
```

* 订阅方实现

```python
#! /usr/bin/env python

import rospy
from tf2_ros import Buffer, TransformListener
from geometry_msgs.msg import PointStamped
from tf2_geometry_msgs import tf2_geometry_msgs

if __name__ == "__main__":
    
    rospy.init_node("dynamic_subp")
    
    buff = Buffer()
    sub = TransformListener(buff)
    
    ps = tf2_geometry_msgs.PointStamped()
    
    ps.header.frame_id = "turtle1"
    ps.header.stamp = rospy.Time()
    
    ps.point.x = 0
    ps.point.y = 0
    ps.point.z = 0
    
    r = rospy.Rate(2)
    while not rospy.is_shutdown():
        
        try:
            nps: tf2_geometry_msgs.PointStamped = buff.transform(ps, "world")
            rospy.loginfo("变换后坐标: (%.2f, %.2f, %.2f),参考的坐标系是:%s",
                          nps.point.x,
                          nps.point.y,
                          nps.point.z,
                          nps.header.frame_id
                          )
        except Exception as e:
            rospy.loginfo("%s", str(e))
        
        r.sleep()
    
```


# 多坐标系变换

启动launch

```xml
<launch>
    <node pkg="tf2_ros" type="static_transform_publisher" name="son1" args="5 0 0 0 0 0 /world /son1" output="screen" /> 
    <node pkg="tf2_ros" type="static_transform_publisher" name="son2" args="3 0 0 0 0 0 /world /son2" output="screen" /> 
</launch>
```

## CPP实现

```cpp
#include "ros/ros.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

/**
 * 订阅方实现
 * 1. 计算两个静态坐标系的相对关系
 * 2. 计算某个坐标系中的点在另一个坐标系中的坐标值
 *
 * 流程：
 * 1. 包含头文件
 * 2. 编码，初始化
 * 3. 创建订阅对象
 * 4. 编写解析逻辑
 * 5. spinOnce
 */

int main(int argc, char *argv[])
{
    setlocale(LC_ALL, "");
    ros::init(argc, argv, "mtfc");
    ros::NodeHandle nh;

    tf2_ros::Buffer buff;
    tf2_ros::TransformListener listener(buff);

    ros::Rate r(2);

    geometry_msgs::PointStamped ps1;

    ps1.header.frame_id = "son1";
    ps1.header.stamp = ros::Time::now();

    ps1.point.x = 1;
    ps1.point.y = 2;
    ps1.point.z = 3;

    while (ros::ok())
    {
        try
        {
            // 计算 son1 和 son2的相对关系
            // ros::Time(0):取间隔最短的两个关系帧计算相对关系
            geometry_msgs::TransformStamped tfs = buff.lookupTransform("son2", "son1", ros::Time(0));
            ROS_INFO("\n坐标系相对关系：\n"
                     "目标坐标系：%s, 基础坐标系: %s\n"
                     "偏移量:(%.2f, %.2f, %.2f)\n"
                     "四元数:(%.2f, %.2f, %.2f, %.2f)\n",
                     tfs.header.frame_id.c_str(), tfs.child_frame_id.c_str(),
                     tfs.transform.translation.x, tfs.transform.translation.y, tfs.transform.translation.z,
                     tfs.transform.rotation.x, tfs.transform.rotation.y, tfs.transform.rotation.z, tfs.transform.rotation.w);

            // son1 与 son2 点的转换，与两个坐标系转换相同
            geometry_msgs::PointStamped nps = buff.transform(ps1, "son2");
            ROS_INFO("转换后的坐标为(%.2f, %.2f, %.2f), 基准坐标系是:%s\n",
                     nps.point.x, nps.point.y, nps.point.z, nps.header.frame_id.c_str());
        }
        catch (const std::exception &e)
        {
            ROS_INFO("%s", e.what());
        }

        r.sleep();
        ros::spinOnce();
    }
    return 0;
}

```

## Python实现

```python
#! /usr/bin/env python

import rospy
from tf2_ros import Buffer, TransformListener
from geometry_msgs.msg import PointStamped, TransformStamped
from tf2_geometry_msgs import tf2_geometry_msgs

if __name__ == "__main__":
    
    rospy.init_node("dynamic_subp")
    
    buff = Buffer()
    sub = TransformListener(buff)
    
    ps = tf2_geometry_msgs.PointStamped()
    
    ps.header.frame_id = "son1"
    ps.header.stamp = rospy.Time.now()
    
    ps.point.x = 1
    ps.point.y = 2
    ps.point.z = 3
    
    r = rospy.Rate(2)
    while not rospy.is_shutdown():
        
        try:
            ntfs: TransformStamped = buff.lookup_transform("son2", "son1", rospy.Time(0))
            rospy.loginfo("\n1. %s\n2. %s\n(%.2f, %.2f, %.2f)",
                          ntfs.header.frame_id, ntfs.child_frame_id, 
                          ntfs.transform.translation.x,ntfs.transform.translation.y,ntfs.transform.translation.z
                          )
            
            nps: PointStamped = buff.transform(ps, "son2")
            rospy.loginfo("变换后坐标: (%.2f, %.2f, %.2f),参考的坐标系是:%s",
                          nps.point.x,
                          nps.point.y,
                          nps.point.z,
                          nps.header.frame_id
                          )
        except Exception as e:
            rospy.loginfo("%s", str(e))
        
        r.sleep()
           
```

# 坐标系关系查看

1. 命令行查看
2. rviz
3. tf2_tools

```bash
$ sudo apt install ros-noetic-tf2-tools
```

	1. 启动节点
	2. `$ rosrun tf2_tools view_frames.py`

![](A-TF坐标变换.png)


# 乌龟追踪

1. 创建新的乌龟

```cpp
// create.cpp
#include "ros/ros.h"
#include "turtlesim/Spawn.h"

int main(int argc, char *argv[])
{
    
    ros::init(argc, argv, "create", ros::InitOption::AnonymousName);
    ros::NodeHandle nh;

    ros::ServiceClient sc = nh.serviceClient<turtlesim::Spawn>("/spawn");
    turtlesim::Spawn sp;
    sp.request.name = "turtle2";
    sp.request.x = 3;
    sp.request.y = 3;
    sp.request.theta = 0;

    sc.waitForExistence();

    if(sc.call(sp))
    {
        ROS_INFO("create turtle success");
    }
    else
    {
        ROS_WARN("create turtle failed");
    }

    return 0;
}
```

2. 编写发布方,发布两只乌龟的坐标消息

```cpp
#include "ros/ros.h"
#include "turtlesim/Pose.h"
#include "tf2/LinearMath/Quaternion.h"
#include "geometry_msgs/TransformStamped.h"
#include "tf2_ros/transform_broadcaster.h"

std::string name;

void sub2pub(const turtlesim::PoseConstPtr &cptr)
{
    geometry_msgs::TransformStamped tfs;
    static tf2_ros::TransformBroadcaster tb;

    tfs.header.frame_id = "world";
    tfs.header.stamp = ros::Time::now();

    tfs.child_frame_id = name;
    tfs.transform.translation.x = cptr->x;
    tfs.transform.translation.y = cptr->y;
    tfs.transform.translation.z = 0;

    tf2::Quaternion quat;
    quat.setRPY(0, 0, cptr->theta);
    tfs.transform.rotation.x = quat.getX();
    tfs.transform.rotation.y = quat.getY();
    tfs.transform.rotation.z = quat.getZ();
    tfs.transform.rotation.w = quat.getW();

    tb.sendTransform(tfs);
}

int main(int argc, char *argv[])
{
    setlocale(LC_ALL, "");

    ros::init(argc, argv, "dynamic_pubc", ros::InitOption::AnonymousName);
    ros::NodeHandle nh;

    name = argv[argc - 1];

    ros::Subscriber subt = nh.subscribe(name +  "/pose", 10, sub2pub);

    ros::spin();
    return 0;
}


```

3. 订阅方解析坐标信息并产生速度信息

```cpp
#include "ros/ros.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "geometry_msgs/Twist.h"
#include "cmath"

/**
 * 1. 换算出 turtle1 相较于 turtle2 的消息
 * 使用多坐标变换
 * 2. 计算角速度和线速度, 并发布
 */

int main(int argc, char *argv[])
{
    setlocale(LC_ALL, "");
    ros::init(argc, argv, "trace");
    ros::NodeHandle nh;

    tf2_ros::Buffer buff;
    tf2_ros::TransformListener listener(buff);

    ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("/turtle2/cmd_vel", 100);

    ros::Rate r(10);

    while (ros::ok())
    {
        try
        {
            geometry_msgs::TransformStamped tfs = buff.lookupTransform("turtle2", "turtle1", ros::Time(0));
            geometry_msgs::Twist tw;

            tw.linear.x = 0.5 * sqrt(pow(tfs.transform.translation.x, 2) + pow(tfs.transform.translation.y, 2));
            tw.angular.z = 4 * atan2(tfs.transform.translation.y, tfs.transform.translation.x);

            pub.publish(tw);
        }
        catch (const std::exception &e)
        {
            ROS_INFO("%s", e.what());
        }

        r.sleep();
        ros::spinOnce();
    }
    return 0;
}

```

## 整理launch文件

```xml
<launch>
    <!-- 1. 启动节点gui及键盘控制 -->
    <node pkg="turtlesim" type="turtlesim_node" name="turtle1" output="screen" />
    <node pkg="turtlesim" type="turtle_teleop_key" name="key" output="screen" />

    <!-- 2. 创建新的乌龟 -->
    <node pkg="turtle_trace" type="create" name="turtle2" output="screen" />

    <!-- 3. 启动两只乌龟相对世界坐标系的发布者 -->
    <!-- 
        1. 使用同一个节点, 但是节点启动两次
        2. 节点启动动态传参
     -->
    <node pkg="turtle_trace" type="pub_pose" name="pub1" args="turtle1" output="screen" />
    <node pkg="turtle_trace" type="pub_pose" name="pub2" args="turtle2" output="screen" />

    <!-- 
        4. 订阅 turtle1, turtle2 相对于世界坐标系的发布消息
        并转换成turtle2 相对于turtle1的坐标关系
        再生成速度消息
    -->
    <node pkg="turtle_trace" type="trace" name="control" output="screen" />

</launch>
```

## Python实现不再举例

# TF和TF2的关系

## 1.TF2与TF比较_简介

- TF2已经替换了TF，TF2是TF的超集，建议学习 TF2 而非 TF
    
- TF2 功能包的增强了内聚性，TF 与 TF2 所依赖的功能包是不同的，TF 对应的是`tf`包，TF2 对应的是`tf2`和`tf2_ros`包，在 TF2 中不同类型的 API 实现做了分包处理。
    
- TF2 实现效率更高，比如在:TF2 的静态坐标实现、TF2 坐标变换监听器中的 Buffer 实现等
    

## 2.TF2与TF比较_静态坐标变换演示

接下来，我们通过静态坐标变换来演示TF2的实现效率。

### 2.1启动 TF2 与 TF 两个版本的静态坐标变换

TF2 版静态坐标变换:`rosrun tf2_ros static_transform_publisher 0 0 0 0 0 0 /base_link /laser`

TF 版静态坐标变换:`rosrun tf static_transform_publisher 0 0 0 0 0 0 /base_link /laser 100`

会发现，TF 版本的启动中最后多一个参数，该参数是指定发布频率

### 2.2运行结果比对

使用`rostopic`查看话题，包含`/tf`与`/tf_static`, 前者是 TF 发布的话题，后者是 TF2 发布的话题，分别调用命令打印二者的话题消息

`rostopic echo /tf`: 当前会循环输出坐标系信息

`rostopic echo /tf_static`: 坐标系信息只有一次

### 2.3结论

如果是静态坐标转换，那么不同坐标系之间的相对状态是固定的，既然是固定的，那么没有必要重复发布坐标系的转换消息，很显然的，tf2 实现较之于 tf 更为高效

# END