nodelet, 可以将多个节点集合成一个进程, 应用于大容量数据传输的场景,提高节点间的数据交互效率,避免延时与阻塞


nodelet语法

```bash
$ rosrun nodelet nodelet 

# nodelet load pkg/Type manager [--no-bond] 加载一个进程进入manager
# nodelet standalone pkg/Type   该节点不进入manager
# nodelet unload name manager   使一个进程离开manager
# nodelet manager               启动manager
```

整理为launch文件如下

```xml(launch)
<launch>
    <!-- 设置nodelet管理器 -->
    <node pkg="nodelet" type="nodelet" name="mymanager" args="manager" output="screen" />
    <!-- 启动节点1，名称为 n1, 参数 /n1/value 为100 -->
    <node pkg="nodelet" type="nodelet" name="n1" args="load nodelet_tutorial_math/Plus mymanager" output="screen" >
        <param name="value" value="100" />
    </node>
    <!-- 启动节点2，名称为 n2, 参数 /n2/value 为-50 -->
    <node pkg="nodelet" type="nodelet" name="n2" args="load nodelet_tutorial_math/Plus mymanager" output="screen" >
        <param name="value" value="-50" />
        <remap from="/n2/in" to="/n1/out" />
    </node>
</launch>
```

# 编程实现

nodelet本身就是一个插件,因此其使用与插件的使用类似

1. 新建功能包,需要包含依赖 `roscpp nodelet pluginlib`

2. 实现插件类,插件需要继承自nodelet::Nodelet,并重写onInit()函数

3. 导出插件类

```cpp
#include "ros/ros.h"
#include "nodelet/nodelet.h"
#include "pluginlib/class_list_macros.h"

namespace my_nodelet
{
class MyPlus: public nodelet::Nodelet
{
public:
    MyPlus(){}

    void onInit()
    {
        ROS_INFO("hello nodelet");
    }
};
};

PLUGINLIB_EXPORT_CLASS(my_nodelet::MyPlus, nodelet::Nodelet)
```

4. 创建xml文件

```xml
<library path="lib/libmyplus">
    <!-- name一般设为包名 -->
    <class type="my_nodelet::MyPlus" base_class_type="nodelet::Nodelet" name="ndlet/MyPlus">
        <description>myplus</description>
    </class>
</library>
```

5. 修改package.xml文件

```xml
  <!-- The export tag contains other, unspecified, tags -->
  <export>
    <!-- Other tools can request additional information be placed here -->
    <nodelet plugin="${prefix}/myplus.xml" />
  </export>
```

6. 进行测试

```bash
# 第一个终端
$ rosrun nodelet nodelet manager __name:=xiaosun

# 第二个终端
$ rosrun nodelet nodelet load ndlet/MyPlus xiaosun __name:=haha
```

如果第一个终端中输出了`hello nodelet`那么就说明配置成功

# 完善上述nodelet程序

主要是修改onInit函数的实现

```cpp
#include "ros/ros.h"
#include "nodelet/nodelet.h"
#include "pluginlib/class_list_macros.h"
#include "std_msgs/Float64.h"

/**
 * 1. 确定需要的变量,订阅对象,发布对象,存储参数的变量
 * 2. 获取NodeHandle
 * 3. 通过NodeHandle创建订阅对象和发布对象
 * 4. 回调函数需要处理数据,并通过发布对象发布数据
*/

namespace my_nodelet
{
class MyPlus: public nodelet::Nodelet
{
public:
    MyPlus():value(0){}

    void onInit()
    {
        // 一般获取私有的NodeHandle,因为在nodelet节点下的数据
        ros::NodeHandle nh = getPrivateNodeHandle();

        nh.getParam("value", value);

        pub = nh.advertise<std_msgs::Float64>("out", 10);
        // 方法1:传入this指针
        sub = nh.subscribe("in", 100, &MyPlus::cb, this);
        // 方法2:使用boost::bind传参
        // nh.subscribe<std_msgs::Float64>("in", 100, boost::bind(&MyPlus::cb, this, _1));
    }

    void cb(const std_msgs::Float64::ConstPtr &p)
    {
        double a = p->data + value;
        std_msgs::Float64 out;
        out.data = a;
        pub.publish(out);
    }

private:
    ros::Publisher pub;
    ros::Subscriber sub;
    double value;
};
};

PLUGINLIB_EXPORT_CLASS(my_nodelet::MyPlus, nodelet::Nodelet)
```

使用的launch文件如下:

```xml
<launch>
    <node name="mymanager" pkg="nodelet" type="nodelet" args="manager" output="screen" />
    <node name="p1" pkg="nodelet" type="nodelet" args="load ndlet/MyPlus mymanager" output="screen" >
        <param name="value" value="100" />
    </node>
    <node name="p2" pkg="nodelet" type="nodelet" args="load ndlet/MyPlus mymanager" output="screen" >
        <param name="value" value="-10" />
        <remap from="/p2/in" to="/p1/out" />
    </node>
</launch>
```

回调函数不能使用非静态函数!