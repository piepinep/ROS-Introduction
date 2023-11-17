机器人传感器获取到的信息，可能需要我们实时处理，又是可能只需要 采集数据，事后分析。在ROS中关于数据的留存以及读取实现，提供了专门的工具`rosbag`

rosbag本质也是ros节点,在录制时,rosbag是一个订阅节点,可以订阅话题消息并将订阅到的数据写入磁盘文件,当重放时,rosbag是一个发布节点,可以读取磁盘文件,发布文件中的话题信息

# 命令行使用

1. 新建目录,保存录制的文件
2. 开始录制

```bash
$ rosbag record -a -o path/to/bag/folder/xxx.bag
```

3. 查看文件

```bash
$ rosbag info yourbag.bag
```

4. 回放文件

```bash
$ rosbag play yourbag.bag
```

# 编码实现

## CPP 

* 写文件

```cpp
#include "ros/ros.h"
#include "rosbag/bag.h"
#include "std_msgs/String.h"

/**
 * 使用rosbag向磁盘写出数据
 * 
*/

int main(int argc, char *argv[])
{
    setlocale(LC_ALL, "");

    ros::init(argc, argv, "write_bagc");
    ros::NodeHandle nh;

    // 新建bag对象
    rosbag::Bag bag;

    // 打开bag文件
    bag.open("hello.bag", rosbag::BagMode::Write);

    std_msgs::String msg;
    msg.data = "hello world";
    
    // 写入消息
    // "topic", timestamp, msg
    bag.write("/chatter", ros::Time::now(), msg);
    bag.write("/chatter", ros::Time::now(), msg);
    bag.write("/chatter", ros::Time::now(), msg);
    bag.write("/chatter", ros::Time::now(), msg);

    // 关闭 bag 文件
    bag.close();
    return 0;
}
```

* 读文件

```cpp
#include "ros/ros.h"
#include "rosbag/bag.h"
#include "std_msgs/String.h"
#include "rosbag/view.h"

/**
 * 使用rosbag从磁盘读入数据
 */

int main(int argc, char *argv[])
{
    setlocale(LC_ALL, "");

    ros::init(argc, argv, "write_bagc");
    ros::NodeHandle nh;

    // 新建bag对象
    rosbag::Bag bag;

    // 打开bag文件
    bag.open("hello.bag", rosbag::BagMode::Read);

    std_msgs::String msg;
    msg.data = "hello world";

    // 取出话题，时间戳和消息
    rosbag::View v(bag);

    for(auto &m: v)
    {
        std::string topic = m.getTopic();
        ros::Time timestamp = m.getTime();
        std_msgs::StringPtr p = m.instantiate<std_msgs::String>();
        ROS_INFO("\n解析的内容:\n"
                 "topic:%s\n"
                 "timestamp:%.2f\n"
                 "msg:%s",
                 topic.c_str(), timestamp.toSec(), p->data.c_str()
                );
    }

    // 关闭 bag 文件
    bag.close();
    return 0;
}

```

## Python实现

```python
#! /usr/bin/env python
import rospy
import rosbag
from std_msgs.msg import String

if __name__ == "__main__":
    #初始化节点 
    rospy.init_node("w_bag_p")

    # 创建 rosbag 对象
    bag = rosbag.Bag("/home/rosdemo/demo/test.bag",'w')

    # 写数据
    s = String()
    s.data= "hahahaha"

    bag.write("chatter",s)
    bag.write("chatter",s)
    bag.write("chatter",s)
    # 关闭流
    bag.close()
```

```python
#! /usr/bin/env python
import rospy
import rosbag
from std_msgs.msg import String

if __name__ == "__main__":
    #初始化节点 
    rospy.init_node("w_bag_p")

    # 创建 rosbag 对象
    bag = rosbag.Bag("/home/rosdemo/demo/test.bag",'r')
    # 读数据
    bagMessage = bag.read_messages("chatter")
    for topic,msg,t in bagMessage:
        rospy.loginfo("%s,%s,%s",topic,msg,t)
    # 关闭流
    bag.close()
```

