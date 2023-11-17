Action通信是一种类似于服务通信的实现,其实现模型也包含请求和响应,但是不同的是,在请求和响应的过程中,服务端还可以连续返回当前任务进度,客户端可以接收连续反馈并且可以取消任务.

ROS中提供了actionlib功能包集实现action通信

![](A-Action通信.png)

![](A-Action通信-1.png)

- goal:目标任务;
- cacel:取消任务;
- status:服务端状态;
- result:最终执行结果(只会发布一次);
- feedback:连续反馈(可以发布多次)。

Action一般用于耗时的请求响应场景

# 自定义action文件

使用action功能需要添加依赖`actionlib和actionlib_msgs`

1. 在功能包下新建action目录
2. 在action目录中新建action文件(xxx.action)
3. 在action文件中按照如下格式组织

```action
# 目标变量
---
# 最终响应变量
---
# 连续反馈变量
```

4. 配置cmake

在cmake文件中

a. 找到`add_action_files`语句,并将自己的action文件路径填充其中,add_action_files会自动寻找action目录下的文件

```cmake
add_action_files(
	FILES
	Addints.action
)
```

b. 找到`generate_messages`语句,并添加actionlib_msgs和std_msgs依赖

```cmake
generate_messages(
  DEPENDENCIES
  actionlib_msgs
  std_msgs
)
```

c.找到`catkin_package`语句,并将`CATKIN_DEPENDS`依赖语句取消注释

```cmake
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES act
 CATKIN_DEPENDS actionlib actionlib_msgs roscpp rospy std_msgs
#  DEPENDS system_lib
)
```

完成上述三个操作后使用cmake进行编译,无报错即说明自定义action编写完成

# action通信实现

## CPP实现

* 服务端

```cpp
#include "ros/ros.h"
#include "actionlib/server/simple_action_server.h"
#include "act/AddintsAction.h"

/**
 * 1. 包含头文件
 * 2. 初始化ros节点
 * 3. 创建NodeHandle
 * 4. 创建action服务对象
 * 5. 请求处理
 *    5.1 解析提交的目标值
 *    5.2 产生反馈
 *    5.3 最终结果响应
 * 6.spin
 */

typedef actionlib::SimpleActionServer<act::AddintsAction> Server;

// 注意!这里是GoalConstPtr而不是ActionGoalConstPtr
void cb(const act::AddintsGoalConstPtr &cptr, Server *s)
{
    // 解析提交的目标值
    int goal_num = cptr->num;

    ros::Rate r(0.1);
    int result = 0;
    for (int i = 1; i <= goal_num; i++)
    {
        result += i;
        r.sleep();
        // 产生连续反馈
        act::AddintsFeedback fb;
        fb.percent = 1. * i / goal_num;
        s->publishFeedback(fb);
    }
    
    // 最终结果响应
    act::AddintsResult ret;
    ret.result = result;
    s->setSucceeded(ret);
    return;
}

int main(int argc, char *argv[])
{
    setlocale(LC_ALL, "");
    ros::init(argc, argv, "action_server_cpp");
    ros::NodeHandle nh;

    // 创建action对象
    // NodeHandle, topic, callback_function,auto_start
    // 注意!这里的绑定函数需要属于boost空间,使用std空间则会报错
    Server server(nh, "addints", boost::bind(cb, _1, &server), false);
	server.start();

    ros::spin();
    return 0;
}

```

* 客户端

```cpp
#include "ros/ros.h"
#include "actionlib/client/simple_action_client.h"
#include "act/AddintsAction.h"

/**
 * 1. 包含头文件
 * 2. 初始化ros节点
 * 3. 创建NodeHandle
 * 4. 创建action客户端对象
 * 5. 发送处理
 *    5.1 建立连接
 *    5.2 处理反馈
 *    5.3 处理最终结果响应
 * 6.spin
 */

typedef actionlib::SimpleActionClient<act::AddintsAction> Client;

/*
void Client::sendGoal(
    const act::AddintsGoal &goal,
    boost::function<void (
        const actionlib::SimpleClientGoalState &state,
        const act::AddintsResultConstPtr &result)>done_cb,
    boost::function<void ()> active_cb,
    boost::function<void (const act::AddintsFeedbackConstPtr &feedback)> feedback_cb
)
*/

void done_cb(const actionlib::SimpleClientGoalState &state,
             const act::AddintsResultConstPtr &result)
{
    if(state.state_ == state.SUCCEEDED)
    {
        ROS_INFO("process success! result = %d", result->result);
    }
    else
    {
        ROS_INFO("error in process");
    }
}

void active_cb()
{
    // 建立连接时调用此函数
    ROS_INFO("successfully connect to server");
}

void feedback_cb(const act::AddintsFeedbackConstPtr &feedback)
{
    ROS_INFO("percent = %.2f%%/100%%", feedback->percent * 100);
}

int main(int argc, char *argv[])
{
    setlocale(LC_ALL, "");
    ros::init(argc, argv, "action_client_cpp");
    ros::NodeHandle nh;

    Client client(nh, "addints");

    client.waitForServer();

    act::AddintsGoal goal;
    goal.num = 10;
    client.sendGoal(goal, done_cb, active_cb, feedback_cb);
    ros::spin();
    return 0;
}

```

## Python实现

* 服务器端

```python
#! /usr/bin/env python

import rospy
from actionlib import SimpleActionServer
from act.msg import *

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
        self.server = SimpleActionServer(topic, AddintsAction, self.cb, False)
        self.server.start()
        
    def cb(self, goal: AddintsGoal):
        num = goal.num
        
        result = 0
        r = rospy.Rate(1)
        for i in range(num):
            result += (i + 1)
            fb = AddintsFeedback()
            fb.percent = (i + 1) / num
            self.server.publish_feedback(fb)
            r.sleep()
        
        res = AddintsResult()
        res.result = result
        self.server.set_succeeded(res)
        
        
if __name__ == "__main__":
    rospy.init_node("action_server_py")
    
    ma = MyAction("addints")
    
    rospy.spin()

```

* 客户端

```python
#! /usr/bin/env python

import rospy
import actionlib
from actionlib import SimpleActionClient, SimpleGoalState
from act.msg import *

def activate_cb():
    rospy.loginfo("connect to server!")
    
def done_cb(state: SimpleGoalState, result: AddintsResult):
    if state == actionlib.GoalStatus.SUCCEEDED:
        rospy.loginfo("successful! result = %.2f", result.result)
    else:
        rospy.loginfo("error")
    
def feedback_cb(feedback: AddintsFeedback):
    rospy.loginfo("now is %.2f%%", feedback.percent * 100)

if __name__ == "__main__":
    rospy.init_node("action_client_py")
    
    sac = SimpleActionClient("addints", AddintsAction)
    goal = AddintsGoal()
    goal.num = 10
    
    sac.wait_for_server()
    sac.send_goal(goal, done_cb, activate_cb, feedback_cb)
    
    rospy.spin()
    
```