参数服务器主要用于实现不同节点之间的数据共享,参数服务器是独立于所有节点的一个公共容器,可以将数据存储在这个容器里,被不同节点调用

参数服务器一般用于存在数据共享的应用场景

# 参数服务器的理论模型

![](D-参数服务器-理论模型.png)

角色:
* master
* talker
* listener

步骤
1. talker 通过RPC向参数服务器发送参数,master将参数保存在参数列表中
2. listene通过RPC向参数服务器发送查找请求
3. master查找参数列表,并通过RPC向listener发送结果

参数可以使用的类型
* 32-bit integers
* booleans
* strings
* doubles
* iso8601 dates
* lists
* base64-encoded binary data
* 字典

>注意:参数服务器不是为高性能而设计的，因此最好用于存储静态的非二进制的简单数据

# 参数操作

## CPP实现

参数服务器数据的增删改查可以通过两套 `API` 实现,
* ros::NodeHandle
* ros::param

### 增加和修改

```cpp
#include "ros/ros.h"

/**
 * 设置机器人的共享参数并修改参数
 * ros::NodeHandle
 *     setParam
 * ros::param
 *     set
*/


int main(int argc, char *argv[])
{
    ros::init(argc, argv, "paramc_set");

    ros::NodeHandle nh;

    // 设置
    // 使用 ros::NodeHandle
    nh.setParam("type", "car");
    nh.setParam("radius", 0.15);

    // 使用 ros::param
    ros::param::set("color", "yellow");
    ros::param::set("price", 5000);

    // 修改,直接覆盖即可
    nh.setParam("type", "plane");
    ros::param::set("price", 500000);
    return 0;
}

```

### 读取

```cpp
/**
 * 设置机器人的共享参数并修改参数
 * ros::NodeHandle
 *  param(key, default): 存在返回值,否则返回default
 *  getParam(key, var): 存在返回true, 值赋值给var,否则返回false, 不赋值
 *  gerParamCached(key, bar): 同上,但是使用cache
 *  getParamNames(std::vector<std::string>): 获取所有的键.并存储在vector中
 *  hasParam(key): 查看是否存在key
 *  searchParam(key, str): 判断是否存在key,如果存在,将key放在str变量中(有前缀/)
 * ros::param
 *     类似NodeHandle
 */
```

### 删除

```cpp
/**
 * 设置机器人的共享参数并修改参数
 * ros::NodeHandle
 *  deleteParam(key) -> bool
 * ros::param
 *  delete(key) -> bool
 */
```

## PYTHON实现

```python
# 增加/修改
rospy.set_param("p_int",10)

# 获取
data = rospy.get_param("p_int",10000)
data = rospy.get_param_cached("p_int")
names = rospy.get_param_names()
flag = rospy.has_param("p_int")
key = rospy.search_param("p_int")

# 删除
rospy.delete_param("key") # -> raise Exception() if can't find key
```

