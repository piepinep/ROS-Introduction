# 工作空间覆盖

所谓工作空间覆盖，是指不同工作空间中，存在重名的功能包的情形

>ROS 开发中，会自定义工作空间且自定义工作空间可以同时存在多个，可能会出现一种情况: 虽然特定工作空间内的功能包不能重名，但是自定义工作空间的功能包与内置的功能包可以重名或者不同的自定义的工作空间中也可以出现重名的功能包.

**后刷新的优先级要高于先刷新的优先级!**

刷新操作是`source workshop/devel/setup.bash`

# ROS节点名称重名

ROS不允许节点重名,节点重构吗会导致调用时产生混淆,常见解决这种重名问题的方法有三种
* rosrun
* launch
* 编码实现

## rosrun

1. 设置命名空间(两个`'_'`)
```bash
$ rosrun yourpackage yournode __ns:=/namespace
```

2. 起别名(两个`'_'`)
```bash
$ rosrun yourpackage yournode __name:=newname
```

3. 命名空间+起别名

## launch

1. 设置命名空间
```xml
<launch>
	<node pkg="yourpackage" type="yournode" name="newname" />
</launch>
```

2. 起别名
```xml
<launch>
	<node pkg="yourpackage" type="yournode" name="name" ns="namespace" />
</launch>
```

3. 命名空间 + 起别名


## 编码方式

### CPP

1. 重映射:
在`ros::init`的option参数中选择ros::options::AnonymousName

2. 命名空间
```cpp
std::map<std::string, std::string> map;
map["__ns"] = "namespace"
ros::init(map, "xxxx");
```

### Python
重命名
```python
rospy.init_node("name", anonymous=True)
```

# 话题重名

## rosrun

```bash
$ rosrun yourpackage your_node /old_topic_name:=/new_topic_name
```

## launch

```xml
<launch>
	<node pkg="yourpackage" type="yournode/yourpy.py" name="yourname" >
		<remap from="xxx" to="yyy" />
	</node>
</launch>
```

## 编码设置

话题的名称与节点的命名空间,节点名称有一定关系,话题可以分为三类
* 全局:话题参考ROS系统,与节点命名空间评级
* 相对:话题参考命名空间,与节点名称平级
* 私有:话题参考节点名称,是节点名称的子集

![](D-重名问题.png)

### CPP

```cpp
// 初始化节点,这只节点名称
ros::init(argc, argv, "name");
ros::NodeHandle nh;

// 设置不同的话题
// 全局话题, 以 '/' 开头, 和节点的命名空间及节点名无关
// 话题可以设置自己的命名空间
ros::Publisher gtopic_pub = nh.advertise<std_msgs::String>("/global", 10);
ros::Publisher gtopic_pub_ns = nh.advertise<std_msgs::String>("/ns/global", 10);

// 相对话题, 非 '/' 开头
ros::Publisher rtopic_pub = nh.advertise<std_msgs::String>("relative", 10);
ros::Publisher rtopic_pub_ns = nh.advertise<std_msgs::String>("ns/relative", 10);

// 私有话题, 结合NodeHandle实现
// 私有的nodehandle以 '/' 开头,生成的话题也是全局的(全局话题优先级更高)
ros::NodeHandle nh("~");
ros::Publisher ptopic_pub = nh.advertise<std_msgs::String>("private", 10);
ros::Publisher ptopic_pub_ns = nh.advertise<std_msgs::String>("ns/private", 10);
```

### Python

```python
# 全局
rospy.Publisher("/global")
rospy.Publisher("/ns/global")

# 相对
rospy.Publisher("relative")
rospy.Publisher("ns/relative")

# 私有
rospy.Publisher("~private")
rospy.Publisher("~ns/global")
```

# 参数重名

参数也具有全局/相对/私有三种

## rosrun
```bash
# 设置参数, 一个 '_', 私有参数
$ rosrun yourpackage yournode _param:=value
```

## roslaunch
```xml
<launch>
	<param name="global" value="100" />
	<node pkg="yourpackage" type="yournode" name="n1">
		<param name="private" value="200" />
	</node>
</launch>
```

## 编码方式

### CPP
使用`ros::param`和`nh::setParam`

```cpp
// 全局
ros::param::set("/global", 100);
nh.setParam("/global", 100);

// 相对
ros::param::set("relative", 100);
nh.setParam("relative", 100);

// 私有
ros::NodeHandle nh("~");
ros::param::set("~private", 100);
nh.setParam("private", 100);
```

### Python

```python
# 全局
rospy.setParam("/global", 100)

# 相对
rospy.setParam("relative", 100)

# 私有
rospy.setParam("~private", 100)
```


