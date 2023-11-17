# 初始化

`ros::init()`
```txt
作用:初始化ros节点
参数:
1. argc: (int)
2. argv: (char **)
3. name: (const std::string &) ,节点名称, 不可重复
4. options: (uint32_t)
返回值: 
void
使用:
1. argc, argv
    如果按照ros中特定格式传入实参,那么ros可以加以使用,比如用来设置全局参数,给节点重命名等
    比如
        rosrun packname nodename _param:=val
2. options
	节点名称具有唯一性,因此同一节点不能重复启动
	ros::init_options::xxx
	* xxx := AnonymousName: 节点可以重复启动 
```

# 发布者

`nodeHandler.advertise<T>()`
```txt
作用:创建发布者对象
参数:
1. topic: (const std::string &) 话题名称
2. queue_size: (uint32_t) 最大发送队列
3. latch (optional): (bool) If true, the last message published on this topic will be saved and sent to new subscribers when they connect
返回值:
发布者对象
使用:
latch:
	以静态地图发布为例:
	1. 可以使用固定地图发布数据
	2. 可以将地图发布对象的latch设置为true,并且发送方只发送一次数据,每当订阅者连接时,将地图数据发送一次
```

# 自旋

`spin()/spinOnce()`
```txt
spin(): 多次回调
spinOnce(): 只进行一次回调
```

# 时间相关

使用时间API必须要建立节点句柄,因为节点句柄中有初始化时间的操作.

```cpp
// 获取当前时刻
ros::Time n = ros::Time::now(); // 获取当前时刻封装 
n.toSec(); // 转换为秒(浮点型)
n.sec(); // 返回秒(整型)

// 设置时刻
ros::Time t(int sec, int nsec); // 构造函数, 秒和纳秒
ros::Time t(double sec); // 构造函数, 秒

// 持续时间
ros::Duration dur(double sec);
dur.sleep();

// 时间运算
ros::Time = ros::Time +/- ros::Duration;
ros::Duration = ros::Time - ros::Time;
ros::Duration = ros::Duration +/- ros::Duration;

// 运行频率
ros::Rate r(double sec);
r.sleep();

// 定时器
ros::Timer timer = nh.createTimer(
					ros::Duration period, 
					const ros::TimerCallback &callback,
					bool oneshot = false, // 只执行一次
					bool autostart = true //
				   );

timer.start(); //手动启动
// 回调函数
void callback(const ros::TimerEvent& event){
	event.current_real: ros::Time;
}
```

# 其他函数

```cpp
ros::ok(); // 节点是否存活

ros::shutdown(); // 关闭节点

ROS_DEBUG(); // 调试使用,不会输出到控制台
ROS_INFO(); // 标准消息 
ROS_WARN(); // 警告, 不会影响运行
ROS_ERROR(); // 错误,可能会影响运行
ROS_FATAL(); // 致命错误,会影响节点运行
```

