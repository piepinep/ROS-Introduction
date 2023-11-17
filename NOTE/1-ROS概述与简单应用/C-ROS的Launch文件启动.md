# launch文件

多ros节点的启动可以使用ros的launch文件

# 1. 选定功能包,创建launch目录

![](1-ROS程序简单实现-创建launch目录.png)

# 2. 在launch目录中创建launch文件(xml格式,后缀为launch)

![](1-ROS程序简单实现-创建launch文件.png)

# 3. 编辑launch文件内容

```xml
<launch>
    <node pkg="" type="" name="" />
</launch>
```

其中node是包名, 节点(cmake中的可执行对象/python文件)对应type,name可以对节点自己命名

```xml
<launch>
    <node pkg="turtlesim" type="turtlesim_node" name="gui" />
    <node pkg="turtlesim" type="turtle_teleop_key" name="key" />
</launch>
```


```xml
<launch>
    <node pkg="helloworld" type="helloworld_node" name="hi" output="screen" />
</launch>
```

# 4. 运行launch文件

```bash
$ source ./devel/setup.bash
$ roslaunch helloworld hello.launch
```

**launch会自动启动roscore**