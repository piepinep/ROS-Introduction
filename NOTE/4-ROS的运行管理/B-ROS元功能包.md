ROS提供了一种方式，可以将不同的功能包打包成一个功能包，当安装某个功能模块时，直接调用打包后的功能包即可，该功能包称为元功能包（meta package）

元功能包是linux文件管理系统的一个概念，是ros中的一个虚包，但是其依赖于其他的软件包，可以通过这种方式将其他包组合起来。可以认为这是一本书的目录索引，高所包集中有哪些子包，并且去哪里下载。

# 作用

方便用户的安装，根据元功能包就可以知道当前功能包的依赖关系，并且将依赖进行安装

# 实现

1. 新建一个功能包, 该功能包的依赖为`空`

![](B-ROS元功能包.png)


2. 修改 `package.xml`

a. 添加一个可执行依赖 `<exec_depend>your_package</exec_depend>`
```xml
<exec_depend>被集成的功能包</exec_depend>
```

b. 在export标签中添加 `<metapackage />` 标签内容
```xml
<export>
	<metapackage />
</export>
```

修改结果如下:

```xml
<?xml version="1.0"?>
<package format="2">
  <name>mypack</name>
  <version>0.0.0</version>
  <description>The mypack package</description>
  <maintainer email="xxx@todo.todo">xxx</maintainer>
  <license>TODO</license>
  
  <buildtool_depend>catkin</buildtool_depend>
  <exec_depend>pubsub</exec_depend>

  <export>
    <metapackage />
  </export>
</package>
```

3. 修改cmake文件

a. 删除多余内容,只保留三行
```cmake
cmake_minimum_required(VERSION 3.0.2)
project(mypack)
find_package(catkin REQUIRED)
```

b.第四行添加函数
`catkin_metapackage()`

修改结果如下:
```cmake
cmake_minimum_required(VERSION 3.0.2)
project(mypack)
find_package(catkin REQUIRED)
catkin_metapackage()
```

>注意:
>cmake文件中不能存在空行,否则编译会失败

