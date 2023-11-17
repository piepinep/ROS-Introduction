xarco是xml macro的简写是一种可编程的xml文件

# 基础

xarco在根标签robot中,必须包含命名空间`xmlns:xacro="http://www.ros.org/wiki/xacro"`

# 属性

语法:
* 定义:`<xacro:property name="" value="" />`
* 调用:`${属性名称}`
* 算术运算:`${数学表达式}`

# 宏

语法:
* 宏定义

```xml
<xacro:macro name="name" params="param1 param2 ...">
	...
</xacro:macro>
```

* 宏调用

```xml
<xacro:宏名称 param1="" param2=""  />?
```

# 导包

```xml
<robot name="xxx" xmlns:xacro="http://wiki.ros.org/xacro">
      <xacro:include filename="my_base.xacro" />
      <xacro:include filename="my_camera.xacro" />
      <xacro:include filename="my_laser.xacro" />
      ....
</robot>
```

# xacro的使用

1. 使用指令将xacro转换为urdf文件后再进行使用

```bash
$ rosrun xxx.xacro > yyy.urdf
$ roslaunch pkgname launchname.launch
```

2. 再launch文件中直接加载xacro

```xml
<launch>
	<param name="robot_description" command="$(find xacro)/xcaro $(find pkgname)/urdf/xcaro/yoururdf.xcaro" />
	...
</launch>
```


