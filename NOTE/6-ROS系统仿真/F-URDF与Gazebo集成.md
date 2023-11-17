gazebo需要的urdf文件中再link标签下,还需要添加collision和inertial标签

# collision标签

1. 如果是标准几何体,那么collision标签中直接复制link标签visual中的参数geometry和origin即可


# inertial标签

子级标签:
* origin:重心位置
	* xyz="x y z"
* mass:质量
	* value="kg"
* inertia:不同维度的惯性参数
	* ixx
	* ixy
	* ixz
	* iyy
	* iyz
	* izz

# gazebo标签

gazebo标签与link平级

参数:
* reference="参考"

子级标签:
* material:颜色 `<material>Gazebo/Red</material>`


# launch文件配置

1. 载入urdf,与rviz使用时一致

```xml
<param name="robot_description" textfile="$(find pkg)/urdf/urdf/yours.urdf" />
```

2. 启动gazebo仿真环境

```xml
<include file="$(find gazebo_ros)/launch/empty_world.launch" />
```

3. 在gazebo中加载机器人模型

```xml
<node pkg="gazebo_ros" type="spawn_model" name="spawn_model" args="-urdf -model mycar -param robot_description" />
```

# 惯性矩阵的xacro宏

```xml
<xacro:macro name="sphere_inertial_matrix" params="m r">
	<inertial>
		<mass value="${m}" />
		<inertia ixx="${2*m*r*r/5}" ixy="0" ixz="0"
			iyy="${2*m*r*r/5}" iyz="0" 
			izz="${2*m*r*r/5}" />
	</inertial>
</xacro:macro>

<xacro:macro name="cylinder_inertial_matrix" params="m r h">
	<inertial>
		<mass value="${m}" />
		<inertia ixx="${m*(3*r*r+h*h)/12}" ixy = "0" ixz = "0"
			iyy="${m*(3*r*r+h*h)/12}" iyz = "0"
			izz="${m*r*r/2}" /> 
	</inertial>
</xacro:macro>

<xacro:macro name="Box_inertial_matrix" params="m l w h">
   <inertial>
		   <mass value="${m}" />
		   <inertia ixx="${m*(h*h + l*l)/12}" ixy = "0" ixz = "0"
			   iyy="${m*(w*w + l*l)/12}" iyz= "0"
			   izz="${m*(w*w + h*h)/12}" />
   </inertial>
</xacro:macro>
```

# 将world文件导入到gazebo

1. 必须新建一个worlds目录,将worlds文件放入目录中

2. 修改launch中的include标签

```xml
<include file="$(find gazebo_ros)/launch/empty_world.launch" >
    <arg name="world_name" value="$(find urdf02)/worlds/box_house.world" />
</include>
```

# 如何搭建worlds环境

方法一:手动拖拽添加

![](F-URDF与Gazebo集成.png)

添加完成后保存即可

方式二:绘制仿真环境

"edit->building editor"

