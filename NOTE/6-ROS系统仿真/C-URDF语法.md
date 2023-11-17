在ROS中,URDF的组件包含两个部分
1. 连杆(link):可见刚体
2. 关节(joint):不可见

# 基础语法

1. robot标签

是urdf文件的根标签,所有的link和joint以及其他标签都必须高喊在robot标签内

* 属性:
	* name:指定机器人的名字

* 子标签:
	* 所有其他标签都是其子标签

2. link标签

link标签用于描述机器人的某个部件的外观和物理属性,每一个部件对应一个link,在link标签中可以设置该部件的形状,尺寸,颜色.惯性扭阵,碰撞参数等一系列属性

![](C-URDF语法-link.png)

* 属性:
	* name:为link命名

* 子标签:
	* visual:描述外观
		* geometry:设置形状
			* box:盒装
				* size="长(x) 宽(y) 高(z)"
			* cylinder:圆柱
				* radius="半径"
				* length="高度"
			* sphere:球体
				* radius="半径"
			* mesh:贴图
				* filename="资源路径"(`格式package://<packagename>/<path>/文件`)
		* origin:偏移量和倾斜弧度
			* xyz="x偏移 y偏移 z偏移"
			* rpy="roll(x) pitch(y) yaw(z)"
		* material:设置材料
			* name="名字"
			* color
				* rgba="r g b a"(0-1的权重值)
	* collision:碰撞属性
	* inertial:惯性矩阵


3. joint标签

urdf 中的 joint 标签用于描述机器人关节的运动学和动力学属性，还可以指定关节运动的安全极限，机器人的两个部件(分别称之为 parent link 与 child link)以"关节"的形式相连接，不同的关节有不同的运动形式: 旋转、滑动、固定、旋转速度、旋转角度限制

![](C-URDF语法-joint.png)

* 属性:
	* name:关节名
	* type:关节运动形式
		* continuous:旋转关节,可以绕单轴无限旋转
		* revolute:旋转关节,有旋转角度限制
		* prismatic:滑动关节,沿某一轴线移动的关节,有位置极限
		* planer:平面关节,允许平面正交方向平移或者旋转
		* floating:浮动关节,不允许平移和旋转运动
		* fixed:固定关节,不允许运动的特殊关节

* 子标签:
	* parent:必需项
		* link:父级连杆的名称
	* child:必需项
		* link:子级连杆的名称
	* origin
		* xyz
		* rpy
	* axis:
		* xyz="0 0 1"(可绕z轴转动)用于设置围绕哪个关节轴运动


具有joint的urdf文件需要在launch文件中添加新的节点
* `joint_state_publisher`: (必需)关节发布节点
* `robot_state_publisher`: (必需)机器人状态发布节点
* `joint_state_publisher_gui`: (可选)用于控制关节运动的节点


**使用base_printlink进行优化**

前面实现的机器人模型是半沉到地下的，因为默认情况下: 底盘的中心点位于地图原点上，所以会导致这种情况产生，可以使用的优化策略，将初始 link 设置为一个尺寸极小的 link(比如半径为 0.001m 的球体，或边长为 0.001m 的立方体)，然后再在初始 link 上添加底盘等刚体，这样实现，虽然仍然存在初始link半沉的现象，但是基本可以忽略了。这个初始 link 一般称之为 base_footprint

```xml
<!--

    使用 base_footprint 优化

-->
<robot name="mycar">
    <!-- 设置一个原点(机器人中心点的投影) -->
    <link name="base_footprint">
        <visual>
            <geometry>
                <sphere radius="0.001" />
            </geometry>
        </visual>
    </link>

    <!-- 添加底盘 -->
    <link name="base_link">
        <visual>
            <geometry>
                <box size="0.5 0.2 0.1" />
            </geometry>
            <origin xyz="0 0 0" rpy="0 0 0" />
            <material name="blue">
                <color rgba="0 0 1.0 0.5" />
            </material>
        </visual>
    </link>

    <!-- 底盘与原点连接的关节 -->
    <joint name="base_link2base_footprint" type="fixed">
        <parent link="base_footprint" />
        <child link="base_link" />
        <origin xyz="0 0 0.05" />
    </joint>

    <!-- 添加摄像头 -->
    <link name="camera">
        <visual>
            <geometry>
                <box size="0.02 0.05 0.05" />
            </geometry>
            <origin xyz="0 0 0" rpy="0 0 0" />
            <material name="red">
                <color rgba="1 0 0 0.5" />
            </material>
        </visual>
    </link>
    <!-- 关节 -->
    <joint name="camera2baselink" type="continuous">
        <parent link="base_link"/>
        <child link="camera" />
        <origin xyz="0.2 0 0.075" rpy="0 0 0" />
        <axis xyz="0 0 1" />
    </joint>

</robot>

```


# URDF工具

```bash
$ sudo apt install liburdfdom-tools
```

1. check_urdf

检查urdf是否合法

2. urdf_to_graphiz

生成urdf结构图,pdf形式


