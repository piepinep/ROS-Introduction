# 准备工作

1. 安装相关ros包
* `$ sudo apt install ros-noetic-gmapping` 用于构建地图
* `$ sudo apt install ros-noetic-map-server` 用于保存和读取地图
* `$ sudo apt install ros-noetic-navigation` 用于定位和路径规划

2. 新建功能包,导入相关依赖

`gmapping map_server amcl move_base`

# SLAM建图

## 1. gmapping

gmapping是简单且成熟的SLAM算法之一,可以根据里程计数据和激光雷达绘制二维栅格地图,gmapping对机器人有硬件要求
* 可以发布里程计消息
* 可以发布雷达消息

## 2. gmapping的节点

gmapping功能包的核心节点是`slam_gmapping`

1. 订阅的话题:
* tf(tf/tfMessage):用于雷达,底盘与里程计之间的坐标变换消息
* scan(sensor_message/LaserScan):SLAM所需要的雷达消息

2. 发布的话题
* map_metadata(nav_msgs/MapMetaData):地图元数据,包括地图的宽度,高度,分辨率等
* map(nav_msgs/OccupancyGrid):地图栅格数据,一般在rviz中以图形化方式显示
* entropy(std_msgs/Float64):机器人姿态分布熵估计,值越大不稳定性越大

3. 服务
* dynamic_map(nav_msgs/GetMap):用来获取地图数据

4. 参数
* base_frame(string, default:"base_link") 机器人基坐标系
* map_frame(string, default:"map") 地图坐标系
* odom_frame(string, default:"odom") 里程计坐标系
* map_update_interval(float, default:5.0) 更新间隔(s)
* maxUrange(float, default:80.0) 激光探测的最大可用范围(m)
* maxRange(float) 激光探测的最大范围(m) 

5. 坐标变换
* 雷达坐标系->基坐标系: 一般由robot_state_publisher或者static_transform_publisher发布
* 基坐标系->里程计坐标系:由里程计节点发布

6. 发布的坐标系
* 地图坐标系->里程计坐标系:地图到里程计坐标系的变换

## 3. gmapping的使用

重点关注几个参数即可

```xml
<launch>
<!-- 仿真情况下，将use_sim_time设置为true -->
<param name="use_sim_time" value="true"/>
    <node pkg="gmapping" type="slam_gmapping" name="slam_gmapping" output="screen">
        <!-- 如果不需要改变订阅话题,可以不进行remap -->
        <remap from="scan" to="scan"/>
        <!-- 
            可选设置三个坐标系
            base_frame:默认为base_link
            odom_frame:默认为odom
            map_frame :默认为map
        -->
        <param name="base_frame" value="base_footprint"/> <!--底盘坐标系-->
        <param name="odom_frame" value="odom"/> <!--里程计坐标系-->
        <param name="map_frame" value="map"/> <!--里程计坐标系-->
        
        <!-- 其他参数 -->
        <param name="map_update_interval" value="5.0"/>
        <param name="maxUrange" value="16.0"/>
        <param name="sigma" value="0.05"/>
        <param name="kernelSize" value="1"/>
        <param name="lstep" value="0.05"/>
        <param name="astep" value="0.05"/>
        <param name="iterations" value="5"/>
        <param name="lsigma" value="0.075"/>
        <param name="ogain" value="3.0"/>
        <param name="lskip" value="0"/>
        <param name="srr" value="0.1"/>
        <param name="srt" value="0.2"/>
        <param name="str" value="0.1"/>
        <param name="stt" value="0.2"/>
        <param name="linearUpdate" value="1.0"/>
        <param name="angularUpdate" value="0.5"/>
        <param name="temporalUpdate" value="3.0"/>
        <param name="resampleThreshold" value="0.5"/>
        <param name="particles" value="30"/>
        <param name="xmin" value="-50.0"/>
        <param name="ymin" value="-50.0"/>
        <param name="xmax" value="50.0"/>
        <param name="ymax" value="50.0"/>
        <param name="delta" value="0.05"/>
        <param name="llsamplerange" value="0.01"/>
        <param name="llsamplestep" value="0.01"/>
        <param name="lasamplerange" value="0.005"/>
        <param name="lasamplestep" value="0.005"/>
    </node>

    <node pkg="joint_state_publisher" name="joint_state_publisher" type="joint_state_publisher" />
    <node pkg="robot_state_publisher" name="robot_state_publisher" type="robot_state_publisher" />

    <node pkg="rviz" type="rviz" name="rviz" />
    <!-- 可以保存 rviz 配置并后期直接使用-->
    <!--
        <node pkg="rviz" type="rviz" name="rviz" args="-d $(find my_nav_sum)/rviz/gmapping.rviz"/>
    -->
</launch>
```

1. 启动gazebo仿真launch文件
2. 启动上述的slam仿真文件,并在rviz中添加相关组件
3. 启动键盘控制文件

# 地图服务

使用`map_server`功能包实现地图的序列化和反序列化

map_server包含两个节点:map_saver和map_server,前者用于保存地图,后者用于读取地图并以服务的方式发送出去

## 1.map_saver

1. 订阅的话题:
* map(nav_msgs/OccupancyGrid)

2. 地图保存launch文件

```xml
<launch>
	<arg name="filename" value="yourpath" />
	<node name="map_save" pkg="map_server" type="map_saver" args="-f $(arg filename)" />
</launch>
```

在建图完成后调用map_saver即可保存地图

### 地图yaml文件

```yaml
# 图片文件位置
image: /path/src/navi/maps/nav.pgm 
# 分辨率,m/pixel
resolution: 0.050000  
# 地图的位姿信息(相对于rviz的位姿)
# origin 的前两个坐标是x,y两个方向的偏移量 第三个坐标是z轴的偏航角度,单位是rad
origin: [-50.000000, -50.000000, 0.000000]
# 是否取反,黑白块颜色
negate: 0 

# 地图中,白色是可通行区域,黑色是障碍物,蓝灰是未知区域
# 1. 地图中的每一个像素取值在 [0,255] 之间，白色为 255，黑色为 0，该值设为 x；
# 2. map_server 会将像素值作为判断是否是障碍物的依据，首先计算比例: p = (255 - x) / 255.0，白色为0，黑色为1(negate为true，则p = x / 255.0)；
# 3. 根据步骤2计算的比例判断是否是障碍物，如果 p > occupied_thresh 那么视为障碍物，如果 p < free_thresh 那么视为无物。

# 占用阈值,判断是否是障碍物
occupied_thresh: 0.65
# 空闲阈值,可通行区域
free_thresh: 0.196
```

## 2.map_server

1. 订阅的话题
* map_metadata(nav_msgs/MapMetaData)
* map(nav_msgs/OccupancyGrid)

2. 服务
* static_map(nav_msgs/GetMap)

3. 参数
* frame_id(string, default: "map") 地图坐标系


```xml
<launch>
	<arg name="ymalname" value="yourfilename" />
	<node name="map_server" pkg="map_server" type="map_server" args="$(find pkg)/path/to/yaml/$(arg yamlname)" />
</launch>
```

# 定位

使用amcl包进行定位,amcl是用于2d移动机器人的概率定位系统,采用了自适应的蒙特卡罗算法,包含在navigation功能包中

## 1. amcl节点说明

amcl 功能包中的核心节点是:amcl。为了方便调用，需要先了解该节点订阅的话题、发布的话题、服务以及相关参数。

1. 订阅的Topic
* scan(sensor_msgs/LaserScan):激光雷达数据。
* tf(tf/tfMessage):坐标变换消息.
* initialpose(geometry_msgs/PoseWithCovarianceStamped):用来初始化粒子滤波器的均值和协方差。
* map(nav_msgs/OccupancyGrid):获取地图数据。

2. 发布的Topic
* amcl_pose(geometry_msgs/PoseWithCovarianceStamped):机器人在地图中的位姿估计。
* particlecloud(geometry_msgs/PoseArray):位姿估计集合，rviz中可以被 PoseArray 订阅然后图形化显示机器人的位姿估计集合。
* tf(tf/tfMessage):发布从 odom 到 map 的转换。

3. 服务
* global_localization(std_srvs/Empty):初始化全局定位的服务。
* request_nomotion_update(std_srvs/Empty):手动执行更新和发布更新的粒子的服务。
* set_map(nav_msgs/SetMap):手动设置新地图和姿态的服务。

4. 调用的服务
* static_map(nav_msgs/GetMap):调用此服务获取地图数据。

5. 参数
* odom_model_type(string, default:"diff"):里程计模型选择: "diff","omni","diff-corrected","omni-corrected" (diff 差速、omni 全向轮)
* odom_frame_id(string, default:"odom"):里程计坐标系。
* base_frame_id(string, default:"base_link"):机器人极坐标系。
* global_frame_id(string, default:"map"):地图坐标系。

6. 坐标变换

里程计本身也是可以协助机器人定位的，不过里程计存在累计误差且一些特殊情况时(车轮打滑)会出现定位错误的情况，amcl 则可以通过估算机器人在地图坐标系下的姿态，再结合里程计提高定位准确度。

- 里程计定位:只是通过里程计数据实现 /odom_frame 与 /base_frame 之间的坐标变换。
- amcl定位: 可以提供 /map_frame 、/odom_frame 与 /base_frame 之间的坐标变换。

![](B-导航实现.png)

## 2. amcl的使用

1. 编写amcl的launch文件

```xml
<launch>
<node pkg="amcl" type="amcl" name="amcl" output="screen">
  <!-- Publish scans from best pose at a max of 10 Hz -->
  <param name="odom_model_type" value="diff"/><!-- 里程计模式为差分 -->
  <param name="odom_alpha5" value="0.1"/>
  <param name="transform_tolerance" value="0.2" />
  <param name="gui_publish_rate" value="10.0"/>
  <param name="laser_max_beams" value="30"/>
  <param name="min_particles" value="500"/>
  <param name="max_particles" value="5000"/>
  <param name="kld_err" value="0.05"/>
  <param name="kld_z" value="0.99"/>
  <param name="odom_alpha1" value="0.2"/>
  <param name="odom_alpha2" value="0.2"/>
  <!-- translation std dev, m -->
  <param name="odom_alpha3" value="0.8"/>
  <param name="odom_alpha4" value="0.2"/>
  <param name="laser_z_hit" value="0.5"/>
  <param name="laser_z_short" value="0.05"/>
  <param name="laser_z_max" value="0.05"/>
  <param name="laser_z_rand" value="0.5"/>
  <param name="laser_sigma_hit" value="0.2"/>
  <param name="laser_lambda_short" value="0.1"/>
  <param name="laser_lambda_short" value="0.1"/>
  <param name="laser_model_type" value="likelihood_field"/>
  <!-- <param name="laser_model_type" value="beam"/> -->
  <param name="laser_likelihood_max_dist" value="2.0"/>
  <param name="update_min_d" value="0.2"/>
  <param name="update_min_a" value="0.5"/>

  <param name="odom_frame_id" value="odom"/><!-- 里程计坐标系 -->
  <param name="base_frame_id" value="base_footprint"/><!-- 添加机器人基坐标系 -->
  <param name="global_frame_id" value="map"/><!-- 添加地图坐标系 -->

  <param name="resample_interval" value="1"/>
  <param name="transform_tolerance" value="0.1"/>
  <param name="recovery_alpha_slow" value="0.0"/>
  <param name="recovery_alpha_fast" value="0.0"/>
</node>
</launch>

```

2. 编写测试launch

```xml
<launch>
    
    <node pkg="joint_state_publisher" name="joint_state_publisher" type="joint_state_publisher" />
    <node pkg="robot_state_publisher" name="robot_state_publisher" type="robot_state_publisher" />
    
    <node pkg="rviz" type="rviz" name="rviz" />

    <include file="$(find navi)/launch/read.launch" />

    <include file="$(find navi)/launch/am.launch" />
    
</launch>
```

3. 在rviz中添加map,robotmodel和posearray组件

# 路径规划

ros提供了move_base功能包,move_base包中提供了路径规划实现

## 1.move_base节点说明

1. 动作

**动作订阅**
* move_base/goal(move_base_msgs/MoveBaseActionGoal):move_base 的运动规划目标。
* move_base/cancel(actionlib_msgs/GoalID):取消目标。

**动作发布**

* move_base/feedback(move_base_msgs/MoveBaseActionFeedback):连续反馈的信息，包含机器人底盘坐标。
* move_base/status(actionlib_msgs/GoalStatusArray):发送到move_base的目标状态信息。
* move_base/result(move_base_msgs/MoveBaseActionResult):操作结果(此处为空)。

2.订阅的Topic
* move_base_simple/goal(geometry_msgs/PoseStamped):运动规划目标(与action相比，没有连续反馈，无法追踪机器人执行状态)。

3. 发布的Topic
* cmd_vel(geometry_msgs/Twist):输出到机器人底盘的运动控制消息。

4. 服务
* make_plan(nav_msgs/GetPlan):请求该服务，可以获取给定目标的规划路径，但是并不执行该路径规划。
* clear_unknown_space(std_srvs/Empty):允许用户直接清除机器人周围的未知空间。
* clear_costmaps(std_srvs/Empty):允许清除代价地图中的障碍物，可能会导致机器人与障碍物碰撞，请慎用。

5. 参数
...

## 2.代价地图

机器人导航(尤其是路径规划模块)是依赖于地图的，地图在SLAM时已经有所介绍了，ROS中的地图其实就是一张图片，这张图片有宽度、高度、分辨率等元数据，在图片中使用灰度值来表示障碍物存在的概率。不过SLAM构建的地图在导航中是不可以直接使用的，因为：

1. SLAM构建的地图是静态地图，而导航过程中，障碍物信息是可变的，可能障碍物被移走了，也可能添加了新的障碍物，导航中需要时时的获取障碍物信息；
2. 在靠近障碍物边缘时，虽然此处是空闲区域，但是机器人在进入该区域后可能由于其他一些因素，比如：惯性、或者不规则形体的机器人转弯时可能会与障碍物产生碰撞，安全起见，最好在地图的障碍物边缘设置警戒区，尽量禁止机器人进入...

所以，静态地图无法直接应用于导航，其基础之上需要添加一些辅助信息的地图，比如时时获取的障碍物数据，基于静态地图添加的膨胀区等数据。

代价地图有两张:global_costmap(全局代价地图) 和 local_costmap(本地代价地图)，前者用于全局路径规划，后者用于本地路径规划。

两张代价地图都可以多层叠加,一般有以下层级:
- Static Map Layer：静态地图层，SLAM构建的静态地图。
- Obstacle Map Layer：障碍地图层，传感器感知的障碍物信息。
- Inflation Layer：膨胀层，在以上两层地图上进行膨胀（向外扩张），以避免机器人的外壳会撞上障碍物。
- Other Layers：自定义costmap。

多个layer可以按需自由搭配。

![](B-导航实现-1.png)

**碰撞算法**

在ROS中，如何计算代价值呢？请看下图:

![](B-导航实现-2.png)

上图中，横轴是距离机器人中心的距离，纵轴是代价地图中栅格的灰度值。

- 致命障碍:栅格值为254，此时障碍物与机器人中心重叠，必然发生碰撞；
- 内切障碍:栅格值为253，此时障碍物处于机器人的内切圆内，必然发生碰撞；
- 外切障碍:栅格值为[128,252]，此时障碍物处于其机器人的外切圆内，处于碰撞临界，不一定发生碰撞；
- 非自由空间:栅格值为(0,127]，此时机器人处于障碍物附近，属于危险警戒区，进入此区域，将来可能会发生碰撞；
- 自由区域:栅格值为0，此处机器人可以自由通过；
- 未知区域:栅格值为255，还没探明是否有障碍物。

膨胀空间的设置可以参考非自由空间。

## 3.move_base的使用

1. 编写launch文件

```xml
<launch>
    <node pkg="move_base" type="move_base" respawn="false" name="move_base" output="screen" clear_params="true">
        <rosparam file="$(find 功能包)/param/costmap_common_params.yaml" command="load" ns="global_costmap" />
        <rosparam file="$(find 功能包)/param/costmap_common_params.yaml" command="load" ns="local_costmap" />
        <rosparam file="$(find 功能包)/param/local_costmap_params.yaml" command="load" />
        <rosparam file="$(find 功能包)/param/global_costmap_params.yaml" command="load" />
        <rosparam file="$(find 功能包)/param/base_local_planner_params.yaml" command="load" />
    </node>
</launch>
```

2. 设置配置文件

	1. 新建param目录
	2. 在目录中新建上述launch文件中的参数yaml文件
	3. 在参数文件中填充参数

* costmap_common_params.yaml

```yaml
#机器人几何参数，如果机器人是圆形，设置 robot_radius,如果是其他形状设置 footprint
robot_radius: 0.12 #圆形
# footprint: [[-0.12, -0.12], [-0.12, 0.12], [0.12, 0.12], [0.12, -0.12]] #其他形状

obstacle_range: 3.0 # 用于障碍物探测，比如: 值为 3.0，意味着检测到距离小于 3 米的障碍物时，就会引入代价地图
raytrace_range: 3.5 # 用于清除障碍物，比如：值为 3.5，意味着清除代价地图中 3.5 米以外的障碍物


#膨胀半径，扩展在碰撞区域以外的代价区域，使得机器人规划路径避开障碍物
inflation_radius: 0.2
#代价比例系数，越大则代价值越小
cost_scaling_factor: 3.0

#地图类型
map_type: costmap
#导航包所需要的传感器
observation_sources: scan
#对传感器的坐标系和数据进行配置。这个也会用于代价地图添加和清除障碍物。例如，你可以用激光雷达传感器用于在代价地图添加障碍物，再添加kinect用于导航和清除障碍物。
scan: {sensor_frame: laser, data_type: LaserScan, topic: scan, marking: true, clearing: true}

```

* local_costmap_params.yaml

```yaml
local_costmap:
  global_frame: odom #里程计坐标系
  robot_base_frame: base_footprint #机器人坐标系

  update_frequency: 10.0 #代价地图更新频率
  publish_frequency: 10.0 #代价地图的发布频率
  transform_tolerance: 0.5 #等待坐标变换发布信息的超时时间

  static_map: false  #不需要静态地图，可以提升导航效果
  rolling_window: true #是否使用动态窗口，默认为false，在静态的全局地图中，地图不会变化
  width: 3 # 局部地图宽度 单位是 m
  height: 3 # 局部地图高度 单位是 m
  resolution: 0.05 # 局部地图分辨率 单位是 m/pixel，一般与静态地图分辨率保持一致

```

* global_costmap_params.yaml

```yaml
global_costmap:
  global_frame: map #地图坐标系
  robot_base_frame: base_footprint #机器人坐标系
  # 以此实现坐标变换

  update_frequency: 1.0 #代价地图更新频率
  publish_frequency: 1.0 #代价地图的发布频率
  transform_tolerance: 0.5 #等待坐标变换发布信息的超时时间

  static_map: true # 是否使用一个地图或者地图服务器来初始化全局代价地图，如果不使用静态地图，这个参数为false.

```

* base_local_planner_params.yaml

```yaml
TrajectoryPlannerROS:

# Robot Configuration Parameters
  max_vel_x: 0.5 # X 方向最大速度
  min_vel_x: 0.1 # X 方向最小速速

  max_vel_theta:  1.0 # 
  min_vel_theta: -1.0
  min_in_place_vel_theta: 1.0

  acc_lim_x: 1.0 # X 加速限制
  acc_lim_y: 0.0 # Y 加速限制
  acc_lim_theta: 0.6 # 角速度加速限制

# Goal Tolerance Parameters，目标公差
  xy_goal_tolerance: 0.10
  yaw_goal_tolerance: 0.05

# Differential-drive robot configuration
# 是否是全向移动机器人
  holonomic_robot: false

# Forward Simulation Parameters，前进模拟参数
  sim_time: 0.8
  vx_samples: 18
  vtheta_samples: 20
  sim_granularity: 0.05

```

参数配置技巧

以上配置在实操中，可能会出现机器人在本地路径规划时与全局路径规划不符而进入膨胀区域出现假死的情况，如何尽量避免这种情形呢？

> 全局路径规划与本地路径规划虽然设置的参数是一样的，但是二者路径规划和避障的职能不同，可以采用不同的参数设置策略:
> 
> - 全局代价地图可以将膨胀半径和障碍物系数设置的偏大一些；
> - 本地代价地图可以将膨胀半径和障碍物系数设置的偏小一些。
> 
> 这样，在全局路径规划时，规划的路径会尽量远离障碍物，而本地路径规划时，机器人即便偏离全局路径也会和障碍物之间保留更大的自由空间，从而避免了陷入“假死”的情形。

3. launch文件的集成

```xml
<launch>
    <!-- map_server -->
    <include file="$(find navi)/launch/read.launch" />
    
    <!-- amcl -->
    <include file="$(find navi)/launch/am.launch" />
    
    <!-- movebase -->
    <include file="$(navi)/launch/mb.launch" />

    <!-- rviz -->
    <node pkg="joint_state_publisher" name="joint_state_publisher" type="joint_state_publisher" />
    <node pkg="robot_state_publisher" name="robot_state_publisher" type="robot_state_publisher" />
    
    <node pkg="rviz" type="rviz" name="rviz" />
</launch>
```

4. amcl启动

	1. 启动gazebo
	2. 启动launch文件
	3. 添加rviz组件


# 导航与SLAM

slam本身就可以实现定位,这个过程实现较为简单

1. 编写launch文件

```xml
<launch>
    <!-- 启动SLAM节点 -->
    <include file="$(find mycar_nav)/launch/slam.launch" />
    <!-- 运行move_base节点 -->
    <include file="$(find mycar_nav)/launch/path.launch" />
    <!-- 运行rviz -->
    <node pkg="rviz" type="rviz" name="rviz" args="-d $(find mycar_nav)/rviz/nav.rviz" />
</launch>
```

2. 运行gazebo仿真
3. 运行launch文件

