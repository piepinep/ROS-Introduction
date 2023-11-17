# 地图

1. `nav_msgs/MapMetaData`

```msg
time map_load_time
float32 resolution
uint32 width
uint32 height
geometry_msgs/Pose origin
  geometry_msgs/Point position
    float64 x
    float64 y
    float64 z
  geometry_msgs/Quaternion orientation
    float64 x
    float64 y
    float64 z
    float64 w
```

2. `nav_msgs/OccupancyGrid`

```msg
std_msgs/Header header
  uint32 seq
  time stamp
  string frame_id
nav_msgs/MapMetaData info
  time map_load_time
  float32 resolution
  uint32 width
  uint32 height
  geometry_msgs/Pose origin
    geometry_msgs/Point position
      float64 x
      float64 y
      float64 z
    geometry_msgs/Quaternion orientation
      float64 x
      float64 y
      float64 z
      float64 w
int8[] data # 地图数组内容,数组长度等于宽度×高度
```

# 里程计

`nav_msgs/Odometry`

```msg
std_msgs/Header header
  uint32 seq
  time stamp
  string frame_id
string child_frame_id
geometry_msgs/PoseWithCovariance pose
  geometry_msgs/Pose pose
    geometry_msgs/Point position
      float64 x
      float64 y
      float64 z
    geometry_msgs/Quaternion orientation
      float64 x
      float64 y
      float64 z
      float64 w
  float64[36] covariance
geometry_msgs/TwistWithCovariance twist
  geometry_msgs/Twist twist
    geometry_msgs/Vector3 linear
      float64 x
      float64 y
      float64 z
    geometry_msgs/Vector3 angular
      float64 x
      float64 y
      float64 z
  float64[36] covariance
```

# 坐标变换

`tf/tfMessage`

```msg
geometry_msgs/TransformStamped[] transforms
  std_msgs/Header header
    uint32 seq
    time stamp
    string frame_id
  string child_frame_id
  geometry_msgs/Transform transform
    geometry_msgs/Vector3 translation
      float64 x
      float64 y
      float64 z
    geometry_msgs/Quaternion rotation
      float64 x
      float64 y
      float64 z
      float64 w
```

# 定位

`geometry_msgs/PoseArray`

```msg
std_msgs/Header header
  uint32 seq
  time stamp
  string frame_id
geometry_msgs/Pose[] poses
  geometry_msgs/Point position
    float64 x
    float64 y
    float64 z
  geometry_msgs/Quaternion orientation
    float64 x
    float64 y
    float64 z
    float64 w
```

# 路径规划

`move_base_msgs/MoveBaseActionGoal`

```msg
std_msgs/Header header
  uint32 seq
  time stamp
  string frame_id
actionlib_msgs/GoalID goal_id
  time stamp
  string id
move_base_msgs/MoveBaseGoal goal
  geometry_msgs/PoseStamped target_pose
    std_msgs/Header header
      uint32 seq
      time stamp
      string frame_id
    geometry_msgs/Pose pose
      geometry_msgs/Point position
        float64 x
        float64 y
        float64 z
      geometry_msgs/Quaternion orientation
        float64 x
        float64 y
        float64 z
        float64 w
```

# 激光雷达

`sensor_msgs/LaserScan`

```mag
std_msgs/Header header
  uint32 seq
  time stamp
  string frame_id
float32 angle_min
float32 angle_max
float32 angle_increment
float32 time_increment
float32 scan_time
float32 range_min
float32 range_max
float32[] ranges
float32[] intensities
```

# 相机

1. `sensor_msgs/Image`

```msg
std_msgs/Header header
  uint32 seq
  time stamp
  string frame_id
uint32 height
uint32 width
string encoding
uint8 is_bigendian
uint32 step
uint8[] data
```

2. `sensor_msgs/CompressedImage`

```msg
std_msgs/Header header
  uint32 seq
  time stamp
  string frame_id
string format
uint8[] data
```

3. `sensor_msgs/PointCloud2`

```msg
std_msgs/Header header
  uint32 seq
  time stamp
  string frame_id
uint32 height
uint32 width
sensor_msgs/PointField[] fields
  uint8 INT8=1
  uint8 UINT8=2
  uint8 INT16=3
  uint8 UINT16=4
  uint8 INT32=5
  uint8 UINT32=6
  uint8 FLOAT32=7
  uint8 FLOAT64=8
  string name
  uint32 offset
  uint8 datatype
  uint32 count
bool is_bigendian
uint32 point_step
uint32 row_step
uint8[] data
bool is_dense
```

# 深度图像转激光数据

ros中提供了一个功能包`depthimage_to_laserscan`,这个功能包可以将深度信息转换成激光雷达

1. 安装

```bash
$ sudo apt install ros-noetic-depthimage-to-laserscan
```

2. 订阅的topic
* image(sensor_msgs/Image): 输入图像数据
* camera_info(sensor_msgs/CameraInfo): 关联图像的相机信息

3. 发布的topic
* scan(sensor_msgs/LaserScane)

4. 参数
* scan_height(int, default: 1 pixel):设置用于生成激光雷达信息的象素行数。
* scan_time(double, default: 1/30.0Hz (0.033s)):两次扫描的时间间隔。
* range_min(double, default: 0.45m):返回的最小范围。结合range_max使用，只会获取 range_min 与 range_max 之间的数据。
* range_max(double, default: 10.0m): 返回的最大范围。结合range_min使用，只会获取 range_min 与 range_max 之间的数据。
* output_frame_id(str, default: camera_depth_frame):激光信息的ID。

5. 使用

1. 编写launch文件
2. 修改/注释xacro文件中与激光雷达相关的内容
3. 启动gazebo仿真环境
4. 启动rviz并添加相关组件

```xml
<launch>
    <node pkg="depthimage_to_laserscan" type="depthimage_to_laserscan" name="depthimage_to_laserscan">
        <remap from="image" to="/camera/depth/image_raw" />
        <param name="output_frame_id" value="camera"  />
    </node>
</launch>
```

