# 在rviz中创建一个盒子机器人

1. 新建工作空间
2. 建立新的功能包, 依赖至少包含 `urdf` 和 `xacro` 
3. 创建 `launch`, `urdf`,`meshes`,`config`四个目录
4. 在`urdf`目录下再新建两个目录`urdf`和`xacro`
5. 在`urdf/urdf`下新建`urdf`文件,urdf文件是一个xml文件
6. 在urdf文件中添加信息

```xml
<robot name="mycar">
    <link name="base_link">
        <visual>
            <geometry>
                <box size="0.5 0.2 0.1" />
            </geometry>
        </visual>
    </link>
</robot>

```

> robot标签:机器人
> link标签:刚性连杆
> visual标签:可视化
> geometry标签:模型
> box:盒子组件

7. 创建launch文件

```xml
<launch>

    <!-- 在参数服务器载入urdf文件 -->
    <param name="robot_description" textfile="$(find 包名)/urdf/urdf/your.urdf" />

    <!-- 启动 rviz -->
    <node pkg="rviz" type="rviz" name="rviz" />

</launch>
```

8. 启动launch文件
9. rviz中"add-> RobotModel"
10. 修改参考坐标系

# 保存与读取配置

读取配置,在launch文件中添加
* `<node pkg="rviz" type="rviz" name="rviz" args="-d your/rviz/folder/xxx.rviz" />`

