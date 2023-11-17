launch文件时一个xml文件,可以启动本地和远程多个节点,还可以在参数服务器中设置参数

# launch文件的使用

1. 在功能包目录下新建 `launch` 目录
![](C-launch文件.png)

2. 在launch目录下新建 `xxx.launch` 文件

```xml
<launch>
    <!-- launch label-->
</launch>
```

3. 使用

```bash
$ roslaunch yourpackage yourlaunch.launch
```

# launch文件的标签

## launch

`<launch>`标签是所有launch文件的跟标签,充当其他标签的容器,只有一个属性 `deprecated ="弃用声明"`
```xml
<launch [deprecated="弃用"]>
	<!--launch labels -->
</launch>
```

## node

`<node>` 标签是最常见的标签,用来指定ROS节点. 需要注意的是roslaunch不能保证按照 **node的顺序** 启动node(节点的启动是异步多进程的)

### 属性

**required**:
* pkg="包名"
* type="节点名/可执行程序名"
* name="节点在拓扑网络中的命名", 可以与节点名不同

**optional**:
* args="args lists"参数列表
* machine="机器名",指定机器上运行
* respawn="true|false", 节点退出是否自动重启
* respawn_delay="N",延迟多久重启,单位s
* required="true|false",该节点是否必要,如果必要,则该节点退出后roslaunch退出
* ns="namespace", 节点的前缀,可以避免重名问题
* clear_params="true|false",启动节点是否删除私有空间的参数
* output="log|screen",log的输出目录,默认为log

### 子级标签
* env:环境变量
* remap:重映射节点名称
* rosparam:参数设置
* param:参数设置

## include

`<include>`用于将另一个xml格式的launch文件导入到当前文件

### 属性
* file=`"$(find 包名)/launch/xxx.launch"`,要包含的文件路径
* ns="namespace", 在指定的ns下导入

### 子级标签
* env
* arg


## remap

`<remap>`用于
1. 话题重命名
2. 更改通信使用的话题

### 属性:
* from="xxx"
* to="yyy"

## param

可以使用在 `<launch>` 和 `<node>` 下
`<param>`主要用于在参数服务器上设置参数,参数源可以通过标签 value 实现指定,也可以通过外部文件加载, 在`<node>`标签中时,相当于私有命名空间

### 属性
* name="[命名空间/]参数名",参数名可以包含命名空间
* value="xxx",可选,此处省略则必须指定外部文件作为参数源
* type="str|int|double|bool|yaml",可选,如果未指定:如果包含`.`的数据认为是浮点数,否则为整数,true|false是bool类型,其他是字符串

## rosparam

可以使用在 `<launch>` 和 `<node>` 下
`<rosparam>`标签可以从yaml文件中导入参数或者导出参数到yaml文件,也可以删除参数,在`<node>`标签中时,相当于私有命名空间

### 属性
* command="load|dump|delete"(可选,默认为load)
* file=`"$(find 包名)/xxxx/xxx.yaml"`, 加载或者导出的yaml文件
* param="参数名称"
* ns="namespace",可选

## group

对节点进行分组,具有ns属性,可以让节点归属于某个命名空间

### 属性
* ns
* clear_params="true|false"(可选)

### 子级标签
除了launch以为的其余标签

## arg

用于动态传参,可以增强launch文件的灵活性

### 属性
* name="参数名"
* default="默认值"(可选)
* value="数值"(可选,不可以与default共存)
* doc="描述信息"

```xml
...
<arg name="arg_name" default="0.5" />
<param name="A", value="$(arg arg_name)" /> 
...
```

arg还可以动态传参

```bash
$ roslaunch youpackage yourlaunch.launch arg.arg_name:=0.7
```