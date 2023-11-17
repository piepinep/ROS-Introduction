ROS中提供了一些实用的命令行工具，这些工具可以获取不同节点的各类信息，常用命令有
* rosnode
* rostopic
* rosservice
* rosmsg
* rossrv
* rosparam

# ROSNODE

```bash
$ rosnode ping    # test connectivity to node
$ rosnode list    # list active nodes
$ rosnode info    # print information about node
$ rosnode machine # list nodes running on a particular machine or list machines
$ rosnode kill    # kill a running node
$ rosnode cleanup # purge registration information of unreachable nodes
```

> rosnode list: 当前运行的节点
> rosnode ping：节点是否可以连接，`rosnode ping /nodename`
> rosnode info: 查看信息
> rosnode machine: 某个机器上的正在运行的节点
> rosnode kill: 终结某个节点
> rosnode cleanup: 清除已注册的僵尸节点 

# ROSTOPIC

```bash
$ rostopic bw     # display bandwidth used by topic
$ rostopic delay  # display delay of topic from timestamp in header
$ rostopic echo   # print messages to screen
$ rostopic find   # find topics by type
$ rostopic hz     # display publishing rate of topic    
$ rostopic info   # print information about active topic
$ rostopic list   # list active topics
$ rostopic pub    # publish data to topic
$ rostopic type   # print topic or field type
```

# ROSSERVICE

```bash
$ rosservice args # print service arguments
$ rosservice call # call the service with the provided args
$ rosservice find # find services by service type
$ rosservice info # print information about service
$ rosservice list # list active services
$ rosservice type # 打印服务的数据类型
$ rosservice uri  # print service ROSRPC uri
```

# ROSMSG

显示消息载体相关的信息

```bash
$ rosmsg show     # 查看消息的具体格式
$ rosmsg info     # 同show
$ rosmsg list     # List all messages
$ rosmsg md5      # Display message md5sum
$ rosmsg package  # List messages in a package
$ rosmsg packages # List packages that contain messages
```

# ROSSRV

显示服务数据类型的相关信息

```bash
$ rossrv show     # Show service description
$ rossrv info     # Alias for rossrv show
$ rossrv list     # List all services
$ rossrv md5      # Display service md5sum
$ rossrv package  # List services in a package
$ rossrv packages # List packages that contain services
```

# ROSPARAM

用于使用YAML编码文件在参数服务器上获取和设置ROS参数

```bash
$ rosparam set    # set parameter
$ rosparam get    # get parameter
$ rosparam load   # 从yaml文件中导入参数
$ rosparam dump   # 导出参数至yaml文件
$ rosparam delete # delete parameter
$ rosparam list   # list parameter names
```

