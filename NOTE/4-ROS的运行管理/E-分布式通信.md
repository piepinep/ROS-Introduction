ROS是一个分布式计算环境，一个运行中的ROS系统可以包含分布在多台计算机上的多个节点。ROS对网络配置有某些要求：
1. 所有端口上的机器之间必须有完整的双向链接
2. 每台计算机必须通过所有其他计算机都可以解析的名称来公告自己

# 准备
确保计算机处于同一网络中，最好设置固定ip

# 配置文件的修改

分别修改两台计算机的 `/etc/hosts` 文件，在该文件中加入对方的ip地址和计算机名

查看本机主机名称: `hostname`

```hosts
IP地址 hostname
```

host文件生效的方式:重启或者调用命令

# 配置主/从机的IP

**主机**: 启动roscore的计算机称为主机
**从机**:其余计算机称为从机

## 配置主机的IP地址

在`.bashrc`文件中添加

```bash
export ROS_MASTER_URI=http://主机ip:11311
export ROS_HOSTNAME=主机ip
```

## 配置从机的IP地址

在`.bashrc`文件中添加

```bash
export ROS_MASTER_URI=http://主机ip:11311
export ROS_HOSTNAME=从机ip
```

# 测试

1. 在主机启动roscore
2. 在主/从机上分别启动订阅节点/发布节点,测试是否可以顺利接收数据

