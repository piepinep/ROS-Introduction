# 使用VSCode完成ROS初始工作

# 创建ros工作空间

```bash
$ mkdir -p workspace02/src
$ cd workspace02
$ catkin_make
$ code .
```

![](1-ROS程序简单实现-进入vscode.png)

# 创建ros功能包

打开点击src目录,右击选择创建功能包

![](1-ROS程序简单实现-创建功能包.png)

分别输入包名和依赖

![](1-ROS程序简单实现-包名.png)

![](1-ROS程序简单实现-依赖.png)

# 代码编写与文件配置

此部分与[代码编写与文件配置](A-编写一个简单的ROS程序.md)一致

# 编译

在vscode中使用快捷键`ctrl+shift+b`,选择`catkin_make:build`,点击齿轮生成`tasks.json`文件

![](1-ROS程序简单实现-设置快捷编译.png)

`task.json`可以使用默认参数,也可以使用如下参数
```json
{
// 有关 tasks.json 格式的文档，请参见
    // https://go.microsoft.com/fwlink/?LinkId=733558
    "version": "2.0.0",
    "tasks": [
        {
            "label": "catkin_make:debug", //代表提示的描述性信息
            "type": "shell",  //可以选择shell或者process,如果是shell代码是在shell里面运行一个命令，如果是process代表作为一个进程来运行
            "command": "catkin_make",//这个是我们需要运行的命令
            "args": [],//如果需要在命令后面加一些后缀，可以写在这里，比如-DCATKIN_WHITELIST_PACKAGES=“pac1;pac2”
            "group": {"kind":"build","isDefault":true},
            "presentation": {
                "reveal": "always"//可选always或者silence，代表是否输出信息
            },
            "problemMatcher": "$catkin-gcc"
        }
    ]
}
```
# 执行

在一个终端分支中使用 `roscore`

另一个终端中:
![](1-ROS程序简单实现-执行.png)


# 中文输出乱码问题(CPP)

在main函数中添加
```cpp
int main(...)
{
	// 方法一
	setlocale(LC_ALL, "");
	// 方法二
	setlocale(LC_CTYPE, "zh_CN.utf8");
	...
}
```
