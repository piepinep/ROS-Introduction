# 编写一个简单的ROS程序-以hello world为例

ROS程序的主要实现流程大致为五条:
1. 创建工作空间
2. 创建功能包
3. 编辑源文件
4. 编辑配置信息
5. 编译执行


# 创建工作空间

工作空间就是你的项目所在的空间, 一个工作空间可以有很多自己编写的功能包.

* 命令
```bash
# 创建工作空间目录
mkdir -p <workspace_name>/src
# 进入工作空间
cd <workspace_name>
# 创建 cmake 环境
catkin_make
```

* 实际操作
``` bash
$ pwd
# /home/your/path

$ mkdir -p workspace01/src
$ cd workspace01
$ catkin_make

# Base path: /home/your/path/workspace01
# Source space: /home/your/path/workspace01/src
# Build space: /home/your/path/workspace01/build
# Devel space: /home/your/path/workspace01/devel
# Install space: /home/your/path/workspace01/install
# Creating symlink "/home/your/path/workspace01/src/CMakeLists.txt" pointing to "/opt/ros/noetic/share/catkin/cmake/toplevel.cmake"
# ####
# #### Running command: "cmake /home/your/path/workspace01/src -DCATKIN_DEVEL_PREFIX=/home/your/path/workspace01/devel -DCMAKE_INSTALL_PREFIX=/home/your/path/workspace01/install -G Unix Makefiles" in "/home/your/path/workspace01/build"
# ####
# -- The C compiler identification is GNU 9.4.0
# -- The CXX compiler identification is GNU 9.4.0
# -- Detecting C compiler ABI info
# -- Detecting C compiler ABI info - done
# -- Check for working C compiler: /usr/bin/cc - skipped
# -- Detecting C compile features
# -- Detecting C compile features - done
# -- Detecting CXX compiler ABI info
# -- Detecting CXX compiler ABI info - done
# -- Check for working CXX compiler: /usr/bin/c++ - skipped
# -- Detecting CXX compile features
# -- Detecting CXX compile features - done
# -- Using CATKIN_DEVEL_PREFIX: /home/your/path/workspace01/devel
# -- Using CMAKE_PREFIX_PATH: /opt/ros/noetic
# -- This workspace overlays: /opt/ros/noetic
# -- Found PythonInterp: /usr/bin/python3 (found suitable version "3.8.10", minimum required is "3")
# -- Using PYTHON_EXECUTABLE: /usr/bin/python3
# -- Using Debian Python package layout
# -- Found PY_em: /usr/lib/python3/dist-packages/em.py
# -- Using empy: /usr/lib/python3/dist-packages/em.py
# -- Using CATKIN_ENABLE_TESTING: ON
# -- Call enable_testing()
# -- Using CATKIN_TEST_RESULTS_DIR: /home/your/path/workspace01/build/test_results
# -- Forcing gtest/gmock from source, though one was otherwise available.
# -- Found gtest sources under '/usr/src/googletest': gtests will be built
# -- Found gmock sources under '/usr/src/googletest': gmock will be built
# -- Found PythonInterp: /usr/bin/python3 (found version "3.8.10")
# -- Found Threads: TRUE
# -- Using Python nosetests: /usr/bin/nosetests3
# -- catkin 0.8.10
# -- BUILD_SHARED_LIBS is on
# -- BUILD_SHARED_LIBS is on
# -- Configuring done
# -- Generating done
# -- Build files have been written to: /home/your/path/workspace01/build
####
#### Running command: "make -j16 -l16" in "/home/your/path/workspace01/build"
####
```

当看到 `Running command: "make -j16 -l16"` 时可以认为`workspace`创建成功.此时workspace目录中的路径为:

```bash
# tree . -L 2
.
├── build
│   ├── CATKIN_IGNORE
│   ├── CMakeCache.txt
│   ├── CMakeFiles
│   ├── CTestConfiguration.ini
│   ├── CTestCustom.cmake
│   ├── CTestTestfile.cmake
│   ├── Makefile
│   ├── atomic_configure
│   ├── bin
│   ├── catkin
│   ├── catkin_generated
│   ├── catkin_make.cache
│   ├── cmake_install.cmake
│   ├── gtest
│   └── test_results
├── devel
│   ├── _setup_util.py
│   ├── cmake.lock
│   ├── env.sh
│   ├── lib
│   ├── local_setup.bash
│   ├── local_setup.sh
│   ├── local_setup.zsh
│   ├── setup.bash
│   ├── setup.sh
│   └── setup.zsh
└── src
    └── CMakeLists.txt -> /opt/ros/noetic/share/catkin/cmake/toplevel.cmake
```

# 创建功能包

* 命令
```bash
# 进入 src 目录
cd src
# 创建自定义包, 需要指定依赖
catkin_create_pkg <ros_package_name> <ros_pack_dependence_list> 
```

* 实际操作
```bash
$ pwd
# /home/your/path/workspace01

$ cd src
$ catkin_create_pkg helloworld roscpp rospy std_msgs

# Created file helloworld/package.xml
# Created file helloworld/CMakeLists.txt
# Created folder helloworld/include/helloworld
# Created folder helloworld/src
# Successfully created files in /home/your/path/workspace01/src/helloworld. Please adjust the values in package.xml.
```

1. 当出现 `Successfully created files` 时,说明功能包创建完毕
2. roscpp是ros的cpp依赖, rospy是ros的python依赖, std_msgs是标准消息库依赖
3. 使用依赖可以在`helloworld/package.xml`中修改

查看目录组织

```bash
# $ tree
.
├── CMakeLists.txt -> /opt/ros/noetic/share/catkin/cmake/toplevel.cmake
└── helloworld
    ├── CMakeLists.txt
    ├── include
    │   └── helloworld
    ├── package.xml
    └── src
```


# 代码编写与文件配置

## 以下流程为 CPP 版本
### 编辑源文件

```bash
$ pwd
# /home/your/path/workspace01/src

$ ls
# CMakeLists.txt  helloworld

$ cd helloworld/src
$ vim cpp_helloworld.cpp
```

在文件中输入如下代码

```cpp
// 1. 包含 ros 头文件
#include "ros/ros.h"

// 2. 编写 main 函数
int main(int argc, char *argv[])
{
    // 3. 初始化 ros 节点
    ros::init(argc, argv, "helloworld");
    // 4. 输出日志
    ROS_INFO("hello world");
    return 0;
}
```

### 编辑配置文件

```bash
$ pwd
# /home/your/path/workspace01/src/helloworld/src

$ cd ..
$ ls
# CMakeLists.txt  include  package.xml  src
```

打开 CMakeLists.txt, 修改两个位置

此处命令的使用见 
* [简单cmake的使用](/Tools/cmake/1-ASimpleCMakeListsTxt.md)
* [cmake库相关语法](/Tools/cmake/2-Libraries.md)

```cmake
## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
## add_executable(${PROJECT_NAME}_node src/helloworld_node.cpp)

# 添加一行
add_executable(${PROJECT_NAME}_node src/cpp_helloworld.cpp)
```


```bash
## Specify libraries to link a library or executable target against
# target_link_libraries(${PROJECT_NAME}_node
#   ${catkin_LIBRARIES}
# )

# 添加一段代码
target_link_libraries(
	${PROJECT_NAME}_node 
	${catkin_LIBRARIES}
)
```

## 以下流程为 PYTHON 版本

### 编辑源文件
```bash
$ pwd
# /home/your/path/workspace01/src/helloworld

# 创建scripts目录
$ mkdir scripts

# 进入scripts目录,并创建py文件
$ cd scripts/
$ vim py_helloworld.py
```

在文件中输入如下代码

```python
#! /usr/bin/env python
## 指定解释器

# 导包
import rospy

# 编写主入口
if __name__ == "__main__":
    
    # 初始化ros节点
    rospy.init_node("hello_python_world")
    
    # 输出日志
    rospy.loginfo("hello world python")
    
```

### 添加可执行权限
* 命令
```bash
chmod +x <your_python_file>
```

* 实际操作
```bash
$ chmod +x py_helloworld.py
```

### 编辑配置文件

```bash
$ pwd
# /home/your/path/workspace01/src/helloworld/scripts

$ cd ..
$ ls
# CMakeLists.txt  include  package.xml  src
```

打开 CMakeLists.txt, 找到install部分进行修改

```cmake
## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
## catkin_install_python(PROGRAMS
##   scripts/my_python_script
##   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
## )

# 添加一段代码
catkin_install_python(PROGRAMS
  scripts/py_helloworld.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
```

# 编译

* 命令:
在工作空间目录下执行
```bash
catkin_make
```

- 实际操作
```bash
$ pwd
# /home/your/path/workspace01

$ catkin_make

# Base path: /home/your/path/workspace01
# Source space: /home/your/path/workspace01/src
# Build space: /home/your/path/workspace01/build
# Devel space: /home/your/path/workspace01/devel
# Install space: /home/your/path/workspace01/install
# ####
# #### Running command: "cmake /home/your/path/workspace01/src -DCATKIN_DEVEL_PREFIX=/home/your/path/workspace01/devel -DCMAKE_INSTALL_PREFIX=/home/your/path/workspace01/install -G Unix Makefiles" in "/home/your/path/workspace01/build"
# ####
# -- Using CATKIN_DEVEL_PREFIX: /home/your/path/workspace01/devel
# -- Using CMAKE_PREFIX_PATH: /opt/ros/noetic
# -- This workspace overlays: /opt/ros/noetic
# -- Found PythonInterp: /usr/bin/python3 (found suitable version "3.8.10", minimum required is "3")
# -- Using PYTHON_EXECUTABLE: /usr/bin/python3
# -- Using Debian Python package layout
# -- Using empy: /usr/lib/python3/dist-packages/em.py
# -- Using CATKIN_ENABLE_TESTING: ON
# -- Call enable_testing()
# -- Using CATKIN_TEST_RESULTS_DIR: /home/your/path/workspace01/build/test_results
# -- Forcing gtest/gmock from source, though one was otherwise available.
# -- Found gtest sources under '/usr/src/googletest': gtests will be built
# -- Found gmock sources under '/usr/src/googletest': gmock will be built
# CMake Deprecation Warning at /usr/src/googletest/CMakeLists.txt:4 (cmake_minimum_required):
#   Compatibility with CMake < 2.8.12 will be removed from a future version of
#   CMake.

#   Update the VERSION argument <min> value or use a ...<max> suffix to tell
#   CMake that the project does not need compatibility with older versions.


# CMake Deprecation Warning at /usr/src/googletest/googlemock/CMakeLists.txt:45 (cmake_minimum_required):
#   Compatibility with CMake < 2.8.12 will be removed from a future version of
#   CMake.

#   Update the VERSION argument <min> value or use a ...<max> suffix to tell
#   CMake that the project does not need compatibility with older versions.


# CMake Deprecation Warning at /usr/src/googletest/googletest/CMakeLists.txt:56 (cmake_minimum_required):
#   Compatibility with CMake < 2.8.12 will be removed from a future version of
#   CMake.

#   Update the VERSION argument <min> value or use a ...<max> suffix to tell
#   CMake that the project does not need compatibility with older versions.


# -- Found PythonInterp: /usr/bin/python3 (found version "3.8.10")
# -- Using Python nosetests: /usr/bin/nosetests3
# -- catkin 0.8.10
# -- BUILD_SHARED_LIBS is on
# -- BUILD_SHARED_LIBS is on
# -- ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
# -- ~~  traversing 1 packages in topological order:
# -- ~~  - helloworld
# -- ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
# -- +++ processing catkin package: 'helloworld'
# -- ==> add_subdirectory(helloworld)
# -- Configuring done
# -- Generating done
# -- Build files have been written to: /home/your/path/workspace01/build
# ####
# #### Running command: "make -j16 -l16" in "/home/your/path/workspace01/build"
# ####
# [ 50%] Building CXX object helloworld/CMakeFiles/helloworld_node.dir/src/cpp_helloworld.cpp.o
# [100%] Linking CXX executable /home/your/path/workspace01/devel/lib/helloworld/helloworld_node
# [100%] Built target helloworld_node
```

当出现 `[100%] Built target helloworld_node` 则说明程序正常

# 执行

* 命令:
在一个终端中开启roscore, 另一个终端中执行程序

```bash
# t1
roscore

# t2
source ./devel/setup.bash
# cpp
rosrun <your_package_name> <your_cmake_target_name>
# python
rosrun helloworld rosrun <your_package_name> <your_python_file>.py
```

新开终端

* 终端1
```bash
$ roscore
```

* 终端2
```bash
$ pwd
# /home/your/path/workspace01

$ source ./devel/setup.bash # 修改环境变量, 使当前 session/terminal 具有执行环境

# cpp 执行
$ rosrun helloworld helloworld_node
# [ INFO] [1699234365.796380282]: hello world

# python执行
$ rosrun helloworld py_helloworld.py
# [INFO] [1699235903.894436]: hello world python
```
