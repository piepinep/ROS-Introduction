# ROS的头文件与源文件

在功能包`include/packname`目录下新建 `.h` 文件, 想要使用头文件还需要进行配置, 在workspace目录下的.vscode的`c_cpp_properities.json`中添加头文件目录

在编译时需要配置cmakelists.txt文件
1. 找到`include_directories`项, 并取消`include`关键字的注释

```cmake
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)
```

2. 找到`add_libraries`项,添加需要的头文件和cpp文件目录
```cmake
## add_library(${PROJECT_NAME}
##   src/${PROJECT_NAME}/pubsub.cpp
## )
add_library(
	library_name
	source/files.cpp
)
```

3. 找到 `add_dependences`, 进行`${PROJECT_NAME}`部分的修改
```cmake
## add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(your_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
```

4. 找到`add_executable`,将所有的`.cpp`文件添加进入
5. 找到`target_link_libraries`,按照依赖顺序添加链接库,**注意链接顺序**

# Python的导入

添加环境变量
```python
import sys
sys.path.insert(pos, env_variables)

import your_pack
```

