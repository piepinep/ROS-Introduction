pluginlib是插件库,用来在ros功能包中加载和卸载插件.插件时指从运行时动态加载的类,通过这个库,不必将莫i个应用显示连接到包含某个类的库,pluginlib可以随时打开包含类的库,而不需要应用程序实现包含类的定义或者头文件

# 案例:

以插件的方式实现正多边形的计算

# 流程

1. 新建功能包,导入依赖 `roscpp pluginlib`

2. 配置include路径
在.vscode目录下,添加**功能包**下的`include`路径

3. 创建基类

```cpp
#ifndef DBX_H
#define DBX_H

namespace dbx_ns
{
class DBX
{
protected:
	// 基类需要提供无参构造函数
    DBX(){};
public:
    // 计算周长的函数
    virtual double get_length() = 0;
    // 设置边长的函数
    virtual void set_length(double side_length) = 0;
};
};

#endif
```

4. 创建插件类

注意:
>教程中插件类的命名空间与基类的命名空间不同,不清楚是否是规范

```cpp
#ifndef DBX_PLUGIN_H
#define DBX_PLUGIN_H

#include "dbx.h"

namespace dbx_ns
{
class SanBian: public DBX
{
private:
    double side_length;
public: 
    SanBian():side_length(0){}
    void set_length(double side_length)
    {
        this->side_length = side_length;
    }
    double get_length()
    {
        return 3 * side_length;
    }
};

class SiBian : public DBX
{
private:
    double side_length;

public:
    SiBian() : side_length(0) {}
    void set_length(double side_length)
    {
        this->side_length = side_length;
    }
    double get_length()
    {
        return 4 * side_length;
    }
};
};

#endif
```

5. 注册插件类

在src下新建cpp文件,并进行注册

```cpp
#include "pluginlib/class_list_macros.h"
#include "pi/dbx.h"
#include "pi/dbx_plugin.h"

// 参数1 是派生类, 参数2 是基类
PLUGINLIB_EXPORT_CLASS(dbx_ns::SanBian, dbx_ns::DBX)
PLUGINLIB_EXPORT_CLASS(dbx_ns::SiBian, dbx_ns::DBX)
```

6. 配置cmake

a. 找到 `include_directories` 语句,放开 `include` 的注释

```cmake
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)
```

b. 找到 `add_library` 语句,添加上文中的注册cpp文件

```cmake
add_library(plu
  src/plu.cpp
)
```

完成上述操作进行编译,如果成功,则会在 `devel/lib`目录下生成libxxx.so动态链接库

7. 使插件可用于ros

a. 在功能包下(package.xml同级目录)新建一个xml文件,在该xml中声明链接库以及声明子类与父类

b. 添加library标签并指明路径
注意:
> 目录的相对起点是devel
> 不需要添加.so后缀
> 注意需要包含命名空间!

c. 添加class标签,并知名子类与父类

d. 可看心情添加description标签

```xml
<library path="lib/libplu">
    <class type="dbx_ns::SanBian" base_class_type="dbx_ns::DBX">
        <description>triangle</description>
    </class>
    <class type="dbx_ns::SiBian" base_class_type="dbx_ns::DBX">
        <description>square</description>
    </class>
</library>
```

e.在package.xml中导出插件

找到export标签,在这个标签中根据功能包标签添加信息

```xml
  <export>
    <!-- Other tools can request additional information be placed here -->
    <!--这里的 pi 是基类的包名-->
    <pi plugin="${prefix}/plu.xml" />
  </export>
```

f.编译

g.使用

```cpp
#include "ros/ros.h"
#include "pluginlib/class_loader.h"
#include "pi/dbx.h"

/**
 * 1. 创建类加载器
 * 2. 使用类加载器实例化某个插件对象
 * 3. 使用插件
*/

int main(int argc, char *argv[])
{
	// 第一个参数功能包名,第二个参数是基类的完整名称
    pluginlib::ClassLoader<dbx_ns::DBX> cl("pi", "dbx_ns::DBX");
	// 参数是想要调用的派生类数据
    boost::shared_ptr<dbx_ns::DBX> san = cl.createInstance("dbx_ns::SanBian");

    san->set_length(10);
   
    ROS_INFO("sanjiaoxing zhouchang = %.2f", san->get_length());

    return 0;
}

```

