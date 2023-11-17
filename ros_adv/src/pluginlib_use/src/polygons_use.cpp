#include "ros/ros.h"
#include "pluginlib/class_loader.h"
#include "pluginlib_use/polygons.h"

/**
 * 1. 创建类加载器
 * 2. 使用类加载器实例化某个插件对象
 * 3. 使用插件
 */

int main(int argc, char *argv[])
{
    pluginlib::ClassLoader<polygon_ns::PolygonBase> cl("pluginlib_use", "polygon_ns::PolygonBase");

    boost::shared_ptr<polygon_ns::PolygonBase> tri = cl.createInstance("polygons_ns::Triangle");

    tri->set_side_length(10);

    ROS_INFO("triangle perimeter = %.2f", tri->get_perimeter());

    return 0;
}
