#include "ros/ros.h"

/**
 * 设置机器人的共享参数并修改参数
 * ros::NodeHandle
 *     setParam
 * ros::param
 *     set
 * 
 * 获取参数
 * ros::NodeHandle
 *  param(key, default): 存在返回值,否则返回default
 *  getParam(key, var): 存在返回true, 值赋值给var,否则返回false, 不赋值
 *  gerParamCached(key, bar): 同上,但是使用cache
 *  getParamNames(std::vector<std::string>): 获取所有的键.并存储在vector中
 *  hasParam(key): 查看是否存在key
 *  searchParam(key, str): 判断是否存在key,如果存在,将key放在str变量中(有前缀/)
 * ros::param
 *     类似NodeHandle
 */

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "paramc_set");

    ros::NodeHandle nh;

    // 使用 ros::NodeHandle
    nh.setParam("type", "car");
    nh.setParam("radius", 0.15);

    // 使用 ros::param
    ros::param::set("color", "yellow");
    ros::param::set("price", 5000);

    // 修改
    nh.setParam("type", "plane");
    ros::param::set("price", 500000);

    return 0;
}
