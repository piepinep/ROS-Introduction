#include "ros/ros.h"
#include "services_communication/TwoIntSum.h"

/**
 * 服务端实现,解析客户端的数据并响应
 * 1. 包含相关头文件
 * 2. 初始化ros节点
 * 3. 创建节点句柄
 * 4. 创建服务器对象
 * 5. 处理请求并产生响应
 * 6. spin()
 */

bool callback(services_communication::TwoIntSum::Request &req,
            services_communication::TwoIntSum::Response &res)
{
    // 处理请求
    int a = req.int1, b = req.int2;

    // 组织响应
    res.sum = a + b;

    ROS_INFO("num1 = %d, num2 = %d, sum = %d", a, b, res.sum);
    return true;
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "services_server_cpp");

    ros::NodeHandle nh;

    ros::ServiceServer ss = nh.advertiseService("services", callback);

    ros::spin();
    return 0;
}