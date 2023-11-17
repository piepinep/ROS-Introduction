#include "ros/ros.h"
#include "services_communication/TwoIntSum.h"

/**
 * 服务端实现,解析客户端的数据并响应
 * 1. 包含相关头文件
 * 2. 初始化ros节点
 * 3. 创建节点句柄
 * 4. 创建客户端对象
 * 5. 提交请求
 * 6. 处理响应
 */

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "services_client_cpp");

    ros::NodeHandle nh;

    ros::ServiceClient sc = nh.serviceClient<services_communication::TwoIntSum>("services");

    services_communication::TwoIntSum ai;
    ai.request.int1 = atoi(argv[argc - 2]);
    ai.request.int2 = atoi(argv[argc - 1]);

    sc.waitForExistence();
    // ros::service::waitForService("services");
    
    if (sc.call(ai))
    {
        ROS_INFO("response = %d", ai.response.sum);
    }
    else
    {
        ROS_INFO("failed");
    }

    return 0;
}