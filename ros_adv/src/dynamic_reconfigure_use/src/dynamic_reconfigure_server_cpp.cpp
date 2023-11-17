#include "ros/ros.h"
#include "dynamic_reconfigure/server.h"
#include "dynamic_reconfigure_use/dynamic_reconfigure_clientConfig.h"

/**
 * 1. 包含头文件
 * 2. 初始化节点
 * 3. 创建服务器对象
 * 4. 回调对象解析
 * 5. spin
*/

// const boost::function<void (dynamic_reconfigure_use::dynamic_reconfigure_clientConfig &, uint32_t level)> &callback
// 需要使用boost::bind进行绑定/经检验不绑定也可以使用
void cb(dynamic_reconfigure_use::dynamic_reconfigure_clientConfig &dyc, uint32_t level)
{
    ROS_INFO("修改后的参数是 %d",dyc.int_param);
}

int main(int argc, char *argv[])
{
    setlocale(LC_ALL, "");
    ros::init(argc, argv, "dynamic_reconfigure_server_cpp");

    dynamic_reconfigure::Server<dynamic_reconfigure_use::dynamic_reconfigure_clientConfig> server;

    // server.setCallback(boost::bind(&cb, _1, _2));
    server.setCallback(cb);

    ros::spin();
    return 0;
}
