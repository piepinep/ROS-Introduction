#include "ros/ros.h"
#include "actionlib/server/simple_action_server.h"
#include "action_use/TwoIntSumAction.h"

/**
 * 1. 包含头文件
 * 2. 初始化ros节点
 * 3. 创建NodeHandle
 * 4. 创建action服务对象
 * 5. 请求处理
 *    5.1 解析提交的目标值
 *    5.2 产生反馈
 *    5.3 最终结果响应
 * 6.spin
 */

typedef actionlib::SimpleActionServer<action_use::TwoIntSumAction> Server;

void cb(const action_use::TwoIntSumGoalConstPtr &cptr, Server *s)
{
    // 解析提交的目标值
    int a = cptr->a, b = cptr->b;

    ros::Rate r(1);
    int result = 0;
    for (int i = a; i <= b; i++)
    {
        result += i;
        // 产生连续反馈
        action_use::TwoIntSumFeedback fb;
        fb.percent = 1. * (i - a + 1) / (b - a + 1);
        s->publishFeedback(fb);
        r.sleep();
    }

    // 最终结果响应
    action_use::TwoIntSumResult ret;
    ret.result = result;
    s->setSucceeded(ret);
}

int main(int argc, char *argv[])
{
    setlocale(LC_ALL, "");
    ros::init(argc, argv, "action_server_cpp");
    ros::NodeHandle nh;

    // 创建action对象
    // NodeHandle, topic, callback_function,auto_start
    Server server(nh, "actions", boost::bind(cb, _1, &server), false);
    server.start();

    ros::spin();
    return 0;
}
