#include "ros/ros.h"
#include "actionlib/client/simple_action_client.h"
#include "action_use/TwoIntSumAction.h"

/**
 * 1. 包含头文件
 * 2. 初始化ros节点
 * 3. 创建NodeHandle
 * 4. 创建action客户端对象
 * 5. 发送处理
 *    5.1 建立连接
 *    5.2 处理反馈
 *    5.3 处理最终结果响应
 * 6.spin
 */

typedef actionlib::SimpleActionClient<action_use::TwoIntSumAction> Client;

/*
void Client::sendGoal(
    const act::AddintsGoal &goal,
    boost::function<void (
        const actionlib::SimpleClientGoalState &state,
        const action_use::TwoIntSumResultConstPtr &result)>done_cb,
    boost::function<void ()> active_cb,
    boost::function<void (const action_use::TwoIntSumFeedbackConstPtr &feedback)> feedback_cb
)
*/

void done_cb(const actionlib::SimpleClientGoalState &state,
             const action_use::TwoIntSumResultConstPtr &result)
{
    if (state.state_ == state.SUCCEEDED)
    {
        ROS_INFO("process success! result = %d", result->result);
    }
    else
    {
        ROS_INFO("error in process");
    }
}

void active_cb()
{
    // 建立连接时调用此函数
    ROS_INFO("successfully connect to server");
}

void feedback_cb(const action_use::TwoIntSumFeedbackConstPtr &feedback)
{
    ROS_INFO("percent = %.2f%%/100%%", feedback->percent * 100);
}

int main(int argc, char *argv[])
{
    setlocale(LC_ALL, "");
    ros::init(argc, argv, "action_client_cpp");
    ros::NodeHandle nh;

    Client client(nh, "actions");

    client.waitForServer();

    action_use::TwoIntSumGoal goal;
    goal.a = 1;
    goal.b = 10;
    client.sendGoal(goal, done_cb, active_cb, feedback_cb);
    ros::spin();
    return 0;
}
