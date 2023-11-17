#include "ros/ros.h"
#include "turtlesim/Spawn.h"

int main(int argc, char *argv[])
{

    ros::init(argc, argv, "spawn_tutle", ros::InitOption::AnonymousName);
    ros::NodeHandle nh;

    ros::ServiceClient sc = nh.serviceClient<turtlesim::Spawn>("/spawn");
    turtlesim::Spawn sp;
    sp.request.name = "turtle2";
    sp.request.x = 3;
    sp.request.y = 3;
    sp.request.theta = 0;

    sc.waitForExistence();

    if (sc.call(sp))
    {
        ROS_INFO("create turtle success");
    }
    else
    {
        ROS_WARN("create turtle failed");
    }

    return 0;
}
