#include "ros/ros.h"
#include "nodelet/nodelet.h"
#include "pluginlib/class_list_macros.h"
#include "std_msgs/Float64.h"

/**
 * 1. 确定需要的变量,订阅对象,发布对象,存储参数的变量
 * 2. 获取NodeHandle
 * 3. 通过NodeHandle创建订阅对象和发布对象
 * 4. 回调函数需要处理数据,并通过发布对象发布数据
*/

namespace my_nodelet
{
    class MyPlus: public nodelet::Nodelet
    {
    public:
        MyPlus():value(0){}

        void onInit()
        {
            // 一般获取私有的NodeHandle,因为在nodelet节点下的数据
            ros::NodeHandle nh = getPrivateNodeHandle();

            nh.getParam("value", value);

            pub = nh.advertise<std_msgs::Float64>("out", 10);
            // 方法1:传入this指针
            sub = nh.subscribe("in", 100, &MyPlus::cb, this);
            // 方法2:使用boost::bind传参
            // nh.subscribe<std_msgs::Float64>("in", 100, boost::bind(&MyPlus::cb, this, _1));
        }

        void cb(const std_msgs::Float64::ConstPtr &p)
        {
            double a = p->data + value;
            std_msgs::Float64 out;
            out.data = a;
            pub.publish(out);
        }

    private:
        ros::Publisher pub;
        ros::Subscriber sub;
        double value;
    };
};

PLUGINLIB_EXPORT_CLASS(my_nodelet::MyPlus, nodelet::Nodelet)