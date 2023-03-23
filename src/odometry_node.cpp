#include "odometry/odomTFPublisher.h"

#include "ros/ros.h"
#include <string>

using namespace std;

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "odometry_node");
    ros::NodeHandle nh;
    ros::Rate rate(200);
    string odom_type;
    nh.param<string>("odom_type", odom_type, "tf_pub");
    if (!odom_type.compare("tf_pub"))
    {
        auto node = OdometryTFPublisher(nh);
        while(ros::ok())
        {
            ros::spinOnce();
            rate.sleep();
        }
    }
    else
    {
        ROS_ERROR("Incompatible odom_type");
        exit(1);
    }
    return 0;
}
