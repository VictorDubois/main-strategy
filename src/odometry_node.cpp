#include "odometry/lightOdometry.h"

#include "ros/ros.h"
#include <string>

using namespace std;

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "odometry_node");
    ros::NodeHandle nh;
    string odom_type;
    nh.param<string>("odom_type", odom_type, "light");
    if (!odom_type.compare("light"))
    {
        auto node = OdometryLightNode(nh);
        ros::spin();
    }
    else
    {
        ROS_ERROR("Incompatible odom_type");
        exit(1);
    }
    return 0;
}