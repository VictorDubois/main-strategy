#include "odometry/fullOdometry.h"
#include "odometry/lightOdometry.h"

#include "ros/ros.h"
#include <string>

using namespace std;

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "odometry_node");
    ros::NodeHandle nh;
    string odom_type;
    nh.param<string>("odom_type", odom_type, "full");
    if (!odom_type.compare("full"))
    {
        auto node = fullOdometry(nh);
        node.run()
    }
    else if (!odom_type.compare("full"))
    {
        auto node = lightOdometry(nh);
        node.run()
    }
    else
    {
        ROS_ERROR("Incompatible odom_type");
    }
}