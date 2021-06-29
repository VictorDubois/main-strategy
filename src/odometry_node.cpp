#include "odometry/odomTFPublisher.h"

#include "ros/ros.h"
#include <string>

using namespace std;

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "odometry_node");
    ros::NodeHandle nh;
    ros::Rate rate(10);
    string odom_type;
    nh.param<string>("odom_type", odom_type, "tf_pub");

    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);
    if (!odom_type.compare("tf_pub"))
    {
        OdometryTFPublisher node(nh, &tfBuffer, &tfListener);
        ros::spin();
    }
    else
    {
        ROS_ERROR("Incompatible odom_type");
        exit(1);
    }
    return 0;
}
