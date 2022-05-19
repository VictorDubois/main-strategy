#include "core.h"
#include "ros/ros.h"
#include <std_msgs/Duration.h>
#include <stdexcept>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#define MAX_ALLOWED_ANGULAR_SPEED 0.2f

#include "lidarStrat.h"

#include <krabi_msgs/motors_cmd.h>

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "mainStrat");
    ros::start();
    ros::Rate loop_rate(UPDATE_RATE);
    ros::NodeHandle nh;
    Core my_core(nh);
    my_core.Setup();

    while (!my_core.isOver() && ros::ok())
    {
        loop_rate.sleep();
        ros::spinOnce();
    }
}
