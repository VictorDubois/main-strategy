#include "core.h"
#include <stdexcept>
#include <tf2/LinearMath/Quaternion.h>
#define MAX_ALLOWED_ANGULAR_SPEED 5.0f

//#include "lidarStrat.h" @TODO fix this

#include <krabi_msgs/msg/motors_cmd.hpp>

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Core>();

    node->Setup();

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    while(rclcpp::ok)
    {
        executor.spin();
    }
    
    rclcpp::shutdown();
    return 0;
}

/*int main(int argc, char* argv[])
{
    
    ros::init(argc, argv, "mainStrat");
    ros::start();
    ros::Rate loop_rate(UPDATE_RATE);
    ros::NodeHandle nh;
    Core my_core(nh);
    my_core.Setup();

    while (ros::ok())
    {
        loop_rate.sleep();
        ros::spinOnce();
        my_core.Loop();
    }
}*/
