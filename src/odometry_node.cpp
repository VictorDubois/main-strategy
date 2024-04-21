#include "odometry/odomTFPublisher.h"

#include "rclcpp/rclcpp.hpp"
#include <string>

using namespace std;


int main(int argc, char* argv[])
{

    rclcpp::init(argc, argv);
    auto node = std::make_shared<OdometryTFPublisher>();

    node->declare_parameter("odom_type", "tf_pub");
    string odom_type = node->get_parameter("odom_type").as_string();

    if (odom_type.compare("tf_pub"))
    {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Incompatible odom_type");
        exit(1);
        return 1;
    }

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    
    rclcpp::shutdown();
    return 0;
}
