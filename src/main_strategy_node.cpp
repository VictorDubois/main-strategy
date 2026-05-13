#include "core.h"
#include <stdexcept>
#include <tf2/LinearMath/Quaternion.h>
#define MAX_ALLOWED_ANGULAR_SPEED 5.0f

#include <krabi_msgs/msg/motors_cmd.hpp>

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Core>();

    diagnostic_updater::Updater updater(node);
    updater.setHardwareID("MainStrat");
    updater.add("MainStrat", node.get(), &Core::produce_diagnostics);

    node->Setup();

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();

    rclcpp::shutdown();
    return 0;
}