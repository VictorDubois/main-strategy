#include "core.h"

void Core::create_subscribers()
{

    std::function<void(std::shared_ptr<geometry_msgs::msg::PoseStamped>)> l_lidar_sub_func
      = std::bind(&Core::updateLidar, this, std::placeholders::_1, true);
    std::function<void(std::shared_ptr<geometry_msgs::msg::PoseStamped>)> l_lidar_behind_sub_func
      = std::bind(&Core::updateLidar, this, std::placeholders::_1, false);

    m_lidar_sub = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "obstacle_pose_stamped", 5, l_lidar_sub_func); //, l_sub_options);
    m_lidar_behind_sub = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "obstacle_behind_pose_stamped", 5, l_lidar_behind_sub_func); //, l_sub_options);
    m_tirette_sub = this->create_subscription<std_msgs::msg::Bool>(
      "tirette",
      5,
      std::bind(&Core::updateTirette, this, std::placeholders::_1)); //, l_sub_options);
    m_odometry_sub = this->create_subscription<nav_msgs::msg::Odometry>(
      "odom", 5, std::bind(&Core::updateOdom, this, std::placeholders::_1)); //, l_sub_options);
    m_odometry_slash_sub = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom", 5, std::bind(&Core::updateOdom, this, std::placeholders::_1)); //, l_sub_options);
    m_strat_movement_sub = this->create_subscription<krabi_msgs::msg::StratMovement>(
      "strat_movement",
      5,
      std::bind(&Core::updateStratMovement, this, std::placeholders::_1)); //, l_sub_options);
}