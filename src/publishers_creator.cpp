#include "core.h"

void Core::create_publishers()
{
    m_motors_cmd_slash_pub = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 5);
    m_motors_cmd_pub = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 5);
    m_motors_enable_pub = this->create_publisher<std_msgs::msg::Bool>("enable_motor", 5);
    m_motors_parameters_pub
      = this->create_publisher<krabi_msgs::msg::MotorsParameters>("motors_parameters", 5);
    m_motors_pwm_pub = this->create_publisher<krabi_msgs::msg::MotorsCmd>("motors_cmd", 5);
    m_target_orientation_pub
      = this->create_publisher<geometry_msgs::msg::PoseStamped>("target_orientation", 5);

    m_motion_debug_pub
      = this->create_publisher<krabi_msgs::msg::MotionDebug>("motion_debug", 5);
    m_chrono_pub = this->create_publisher<builtin_interfaces::msg::Duration>("/remaining_time", 5);

    // DNF debug arrays for Foxglove visualisation (Plot panel on each topic)
    m_dnf_goal_pub
      = this->create_publisher<std_msgs::msg::Float32MultiArray>("dnf_goal_output", 1);
    m_dnf_lidar_pub
      = this->create_publisher<std_msgs::msg::Float32MultiArray>("dnf_lidar_output", 1);
    m_dnf_landscape_pub
      = this->create_publisher<std_msgs::msg::Float32MultiArray>("dnf_angular_landscape", 1);
}