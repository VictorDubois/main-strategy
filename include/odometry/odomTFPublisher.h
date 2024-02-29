#pragma once
#include <krabi_msgs/msg/odom_light.hpp>
#include <krabi_msgs/msg/odom_lighter.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <krabi_msgs/srv/set_odom.hpp>
#include <krabilib/pose.h>
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

class OdometryTFPublisher: public rclcpp::Node
{
public:
    OdometryTFPublisher();

private:
    void updateLightOdom(krabi_msgs::msg::OdomLight odommsg);
    void updateLighterOdom(krabi_msgs::msg::OdomLighter odommsg);
    void publishOdom(krabi_msgs::msg::OdomLight odommsg);
    void publishOdom(krabi_msgs::msg::OdomLighter odommsg, geometry_msgs::msg::Pose odompose);

    void publishTf(const geometry_msgs::msg::Pose& pose,
                   const std::string& frame_id,
                   const std::string& child_frame_id);

    std::shared_ptr<tf2_ros::TransformListener> m_tf_listener_{nullptr};
    std::unique_ptr<tf2_ros::Buffer> m_tf_buffer_;

    rclcpp::Subscription<krabi_msgs::msg::OdomLight>::SharedPtr m_odom_sub;
    rclcpp::Subscription<krabi_msgs::msg::OdomLighter>::SharedPtr m_odom_lighter_sub;

    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr m_init_pose_pub;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr m_odom_pub;

    bool m_odom_reset;
    void resetOdometry(float x, float y, float theta);
    void resetOdometry();
};
