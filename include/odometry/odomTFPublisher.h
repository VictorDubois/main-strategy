#pragma once
#include "rclcpp/node.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <krabi_msgs/msg/odom_light.hpp>
#include <krabi_msgs/msg/odom_lighter.hpp>
#include <krabi_msgs/srv/set_odom.hpp>
#include <krabilib/pose.h>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/transform_broadcaster.h>

class OdometryTFPublisher : public rclcpp::Node
{
public:
    OdometryTFPublisher();

private:
    void updateLightOdom(const krabi_msgs::msg::OdomLight odommsg);
    void updateLighterOdom(const krabi_msgs::msg::OdomLighter odommsg);
    void updateOdom(const nav_msgs::msg::Odometry odommsg);

    void publishOdom(const krabi_msgs::msg::OdomLight& odommsg);
    void publishOdom(const krabi_msgs::msg::OdomLighter& odommsg,
                     const geometry_msgs::msg::Pose& odompose);
    void publishOdom(const nav_msgs::msg::Odometry& odommsg);

    void publishTf(const geometry_msgs::msg::Pose& pose,
                   const std::string& frame_id,
                   const std::string& child_frame_id);

    std::shared_ptr<tf2_ros::TransformListener> m_tf_listener_{ nullptr };
    std::unique_ptr<tf2_ros::Buffer> m_tf_buffer_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> m_tf_broadcaster;

    rclcpp::Subscription<krabi_msgs::msg::OdomLight>::SharedPtr m_odom_sub;
    rclcpp::Subscription<krabi_msgs::msg::OdomLighter>::SharedPtr m_odom_lighter_sub;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr m_odom_odom_sub;

    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr m_init_pose_pub;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr m_odom_pub;

    bool m_odom_reset;
    void publishInitialPose();
    void resetOdometry(float x, float y, float theta);
    void resetOdometry();

    nav_msgs::msg::Odometry odom_msg;
};
