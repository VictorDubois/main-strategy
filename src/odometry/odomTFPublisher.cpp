#include "odometry/odomTFPublisher.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>


OdometryTFPublisher::OdometryTFPublisher()
: Node("odometry_node")
//  : m_nh(nh)
  , m_odom_reset(false)
{
    m_tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    m_tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*m_tf_buffer_);
    m_tf_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    m_init_pose_pub = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("initialpose", 5);
    m_odom_pub = this->create_publisher<nav_msgs::msg::Odometry>("odom", 5);

    // If problem with spin called multiple times
    /*auto my_callback_group = create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    rclcpp::SubscriptionOptions l_sub_options;
    l_sub_options.callback_group = my_callback_group;*/

    m_odom_sub = this->create_subscription<krabi_msgs::msg::OdomLight>("odom_light", 5, std::bind(&OdometryTFPublisher::updateLightOdom, this, std::placeholders::_1));//, l_sub_options);
    m_odom_lighter_sub = this->create_subscription<krabi_msgs::msg::OdomLighter>("odom_lighter", 5, std::bind(&OdometryTFPublisher::updateLighterOdom, this, std::placeholders::_1));//, l_sub_options);

    this->declare_parameter("/init_pose/x", 0.f);
    this->declare_parameter("/init_pose/y", 0.f);
    this->declare_parameter("/init_pose/theta", 0.f);
    this->declare_parameter("/publish_tf_odom", true);
}

void OdometryTFPublisher::resetOdometry()
{
    float init_x, init_y, init_theta;
    this->get_parameter("/init_pose/x", init_x);
    this->get_parameter("/init_pose/y", init_y);
    this->get_parameter("/init_pose/theta", init_theta);

    geometry_msgs::msg::PoseWithCovarianceStamped init_pose_msg
      = geometry_msgs::msg::PoseWithCovarianceStamped();
    init_pose_msg.pose.pose.position.x = init_x;
    init_pose_msg.pose.pose.position.y = init_y;
    init_pose_msg.pose.pose.position.z = 0;
    init_pose_msg.pose.covariance = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                                      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
    init_pose_msg.pose.covariance[0] = 0.1f; // X covariance
    init_pose_msg.pose.covariance[7] = 0.1f; // Y covariance
    init_pose_msg.pose.covariance[35] = 0.2; // Rz covariance
    tf2::Quaternion quat_tf_orientation;
    quat_tf_orientation.setRPY(0, 0, init_theta);
    quat_tf_orientation.normalize();
    geometry_msgs::msg::Quaternion quat_msg;
    quat_msg = tf2::toMsg(quat_tf_orientation);
    init_pose_msg.pose.pose.orientation = quat_msg;

    init_pose_msg.header.frame_id = "map";
    init_pose_msg.header.stamp = this->now();

    m_init_pose_pub->publish(init_pose_msg);

    resetOdometry(0, 0, 0);//init_x, init_y, init_theta);
}

/**
 * @brief OdometryTFPublisher::publishOdom publish a full-fledged odom message. It takes too long
 * for rosserial to publish it directly
 * @param odom_light_msg the partial message published by rosserial
 */
void OdometryTFPublisher::publishOdom(krabi_msgs::msg::OdomLight odom_light_msg)
{
    nav_msgs::msg::Odometry odom_msg = nav_msgs::msg::Odometry();
    odom_msg.header.stamp = odom_light_msg.header.stamp;
    if(odom_light_msg.header.stamp.sec == 0 && odom_light_msg.header.stamp.nanosec == 0)
    {
        odom_msg.header.stamp = this->now();
    }
    odom_msg.pose.pose = odom_light_msg.pose;
    odom_msg.twist.twist = odom_light_msg.speed;
    odom_msg.header.frame_id = "odom";
    odom_msg.child_frame_id = "base_link";

    odom_msg.pose.covariance = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                                 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
    odom_msg.pose.covariance[0] = 0.03f; // X covariance
    odom_msg.pose.covariance[7] = 0.03f; // Y covariance
    odom_msg.pose.covariance[35] = 0.1f; // Rz covariance
    m_odom_pub->publish(odom_msg);
}

/**
 * @brief OdometryTFPublisher::publishOdom publish a full-fledged odom message. It takes too long
 * for rosserial to publish it directly
 * @param odom_lighter_msg the partial message published by rosserial (even lighter than odom_light)
 */
void OdometryTFPublisher::publishOdom(krabi_msgs::msg::OdomLighter odom_lighter_msg,
                                      geometry_msgs::msg::Pose pose)
{
    nav_msgs::msg::Odometry odom_msg = nav_msgs::msg::Odometry();
    odom_msg.header.stamp = odom_lighter_msg.header.stamp;
    if(odom_lighter_msg.header.stamp.sec == 0 && odom_lighter_msg.header.stamp.nanosec == 0)
    {
        odom_msg.header.stamp = this->now();
    }
    
    odom_msg.pose.pose = pose;
    odom_msg.twist.twist.linear.x = odom_lighter_msg.speed_vx;
    odom_msg.twist.twist.angular.z = odom_lighter_msg.speed_wz;
    odom_msg.header.frame_id = "odom";
    odom_msg.child_frame_id = "base_link";

    odom_msg.pose.covariance = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                                 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
    odom_msg.pose.covariance[0] = 0.03f; // X covariance
    odom_msg.pose.covariance[7] = 0.03f; // Y covariance
    odom_msg.pose.covariance[35] = 0.1f; // Rz covariance

    m_odom_pub->publish(odom_msg);
}

void OdometryTFPublisher::updateLightOdom(krabi_msgs::msg::OdomLight odommsg)
{
    if (!m_odom_reset)
    {
        resetOdometry();
        return;
    }
    auto base_link_id = "base_link"; // tf::resolve(ros::this_node::getNamespace(), "base_link");
    auto odom_id = "odom"; // tf::resolve(ros::this_node::getNamespace(), "odom");
    publishTf(odommsg.pose, odom_id, base_link_id);
    publishOdom(odommsg);
}

void OdometryTFPublisher::updateLighterOdom(krabi_msgs::msg::OdomLighter odom_lighter_msg)
{
    if (!m_odom_reset)
    {
        resetOdometry();
        return;
    }
    auto base_link_id = "base_link"; // tf::resolve(ros::this_node::getNamespace(), "base_link");
    auto odom_id = "odom"; // tf::resolve(ros::this_node::getNamespace(), "odom");
    geometry_msgs::msg::Pose odom_pose;
    odom_pose.position.x = odom_lighter_msg.pose_x;
    odom_pose.position.y = odom_lighter_msg.pose_y;
    odom_pose.position.z = 0;
    tf2::Quaternion odom_orientation_quat;
    odom_orientation_quat.setRPY(0, 0, odom_lighter_msg.angle_rz);
    odom_pose.orientation = tf2::toMsg(odom_orientation_quat);
    publishTf(odom_pose, odom_id, base_link_id);
    publishOdom(odom_lighter_msg, odom_pose);
}

void OdometryTFPublisher::publishTf(const geometry_msgs::msg::Pose& pose,
                                    const std::string& frame_id,
                                    const std::string& child_frame_id)
{
    bool publish_tf_odom = false;
    this->get_parameter("/publish_tf_odom", publish_tf_odom);
    if (!publish_tf_odom)
    {
        return;
    }

    // first, we'll publish the transform over tf
    geometry_msgs::msg::TransformStamped odom_trans;
    odom_trans.header.stamp = this->now();
    odom_trans.header.frame_id = frame_id;
    odom_trans.child_frame_id = child_frame_id;

    odom_trans.transform.translation.x = pose.position.x;
    odom_trans.transform.translation.y = pose.position.y;
    odom_trans.transform.translation.z = pose.position.z;
    odom_trans.transform.rotation = pose.orientation;

    // send the transform
    m_tf_broadcaster->sendTransform(odom_trans);
}

void OdometryTFPublisher::resetOdometry(float x, float y, float theta)
{
    m_odom_reset = true;
    m_odom_reset = true;

    rclcpp::Client<krabi_msgs::srv::SetOdom>::SharedPtr client = this->create_client<krabi_msgs::srv::SetOdom>("set_odom");

    auto request = std::make_shared<krabi_msgs::srv::SetOdom::Request>();

    request->x = x;
    request->y = y;
    request->theta = theta;
  

    auto result = client->async_send_request(request);
    // Wait for the result.
    /*if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result) ==
        rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Odometry calibrated");
    } else {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to calibrate odometry");
    }*/
}
