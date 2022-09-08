#include "odometry/odomTFPublisher.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>


OdometryTFPublisher::OdometryTFPublisher(ros::NodeHandle& nh)
  : m_nh(nh)
  , m_odom_reset(false)
{
<<<<<<< HEAD
    m_init_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("initialpose", 5, true);
=======
    m_init_pose_pub = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("initialpose", 5, true);
>>>>>>> d5e3e0936af1d342f515ba2e9dff42b5528fc00e
    m_odom_sub = nh.subscribe("odom_light", 10, &OdometryTFPublisher::updateLightOdom, this);
    m_odom_lighter_sub = nh.subscribe("odom_lighter", 10, &OdometryTFPublisher::updateLighterOdom, this);
    m_odom_pub = nh.advertise<nav_msgs::Odometry>("odom", 10);
}

void OdometryTFPublisher::resetOdometry()
{
    float init_x, init_y, init_theta;
    m_nh.param<float>("/init_pose/x", init_x, 0);
    m_nh.param<float>("/init_pose/y", init_y, 0);
    m_nh.param<float>("/init_pose/theta", init_theta, 0);

    geometry_msgs::PoseWithCovarianceStamped init_pose_msg = geometry_msgs::PoseWithCovarianceStamped();
    init_pose_msg.pose.pose.position.x = init_x;
    init_pose_msg.pose.pose.position.y = init_y;
    init_pose_msg.pose.pose.position.z = 0;
    init_pose_msg.pose.covariance = {0, 0, 0,
                                     0, 0, 0,
                                     0, 0, 0,
                                     0, 0, 0, 0, 0, 0, 0, 0, 0,
                                     0, 0, 0, 0, 0, 0, 0, 0, 0, 
                                     0, 0, 0, 0, 0, 0, 0, 0, 0};
    init_pose_msg.pose.covariance[0] = 0.1f;// X covariance
    init_pose_msg.pose.covariance[7] = 0.1f;// Y covariance
    init_pose_msg.pose.covariance[35] = 0.2;// Rz covariance
    tf2::Quaternion quat_tf_orientation;
    quat_tf_orientation.setRPY(0, 0, init_theta);
    quat_tf_orientation.normalize();
    geometry_msgs::Quaternion quat_msg;
    quat_msg = tf2::toMsg(quat_tf_orientation);
    init_pose_msg.pose.pose.orientation = quat_msg;

    init_pose_msg.header.frame_id = "map";
    init_pose_msg.header.stamp = ros::Time::now();

    m_init_pose_pub.publish(init_pose_msg);

    resetOdometry(init_x, init_y, init_theta);
}

/**
 * @brief OdometryTFPublisher::publishOdom publish a full-fledged odom message. It takes too long for rosserial to publish it directly
 * @param odom_light_msg the partial message published by rosserial
 */
void OdometryTFPublisher::publishOdom(krabi_msgs::odom_light odom_light_msg)
{
    nav_msgs::Odometry odom_msg = nav_msgs::Odometry();
    odom_msg.pose.pose = odom_light_msg.pose;
    odom_msg.twist.twist = odom_light_msg.speed;
    odom_msg.header.stamp = odom_light_msg.header.stamp;
    odom_msg.header.frame_id = "krabby/odom";
    odom_msg.child_frame_id = "krabby/base_link";
    m_odom_pub.publish(odom_msg);
}

/**
 * @brief OdometryTFPublisher::publishOdom publish a full-fledged odom message. It takes too long for rosserial to publish it directly
 * @param odom_lighter_msg the partial message published by rosserial (even lighter than odom_light)
 */
void OdometryTFPublisher::publishOdom(krabi_msgs::odom_lighter odom_lighter_msg, geometry_msgs::Pose pose)
{
    nav_msgs::Odometry odom_msg = nav_msgs::Odometry();
    odom_msg.twist.twist.linear.x = odom_lighter_msg.speedVx;
    odom_msg.twist.twist.angular.z = odom_lighter_msg.speedWz;
    odom_msg.header.stamp = odom_lighter_msg.header.stamp;
    odom_msg.header.frame_id = "krabby/odom";
    odom_msg.child_frame_id = "krabby/base_link";
    m_odom_pub.publish(odom_msg);
}

void OdometryTFPublisher::updateLightOdom(krabi_msgs::odom_light odommsg)
{
    if (!m_odom_reset)
    {
        resetOdometry();
        return;
    }
    auto base_link_id = tf::resolve(ros::this_node::getNamespace(), "base_link");
    auto odom_id = tf::resolve(ros::this_node::getNamespace(), "odom");
    publishTf(odommsg.pose, odom_id, base_link_id);
    publishOdom(odommsg);
}

void OdometryTFPublisher::updateLighterOdom(krabi_msgs::odom_lighter odom_lighter_msg)
{
    if (!m_odom_reset)
    {
        resetOdometry();
        return;
    }
    auto base_link_id = tf::resolve(ros::this_node::getNamespace(), "base_link");
    auto odom_id = tf::resolve(ros::this_node::getNamespace(), "odom");
    geometry_msgs::Pose odom_pose;
    odom_pose.position.x = odom_lighter_msg.poseX;
    odom_pose.position.y = odom_lighter_msg.poseY;
    tf2::Quaternion odom_orientation_quat;
    odom_orientation_quat.setRPY(0, 0, odom_lighter_msg.angleRz);
    odom_pose.orientation = tf2::toMsg(odom_orientation_quat);
    publishTf(odom_pose, odom_id, base_link_id);
    publishOdom(odom_lighter_msg, odom_pose);
}

void OdometryTFPublisher::publishTf(const geometry_msgs::Pose& pose,
                                    const std::string& frame_id,
                                    const std::string& child_frame_id)
{
    // first, we'll publish the transform over tf
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = ros::Time::now();
    odom_trans.header.frame_id = frame_id;
    odom_trans.child_frame_id = child_frame_id;

    odom_trans.transform.translation.x = pose.position.x;
    odom_trans.transform.translation.y = pose.position.y;
    odom_trans.transform.translation.z = pose.position.z;
    odom_trans.transform.rotation = pose.orientation;

    // send the transform
    m_tf_broadcaster.sendTransform(odom_trans);
}

void OdometryTFPublisher::resetOdometry(float x, float y, float theta)
{
    m_odom_reset = true;
    ros::ServiceClient client = m_nh.serviceClient<krabi_msgs::SetOdom>("set_odom");
    krabi_msgs::SetOdom srv;
    srv.request.x = x;
    srv.request.y = y;
    srv.request.theta = theta;

    if (client.call(srv))
    {
        ROS_INFO("Odometry calibrated");
    }
    else
    {
        ROS_INFO("Failed to calibrate odometry");
    }
}
