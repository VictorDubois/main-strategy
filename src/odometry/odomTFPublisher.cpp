#include "odometry/odomTFPublisher.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

OdometryTFPublisher::OdometryTFPublisher(ros::NodeHandle& nh)
  : m_nh(nh)
  , m_odom_reset(false)
{
    m_odom_sub = nh.subscribe("odom", 10, &OdometryTFPublisher::updateLightOdom, this);
}

void OdometryTFPublisher::resetOdometry()
{
    float init_x, init_y, init_theta;
    m_nh.param<float>("init_pose/x", init_x, 0);
    m_nh.param<float>("init_pose/y", init_y, 0);
    m_nh.param<float>("init_pose/theta", init_theta, 0);

    resetOdometry(init_x, init_y, init_theta);
}

void OdometryTFPublisher::updateLightOdom(nav_msgs::Odometry odommsg)
{
    if (!m_odom_reset)
    {
        resetOdometry();
        return;
    }
    auto base_link_id = tf::resolve(ros::this_node::getNamespace(), "base_link");
    auto odom_id = tf::resolve(ros::this_node::getNamespace(), "odom");
    publishTf(odommsg.pose.pose, odom_id, base_link_id);
}

void OdometryTFPublisher::publishTf(const geometry_msgs::Pose& pose,
                                    const std::string& frame_id,
                                    const std::string& child_frame_id)
{
    float init_x, init_y, init_theta;
    m_nh.param<float>("init_pose/x", init_x, 0);
    m_nh.param<float>("init_pose/y", init_y, 0);
    m_nh.param<float>("init_pose/theta", init_theta, 0);

    // first, we'll publish the transform over tf
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = ros::Time::now();
    odom_trans.header.frame_id = frame_id;
    odom_trans.child_frame_id = child_frame_id;

    odom_trans.transform.translation.x = pose.position.x + init_x;
    odom_trans.transform.translation.y = pose.position.y + init_y;
    odom_trans.transform.translation.z = pose.position.z;

    tf2::Quaternion myQuaternion;
    tf2::Quaternion quaternion_from_msg;
    tf2::fromMsg(pose.orientation, quaternion_from_msg);

    myQuaternion.setRPY(0, 0, init_theta);

    myQuaternion *= quaternion_from_msg;

    geometry_msgs::Quaternion quat_msg;

    tf2::convert(quat_msg, myQuaternion);

    odom_trans.transform.rotation = quat_msg; // pose.orientation;

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
