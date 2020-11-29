#include "odometry/odomTFPublisher.h"

OdometryTFPublisher::OdometryTFPublisher(ros::NodeHandle& nh)
  : m_nh(nh)
{
    odom_reset = false;
    odom_connected = false;

    resetOdometry();

    auto odom_id = tf::resolve(ros::this_node::getNamespace(), "odom");
    m_odom_sub = nh.subscribe(odom_id, 10, &OdometryTFPublisher::updateLightOdom, this);
    m_odom_sub_2 = nh.subscribe("odom", 10, &OdometryTFPublisher::updateLightOdom, this);
}

void OdometryTFPublisher::resetOdometry()
{
    float init_x, init_y, init_theta;
    m_nh.param<float>("init_pose/x", init_x, 0);
    m_nh.param<float>("init_pose/y", init_x, 0);
    m_nh.param<float>("init_pose/theta", init_x, 0);

    resetOdometry(init_x, init_y, init_theta);
}

void OdometryTFPublisher::updateLightOdom(nav_msgs::Odometry odommsg)
{
    auto base_link_id = tf::resolve(ros::this_node::getNamespace(), "base_link");
    auto odom_id = tf::resolve(ros::this_node::getNamespace(), "odom");
    publishTf(odommsg.pose.pose, odom_id, base_link_id);
    odom_connected = true;
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
    if (!odom_connected || odom_reset) {
        return;
    }
    odom_reset = true;
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
        ROS_ERROR("Failed to calibrate odometry");
    }
}
