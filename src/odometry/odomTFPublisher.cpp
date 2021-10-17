#include "odometry/odomTFPublisher.h"

OdometryTFPublisher::OdometryTFPublisher(ros::NodeHandle& nh)
  : m_nh(nh),m_odom_reset(false)
{
    m_odom_sub = nh.subscribe("odom_light", 10, &OdometryTFPublisher::updateLightOdom, this);
    m_odom_pub = nh.advertise<nav_msgs::Odometry>("odom", 10);
}

void OdometryTFPublisher::resetOdometry()
{
    float init_x, init_y, init_theta;
    m_nh.param<float>("init_pose/x", init_x, 0);
    m_nh.param<float>("init_pose/y", init_y, 0);
    m_nh.param<float>("init_pose/theta", init_theta, 0);

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
    m_odom_pub.publish(odom_msg);
}

void OdometryTFPublisher::updateLightOdom(krabi_msgs::odom_light odommsg)
{
    if(!m_odom_reset){
        resetOdometry();
        return;
    }
    auto base_link_id = tf::resolve(ros::this_node::getNamespace(), "base_link");
    auto odom_id = tf::resolve(ros::this_node::getNamespace(), "odom");
    publishTf(odommsg.pose, odom_id, base_link_id);
    publishOdom(odommsg);
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
