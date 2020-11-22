#include "odometry/lightOdometry.h"

OdometryLightNode::OdometryLightNode(ros::NodeHandle& nh)
  : m_nh(nh)
{
    bool is_blue;
    nh.param<bool>("isBlue",is_blue, true);
    if(is_blue){
        resetOdometry(-1, 0, 0);
    }
    else{
        resetOdometry(1, 0, M_PI);
    }
    m_odom_sub = nh.subscribe("odom",10,&OdometryLightNode::updateLightOdom,this);
}

void OdometryLightNode::updateLightOdom(nav_msgs::Odometry odommsg)
{
    publishTf(odommsg.pose.pose);
}

void OdometryLightNode::publishTf(const geometry_msgs::Pose& pose)
{
    // first, we'll publish the transform over tf
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = ros::Time::now();
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";

    odom_trans.transform.translation.x = pose.position.x;
    odom_trans.transform.translation.y = pose.position.y;
    odom_trans.transform.translation.z = pose.position.z;
    odom_trans.transform.rotation = pose.orientation;

    // send the transform
    m_tf_broadcaster.sendTransform(odom_trans);
}

void OdometryLightNode::resetOdometry(float x, float y, float theta)
{

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