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
    m_odom_pub = nh.advertise<nav_msgs::Odometry>("odom", 10);
    m_odom_light_sub = nh.subscribe("odom_light",10,&OdometryLightNode::updateLightOdom,this);
}

void OdometryLightNode::updateLightOdom(krabi_msgs::odom_light motors_odom)
{
    nav_msgs::Odometry odomsg;
    odomsg.header.frame_id = "odom";
    odomsg.header.stamp = ros::Time::now();
    odomsg.child_frame_id = "base_link";

    odomsg.pose.pose = motors_odom.pose;
    for (unsigned int i = 0; i < 6 * 6; i++)
    {
        odomsg.pose.covariance[i] = 0;
    }

    odomsg.pose.covariance[0 * 6 + 0] = 0.1; // x
    odomsg.pose.covariance[1 * 6 + 1] = 0.1; // y
    odomsg.pose.covariance[5 * 6 + 5] = 0.2; // rz

    odomsg.pose.covariance[2 * 6 + 2] = 100; // z
    odomsg.pose.covariance[3 * 6 + 3] = 100; // rx
    odomsg.pose.covariance[4 * 6 + 4] = 100; // ry

    for (unsigned int i = 0; i < 6 * 6; i++)
    {
        odomsg.twist.covariance[i] = 0;
    }

    odomsg.twist.twist = motors_odom.speed;
    m_odom_pub.publish(odomsg);
    publishTf(odomsg.pose.pose);
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