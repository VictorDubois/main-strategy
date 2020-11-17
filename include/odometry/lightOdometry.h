#pragma once
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <krabi_msgs/SetOdom.h>
#include <krabi_msgs/odom_light.h>
#include <krabilib/pose.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

class OdometryLightNode
{
public:
    OdometryLightNode(ros::NodeHandle& nh);

private:
    void updateLightOdom(krabi_msgs::odom_light motors_odom);
    void resetOdometry(float x, float y, float theta);
    void publishTf(const geometry_msgs::Pose& pose);
   
    tf::TransformBroadcaster m_tf_broadcaster;
    ros::Subscriber m_odom_light_sub;
    ros::Publisher m_odom_pub;
    ros::NodeHandle& m_nh;
};