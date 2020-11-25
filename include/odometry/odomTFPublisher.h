#pragma once
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <krabi_msgs/SetOdom.h>
#include <krabi_msgs/odom_light.h>
#include <krabilib/pose.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

class OdometryTFPublisher
{
public:
    OdometryTFPublisher(ros::NodeHandle& nh);

private:
    void updateLightOdom(nav_msgs::Odometry motors_odom);
    void resetOdometry(float x, float y, float theta);
    void publishTf(const geometry_msgs::Pose& pose, const string& frame_id, const string& child_frame_id);
   
    tf::TransformBroadcaster m_tf_broadcaster;
    ros::Subscriber m_odom_sub;
    ros::Publisher m_odom_pub;
    ros::NodeHandle& m_nh;
};