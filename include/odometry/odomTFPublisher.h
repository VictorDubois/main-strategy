#pragma once
#include <krabi_msgs/SetOdom.h>
#include <krabi_msgs/odom_light.h>
#include <krabilib/pose.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <tf2_ros/transform_listener.h>

class OdometryTFPublisher
{
public:
    OdometryTFPublisher(ros::NodeHandle& nh,
                        tf2_ros::Buffer* buffer,
                        tf2_ros::TransformListener* transform_listener);

    OdometryTFPublisher(const OdometryTFPublisher& a)
      : m_nh(a.m_nh)
    {
    } // user-defined copy ctor

private:
    void updateLightOdom(nav_msgs::Odometry motors_odom);
    void publishTf(const geometry_msgs::Pose& pose,
                   const std::string& frame_id,
                   const std::string& child_frame_id);

    tf::TransformBroadcaster m_tf_broadcaster;
    ros::Subscriber m_odom_sub;
    ros::Publisher m_odom_pub;
    ros::NodeHandle& m_nh;
    bool m_odom_reset;
    void resetOdometry(float x, float y, float theta);
    void resetOdometry();

    bool m_is_blue;

    void updateAruco(boost::shared_ptr<geometry_msgs::PoseStamped const> arucoPose, int id);
    std::map<int, ros::Subscriber> m_arucos_sub;
    std::array<geometry_msgs::PoseStamped, 10> m_arucos;
    tf2_ros::Buffer* m_tfBuffer;
    tf2_ros::TransformListener* m_tfListener;
};
