#pragma once
#include "ros/ros.h"
#include <tf/transform_broadcaster.h>
#include <utility>
#include <vector>

#include "constants.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Vector3.h"
#include "helpers_broker.h"
#include "krabilib/positionPlusAngle.h"
#include "nav_msgs/Odometry.h"
#include <krabi_msgs/motors.h>
#include <krabi_msgs/odom_light.h>
#include <std_msgs/Bool.h>

class OdometryNode
{

public:
    OdometryNode();
    void run();

private:
    ros::Subscriber m_encoders_sub;
    tf::TransformBroadcaster m_odom_broadcaster;
    Pose m_start_pose;
    Pose m_current_pose;
    ros::Time m_last_speed_update_time;

    void updateOdometry(nav_msgs::Odometry odometry);
    void Core::updateCurrentPose(krabi_msgs::encoders encoders);
    geometry_msgs::Pose updateCurrentPose(int32_t encoder1, int32_t encoder2);
    void sendOdometry();
    void updateCurrentSpeed()
};
