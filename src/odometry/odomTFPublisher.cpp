#include "odometry/odomTFPublisher.h"

#include <ros/ros.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>
OdometryTFPublisher::OdometryTFPublisher(ros::NodeHandle& nh,
                                         tf2_ros::Buffer* a_tfBuffer,
                                         tf2_ros::TransformListener* a_tfListener)
  : m_nh(nh)
  , m_odom_reset(false)
  , m_tfBuffer(a_tfBuffer)
  , m_tfListener(a_tfListener)
{
    m_odom_sub = nh.subscribe("odom", 10, &OdometryTFPublisher::updateLightOdom, this);

    nh.param<bool>("isBlue", m_is_blue, true);

    if (!m_is_blue)
    {
        m_arucos_sub[6] = nh.subscribe<geometry_msgs::PoseStamped>(
          "/pose_robots/6", 5, boost::bind(&OdometryTFPublisher::updateAruco, this, _1, 6));
        m_arucos_sub[7] = nh.subscribe<geometry_msgs::PoseStamped>(
          "/pose_robots/7", 5, boost::bind(&OdometryTFPublisher::updateAruco, this, _1, 7));
        m_arucos_sub[8] = nh.subscribe<geometry_msgs::PoseStamped>(
          "/pose_robots/8", 5, boost::bind(&OdometryTFPublisher::updateAruco, this, _1, 8));
        m_arucos_sub[9] = nh.subscribe<geometry_msgs::PoseStamped>(
          "/pose_robots/9", 5, boost::bind(&OdometryTFPublisher::updateAruco, this, _1, 9));
        m_arucos_sub[10] = nh.subscribe<geometry_msgs::PoseStamped>(
          "/pose_robots/10", 5, boost::bind(&OdometryTFPublisher::updateAruco, this, _1, 10));
    }
    else
    {
        m_arucos_sub[1] = nh.subscribe<geometry_msgs::PoseStamped>(
          "/pose_robots/1", 5, boost::bind(&OdometryTFPublisher::updateAruco, this, _1, 1));
        m_arucos_sub[2] = nh.subscribe<geometry_msgs::PoseStamped>(
          "/pose_robots/2", 5, boost::bind(&OdometryTFPublisher::updateAruco, this, _1, 2));
        m_arucos_sub[3] = nh.subscribe<geometry_msgs::PoseStamped>(
          "/pose_robots/3", 5, boost::bind(&OdometryTFPublisher::updateAruco, this, _1, 3));
        m_arucos_sub[4] = nh.subscribe<geometry_msgs::PoseStamped>(
          "/pose_robots/4", 5, boost::bind(&OdometryTFPublisher::updateAruco, this, _1, 4));
        m_arucos_sub[5] = nh.subscribe<geometry_msgs::PoseStamped>(
          "/pose_robots/5", 5, boost::bind(&OdometryTFPublisher::updateAruco, this, _1, 5));
    }
}

void OdometryTFPublisher::updateAruco(boost::shared_ptr<geometry_msgs::PoseStamped const> arucoPose,
                                      int id)
{
    m_arucos[id] = *arucoPose;

    publishTf(arucoPose->pose, "/aruco", "/aruco_raw_pose");
    auto base_link_id = tf::resolve(ros::this_node::getNamespace(), "base_link");
    auto deltaOdom = m_tfBuffer->lookupTransform(base_link_id,
                                                 arucoPose->header.stamp,
                                                 base_link_id,
                                                 ros::Time::now(),
                                                 "map",
                                                 ros::Duration(1.0));

    ROS_INFO_STREAM("ego_aruco_received. Movement since: x = "
                    << deltaOdom.transform.translation.x
                    << ", y = " << deltaOdom.transform.translation.y
                    << ", QuatW = " << deltaOdom.transform.rotation.w
                    << ", QuatX = " << deltaOdom.transform.rotation.x
                    << ", QuatY = " << deltaOdom.transform.rotation.y
                    << ", QuatZ = " << deltaOdom.transform.rotation.z << std::endl);
    auto corrected_pose = arucoPose->pose;
    tf2::Quaternion quat_tf_odom;
    tf2::fromMsg(deltaOdom.transform.rotation, quat_tf_odom);
    double roll, pitch, yaw_odom;
    tf2::Matrix3x3(quat_tf_odom).getRPY(roll, pitch, yaw_odom);

    corrected_pose.position.x = arucoPose->pose.position.x
                                + deltaOdom.transform.translation.x * cos(yaw_odom)
                                - deltaOdom.transform.translation.y * sin(yaw_odom);
    corrected_pose.position.y = arucoPose->pose.position.y
                                + deltaOdom.transform.translation.x * sin(yaw_odom)
                                + deltaOdom.transform.translation.y * cos(yaw_odom);

    tf2::Quaternion quat_tf_aruco;
    tf2::fromMsg(arucoPose->pose.orientation, quat_tf_aruco);

    tf2::Quaternion quat_tf_corrected;

    quat_tf_corrected = quat_tf_odom * quat_tf_aruco;
    quat_tf_corrected.normalize();
    corrected_pose.orientation = tf2::toMsg(quat_tf_corrected);

    ROS_INFO_STREAM("corrected position: x = " << corrected_pose.position.x << ", y = "
                                               << corrected_pose.position.y << std::endl);

    publishTf(corrected_pose, "/aruco", "/corrected_odom");
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
    ROS_INFO_STREAM("coucou, updating odom");
    return;
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
