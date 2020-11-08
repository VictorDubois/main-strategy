#include "core.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

void Core::updateOdometry(nav_msgs::Odometry odometry)
{
    if (!m_encoders_initialized)
    {
        std::cout << "initializing encoders" << std::endl;
        m_starting_position.setX(m_starting_position.getX() - odometry.pose.pose.position.x * 1000);
        m_starting_position.setY(m_starting_position.getY() - odometry.pose.pose.position.y * 1000);
        m_encoders_initialized = true;
    }
    std::cout << "updating Odometry" << std::endl;
    m_X = odometry.pose.pose.position.x + m_starting_position.getX() / 1000.f;
    m_Y = odometry.pose.pose.position.y + m_starting_position.getY() / 1000.f;
    m_current_position = Position(m_X * 1000, m_Y * 1000, false);

    double siny_cosp = 2
                       * (odometry.pose.pose.orientation.w * odometry.pose.pose.orientation.z
                          + odometry.pose.pose.orientation.x * odometry.pose.pose.orientation.y);
    double cosy_cosp
      = 1
        - 2
            * (odometry.pose.pose.orientation.y * odometry.pose.pose.orientation.y
               + odometry.pose.pose.orientation.z * odometry.pose.pose.orientation.z);
    m_current_theta = std::atan2(siny_cosp, cosy_cosp) * 180.f / M_PI;

    geometry_msgs::Pose currentPose = odometry.pose.pose;
    currentPose.position.x = m_X;
    currentPose.position.y = m_Y;

    tf2::Quaternion orientation_quat;
    orientation_quat.setRPY(0, 0, m_current_theta * M_PI / 180);
    currentPose.orientation = tf2::toMsg(orientation_quat);

    m_current_pose_pub.publish(currentPose);
    m_current_linear_speed = odometry.twist.twist.linear.x;
    m_current_angular_speed = odometry.twist.twist.angular.z;

    m_distance_to_goal = (sqrt((m_X - m_goal_position.getPosition().getX() / 1000.f)
                               * (m_X - m_goal_position.getPosition().getX() / 1000.f)
                             + (m_Y - m_goal_position.getPosition().getY() / 1000.f)
                                 * (m_Y - m_goal_position.getPosition().getY() / 1000.f)));
    std::cout << "distance to goal = " << m_distance_to_goal << std::endl;
}


void Core::updateCurrentPose(krabi_msgs::encoders encoders)
{
    return;

    m_encoder1 = encoders.encoder_left;
    m_encoder2 = encoders.encoder_right;

    if (!m_encoders_initialized)
    {
        std::cout << "initializing encoders" << std::endl;
        m_starting_encoder1 = m_encoder1;
        m_starting_encoder2 = m_encoder2;
        m_encoders_initialized = true;
    }

    m_encoder1 -= m_starting_encoder1;
    m_encoder2 -= m_starting_encoder2;

    geometry_msgs::Pose currentPose = updateCurrentPose(m_encoder1, m_encoder2);

    m_current_pose_pub.publish(currentPose);
    updateCurrentSpeed();
    sendOdometry(currentPose);

    m_distance_to_goal = (sqrt((m_X - m_goal_position.getPosition().getX() / 1000.f)
                               * (m_X - m_goal_position.getPosition().getX() / 1000.f)
                             + (m_Y - m_goal_position.getPosition().getY() / 1000.f)
                                 * (m_Y - m_goal_position.getPosition().getY() / 1000.f)));
    std::cout << "distance to goal = " << m_distance_to_goal << std::endl;
}

void Core::updateLightOdom(krabi_msgs::odom_light motors_odom)
{
    // raw values send by the motor board, that are not in the correct frame
    float temp_X = motors_odom.pose.position.x;
    float temp_Y = motors_odom.pose.position.y;
    float temp_theta = motors_odom.pose.orientation.z * 180 / M_PI;

    if (!m_encoders_initialized)
    {
        std::cout << "initializing encoders" << std::endl;
        if (!m_is_blue)
        {
            m_starting_X += temp_X;
            m_starting_Y += temp_Y;
        }
        else
        {
            m_starting_X -= temp_X;
            m_starting_Y -= temp_Y;
        }
        std::cout << "x = " << temp_X << ", Y = " << temp_Y << ", theta = " << temp_theta
                  << "theta_zero = " << m_theta_zero << std::endl;
        // theta_zero -= temp_theta;
        m_last_position.setX(m_starting_X);
        m_last_position.setX(m_starting_Y);
        m_encoders_initialized = true;
    }
    if (!m_is_blue)
    {
        // std::cout << ">>>>>>>>> m_is_blue, temp_X = " << temp_X << ", starting_X = " << starting_X
        // << ", X = " << X << std::endl;
        m_X = -temp_X + m_starting_X;
        m_Y = -temp_Y + m_starting_Y;
    }
    else
    {
        m_X = temp_X + m_starting_X;
        m_Y = temp_Y + m_starting_Y;
    }

    m_current_position = Position(m_X * 1000, m_Y * 1000, false);

    m_current_theta = temp_theta + m_theta_zero;

    geometry_msgs::Pose currentPose = motors_odom.pose;
    currentPose.position.x = m_X;
    currentPose.position.y = m_Y;

    tf2::Quaternion orientation_quat;
    orientation_quat.setRPY(0, 0, m_current_theta * M_PI / 180);
    currentPose.orientation = tf2::toMsg(orientation_quat);

    m_current_pose_pub.publish(currentPose);
    updateCurrentSpeed();
    // m_current_linear_speed = motors_odom.speed.linear.x;// not yet working: returns 0
    // m_current_angular_speed = motors_odom.speed.angular.z;// not yet working: returns 0
    sendOdometry(currentPose);

    m_distance_to_goal = (sqrt((m_X - m_goal_position.getPosition().getX() / 1000.f)
                               * (m_X - m_goal_position.getPosition().getX() / 1000.f)
                             + (m_Y - m_goal_position.getPosition().getY() / 1000.f)
                                 * (m_Y - m_goal_position.getPosition().getY() / 1000.f)));
    std::cout << "distance to goal = " << m_distance_to_goal << std::endl;
}

geometry_msgs::Pose Core::updateCurrentPose(int32_t encoder1, int32_t encoder2)
{
    float linear_dist = compute_linear_dist(encoder1, encoder2);
    m_current_theta = get_orientation_float(encoder1, encoder2) + m_theta_zero;

    m_X += linear_dist * cos(m_current_theta * M_PI / 180.f);
    m_Y += linear_dist * sin(m_current_theta * M_PI / 180.f);

    std::cout << "X = " << m_X << ", Y = " << m_Y << ", theta = " << m_current_theta
              << ",linear_dist = " << linear_dist << std::endl;

    m_current_position = Position(m_X * 1000, m_Y * 1000, false);

    geometry_msgs::Pose currentPose;
    currentPose.position.x = m_X;
    currentPose.position.y = m_Y;

    tf2::Quaternion orientation_quat;
    orientation_quat.setRPY(0, 0, m_current_theta * M_PI / 180.f);
    currentPose.orientation = tf2::toMsg(orientation_quat);
    return currentPose;
}


void Core::sendOdometry(const geometry_msgs::Pose& currentPose)
{
    // first, we'll publish the transform over tf
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = ros::Time::now();
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";

    odom_trans.transform.translation.x = currentPose.position.x;
    odom_trans.transform.translation.y = currentPose.position.y;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = currentPose.orientation;

    // send the transform
    m_odom_broadcaster.sendTransform(odom_trans);

    nav_msgs::Odometry odomsg;
    odomsg.header.frame_id = "odom";
    odomsg.header.stamp = ros::Time::now();
    odomsg.child_frame_id = "base_link";

    for (unsigned int i = 0;
         i < (sizeof(odomsg.pose.covariance) / sizeof(odomsg.pose.covariance[0]));
         i++)
    {
        odomsg.pose.covariance[i] = 0;
    }

    odomsg.pose.pose = currentPose;

    odomsg.pose.covariance[0] = 0.1;
    odomsg.pose.covariance[7] = 0.1;
    odomsg.pose.covariance[35] = 0.2;

    odomsg.pose.covariance[14] = 0000; // set a non-zero covariance on unused
    odomsg.pose.covariance[21] = 0000; // dimensions (z, pitch and roll); this
    odomsg.pose.covariance[28] = 0000; // is a requirement of robot_pose_ekf
    // source:
    // https://github.com/yujinrobot/kobuki/blob/0.6.6/kobuki_node/src/library/odometry.cpp#L145-L154

    for (unsigned int i = 0;
         i < (sizeof(odomsg.twist.covariance) / sizeof(odomsg.twist.covariance[0]));
         i++)
    {
        odomsg.twist.covariance[i] = 0;
    }

    odomsg.twist.twist.linear.x = m_current_linear_speed;
    odomsg.twist.twist.linear.y = 0;
    odomsg.twist.twist.linear.z = 0;

    odomsg.twist.twist.angular.x = 0;
    odomsg.twist.twist.angular.y = 0;
    odomsg.twist.twist.angular.z = -m_current_angular_speed * M_PI / 180;
    m_odom_pub.publish(odomsg);
}

void Core::updateCurrentSpeed()
{
    ros::Time now = ros::Time::now();
    double time_since_last_speed_update = (m_last_speed_update_time - now).toSec();

    if (time_since_last_speed_update == 0)
    {
        std::cout << "Error: null time!" << std::endl;
    }

    float distance_moved = ((m_current_position - m_last_position).getNorme()) / 1000.f;

    m_last_position.setX(m_X * 1000);
    m_last_position.setY(m_Y * 1000);

    m_current_linear_speed = abs(distance_moved / time_since_last_speed_update);
    std::cout << "m_current_linear_speed = " << m_current_linear_speed << std::endl;

    m_current_angular_speed = (m_current_theta - m_last_theta) / time_since_last_speed_update;
    m_last_theta = m_current_theta;

    m_last_speed_update_time = now;
}

void Core::limitLinearSpeedCmdByGoal()
{
    float max_acceleration = 0.15f; // m*s-2
    float max_deceleration = 0.15f; // m*s-2
    float new_speed_order = 0;      // m/s

    float desired_final_speed = 0; // m*s-2

    float time_to_stop = (m_current_linear_speed - desired_final_speed) / max_deceleration;
    std::cout << "time to stop = " << time_to_stop << "s, ";

    float distance_to_stop = time_to_stop * (m_current_linear_speed - desired_final_speed) / 2;
    std::cout << ", distance to stop = " << distance_to_stop << "m, ";

    // Compute extra time if accelerating
    float average_extra_speed
      = (2 * m_current_linear_speed + max_acceleration / 2 + max_deceleration / 2);
    float extra_distance = average_extra_speed / UPDATE_RATE;

    if (m_distance_to_goal < distance_to_stop)
    {
        std::cout << "decelerate";
        new_speed_order = m_current_linear_speed - max_deceleration / UPDATE_RATE;
    }
    else if (m_distance_to_goal < m_current_linear_speed / UPDATE_RATE)
    {
        std::cout << "EMERGENCY BRAKE";
        new_speed_order = 0;
    }
    else if (m_distance_to_goal > distance_to_stop + extra_distance)
    {
        std::cout << "accelerate";
        new_speed_order = m_current_linear_speed + max_acceleration / UPDATE_RATE;
    }
    else
    {
        std::cout << "cruise speed";
        new_speed_order = m_current_linear_speed;
    }
    // m_linear_speed_cmd = MIN(m_default_linear_speed, new_speed_order);
    m_linear_speed_cmd = new_speed_order;
    std::cout << "new speed: " << new_speed_order << " => " << m_linear_speed_cmd << std::endl;
}
