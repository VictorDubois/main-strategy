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