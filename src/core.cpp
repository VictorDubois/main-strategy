#include "core.h"
#include "ros/ros.h"
#include <std_msgs/Duration.h>
#include <stdexcept>

#define MAX_ALLOWED_ANGULAR_SPEED 0.2f

#include "lidarStrat.h"
#define UPDATE_RATE 10

#include <krabi_msgs/motors_cmd.h>

void Core::updateCurrentPose()
{
    try
    {
        auto base_link_id = tf::resolve(ros::this_node::getNamespace(), "base_link");
        const auto& transform
          = m_tf_buffer.lookupTransform("map", base_link_id, ros::Time(0)).transform;
        m_baselink_to_map = transformFromMsg(transform);
        m_map_to_baselink = transformFromMsg(
          m_tf_buffer.lookupTransform(base_link_id, "map", ros::Time(0)).transform);
        m_current_pose = Pose(transform);
    }
    catch (tf2::TransformException& ex)
    {
        ROS_WARN("%s", ex.what());
    }

    ROS_DEBUG_STREAM("updateCurrentPose: " << m_current_pose << std::endl);
    ROS_DEBUG_STREAM("Transform matrix [baselink to map]: " << m_baselink_to_map);
}

Angle Core::getAngleToGoal()
{
    return (m_goal_pose.getPosition()-m_current_pose.getPosition()).getAngle();
}

void Core::updateGoal(geometry_msgs::PoseStamped goal_pose)
{
    m_goal_pose = Pose(goal_pose.pose);
    ROS_DEBUG_STREAM("New goal: " << m_goal_pose);
}

void Core::updateTirette(std_msgs::Bool starting)
{
    if (starting.data && m_state == State::WAIT_TIRETTE)
    {
        ROS_INFO_STREAM("Go for launch!");
        m_state = State::NORMAL;

        // Start counting the time
        m_begin_match = ros::Time::now();
    }
}

void Core::updateLidar(boost::shared_ptr<geometry_msgs::PoseStamped const> closest_obstacle, bool front)
{
    if (front == !reverseGear())
    {
        addObstacle(Position(closest_obstacle->pose.position));
    }
}

void Core::addObstacle(PolarPosition obstacle)
{
    unsigned int closest_obstacle_id = angle_to_neuron_id(obstacle.getAngle());

    Angle normalized_angle = !reverseGear()
                               ? obstacle.getAngle()
                               : AngleTools::wrapAngle(Angle(obstacle.getAngle()+ M_PI));

    float m_speed_inhibition_from_obstacle
      = LidarStrat::speed_inhibition(obstacle.getDistance(), normalized_angle, 1);

    if (m_speed_inhibition_from_obstacle < 0.2f)
    {
        m_speed_inhibition_from_obstacle = 0.f;
    }

    ROS_INFO_STREAM("Speed inhib from obstacle = " << m_speed_inhibition_from_obstacle
                                                   << ". Obstacle @" << obstacle << std::endl);

    for (int j = 0; j < NB_NEURONS; j += 1)
    {
        m_lidar_output[j]
          += -gaussian(50., m_speed_inhibition_from_obstacle, closest_obstacle_id, j);
    }
}

void Core::updateGear(std_msgs::Bool a_reverse_gear_activated)
{
    m_reverse_gear_activated = a_reverse_gear_activated.data;
}

Core::Core(ros::NodeHandle& nh)
  : m_tf_listener(m_tf_buffer)
  , m_nh(nh)
{
    m_begin_match = ros::Time(0);
    m_goal_pose = Pose();
    m_distance_to_goal = 0;

    m_speed_inhibition_from_obstacle = 1;
    m_reverse_gear_activated = false;

    m_motors_cmd_pub =m_nh.advertise<geometry_msgs::Twist>("cmd_vel", 5);
    m_motors_enable_pub =m_nh.advertise<std_msgs::Bool>("enable_motor", 5);
    m_chrono_pub =m_nh.advertise<std_msgs::Duration>("remaining_time", 5);
    m_goal_sub =m_nh.subscribe("goal_pose", 1000, &Core::updateGoal, this);

    m_lidar_sub =m_nh.subscribe<geometry_msgs::PoseStamped>(
          "obstacle_pose_stamped", 5, boost::bind(&Core::updateLidar, this, _1, true));


    m_lidar_behind_sub =m_nh.subscribe<geometry_msgs::PoseStamped>(
      "obstacle_behind_pose_stamped", 5, boost::bind(&Core::updateLidar, this, _1, true));
    m_tirette_sub =m_nh.subscribe("tirette", 1, &Core::updateTirette, this);
    m_odometry_sub =m_nh.subscribe("odom", 1000, &Core::updateOdom, this);

    m_reverse_gear_sub =m_nh.subscribe("reverseGear", 1000, &Core::updateGear, this);

   m_nh.param<bool>("isBlue", m_is_blue, true);

    ROS_INFO_STREAM(m_is_blue ? "Is Blue !" : "Not Blue :'(");

    m_goal_output[NB_NEURONS] = { 0. };
    m_obstacles_output[NB_NEURONS] = { 0. };
    m_lidar_output[NB_NEURONS] = { 0. };
    m_angular_speed_vector[NB_NEURONS] = { 0. };
    m_angular_landscape[NB_NEURONS] = { 0. };
    m_orienting = false;

    ROS_INFO_STREAM("Init done! Proceeding.\nStarting IA.\n");

    /**************************************
     *      Variable initialization       *
     **************************************/
    m_linear_speed = 0;
    m_angular_speed = 0;
    m_linear_speed_cmd = 0;
    m_angular_speed_cmd = 0;
}

void Core::updateOdom(const nav_msgs::Odometry& odometry)
{
    m_linear_speed
      = tf::Vector3(odometry.twist.twist.linear.x, odometry.twist.twist.linear.y, 0).length();
    m_angular_speed = odometry.twist.twist.angular.z;
}

void Core::stopMotors()
{
    setMotorsSpeed(Vitesse(0), VitesseAngulaire(0), false, false);
}

void Core::setMotorsSpeed(Vitesse linearSpeed, VitesseAngulaire angularSpeed)
{
    setMotorsSpeed(linearSpeed, angularSpeed, true, false);
}

void Core::setMotorsSpeed(Vitesse linearSpeed,
                          VitesseAngulaire angularSpeed,
                          bool enable,
                          bool /*resetEncoders*/)
{
    geometry_msgs::Twist new_motor_cmd;
    new_motor_cmd.linear.x = reverseGear() ? -linearSpeed : linearSpeed;
    new_motor_cmd.angular.z = angularSpeed;

    m_motors_cmd_pub.publish(new_motor_cmd);
    std_msgs::Bool new_enable_cmd;
    new_enable_cmd.data = enable;
    m_motors_enable_pub.publish(new_enable_cmd);
}

bool Core::reverseGear()
{
    return m_reverse_gear_activated;
}

int Core::Setup()
{
    // Take time before entering the loop
    usleep(10000); // So the next now-last won't return 0

    stopMotors();

    return 0;
}

Core::~Core()
{
    // We broke out of the loop, stop everything
    stopMotors();

    ROS_INFO_STREAM("Got out of the main loop, stopped everything.\n");
}

Core::State Core::Loop()
{
    if ((m_state != State::WAIT_TIRETTE) && isTimeToStop())
    {
        ROS_INFO_STREAM("Time's up !");
        return m_state;
    }

    if (m_state == State::WAIT_TIRETTE)
    {
        setMotorsSpeed(Vitesse(0), VitesseAngulaire(0), false, false);
    }
    else if (m_state == State::NORMAL)
    {
        updateCurrentPose();
        m_distance_to_goal = (m_goal_pose.getPosition() - m_current_pose.getPosition()).getNorme();

        if (m_distance_to_goal >= Distance(0.05f))
        {
            m_orienting = false;
        }
        if (m_distance_to_goal > Distance(0.02f) && !m_orienting)
        {

            // orient towards the goal's position
            m_target_orientation = getAngleToGoal();
        }
        else
        {
            m_orienting = true;
            // respect the goal's own orientation
            m_target_orientation = m_goal_pose.getAngle();

            ROS_DEBUG_STREAM("########################################"
                            << std::endl
                            << "Positionned, m_orienting to " << m_goal_pose.getAngle() << std::endl
                            << "########################################" << std::endl);
        }

        if (reverseGear() && !m_orienting)
        {
            m_target_orientation = AngleTools::wrapAngle(Angle(m_target_orientation + M_PI));
        }

        // Inhibit linear speed if there are obstacles

        // Compute attractive vectors from positive valence strategies
        // TODO: choose the POSITIVE VALENCE STRATEGY!
        auto delta_orientation
          = AngleTools::diffAngle(m_target_orientation, m_current_pose.getAngle());
        for (int i = 0; i < NB_NEURONS; i += 1)
        {
            m_goal_output[i] = target(207.f, 1.1f, angle_to_neuron_id(delta_orientation), i);
        }

        ROS_INFO_STREAM("relative_target_orientation: "
                        << delta_orientation
                        << ", peak value: " << get_idx_of_max(m_goal_output, NB_NEURONS)
                        << ", central value = " << m_goal_output[angle_to_neuron_id(Angle(0))]);

        // Sum positive and negative valence strategies
        for (int i = 0; i < NB_NEURONS; i += 1)
        {
            m_angular_landscape[i] = m_goal_output[i]; // + m_lidar_output[i];
        }

        // And finally: differentiate the m_angular_landscape vector to get drive
        differentiate(m_angular_landscape, m_angular_speed_vector, NB_NEURONS, 3000.);

        // Set linear speed according to the obstacles strategy & angular speed based on goal +
        // obstacles Robot's vision is now centered on 180 deg
        m_angular_speed_cmd = VitesseAngulaire(m_angular_speed_vector[angle_to_neuron_id(
          Angle(0))]); // - as positive is towards the left in ros, while the
                       // derivation is left to right
        ROS_DEBUG_STREAM(",m_angular_speed_cmd = " << m_angular_speed_cmd << std::endl);

        limitLinearSpeedCmdByGoal();

        m_linear_speed_cmd = std::min(
          m_linear_speed_cmd, Vitesse(m_default_linear_speed * m_speed_inhibition_from_obstacle));

        limitAngularSpeedCmd(m_angular_speed_cmd);

        limitAcceleration();

        if (DISABLE_LINEAR_SPEED || m_orienting)
        {
            m_linear_speed_cmd = 0;
        }

        if (DISABLE_ANGULAR_SPEED)
        {
            m_angular_speed_cmd = 0;
        }

        // Modulate linear speed by angular speed: stop going forward when you want to turn
        // m_linear_speed = MIN(m_linear_speed, m_linear_speed / abs(m_angular_speed));
        if (abs(m_angular_speed_cmd) > 0.8*MAX_ALLOWED_ANGULAR_SPEED)
        {
            m_linear_speed_cmd = 0;
            ROS_DEBUG_STREAM("Want to turn, stop going forward");
        }
        ROS_DEBUG_STREAM("linear speed = " << m_linear_speed << ", m_orienting = " << m_orienting
                                          << ", speed inihib from obstacles = "
                                          << m_speed_inhibition_from_obstacle << " * "
                                          << m_default_linear_speed << std::endl);

        // Set motors speed according to values computed before
        setMotorsSpeed(m_linear_speed_cmd, m_angular_speed_cmd / 5.f, true, false);
    } // End of m_state == State::NORMAL

    publishRemainingTime();

    return m_state;
}

template<typename velocity_t, typename acceleration_t>
velocity_t limit_acceleration(velocity_t current_velocity,
                              velocity_t cmd_velocity,
                              acceleration_t max_acceleration,
                              double dt)
{
    auto delta_vel = cmd_velocity - current_velocity;
    int delta_vel_sign = (delta_vel > 0) ? 1 : ((delta_vel < 0) ? -1 : 0);
    delta_vel = delta_vel_sign * max_acceleration * dt;
    return velocity_t(current_velocity + delta_vel);
}

void Core::limitAcceleration()
{
    /**
    ROS_DEBUG_STREAM("Limiting acceleration:");
    ROS_DEBUG_STREAM("Before: linear_speed: " << m_linear_speed_cmd << "m/s , angular_speed: "
                                              << m_angular_speed_cmd << "rad/s");
    m_linear_speed_cmd = limit_acceleration(
      m_linear_speed, m_linear_speed_cmd, Acceleration(0.025), 1. / float(UPDATE_RATE));
    m_angular_speed_cmd = limit_acceleration(
      m_angular_speed, m_angular_speed_cmd, AccelerationAngulaire(0.00025), 1. / float(UPDATE_RATE));
    ROS_DEBUG_STREAM("After: linear_speed: " << m_linear_speed_cmd << "m/s , angular_speed: "
                                             << m_angular_speed_cmd << "rad/s");
                                             **/
}

void Core::publishRemainingTime()
{
    std_msgs::Duration remaining_time_msg;

    if (m_begin_match.toSec() < 1)
    {
        remaining_time_msg.data = ros::Duration(TIMEOUT_END_MATCH / 1000);
    }
    else
    {
        ros::Time end_of_match = ros::Time(m_begin_match.toSec() + TIMEOUT_END_MATCH / 1000);
        remaining_time_msg.data = end_of_match - ros::Time::now();
    }

    m_chrono_pub.publish(remaining_time_msg);
}

bool Core::isTimeToStop()
{
    // State::EXIT and stop robot if we reached the end of the match
    if (ENABLE_TIMEOUT_END_MATCH == TRUE
        && (ros::Time::now() - m_begin_match).toSec() * 1000 > TIMEOUT_END_MATCH)
    {
        m_state = State::EXIT;
        return true;
    }
    return false;
}

void Core::computeTargetSpeedOrientation()
{
    /*if (s_joystick.output->strength == 1) {
            m_target_orientation = get_idx_of_max(s_joystick.output->neural_field, NB_NEURONS);
            m_linear_speed_cmd = s_joystick.output->speed_inhibition;
            //ROS_INFO_STREAM("Target orientation from joystick: %d\n", m_target_orientation);
    } else if (s_goal.output->strength == 1) {
            m_target_orientation = get_idx_of_max(s_goal.output->neural_field, NB_NEURONS);
            //m_linear_speed_cmd = m_default_linear_speed;
            m_linear_speed_cmd = s_goal.output->speed_inhibition;
            //ROS_INFO_STREAM("Linear speed command from goal: %d\n", m_linear_speed_cmd);
            //ROS_INFO_STREAM("Target orientation from goal: %d\n", m_target_orientation);
    } else {
            /*
             * If no positive valence strategy fires, set the target orientation to the robot's
    orientation
             * which means the robot will go straight on
             */
    // m_target_orientation = orientation;
    m_linear_speed_cmd = m_default_linear_speed;
    //}
}

void Core::limitAngularSpeedCmd(VitesseAngulaire& m_angular_speed)
{
    // TODO: use the winning strategy with weights
    // TODO: restore speed limitations
    /*if (s_goal.output->strength == 1) {
            int speed_limited_by_obstacle = s_goal.output->m_angular_speed_inhibition *
    MAX_ALLOWED_ANGULAR_SPEED / 100; MIN(m_angular_speed, speed_limited_by_obstacle);
            MAX(m_angular_speed, -speed_limited_by_obstacle);
    }*/

    // Cap angular speed, so that the robot doesn't turn TOO FAST on itself
    m_angular_speed = std::max(m_angular_speed, VitesseAngulaire(-MAX_ALLOWED_ANGULAR_SPEED));
    m_angular_speed = std::min(m_angular_speed, VitesseAngulaire(MAX_ALLOWED_ANGULAR_SPEED));
}

void Core::limitLinearSpeedCmdByGoal()
{
    Acceleration max_acceleration = Acceleration(0.15f); // m*s-2
    Acceleration max_deceleration = Acceleration(0.15f); // m*s-2
    Vitesse new_speed_order = Vitesse(0);                // m/s

    Vitesse desired_final_speed = Vitesse(0); // m*s-2

    float time_to_stop = (m_linear_speed - desired_final_speed) / max_deceleration;
    ROS_DEBUG_STREAM("time to stop = " << time_to_stop << "s, ");

    Distance distance_to_stop
      = Distance(time_to_stop * (m_linear_speed - desired_final_speed) / 2.);
    ROS_DEBUG_STREAM(", distance to stop = " << distance_to_stop << "m, ");

    // Compute extra time if accelerating
    // TODO there appears to be a problem here
    Vitesse average_extra_speed
      = Vitesse((Acceleration(2 * m_linear_speed) + max_acceleration / 2. + max_deceleration / 2.));
    Distance extra_distance = Distance(average_extra_speed / float(UPDATE_RATE));

    if (m_distance_to_goal < distance_to_stop)
    {
        ROS_DEBUG_STREAM("decelerate");
        new_speed_order = m_linear_speed - max_deceleration / float(UPDATE_RATE);
    }
    else if (m_distance_to_goal < m_linear_speed / float(UPDATE_RATE))
    {
        ROS_DEBUG_STREAM("EMERGENCY BRAKE");
        new_speed_order = 0;
    }
    else if (m_distance_to_goal > distance_to_stop + extra_distance)
    {
        ROS_DEBUG_STREAM("accelerate");
        new_speed_order = m_linear_speed + max_acceleration / float(UPDATE_RATE);
    }
    else
    {
        ROS_DEBUG_STREAM("cruise speed");
        new_speed_order = m_linear_speed;
    }
    // m_linear_speed_cmd = MIN(m_default_linear_speed, new_speed_order);
    ROS_DEBUG_STREAM("new speed: " << new_speed_order << " => " << m_linear_speed_cmd << std::endl);
    m_linear_speed_cmd = new_speed_order;
}

unsigned int angle_to_neuron_id(Angle a)
{
    return (unsigned int)(((AngleTools::wrapAngle(a) + M_PI) / (2 * M_PI)) * NB_NEURONS);
}