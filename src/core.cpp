#include "core.h"
#include "ros/ros.h"
#include <std_msgs/Duration.h>
#include <stdexcept>

#define MAX_ALLOWED_ANGULAR_SPEED 0.2f

#include "lidarStrat.h"
#define UPDATE_RATE 10

#include <krabi_msgs/motors_cmd.h>

/**
 * Convert a cartesian position to a polar one
 * @param posX the X position, in mm
 * @param posY the Y position, in mm
 * @param theta the angle, in degrees
 * @param posX the distance, in meters
 **/
//#define DEBUG_cartToPolar
void Core::cartToPolar(int posX, int posY, float& theta, float& distance)
{
    theta = ((180.f / M_PI) * atan2((float)posX, (float)posY));
    distance = sqrt((float)(posX * posX + posY * posY));

    // fix angular ambiguity
    if (posY < 0)
    {
        theta += 180;
    }

#ifdef DEBUG_cartToPolar
    std::cout << "posX = " << posX << "posY = " << posY << "theta = " << theta
              << ", distance = " << distance << std::endl;
    float posXafter = distance * cos(theta * M_PI / 180.f);
    float posYafter = distance * sin(theta * M_PI / 180.f);
    std::cout << "posXafter = " << posXafter << ", posYafter = " << posYafter << std::endl;
#endif
}

float Core::vector_to_angle(geometry_msgs::Vector3 vector)
{
    return atan2(vector.y, vector.x) * 180 / M_PI;
}

float Core::vector_to_angle(geometry_msgs::Point vector)
{
    return atan2(vector.y, vector.x) * 180 / M_PI;
}

float Core::vector_to_amplitude(geometry_msgs::Vector3 vector)
{
    return sqrt((vector.x * vector.x) + (vector.y * vector.y));
}

float Core::vector_to_amplitude(geometry_msgs::Point vector)
{
    return sqrt((vector.x * vector.x) + (vector.y * vector.y));
}

float Core::getAngleToGoal()
{
    // Convert absolute position to relative position
    Position relative_m_goal_pose = m_goal_pose.getPosition() - m_current_pose.getPosition();
    // Position relative_m_goal_pose = m_goal_pose - m_starting_position - m_current_pose.getPosition();

    // Orient to goal
    float relative_m_goal_pose_angle = relative_m_goal_pose.getAngle() * 180. / M_PI;
    std::cout << "relative goal position = " << relative_m_goal_pose.getX() << ", "
              << relative_m_goal_pose.getY() << std::endl;
    std::cout << "m_target_orientation = " << relative_m_goal_pose_angle << std::endl;
    return relative_m_goal_pose_angle;
}

void Core::updateGoal(geometry_msgs::PoseStamped goal_pose)
{
    m_goal_pose = PositionPlusAngle(goal_pose.pose);

    // last_goal_max_speed = goal_out.max_speed;
}

void Core::updateTirette(std_msgs::Bool starting)
{
    if (starting.data && m_state == State::WAIT_TIRETTE)
    {
        std::cout << "Go for launch!" << std::endl;
        m_state = State::NORMAL;

        // Start counting the time
        m_begin_match = ros::Time::now();
    }
}

void Core::updateLidar(geometry_msgs::PoseStamped closest_obstacle)
{
    if (reverseGear())
    {
        return;
    }
    // Compute repulsive vector from obstacles
    float closest_obstacle_id = vector_to_angle(closest_obstacle.pose.position);
    float closest_obstacle_angle = closest_obstacle_id + 180;
    float obstacle_distance = vector_to_amplitude(closest_obstacle.pose.position);

    addObstacle(obstacle_distance, closest_obstacle_angle);
}

void Core::addObstacle(float obstacle_distance, float closest_obstacle_angle)
{
    float closest_obstacle_id = closest_obstacle_angle - 180;
    // Compute intensity of obstacle
    float obstacle_dangerouseness = 175. * obstacle_distance;

    m_speed_inhibition_from_obstacle
      = LidarStrat::speed_inhibition(obstacle_distance, closest_obstacle_angle, 1);

    if (m_speed_inhibition_from_obstacle < 0.2f)
    {
        m_speed_inhibition_from_obstacle = 0.f;
    }

    std::cout << "Speed inhib from obstacle = " << m_speed_inhibition_from_obstacle << ". Obstacle @"
              << obstacle_distance << "m, " << closest_obstacle_angle << "°." << std::endl;

    // printf("closest_obstacle_id = %d, peakValue = %f\n", closest_obstacle_id, peakValue);
    // Then apply gaussian function centered on the sensor's angle
    for (int j = 0; j < NB_NEURONS; j += 1)
    {
        // obstacles_output[j] += - gaussian(50., a, (360 + 180 - idx) % 360, j);
        m_lidar_output[j] += -gaussian(
          50., obstacle_dangerouseness, (fmod(360 + 180 + closest_obstacle_id, 360)), j);
    }
}

void Core::updateLidarBehind(geometry_msgs::PoseStamped closest_obstacle)
{
    if (!reverseGear())
    {
        return;
    }
    // Compute repulsive vector from obstacles
    float closest_obstacle_id = vector_to_angle(closest_obstacle.pose.position);
    float closest_obstacle_angle = closest_obstacle_id + 180;
    float obstacle_distance = vector_to_amplitude(closest_obstacle.pose.position);

    addObstacle(obstacle_distance, closest_obstacle_angle);
}

void Core::updateGear(std_msgs::Bool a_reverse_gear_activated)
{
    m_reverse_gear_activated = a_reverse_gear_activated.data;
}

Core::Core()
{
    ros::NodeHandle n;
    m_is_blue = false;
    m_begin_match = ros::Time(0);
    m_goal_pose = PositionPlusAngle();
    m_distance_to_goal = 0;
    geometry_msgs::Twist last_lidar_max_speed;
    last_lidar_max_speed.linear.x = 0;
    last_lidar_max_speed.linear.y = 0;
    last_lidar_max_speed.angular.z = 0;
    geometry_msgs::Twist last_goal_max_speed;
    last_goal_max_speed.linear.x = 0;
    last_goal_max_speed.linear.y = 0;
    last_goal_max_speed.angular.z = 0;
    m_speed_inhibition_from_obstacle = 0;
    m_motors_cmd_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 5);
    m_motors_enable_pub = n.advertise<std_msgs::Bool>("enable_motor", 5);
    m_chrono_pub = n.advertise<std_msgs::Duration>("remaining_time", 5);
    m_goal_sub = n.subscribe("goal_pose", 1000, &Core::updateGoal, this);
    m_lidar_sub = n.subscribe("obstacle_pose_stamped", 1000, &Core::updateLidar, this);
    m_odometry_sub = n.subscribe("odom",1000, &Core::updateOdom, this);
    m_lidar_behind_sub
      = n.subscribe("obstacle_behind_pose_stamped", 1000, &Core::updateLidarBehind, this);
    m_tirette_sub = n.subscribe("tirette", 1, &Core::updateTirette, this);
    m_reverse_gear_sub = n.subscribe("reverseGear", 1000, &Core::updateGear, this);

    n.param<bool>("isBlue", m_is_blue, true);

    if (m_is_blue)
    {
        std::cout << "Is Blue !" << std::endl;
    }
    else
    {
        std::cout << "Not Blue :'(" << std::endl;
    }

    m_goal_output[NB_NEURONS] = { 0. };
    m_obstacles_output[NB_NEURONS] = { 0. };
    m_lidar_output[NB_NEURONS] = { 0. };
    m_angular_speed_vector[NB_NEURONS] = { 0. };
    m_angular_landscape[NB_NEURONS] = { 0. };
    m_orienting = false;

    printf("done! Proceeding.\nStarting IA.\n");

    /**************************************
     *      Variable initialization       *
     **************************************/
    m_linear_speed = 0;
    m_angular_speed = 0;
    m_linear_speed_cmd = 0;
    m_angular_speed_cmd = 0;

}

void Core::updateOdom(const nav_msgs::Odometry& odometry){
    m_current_pose.setX(odometry.pose.pose.position.x);
    m_current_pose.setY(odometry.pose.pose.position.y);
    double roll, pitch, yaw;
    auto quat = odometry.pose.pose.orientation;
    tf::Quaternion q(quat.x, quat.y, quat.z, quat.w);
    tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
    m_current_pose.setAngle(yaw);
    
    m_linear_speed = tf::Vector3(odometry.twist.twist.linear.x, odometry.twist.twist.linear.y,0).length();
    m_angular_speed = odometry.twist.twist.angular.z;
    m_distance_to_goal = sqrt(pow(odometry.pose.pose.position.x,2) + pow(odometry.pose.pose.position.y,2));
}

void Core::stopMotors()
{
    setMotorsSpeed(0, 0, false, false);
}

void Core::setMotorsSpeed(float linearSpeed, float angularSpeed)
{
    setMotorsSpeed(linearSpeed, angularSpeed, true, false);
}

void Core::setMotorsSpeed(float linearSpeed,
                            float angularSpeed,
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

void Core::landscapeFromAngleAndStrength(std::vector<float> landscape, float angle, float strength)
{
    for (int i = 0; i < NB_NEURONS; i++)
    {
        landscape[i] = cos(angle - i) * strength;
    }
}

Core::~Core()
{
    // We broke out of the loop, stop everything
    stopMotors();

    printf("Got out of the main loop, stopped everything.\n");
}

Core::State Core::Loop()
{
    if ((m_state != State::WAIT_TIRETTE) && isTimeToStop())
    {
        std::cout << "Time's up !" << std::endl;
        return m_state;
    }

    if (m_state == State::WAIT_TIRETTE)
    {
        setMotorsSpeed(0, 0, false, false);
    }
    else if (m_state == State::NORMAL)
    {
        // Get the orientation we need to follow to reach the goal
        // TODO: assign priority integers to strategies, take the strategy that has the max of
        // priority * strength

        // computeTargetSpeedOrientation(orientation);

        if (m_distance_to_goal >= 0.05f)
        {
            m_orienting = false;
        }
        if (m_distance_to_goal > 0.02f && !m_orienting)
        {

            // orient towards the goal's position
            m_target_orientation = getAngleToGoal();
        }
        else
        {
            m_orienting = true;
            // respect the goal's own orientation
            m_target_orientation = m_goal_pose.getAngle();
            if (!m_is_blue)
            {
                m_target_orientation += 180; // seems to be needed since odom_light
            }
            std::cout << "########################################" << std::endl
                      << "Positionned, m_orienting to " << m_goal_pose.getAngle() << std::endl
                      << "########################################" << std::endl;
        }

        if (reverseGear() && !m_orienting)
        {
            m_target_orientation += 180.f;
        }

        // Inhibit linear speed if there are obstacles

        // Compute attractive vectors from positive valence strategies
        // TODO: choose the POSITIVE VALENCE STRATEGY!
        for (int i = 0; i < NB_NEURONS; i += 1)
        {
            m_goal_output[i]
              = target(207.f, 1.1f, fmod(360 + 180 - (m_target_orientation - m_current_pose.getAngle()), 360), i);
        }

        std::cout << "relative_m_target_orientation: " << (m_target_orientation - m_current_pose.getAngle())
                  << ", peak value: " << get_idx_of_max(m_goal_output, NB_NEURONS)
                  << ", central value = " << m_goal_output[180];

        // Sum positive and negative valence strategies
        for (int i = 0; i < NB_NEURONS; i += 1)
        {
            // Temporarily check if joystick is active (later: use weighted sum)
            // if (s_joystick.output->strength == 0) {
            m_angular_landscape[i] = m_goal_output[i]; // + m_lidar_output[i];
            /*}
            else {
                    printf("joystick is active\n");
                    fflush(stdout);
                    m_angular_landscape[i] = m_goal_output[i];
            }*/
        }

        // And finally: differentiate the m_angular_landscape vector to get drive
        differentiate(m_angular_landscape, m_angular_speed_vector, NB_NEURONS, 3000.);

        // Set linear speed according to the obstacles strategy & angular speed based on goal +
        // obstacles Robot's vision is now centered on 180 deg
        float m_angular_speed_cmd
          = -m_angular_speed_vector[180]; // - as positive is towards the left in ros, while the
                                        // derivation is left to right
        std::cout << ",m_angular_speed_cmd = " << m_angular_speed_cmd << std::endl;

        // m_linear_speed_cmd = m_default_linear_speed;

        limitLinearSpeedCmdByGoal();

        // Do not be afraid of the lighthouse
        if (m_current_pose.getPosition().getY() < 350)
        {
            m_speed_inhibition_from_obstacle = 1000;
        }

        // Do not be afraid of the manche a air
        if (reverseGear() && m_current_pose.getPosition().getY() > 2000 - 350)
        {
            m_speed_inhibition_from_obstacle = 1000;
        }

        m_linear_speed_cmd
          = std::min(m_linear_speed_cmd, m_default_linear_speed * m_speed_inhibition_from_obstacle);

        limitAngularSpeedCmd(m_angular_speed_cmd);

        update_speed(false, &m_angular_speed, m_angular_speed_cmd);

        if (DISABLE_LINEAR_SPEED || m_orienting)
        {
            m_linear_speed_cmd = 0;
        }

        if (DISABLE_ANGULAR_SPEED)
        {
            m_angular_speed_cmd = 0;
        }

        // m_linear_speed = 0;
        // Modulate linear speed by angular speed: stop going forward when you want to turn
        // m_linear_speed = MIN(m_linear_speed, m_linear_speed / abs(m_angular_speed));
        if (m_angular_speed_cmd < -1 || m_angular_speed_cmd > 1)
        {
            m_linear_speed_cmd = 0;
        }
        std::cout << "linear speed = " << m_linear_speed << ", m_orienting = " << m_orienting
                  << "speed inihib from obstacles = " << m_speed_inhibition_from_obstacle << " * "
                  << m_default_linear_speed << std::endl;

        // Set motors speed according to values computed before
        setMotorsSpeed(m_linear_speed_cmd, m_angular_speed_cmd / 5.f, true, false);
    } // End of m_state == State::NORMAL

    publishRemainingTime();

    return m_state;
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

void Core::computeTargetSpeedOrientation(const unsigned int orientation)
{
    /*if (s_joystick.output->strength == 1) {
            m_target_orientation = get_idx_of_max(s_joystick.output->neural_field, NB_NEURONS);
            m_linear_speed_cmd = s_joystick.output->speed_inhibition;
            //printf("Target orientation from joystick: %d\n", m_target_orientation);
    } else if (s_goal.output->strength == 1) {
            m_target_orientation = get_idx_of_max(s_goal.output->neural_field, NB_NEURONS);
            //m_linear_speed_cmd = m_default_linear_speed;
            m_linear_speed_cmd = s_goal.output->speed_inhibition;
            //printf("Linear speed command from goal: %d\n", m_linear_speed_cmd);
            //printf("Target orientation from goal: %d\n", m_target_orientation);
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

void Core::limitAngularSpeedCmd(float& m_angular_speed)
{
    // TODO: use the winning strategy with weights
    // TODO: restore speed limitations
    /*if (s_goal.output->strength == 1) {
            int speed_limited_by_obstacle = s_goal.output->m_angular_speed_inhibition *
    MAX_ALLOWED_ANGULAR_SPEED / 100; MIN(m_angular_speed, speed_limited_by_obstacle);
            MAX(m_angular_speed, -speed_limited_by_obstacle);
    }*/

    // Cap angular speed, so that the robot doesn't turn TOO FAST on itself
    std::max(m_angular_speed, -MAX_ALLOWED_ANGULAR_SPEED);
    std::min(m_angular_speed, MAX_ALLOWED_ANGULAR_SPEED);
}

void Core::limitLinearSpeedCmdByGoal()
{
    float max_acceleration = 0.15f; // m*s-2
    float max_deceleration = 0.15f; // m*s-2
    float new_speed_order = 0;      // m/s

    float desired_final_speed = 0; // m*s-2

    float time_to_stop = (m_linear_speed - desired_final_speed) / max_deceleration;
    std::cout << "time to stop = " << time_to_stop << "s, ";

    float distance_to_stop = time_to_stop * (m_linear_speed - desired_final_speed) / 2;
    std::cout << ", distance to stop = " << distance_to_stop << "m, ";

    // Compute extra time if accelerating
    float average_extra_speed
      = (2 * m_linear_speed + max_acceleration / 2 + max_deceleration / 2);
    float extra_distance = average_extra_speed / UPDATE_RATE;

    if (m_distance_to_goal < distance_to_stop)
    {
        std::cout << "decelerate";
        new_speed_order = m_linear_speed - max_deceleration / UPDATE_RATE;
    }
    else if (m_distance_to_goal < m_linear_speed / UPDATE_RATE)
    {
        std::cout << "EMERGENCY BRAKE";
        new_speed_order = 0;
    }
    else if (m_distance_to_goal > distance_to_stop + extra_distance)
    {
        std::cout << "accelerate";
        new_speed_order = m_linear_speed + max_acceleration / UPDATE_RATE;
    }
    else
    {
        std::cout << "cruise speed";
        new_speed_order = m_linear_speed;
    }
    // m_linear_speed_cmd = MIN(m_default_linear_speed, new_speed_order);
    m_linear_speed_cmd = new_speed_order;
    std::cout << "new speed: " << new_speed_order << " => " << m_linear_speed_cmd << std::endl;
}
