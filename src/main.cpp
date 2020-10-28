#include "core.h"
#include "ros/ros.h"
#include <std_msgs/Duration.h>
#include <stdexcept>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#define MAX_ALLOWED_ANGULAR_SPEED 0.2f

#include "../../lidar_strategy/include/lidarStrat.h"
#define UPDATE_RATE 10
#ifndef MAX
#define MIN(a, b) (((a) < (b)) ? (a) : (b))
#define MAX(a, b) (((a) > (b)) ? (a) : (b))
#endif

#include <goal_strategy/motors_cmd.h>

/**
 * Convert a cartesian position to a polar one
 * @param posX the X position, in mm
 * @param posY the Y position, in mm
 * @param theta the angle, in degrees
 * @param posX the distance, in meters
 **/
//#define DEBUG_cart_to_polar
void Core::cart_to_polar(int posX, int posY, float& theta, float& distance)
{
    theta = ((180.f / M_PI) * atan2((float)posX, (float)posY));
    distance = sqrt((float)(posX * posX + posY * posY));

    // fix angular ambiguity
    if (posY < 0)
    {
        theta += 180;
    }

#ifdef DEBUG_cart_to_polar
    std::cout << "posX = " << posX << "posY = " << posY << "theta = " << theta
              << ", distance = " << distance << std::endl;
    float posXafter = distance * cos(theta * M_PI / 180.f);
    float posYafter = distance * sin(theta * M_PI / 180.f);
    std::cout << "posXafter = " << posXafter << ", posYafter = " << posYafter << std::endl;
#endif
}

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "mainStrat");
    ros::start();
    ros::Rate staticLoop(UPDATE_RATE);
    Core* my_core = new Core();
    my_core->Setup();

    while (my_core->Loop() && ros::ok())
    {
        staticLoop.sleep();
        ros::spinOnce();
    }

    delete my_core;
}

void Core::updateOdometry(nav_msgs::Odometry odometry)
{
    X = odometry.pose.pose.position.x + starting_position.getX() / 1000.f;
    Y = odometry.pose.pose.position.y + starting_position.getY() / 1000.f;
    current_position = Position(X * 1000, Y * 1000, false);

    double siny_cosp = 2
                       * (odometry.pose.pose.orientation.w * odometry.pose.pose.orientation.z
                          + odometry.pose.pose.orientation.x * odometry.pose.pose.orientation.y);
    double cosy_cosp
      = 1
        - 2
            * (odometry.pose.pose.orientation.y * odometry.pose.pose.orientation.y
               + odometry.pose.pose.orientation.z * odometry.pose.pose.orientation.z);
    current_theta = std::atan2(siny_cosp, cosy_cosp) * 180.f / M_PI;

    current_pose_pub.publish(odometry.pose.pose);

    distance_to_goal = (sqrt((X - goal_position.getPosition().getX() / 1000.f)
                               * (X - goal_position.getPosition().getX() / 1000.f)
                             + (Y - goal_position.getPosition().getY() / 1000.f)
                                 * (Y - goal_position.getPosition().getY() / 1000.f)));
    std::cout << "distance to goal = " << distance_to_goal << std::endl;
}

void Core::updateCurrentPose(goal_strategy::encoders encoders)
{
    return;

    encoder1 = encoders.encoder_left;
    encoder2 = encoders.encoder_right;

    if (!encoders_initialized)
    {
        std::cout << "initializing encoders" << std::endl;
        starting_encoder1 = encoder1;
        starting_encoder2 = encoder2;
        encoders_initialized = true;
    }

    encoder1 -= starting_encoder1;
    encoder2 -= starting_encoder2;

    geometry_msgs::Pose currentPose = update_current_pose(encoder1, encoder2);

    current_pose_pub.publish(currentPose);
    update_current_speed();
    send_odometry(currentPose);

    distance_to_goal = (sqrt((X - goal_position.getPosition().getX() / 1000.f)
                               * (X - goal_position.getPosition().getX() / 1000.f)
                             + (Y - goal_position.getPosition().getY() / 1000.f)
                                 * (Y - goal_position.getPosition().getY() / 1000.f)));
    std::cout << "distance to goal = " << distance_to_goal << std::endl;
}

void Core::updateLightOdom(goal_strategy::odom_light motors_odom)
{
    // raw values send by the motor board, that are not in the correct frame
    float temp_X = motors_odom.pose.position.x;
    float temp_Y = motors_odom.pose.position.y;
    float temp_theta = motors_odom.pose.orientation.z * 180 / M_PI;

    if (!encoders_initialized)
    {
        std::cout << "initializing encoders" << std::endl;
        starting_X -= temp_X;
        starting_Y -= temp_Y;
	std::cout << "x = " << temp_X << ", Y = " << temp_Y << ", theta = " << temp_theta << "theta_zero = " << theta_zero << std::endl;
        //theta_zero -= temp_theta;
        last_position.setX(starting_X);
        last_position.setX(starting_Y);
        encoders_initialized = true;
    }

    X = temp_X + starting_X;
    Y = temp_Y + starting_Y;

    current_position = Position(X * 1000, Y * 1000, false);

    current_theta = temp_theta + theta_zero;

    geometry_msgs::Pose currentPose = motors_odom.pose;
    currentPose.position.x = X;
    currentPose.position.y = Y;

    tf2::Quaternion orientation_quat;
    orientation_quat.setRPY(0, 0, current_theta * M_PI / 180);
    currentPose.orientation = tf2::toMsg(orientation_quat);

    current_pose_pub.publish(currentPose);
    update_current_speed();
    // current_linear_speed = motors_odom.speed.linear.x;// not yet working: returns 0
    // current_angular_speed = motors_odom.speed.angular.z;// not yet working: returns 0
    send_odometry(currentPose);

    distance_to_goal = (sqrt((X - goal_position.getPosition().getX() / 1000.f)
                               * (X - goal_position.getPosition().getX() / 1000.f)
                             + (Y - goal_position.getPosition().getY() / 1000.f)
                                 * (Y - goal_position.getPosition().getY() / 1000.f)));
    std::cout << "distance to goal = " << distance_to_goal << std::endl;
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
    Position relative_goal_position = goal_position.getPosition() - current_position;
    // Position relative_goal_position = goal_position - starting_position - current_position;

    // Orient to goal
    float relative_goal_position_angle = relative_goal_position.getAngle() * 180. / M_PI;
    std::cout << "relative goal position = " << relative_goal_position.getX() << ", "
              << relative_goal_position.getY() << std::endl;
    std::cout << "target_orientation = " << relative_goal_position_angle << std::endl;
    return relative_goal_position_angle;
}

void Core::updateGoal(geometry_msgs::PoseStamped goal_pose)
{
    goal_position = PositionPlusAngle(goal_pose.pose);

    // last_goal_max_speed = goal_out.max_speed;
}

void Core::updateTirette(std_msgs::Bool starting)
{
    if (starting.data && state == WAIT_TIRETTE)
    {
        std::cout << "Go for launch!" << std::endl;
        state = NORMAL;

        // Start counting the time
        begin_match = ros::Time::now();
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

    speed_inhibition_from_obstacle
      = LidarStrat::speed_inhibition(obstacle_distance, closest_obstacle_angle, 1);

    if (speed_inhibition_from_obstacle < 0.2f)
    {
        speed_inhibition_from_obstacle = 0.f;
    }

    std::cout << "Speed inhib from obstacle = " << speed_inhibition_from_obstacle << ". Obstacle @"
              << obstacle_distance << "m, " << closest_obstacle_angle << "°." << std::endl;

    // printf("closest_obstacle_id = %d, peakValue = %f\n", closest_obstacle_id, peakValue);
    // Then apply gaussian function centered on the sensor's angle
    for (int j = 0; j < NB_NEURONS; j += 1)
    {
        // obstacles_output[j] += - gaussian(50., a, (360 + 180 - idx) % 360, j);
        lidar_output[j] += -gaussian(
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
    is_blue = false;
    last_speed_update_time = ros::Time::now();
    begin_match = ros::Time(0);
    current_linear_speed = 0;
    encoders_initialized = false;
    goal_position = PositionPlusAngle();
    distance_to_goal = 0;
    geometry_msgs::Twist last_lidar_max_speed;
    last_lidar_max_speed.linear.x = 0;
    last_lidar_max_speed.linear.y = 0;
    last_lidar_max_speed.angular.z = 0;
    geometry_msgs::Twist last_goal_max_speed;
    last_goal_max_speed.linear.x = 0;
    last_goal_max_speed.linear.y = 0;
    last_goal_max_speed.angular.z = 0;
    speed_inhibition_from_obstacle = 0;
    motors_cmd_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 5);
    motors_enable_pub = n.advertise<std_msgs::Bool>("enable_motor", 5);
    current_pose_pub = n.advertise<geometry_msgs::Pose>("current_pose", 5);
    odom_pub = n.advertise<nav_msgs::Odometry>("odom", 5);
    chrono_pub = n.advertise<std_msgs::Duration>("remaining_time", 5);
    encoders_sub = n.subscribe("encoders", 1000, &Core::updateCurrentPose, this);
    odom_light_sub = n.subscribe("odom_light", 5, &Core::updateLightOdom, this);
    goal_sub = n.subscribe("goal_pose", 1000, &Core::updateGoal, this);
    odometry_sub = n.subscribe("odom_sub", 1000, &Core::updateOdometry, this);
    lidar_sub = n.subscribe("obstacle_pose_stamped", 1000, &Core::updateLidar, this);
    lidar_behind_sub
      = n.subscribe("obstacle_behind_pose_stamped", 1000, &Core::updateLidarBehind, this);
    tirette_sub = n.subscribe("tirette", 1000, &Core::updateTirette, this);
    reverse_gear_sub = n.subscribe("reverseGear", 1000, &Core::updateGear, this);

    n.param<bool>("isBlue", is_blue, true);

    if (is_blue)
    {
        std::cout << "Is Blue !" << std::endl;
    }
    else
    {
        std::cout << "Not Blue :'(" << std::endl;
    }

    goal_output[NB_NEURONS] = { 0. };
    obstacles_output[NB_NEURONS] = { 0. };
    lidar_output[NB_NEURONS] = { 0. };
    angular_speed_vector[NB_NEURONS] = { 0. };
    angular_landscape[NB_NEURONS] = { 0. };
    orienting = false;

    printf("done! Proceeding.\nStarting IA.\n");

    /**************************************
     *      Variable initialization       *
     **************************************/
    linear_speed = 0;
    angular_speed = 0;
    linear_speed_cmd = 0;

    starting_position = Position(200, 800, !is_blue);
    starting_X = starting_position.getX() / 1000.f;
    starting_Y = starting_position.getY() / 1000.f;
    X = starting_X;
    Y = starting_Y;
    if (is_blue)
    {
        theta_zero = 0.f;
    }
    else
    {
        theta_zero = 180.f;
    }

    std::cout << "Starting position: X = " << X << ", Y = " << Y << "Theta_zero = " << theta_zero
              << std::endl;

    last_encoder1 = last_encoder2 = 0;
}

void Core::stop_motors()
{
    set_motors_speed(0, 0, false, false);
}

void Core::set_motors_speed(float linearSpeed, float angularSpeed)
{
    set_motors_speed(linearSpeed, angularSpeed, true, false);
}

void Core::set_motors_speed(float linearSpeed,
                            float angularSpeed,
                            bool enable,
                            bool /*resetEncoders*/)
{
    geometry_msgs::Twist new_motor_cmd;
    new_motor_cmd.linear.x = reverseGear() ? -linearSpeed : linearSpeed;
    new_motor_cmd.angular.z = angularSpeed;

    motors_cmd_pub.publish(new_motor_cmd);
    std_msgs::Bool new_enable_cmd;
    new_enable_cmd.data = enable;
    motors_enable_pub.publish(new_enable_cmd);
}

bool Core::reverseGear()
{
    return m_reverse_gear_activated;
}

int Core::Setup()
{
    // Take time before entering the loop
    usleep(10000); // So the next now-last won't return 0

    stop_motors();

    return 0;
}

void Core::landscapeFromAngleAndStrength(std::vector<float> landscape, float angle, float strength)
{
    for (int i = 0; i < NB_NEURONS; i++)
    {
        landscape[i] = cos(angle - i) * strength;
    }
}

bool Core::digitalRead(int)
{
    return true;
}

Core::~Core()
{
    // We broke out of the loop, stop everything
    stop_motors();

    printf("Got out of the main loop, stopped everything.\n");
}

geometry_msgs::Pose Core::update_current_pose(int32_t encoder1, int32_t encoder2)
{
    float linear_dist = compute_linear_dist(encoder1, encoder2);
    current_theta = get_orientation_float(encoder1, encoder2) + theta_zero;

    X += linear_dist * cos(current_theta * M_PI / 180.f);
    Y += linear_dist * sin(current_theta * M_PI / 180.f);

    std::cout << "X = " << X << ", Y = " << Y << ", theta = " << current_theta
              << ",linear_dist = " << linear_dist << std::endl;

    current_position = Position(X * 1000, Y * 1000, false);

    geometry_msgs::Pose currentPose;
    currentPose.position.x = X;
    currentPose.position.y = Y;

    tf2::Quaternion orientation_quat;
    orientation_quat.setRPY(0, 0, current_theta * M_PI / 180.f);
    currentPose.orientation = tf2::toMsg(orientation_quat);
    return currentPose;
}

void Core::send_odometry(const geometry_msgs::Pose& currentPose)
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
    odom_broadcaster.sendTransform(odom_trans);

    nav_msgs::Odometry odom_msg;
    odom_msg.header.frame_id = "odom";
    odom_msg.header.stamp = ros::Time::now();
    odom_msg.child_frame_id = "base_link";

    for (unsigned int i = 0;
         i < (sizeof(odom_msg.pose.covariance) / sizeof(odom_msg.pose.covariance[0]));
         i++)
    {
        odom_msg.pose.covariance[i] = 0;
    }

    odom_msg.pose.pose = currentPose;

    odom_msg.pose.covariance[0] = 0.1;
    odom_msg.pose.covariance[7] = 0.1;
    odom_msg.pose.covariance[35] = 0.2;

    odom_msg.pose.covariance[14] = 0000; // set a non-zero covariance on unused
    odom_msg.pose.covariance[21] = 0000; // dimensions (z, pitch and roll); this
    odom_msg.pose.covariance[28] = 0000; // is a requirement of robot_pose_ekf
    // source:
    // https://github.com/yujinrobot/kobuki/blob/0.6.6/kobuki_node/src/library/odometry.cpp#L145-L154

    for (unsigned int i = 0;
         i < (sizeof(odom_msg.twist.covariance) / sizeof(odom_msg.twist.covariance[0]));
         i++)
    {
        odom_msg.twist.covariance[i] = 0;
    }

    odom_msg.twist.twist.linear.x = current_linear_speed;
    odom_msg.twist.twist.linear.y = 0;
    odom_msg.twist.twist.linear.z = 0;

    odom_msg.twist.twist.angular.x = 0;
    odom_msg.twist.twist.angular.y = 0;
    odom_msg.twist.twist.angular.z = -current_angular_speed * M_PI / 180;
    odom_pub.publish(odom_msg);
}

void Core::update_current_speed()
{
    ros::Time now = ros::Time::now();
    double time_since_last_speed_update = (last_speed_update_time - now).toSec();

    if (time_since_last_speed_update == 0)
    {
        std::cout << "Error: null time!" << std::endl;
    }

    float distance_moved = ((current_position - last_position).getNorme()) / 1000.f;

    last_position.setX(X * 1000);
    last_position.setY(Y * 1000);

    current_linear_speed = abs(distance_moved / time_since_last_speed_update);
    std::cout << "current_linear_speed = " << current_linear_speed << std::endl;

    current_angular_speed = (current_theta - last_theta) / time_since_last_speed_update;
    last_theta = current_theta;

    last_speed_update_time = now;
}

void Core::limit_linear_speed_cmd_by_goal()
{
    float max_acceleration = 0.15f; // m*s-2
    float max_deceleration = 0.15f; // m*s-2
    float new_speed_order = 0;      // m/s

    float desired_final_speed = 0; // m*s-2

    float time_to_stop = (current_linear_speed - desired_final_speed) / max_deceleration;
    std::cout << "time to stop = " << time_to_stop << "s, ";

    float distance_to_stop = time_to_stop * (current_linear_speed - desired_final_speed) / 2;
    std::cout << ", distance to stop = " << distance_to_stop << "m, ";

    // Compute extra time if accelerating
    float average_extra_speed
      = (2 * current_linear_speed + max_acceleration / 2 + max_deceleration / 2);
    float extra_distance = average_extra_speed / UPDATE_RATE;

    if (distance_to_goal < distance_to_stop)
    {
        std::cout << "decelerate";
        new_speed_order = current_linear_speed - max_deceleration / UPDATE_RATE;
    }
    else if (distance_to_goal < current_linear_speed / BROKER_FREQ)
    {
        std::cout << "EMERGENCY BRAKE";
        new_speed_order = 0;
    }
    else if (distance_to_goal > distance_to_stop + extra_distance)
    {
        std::cout << "accelerate";
        new_speed_order = current_linear_speed + max_acceleration / UPDATE_RATE;
    }
    else
    {
        std::cout << "cruise speed";
        new_speed_order = current_linear_speed;
    }
    // linear_speed_cmd = MIN(default_linear_speed, new_speed_order);
    linear_speed_cmd = new_speed_order;
    std::cout << "new speed: " << new_speed_order << " => " << linear_speed_cmd << std::endl;
}

int Core::Loop()
{
    if ((state != WAIT_TIRETTE) && is_time_to_stop())
    {
        std::cout << "Time's up !" << std::endl;
        return state;
    }

    if (state == WAIT_TIRETTE)
    {
        set_motors_speed(0, 0, false, false);
    }
    else if (state == NORMAL)
    {
        // Get the orientation we need to follow to reach the goal
        // TODO: assign priority integers to strategies, take the strategy that has the max of
        // priority * strength

        // compute_target_speed_orientation(orientation);

        if (distance_to_goal >= 0.05f)
        {
            orienting = false;
        }
        if (distance_to_goal > 0.02f && !orienting)
        {

            // orient towards the goal's position
            target_orientation = getAngleToGoal();
        }
        else
        {
            orienting = true;
            // respect the goal's own orientation
            target_orientation = goal_position.getAngle();
            std::cout << "########################################" << std::endl
                      << "Positionned, orienting to " << goal_position.getAngle() << std::endl
                      << "########################################" << std::endl;
        }

        if (reverseGear() && !orienting)
        {
            target_orientation += 180.f;
        }

        // Inhibit linear speed if there are obstacles

        // Compute attractive vectors from positive valence strategies
        // TODO: choose the POSITIVE VALENCE STRATEGY!
        for (int i = 0; i < NB_NEURONS; i += 1)
        {
            goal_output[i]
              = target(107.f, 1.1f, fmod(360 + 180 - (target_orientation - current_theta), 360), i);
        }

        std::cout << "relative_target_orientation: " << (target_orientation - current_theta)
                  << ", peak value: " << get_idx_of_max(goal_output, NB_NEURONS)
                  << ", central value = " << goal_output[180];

        // Sum positive and negative valence strategies
        for (int i = 0; i < NB_NEURONS; i += 1)
        {
            // Temporarily check if joystick is active (later: use weighted sum)
            // if (s_joystick.output->strength == 0) {
            angular_landscape[i] = goal_output[i]; // + lidar_output[i];
            /*}
            else {
                    printf("joystick is active\n");
                    fflush(stdout);
                    angular_landscape[i] = goal_output[i];
            }*/
        }

        // And finally: differentiate the angular_landscape vector to get drive
        differentiate(angular_landscape, angular_speed_vector, NB_NEURONS, 3000.);

        // Set linear speed according to the obstacles strategy & angular speed based on goal +
        // obstacles Robot's vision is now centered on 180 deg
        float angular_speed_cmd
          = -angular_speed_vector[180]; // - as positive is towards the left in ros, while the
                                        // derivation is left to right
        std::cout << ",angular_speed_cmd = " << angular_speed_cmd << std::endl;

        // linear_speed_cmd = default_linear_speed;

        limit_linear_speed_cmd_by_goal();

	if (reverseGear() && current_position.getY() < 250)
        {
            speed_inhibition_from_obstacle = 1000;
        }
        linear_speed_cmd
          = MIN(linear_speed_cmd, default_linear_speed * speed_inhibition_from_obstacle);

        limit_angular_speed_cmd(angular_speed_cmd);

        update_speed(FALSE, &angular_speed, angular_speed_cmd);
        // update_speed(FALSE, &linear_speed, linear_speed_cmd);
        linear_speed = linear_speed_cmd;

        if (DISABLE_LINEAR_SPEED || orienting)
        {
            linear_speed = 0;
        }

        if (DISABLE_ANGULAR_SPEED)
        {
            angular_speed = 0;
        }

        // linear_speed = 0;
        // Modulate linear speed by angular speed: stop going forward when you want to turn
        // linear_speed = MIN(linear_speed, linear_speed / abs(angular_speed));
        if (angular_speed < -1 || angular_speed > 1)
        {
            linear_speed = 0;
        }
        std::cout << "linear speed = " << linear_speed << ", orienting = " << orienting
                  << "speed inihib from obstacles = " << speed_inhibition_from_obstacle << " * "
                  << default_linear_speed << std::endl;

        // Set motors speed according to values computed before
        set_motors_speed(linear_speed, angular_speed / 20.f, true, false);
    } // End of state == NORMAL

    publish_remaining_time();

    return state;
}

void Core::publish_remaining_time()
{
    std_msgs::Duration remaining_time_msg;

    if (begin_match.toSec() < 1)
    {
        remaining_time_msg.data = ros::Duration(TIMEOUT_END_MATCH / 1000);
    }
    else
    {
        ros::Time end_of_match = ros::Time(begin_match.toSec() + TIMEOUT_END_MATCH / 1000);
        remaining_time_msg.data = end_of_match - ros::Time::now();
    }

    chrono_pub.publish(remaining_time_msg);
}

bool Core::is_time_to_stop()
{
    // Exit and stop robot if we reached the end of the match
    if (ENABLE_TIMEOUT_END_MATCH == TRUE
        && (ros::Time::now() - begin_match).toSec() * 1000 > TIMEOUT_END_MATCH)
    {
        state = EXIT;
        return true;
    }
    return false;
}

void Core::compute_target_speed_orientation(const unsigned int orientation)
{
    /*if (s_joystick.output->strength == 1) {
            target_orientation = get_idx_of_max(s_joystick.output->neural_field, NB_NEURONS);
            linear_speed_cmd = s_joystick.output->speed_inhibition;
            //printf("Target orientation from joystick: %d\n", target_orientation);
    } else if (s_goal.output->strength == 1) {
            target_orientation = get_idx_of_max(s_goal.output->neural_field, NB_NEURONS);
            //linear_speed_cmd = default_linear_speed;
            linear_speed_cmd = s_goal.output->speed_inhibition;
            //printf("Linear speed command from goal: %d\n", linear_speed_cmd);
            //printf("Target orientation from goal: %d\n", target_orientation);
    } else {
            /*
             * If no positive valence strategy fires, set the target orientation to the robot's
    orientation
             * which means the robot will go straight on
             */
    // target_orientation = orientation;
    linear_speed_cmd = default_linear_speed;
    //}
}

void Core::limit_angular_speed_cmd(float& angular_speed)
{
    // TODO: use the winning strategy with weights
    // TODO: restore speed limitations
    /*if (s_goal.output->strength == 1) {
            int speed_limited_by_obstacle = s_goal.output->angular_speed_inhibition *
    MAX_ALLOWED_ANGULAR_SPEED / 100; MIN(angular_speed, speed_limited_by_obstacle);
            MAX(angular_speed, -speed_limited_by_obstacle);
    }*/

    // Cap angular speed, so that the robot doesn't turn TOO FAST on itself
    MAX(angular_speed, -MAX_ALLOWED_ANGULAR_SPEED);
    MIN(angular_speed, MAX_ALLOWED_ANGULAR_SPEED);
}
