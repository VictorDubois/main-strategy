#include "core.h"
#include "ros/ros.h"
#include <std_msgs/Duration.h>
#include <stdexcept>

#define MAX_ALLOWED_ANGULAR_SPEED 5.f // rad/s

#include "lidarStrat.h"
#define UPDATE_RATE 10

#include <krabi_msgs/motors_cmd.h>
#include <krabi_msgs/motors_parameters.h>

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>

#ifdef PLOT_NEURONS
#include "matplotlib-cpp/matplotlibcpp.h"
namespace plt = matplotlibcpp;
#endif

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

    // std::cout << "updateCurrentPose: " << m_current_pose << std::endl;//Raspi4 doesn't like
    // <<transform std::cout << "Transform matrix [baselink to map]: " << m_baselink_to_map;//Raspi4
    // doesn't like <<transform
}

Angle Core::getAngleToGoal()
{
    return (m_goal_pose.getPosition() - m_current_pose.getPosition()).getAngle();
}

void Core::updateGoal(geometry_msgs::PoseStamped goal_pose)
{
    // m_goal_pose = Pose(goal_pose.pose);
    // ROS_DEBUG_STREAM("New goal: " << m_goal_pose);
}

void Core::updateTirette(std_msgs::Bool starting)
{
    if (starting.data && (m_state == State::WAIT_TIRETTE || m_state == State::INIT_ODOM_TODO))
    {
        ROS_INFO_STREAM("Go for launch!");
        m_state = State::NORMAL;

        // Start counting the time
        m_begin_match = ros::Time::now();
    }
}

void Core::updateLidar(boost::shared_ptr<geometry_msgs::PoseStamped const> closest_obstacle,
                       bool front)
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
                               : AngleTools::wrapAngle(Angle(obstacle.getAngle() + M_PI));

    m_speed_inhibition_from_obstacle
      = LidarStrat::speed_inhibition(obstacle.getDistance(), normalized_angle, 1);

    if (m_speed_inhibition_from_obstacle < 0.2f)
    {
        m_speed_inhibition_from_obstacle = 0.f;
    }

    /*ROS_INFO_STREAM("Speed inhib from obstacle = " << m_speed_inhibition_from_obstacle
                                                   << ". Obstacle @" << obstacle << std::endl);*/

    float obstacle_distance_clean = obstacle.getDistance();
    if (obstacle_distance_clean < 0.01f)
    {
        obstacle_distance_clean = 0.01f;
    }

    float averaving_factor = 0.8f;
    for (int j = 0; j < NB_NEURONS; j += 1)
    {
        float l_sigma = 30; // °  
        float l_strength = 10.0 / obstacle_distance_clean; 

        m_lidar_output[j]
          = averaving_factor * m_lidar_output[j]
            - (1 - averaving_factor)
                * gaussian(l_sigma, l_strength, closest_obstacle_id, j);
    }
}

void Core::updateGear(std_msgs::Bool a_reverse_gear_activated)
{
    m_reverse_gear_activated = a_reverse_gear_activated.data == 1;
}

void Core::updateStratMovement(krabi_msgs::strat_movement move)
{
    m_strat_movement_parameters = move;
    m_goal_pose = Pose(move.goal_pose.pose);
    m_goal_pose_stamped = move.goal_pose;
    return;

    if((m_buffer_strat_movement_parameters.max_speed.linear.x == 0 && m_strat_movement_parameters.max_speed.linear.x != 0) ||
        (m_buffer_strat_movement_parameters.max_speed.angular.z == 0 && m_strat_movement_parameters.max_speed.angular.z != 0) ||
        m_buffer_strat_movement_parameters.goal_pose.pose != m_strat_movement_parameters.goal_pose.pose ||
        m_buffer_strat_movement_parameters.orient != m_strat_movement_parameters.orient ||
        m_buffer_strat_movement_parameters.reverse_gear != m_strat_movement_parameters.reverse_gear)
    {
        stopMotors();
        m_strat_movement_parameters.max_speed.angular.z = 0;
        m_strat_movement_parameters.max_speed.linear.x = 0;
    }
    //    ROS_DEBUG_STREAM("New goal: " << m_goal_pose);

    m_buffer_strat_movement_parameters = m_strat_movement_parameters;
    m_buffer_goal_pose = m_goal_pose;
    m_buffer_goal_pose_stamped = m_goal_pose_stamped;
}

Core::Core(ros::NodeHandle& nh)
  : m_tf_listener(m_tf_buffer)
  , m_nh(nh)
{
    m_previous_angle_to_goal = Angle (0);
    m_previous_goal_pose = Pose();

    m_goal_pose = Pose();
    m_distance_to_goal = 0;

    m_speed_inhibition_from_obstacle = 1;
    m_reverse_gear_activated = false;

    m_nh.param<bool>("isBlue", m_is_blue, true);
    ROS_INFO_STREAM(m_is_blue ? "Is Blue !" : "Not Blue :'(");

    m_motors_cmd_pub = m_nh.advertise<geometry_msgs::Twist>("cmd_vel", 5);
    m_motors_enable_pub = m_nh.advertise<std_msgs::Bool>("enable_motor", 5);
    m_motors_parameters_pub = m_nh.advertise<krabi_msgs::motors_parameters>("motors_parameters", 5);
    m_motors_pwm_pub = m_nh.advertise<krabi_msgs::motors_cmd>("motors_cmd", 5);
    m_target_orientation_pub = m_nh.advertise<geometry_msgs::PoseStamped>("target_orientation", 5);

    m_chrono_pub = m_nh.advertise<std_msgs::Duration>("/remaining_time", 5);
    m_distance_asserv_pub
      = m_nh.advertise<krabi_msgs::motors_distance_asserv>("motors_distance_asserv", 5);
    // m_goal_sub = m_nh.subscribe("goal_pose", 1000, &Core::updateGoal, this);

    m_lidar_sub = m_nh.subscribe<geometry_msgs::PoseStamped>(
      "obstacle_pose_stamped", 5, boost::bind(&Core::updateLidar, this, _1, true));

    m_lidar_behind_sub = m_nh.subscribe<geometry_msgs::PoseStamped>(
      "obstacle_behind_pose_stamped", 5, boost::bind(&Core::updateLidar, this, _1, false));
    m_tirette_sub = m_nh.subscribe("tirette", 1, &Core::updateTirette, this);
    m_odometry_sub = m_nh.subscribe("odom", 1000, &Core::updateOdom, this);
    m_strat_movement_sub = m_nh.subscribe("strat_movement", 5, &Core::updateStratMovement, this);

    m_running = std::thread(&Core::plotAll, this);

    // m_reverse_gear_sub = m_nh.subscribe("reverseGear", 1000, &Core::updateGear, this);

    if (!m_is_blue)
    {
        m_arucos_sub[6] = nh.subscribe<geometry_msgs::PoseStamped>(
          "/pose_robots/6", 5, boost::bind(&Core::updateAruco, this, _1, 6));
        m_arucos_sub[7] = nh.subscribe<geometry_msgs::PoseStamped>(
          "/pose_robots/7", 5, boost::bind(&Core::updateAruco, this, _1, 7));
        m_arucos_sub[8] = nh.subscribe<geometry_msgs::PoseStamped>(
          "/pose_robots/8", 5, boost::bind(&Core::updateAruco, this, _1, 8));
        m_arucos_sub[9] = nh.subscribe<geometry_msgs::PoseStamped>(
          "/pose_robots/9", 5, boost::bind(&Core::updateAruco, this, _1, 9));
        m_arucos_sub[10] = nh.subscribe<geometry_msgs::PoseStamped>(
          "/pose_robots/10", 5, boost::bind(&Core::updateAruco, this, _1, 10));
    }
    else
    {
        m_arucos_sub[1] = nh.subscribe<geometry_msgs::PoseStamped>(
          "/pose_robots/1", 5, boost::bind(&Core::updateAruco, this, _1, 1));
        m_arucos_sub[2] = nh.subscribe<geometry_msgs::PoseStamped>(
          "/pose_robots/2", 5, boost::bind(&Core::updateAruco, this, _1, 2));
        m_arucos_sub[3] = nh.subscribe<geometry_msgs::PoseStamped>(
          "/pose_robots/3", 5, boost::bind(&Core::updateAruco, this, _1, 3));
        m_arucos_sub[4] = nh.subscribe<geometry_msgs::PoseStamped>(
          "/pose_robots/4", 5, boost::bind(&Core::updateAruco, this, _1, 4));
        m_arucos_sub[5] = nh.subscribe<geometry_msgs::PoseStamped>(
          "/pose_robots/5", 5, boost::bind(&Core::updateAruco, this, _1, 5));
    }

    m_goal_output[NB_NEURONS] = { 0. };
    m_obstacles_output[NB_NEURONS] = { 0. };
    m_lidar_output[NB_NEURONS] = { 0. };
    m_angular_speed_vector[NB_NEURONS] = { 0. };
    m_angular_landscape[NB_NEURONS] = { 0. };

    ROS_INFO_STREAM("Init done! Proceeding.\nStarting IA.\n");

    /**************************************
     *      Variable initialization       *
     **************************************/
    m_linear_speed = 0;
    m_angular_speed = 0;
    m_linear_speed_cmd = 0;
    m_angular_speed_cmd = 0;

    m_end_init_odo = ros::Time::now() + ros::Duration(2.5);
}

void Core::updateOdom(const nav_msgs::Odometry& odometry)
{
    m_linear_speed
      = tf::Vector3(odometry.twist.twist.linear.x, odometry.twist.twist.linear.y, 0).length();
    m_angular_speed = odometry.twist.twist.angular.z;
    // Loop();
}

bool Core::isOver()
{
    return m_state == Core::State::EXIT;
}

void Core::stopMotors()
{
    setMotorsSpeed(Vitesse(0), VitesseAngulaire(0), false, false);
}
void Core::brake()
{
    setMotorsSpeed(Vitesse(0), VitesseAngulaire(0), true, false);
}

void Core::setMotorsSpeed(Vitesse linearSpeed, VitesseAngulaire angularSpeed)
{
    setMotorsSpeed(linearSpeed, angularSpeed, true, false);
}

double Core::getReach(const std::string& end_point_frame_id)
{
    if (end_point_frame_id == "")
    {
        return Distance(0);
    }
    try
    {
        auto base_link_id = tf::resolve(ros::this_node::getNamespace(), "base_link");
        const geometry_msgs::Transform& transform
          = m_tf_buffer.lookupTransform(end_point_frame_id, base_link_id, ros::Time(0)).transform;

        return static_cast<double>(sqrt(transform.translation.x * transform.translation.x
                                        + transform.translation.y * transform.translation.y
                                        + transform.translation.z * transform.translation.z));
    }
    catch (tf2::TransformException& ex)
    {
        ROS_WARN("Error while getting reach: %s", ex.what());

        return Distance(0);
    }
}

void Core::setMotorsSpeed(Vitesse linearSpeed,
                          VitesseAngulaire angularSpeed,
                          bool enable,
                          bool resetEncoders)
{
    geometry_msgs::Twist new_motor_cmd;
    new_motor_cmd.linear.x = reverseGear() ? -linearSpeed : linearSpeed;
    new_motor_cmd.angular.z = angularSpeed;

    krabi_msgs::motors_distance_asserv l_distance_asserv_msg;
    l_distance_asserv_msg.use_distance_asserv = true;
    l_distance_asserv_msg.goal_pose = geometry_msgs::PoseStamped();

    const auto l_odom_id = tf::resolve(ros::this_node::getNamespace(), "odom");
    const auto l_map_id = "map";

    std::string l_error_message = "no error";
    try
    {
        if (m_tf_buffer.canTransform(l_map_id, l_odom_id, ros::Time(0), &l_error_message))
        {

            geometry_msgs::TransformStamped l_map_to_odom
              = m_tf_buffer.lookupTransform(l_map_id, l_odom_id, ros::Time(0));
            geometry_msgs::PoseStamped l_goal_pose_in_odom;
            tf2::doTransform(m_goal_pose_stamped, l_goal_pose_in_odom, l_map_to_odom);
            l_distance_asserv_msg.goal_pose = l_goal_pose_in_odom;
            // ROS_WARN_STREAM("goal in map : " << m_goal_pose_stamped.pose.position.x << ", " <<
            // m_goal_pose_stamped.pose.position.y); ROS_WARN_STREAM("goal in odom : " <<
            // l_goal_pose_in_odom.pose.position.x << ", " << l_goal_pose_in_odom.pose.position.y);
        }
        else
        {
            ROS_WARN_STREAM("CanTransform said: Unable to find a transform from "
                            << l_map_id << " to " << l_odom_id
                            << " (=map to odom), disabling distance asserv: " << l_error_message);
            l_distance_asserv_msg.use_distance_asserv = false;
        }
    }
    catch (tf2::LookupException)
    {
        ROS_WARN("Unable to find a transform from map to odom, disabling distance asserv");
        l_distance_asserv_msg.use_distance_asserv = false;
    }

    l_distance_asserv_msg.max_speed_at_arrival = m_strat_movement_parameters.max_speed_at_arrival;

    l_distance_asserv_msg.reach = getReach(m_strat_movement_parameters.endpoint_frame_id);

    m_distance_asserv_pub.publish(l_distance_asserv_msg);
    m_motors_cmd_pub.publish(new_motor_cmd);
    std_msgs::Bool new_enable_cmd;
    new_enable_cmd.data = enable;
    m_motors_enable_pub.publish(new_enable_cmd);


    geometry_msgs::PoseStamped l_target_orientation_msg;
    //l_target_orientation_msg.header.stamp = ros::Time::now();
    l_target_orientation_msg.header.frame_id = "/map";
    l_target_orientation_msg.pose = m_current_pose;
    //l_distance_asserv_msg.goal_pose.pose;
    tf2::Quaternion myQuaternion;
    myQuaternion.setRPY( 0, 0, m_target_orientation );
    myQuaternion.normalize();
    //tf2::convert(l_target_orientation_msg.pose.orientation, myQuaternion);
    l_target_orientation_msg.pose.orientation = tf2::toMsg(myQuaternion);
    l_target_orientation_msg.header.stamp = ros::Time::now();
    m_target_orientation_pub.publish(l_target_orientation_msg);

    krabi_msgs::motors_cmd new_motors_pwm_cmd;
    krabi_msgs::motors_parameters new_parameters;
    new_parameters.max_current = 0.7f;
    new_parameters.max_current_left = 2;
    new_parameters.max_current_right = 2;
    new_motors_pwm_cmd.enable_motors = enable;
    new_motors_pwm_cmd.reset_encoders = resetEncoders;
    new_motors_pwm_cmd.override_PWM = false;
    new_motors_pwm_cmd.PWM_override_left = 0;
    new_motors_pwm_cmd.PWM_override_right = 0;

    if (recalage_bordure())
    {
        new_parameters.max_current = 2.5f;
        new_parameters.max_current_left = 0.4f;
        new_parameters.max_current_right = 0.4f;
        new_motors_pwm_cmd.enable_motors = enable;
        new_motors_pwm_cmd.override_PWM = true;
        new_motors_pwm_cmd.PWM_override_left = 20;
        new_motors_pwm_cmd.PWM_override_right = 20;
        if (reverseGear())
        {
            new_motors_pwm_cmd.PWM_override_left = -new_motors_pwm_cmd.PWM_override_left;
            new_motors_pwm_cmd.PWM_override_right = -new_motors_pwm_cmd.PWM_override_right;
        }
    }

    if (clamp_mode())
    {
        new_parameters.max_current = 2.1f;
        new_parameters.max_current_left = 3;
        new_parameters.max_current_right = 3;
    }

    m_motors_parameters_pub.publish(new_parameters);

    m_motors_pwm_pub.publish(new_motors_pwm_cmd);
}

bool Core::reverseGear()
{
    return m_strat_movement_parameters.reverse_gear == 1;
}

bool Core::orienting()
{
    return m_strat_movement_parameters.orient == 1;
}

bool Core::stop_angular()
{
    return m_strat_movement_parameters.orient == 4;
}

bool Core::recalage_bordure()
{
    return m_strat_movement_parameters.orient == 2;
}

bool Core::clamp_mode()
{
    return m_strat_movement_parameters.orient == 3;
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

void Core::limitLinearSpeedByAngularSpeed(VitesseAngulaire a_angular_speed)
{
    VitesseAngulaire l_sigma_angular_speed = VitesseAngulaire(0.1f); // rad/s
    float l_scale = 1.f / l_sigma_angular_speed;                     // 0.4f;
    // So that gaussian(0) = 1
    Vitesse linear_speed_limit
      = m_default_linear_speed * gaussian(l_sigma_angular_speed, l_scale, 0, a_angular_speed);

    m_linear_speed_cmd = std::min(m_linear_speed_cmd, linear_speed_limit);
    ROS_INFO_STREAM("limit linear by angular speed : " << linear_speed_limit << std::endl);
}

void Core::plotAll()
{
    while (m_state != State::EXIT)
    {
	usleep(100000);
        std::vector<float> l_angular_landscape;
        std::vector<float> l_goal_output;
        std::vector<float> l_lidar_output;
        for (int i = 0; i < NB_NEURONS; i += 1)
        {
            l_angular_landscape.push_back(m_angular_landscape[i]);
            l_goal_output.push_back(m_goal_output[i]);
            l_lidar_output.push_back(m_lidar_output[i]);
        }

#ifdef PLOT_NEURONS
        plt::clf();
        plt::plot(l_angular_landscape);
        plt::plot(l_goal_output);
        plt::plot(l_lidar_output);
        plt::draw();
        plt::pause(0.1);
#endif
    }
#ifdef PLOT_NEURONS
    plt::close();
#endif
}

Core::State Core::Loop()
{
    publishRemainingTime();

    if ((m_state != State::WAIT_TIRETTE && m_state != State::INIT_ODOM_TODO) && isTimeToStop())
    {
        ROS_INFO_STREAM("Time's up !");
        return m_state;
    }

    if (m_state == State::INIT_ODOM_TODO)
    {
        // Init odom once, to allow moving for ICP/EKF init
        setMotorsSpeed(Vitesse(0), VitesseAngulaire(0), false, true);
        if (m_end_init_odo < ros::Time::now())
        {
            m_state = State::WAIT_TIRETTE;
        }
    }
    else if (m_state == State::WAIT_TIRETTE)
    {
        setMotorsSpeed(Vitesse(0), VitesseAngulaire(0), false, false);
    }
    else if (m_state == State::NORMAL)
    {
        updateCurrentPose();
        m_distance_to_goal = (m_goal_pose.getPosition() - m_current_pose.getPosition()).getNorme();

        if (!orienting())
        {
            // orient towards the goal's position
            m_target_orientation = getAngleToGoal();

            // update the goal to 0 when reaching a goal => doesn't do much, is overridden quickly :/
            // Remaining issue: 90° turns => more than 90° margin? => ça a l'air de marcher :) => en fait, il reste des sauts de moins de 3/4 PI => done mais désactivé
            // @todo, check if very close to target + orientation jump (from the previous iteration) => done, mais juste en arrêtant le robot. Pas compatible avec un asserv en position
            // reacting to a goal update? => reserve the new strat_mvnt for the next iteration + limit speed right away. Currently it is possible to lack the synchronisation with reverse gear, for instance. Done

            // Manage position overshoots: do not turn around if the position has been overshoot by 5mm!
            Distance l_too_close_threshold = Distance(0.1f);
            Distance l_reach_goal_dist = Distance(0.02f);
            /*auto l_delta_orientation = AngleTools::diffAngle(m_target_orientation, m_current_pose.getAngle());

            if (m_distance_to_goal < l_too_close_threshold && ((abs(l_delta_orientation) > 3.f*M_PI/4.f) != reverseGear()))
            {
                m_target_orientation = AngleTools::wrapAngle(Angle(m_target_orientation + M_PI));
            }*/

            auto l_delta_orientation = AngleTools::diffAngle(m_target_orientation, m_previous_angle_to_goal);
            auto l_delta_position = (m_goal_pose.getPosition() - m_previous_goal_pose.getPosition()).getNorme();

            //if (m_distance_to_goal < l_too_close_threshold && l_delta_position < Distance(0.01) && (abs(l_delta_orientation) > M_PI/2.f))
            if (m_distance_to_goal < l_reach_goal_dist )
            {
                //m_target_orientation = AngleTools::wrapAngle(Angle(m_target_orientation + M_PI));
                //setMotorsSpeed(Vitesse(0), Vitesse(0), true, false);
                m_target_orientation = m_current_pose.getAngle();
                brake();
                return m_state;
            }

            m_previous_angle_to_goal = getAngleToGoal();
            m_previous_goal_pose = m_goal_pose;
            
        }
        else
        {
            // respect the goal's own orientation
            m_target_orientation = m_goal_pose.getAngle();

            ROS_DEBUG_STREAM("########################################"
                             << std::endl
                             << "Positionned, m_orienting to " << m_goal_pose.getAngle()
                             << std::endl
                             << "########################################" << std::endl);
        }

        if (reverseGear())
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

        /*ROS_INFO_STREAM("relative_target_orientation: "
                        << delta_orientation
                        << ", peak value: " << get_idx_of_max(m_goal_output, NB_NEURONS)
                        << ", central value = " <<
           m_goal_output[angle_to_neuron_id(Angle(0))]);*/

        // Sum positive and negative valence strategies

        for (int i = 0; i < NB_NEURONS; i += 1)
        {
            m_angular_landscape[i] = m_goal_output[i];
                  //+m_lidar_output[i];
        }

        // And finally: differentiate the m_angular_landscape vector to get drive
        differentiate(m_angular_landscape, m_angular_speed_vector, NB_NEURONS, 3000.);

        // Set linear speed according to the obstacles strategy & angular speed based on goal +
        // obstacles Robot's vision is now centered on 180 deg
        m_angular_speed_cmd = VitesseAngulaire(m_angular_speed_vector[angle_to_neuron_id(
          Angle(0))]); // - as positive is towards the left in ros, while the
                       // derivation is left to right
        // ROS_DEBUG_STREAM(",m_angular_speed_cmd = " << m_angular_speed_cmd << std::endl);

        limitLinearSpeedCmdByGoal();

        m_linear_speed_cmd = std::min(
          m_linear_speed_cmd, Vitesse(m_default_linear_speed * m_speed_inhibition_from_obstacle));

        limitAngularSpeedCmd(m_angular_speed_cmd);

        limitAcceleration();

        if (DISABLE_LINEAR_SPEED || orienting())
        {
            m_linear_speed_cmd = 0;
            ROS_INFO_STREAM("linear speed disabled");
        }

        if (DISABLE_ANGULAR_SPEED || stop_angular())
        {
            m_angular_speed_cmd = 0;
            ROS_INFO_STREAM("angular speed disabled");
        }

        // Modulate linear speed by angular speed: stop going forward when you want to turn
        limitLinearSpeedByAngularSpeed(m_angular_speed_cmd);

        ROS_INFO_STREAM("linear speed = "
                        << m_linear_speed << ", m_orienting = " << orienting()
                        << ", speed inihib from obstacles = " << m_speed_inhibition_from_obstacle
                        << " * " << m_default_linear_speed
                        << ", angular_speed_cmd = " << m_angular_speed_cmd
                        << ", linear_speed_cmd = " << m_linear_speed_cmd << std::endl);

        // Set motors speed according to values computed before
        setMotorsSpeed(m_linear_speed_cmd, m_angular_speed_cmd, true, false);

        
    } // End of m_state == State::NORMAL

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
      m_angular_speed, m_angular_speed_cmd, AccelerationAngulaire(0.00025), 1. /
    float(UPDATE_RATE)); ROS_DEBUG_STREAM("After: linear_speed: " << m_linear_speed_cmd << "m/s
    , angular_speed: "
                                             << m_angular_speed_cmd << "rad/s");
                                             **/
}

void Core::publishRemainingTime()
{
    std_msgs::Duration remaining_time_msg;
    remaining_time_msg.data = ros::Duration(TIMEOUT_END_MATCH / 1000);

    if (m_begin_match)
    {
        auto remaining_time
          = ros::Duration(TIMEOUT_END_MATCH / 1000 - (ros::Time::now() - *m_begin_match).toSec());
        remaining_time_msg.data = remaining_time;
    }

    m_chrono_pub.publish(remaining_time_msg);
}

bool Core::isTimeToStop()
{
    // State::EXIT and stop robot if we reached the end of the match
    if (ENABLE_TIMEOUT_END_MATCH == TRUE
        && (ros::Time::now() - *m_begin_match).toSec() * 1000 > TIMEOUT_END_MATCH)
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

void Core::limitAngularSpeedCmd(VitesseAngulaire& a_angular_speed_cmd)
{
    // TODO: use the winning strategy with weights
    // TODO: restore speed limitations
    /*if (s_goal.output->strength == 1) {
            int speed_limited_by_obstacle = s_goal.output->m_angular_speed_inhibition *
    MAX_ALLOWED_ANGULAR_SPEED / 100; MIN(m_angular_speed, speed_limited_by_obstacle);
            MAX(m_angular_speed, -speed_limited_by_obstacle);
    }*/

    // Cap angular speed, so that the robot doesn't turn TOO FAST on itself
    a_angular_speed_cmd
      = std::max(a_angular_speed_cmd, VitesseAngulaire(-MAX_ALLOWED_ANGULAR_SPEED));
    a_angular_speed_cmd
      = std::min(a_angular_speed_cmd, VitesseAngulaire(MAX_ALLOWED_ANGULAR_SPEED));
    a_angular_speed_cmd = std::max(
      a_angular_speed_cmd, VitesseAngulaire(-m_strat_movement_parameters.max_speed.angular.z));
    a_angular_speed_cmd = std::min(
      a_angular_speed_cmd, VitesseAngulaire(m_strat_movement_parameters.max_speed.angular.z));
}

// Limit linear speed, to match the desired speed when reaching the goal
void Core::limitLinearSpeedCmdByGoal()
{
    Acceleration max_acceleration = Acceleration(0.35f); // m*s-2
    Acceleration max_deceleration = Acceleration(0.35f); // m*s-2
    if (m_strat_movement_parameters.max_speed_at_arrival > 0.01f)
    {
        max_acceleration = Acceleration(0.65f); // m*s-2
        max_deceleration = Acceleration(0.45f); // m*s-2
    }

    Vitesse new_speed_order = Vitesse(0); // m/s

    Vitesse desired_final_speed
      = Vitesse(m_strat_movement_parameters.max_speed_at_arrival); // m*s-2

    float time_to_stop = (m_linear_speed - desired_final_speed) / max_deceleration;
    time_to_stop = std::max(0.f, time_to_stop);
    ROS_INFO_STREAM("time to stop = " << time_to_stop << "s, ");

    Distance distance_to_stop
      = Distance(time_to_stop * (m_linear_speed - desired_final_speed) / 2.);
    ROS_INFO_STREAM(", distance to stop = " << distance_to_stop << "m, ");

    // Compute extra time if accelerating
    Vitesse average_extra_speed = Vitesse(
      2 * m_linear_speed + Vitesse((max_acceleration / 2. + max_deceleration / 2.) / UPDATE_RATE));
    Distance extra_distance = Distance(average_extra_speed / float(UPDATE_RATE));

    Distance l_distance_to_goal
      = Distance(m_distance_to_goal - getReach(m_strat_movement_parameters.endpoint_frame_id));

    if (l_distance_to_goal < distance_to_stop)
    {
        ROS_INFO_STREAM("decelerate");
        new_speed_order = m_linear_speed - max_deceleration / float(UPDATE_RATE);
    }
    else if (l_distance_to_goal < m_linear_speed / float(UPDATE_RATE))
    {
        ROS_INFO_STREAM("EMERGENCY BRAKE");
        new_speed_order = 0;
    }
    else if (l_distance_to_goal > distance_to_stop + extra_distance)
    {
        ROS_INFO_STREAM("accelerate");
        new_speed_order = m_linear_speed + max_acceleration / float(UPDATE_RATE);
    }
    else
    {
        ROS_INFO_STREAM("cruise speed");
        new_speed_order = m_linear_speed;
    }
    new_speed_order
      = std::max(new_speed_order, Vitesse(-m_strat_movement_parameters.max_speed.linear.x));
    new_speed_order
      = std::min(new_speed_order, Vitesse(m_strat_movement_parameters.max_speed.linear.x));
    // m_linear_speed_cmd = MIN(m_default_linear_speed, new_speed_order);
    ROS_INFO_STREAM("new speed: " << new_speed_order << " => " << m_linear_speed_cmd << std::endl);
    m_linear_speed_cmd = new_speed_order;
}

unsigned int angle_to_neuron_id(Angle a)
{
    return (unsigned int)(((AngleTools::wrapAngle(a) + M_PI) / (2 * M_PI)) * NB_NEURONS);
}

void Core::publishTf(const geometry_msgs::Pose& pose,
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

void Core::updateAruco(boost::shared_ptr<geometry_msgs::PoseStamped const> arucoPose, int id)
{
    m_arucos[id] = *arucoPose;

    publishTf(arucoPose->pose, "/aruco", "/aruco_raw_pose");
    auto base_link_id = tf::resolve(ros::this_node::getNamespace(), "base_link");
    auto deltaOdom = m_tf_buffer.lookupTransform(base_link_id,
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
