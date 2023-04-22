#pragma once
/*********************************************
 *                  BROKER                   *
 **********************************************/
#include "ros/ros.h"
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf2_ros/transform_listener.h>

#include <thread>
#include <utility>
#include <vector>

#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>
#include <krabi_msgs/motors.h>
#include <krabi_msgs/motors_distance_asserv.h>
#include <krabi_msgs/odom_light.h>
#include <krabi_msgs/strat_movement.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Bool.h>

#include "constants.h"
#include "helpers_broker.h"
#include "krabilib/pose.h"

// Should remove this and put it in a global, "constant.h" file for the whole project
#define NB_NEURONS 360

#define DISABLE_LINEAR_SPEED false
#define DISABLE_ANGULAR_SPEED false

#define UPDATE_RATE 10

#define ENABLE_TIMEOUT_END_MATCH true
#define TIMEOUT_END_MATCH 99e3 // in ms

unsigned int angle_to_neuron_id(Angle a);

class Core
{

public:
    // States ENUM for the main loop (m_state machine)
    enum class State
    {
        EXIT,
        WAIT_TIRETTE,
        NORMAL
    };

    Core(ros::NodeHandle& nh);
    ~Core();
    int Setup();
    State Loop();
    bool isOver();

private:
    void selectColor();

    void updateStratMovement(krabi_msgs::strat_movement move);

    // Has the tirette been pulled?
    bool areWeGoForLaunch();

    // Update time + detect end of match
    // https://www.youtube.com/watch?v=2k0SmqbBIpQ
    bool isTimeToStop();

    // Maintain loop frequency @BROKER_FREQ Hz
    void maintainLoopTiming();

    // Set target speed and orientation
    void computeTargetSpeedOrientation();

    // Enforce limits on angular speed: absolute max and obstacle inhibition
    void limitAngularSpeedCmd(VitesseAngulaire& a_angular_speed);

    // Publish the time remaining on the match's clock
    void publishRemainingTime();

    void stopMotors();
    void setMotorsSpeed(Vitesse linearSpeed,
                        VitesseAngulaire angularSpeed,
                        bool enable,
                        bool resetEncoders);
    void setMotorsSpeed(Vitesse linearSpeed, VitesseAngulaire angularSpeed);
    void updateGoal(geometry_msgs::PoseStamped goal_pose);
    void updateLidar(boost::shared_ptr<geometry_msgs::PoseStamped const> closest_obstacle,
                     bool front);
    void addObstacle(PolarPosition obstacle);
    void updateTirette(std_msgs::Bool starting);
    void updateGear(std_msgs::Bool a_reverse_gear_activated);
    void updateOdom(const nav_msgs::Odometry& odometry);
    void updateCurrentPose();

    bool reverseGear();
    bool orienting();
    bool recalage_bordure();
    bool clamp_mode();
    bool stop_angular();

    double getReach(const std::string& end_point_frame_id);

    /**
     * @brief getGoalAngle returns the relative angle in degres to the goal
     * @return float the angle
     */
    Angle getAngleToGoal();

    void updateCurrentSpeed();
    void limitLinearSpeedCmdByGoal();
    void limitAcceleration();
    void limitLinearSpeedByAngularSpeed(VitesseAngulaire angular_speed);

    void plotAll();
    std::thread m_running;

    // State of the broker loop
    State m_state = State::WAIT_TIRETTE;
    // Line speed to set when no positive valence strategy fires
    const Vitesse m_default_linear_speed = Vitesse(0.5f);
    Angle m_target_orientation;
    Vitesse m_linear_speed, m_linear_speed_cmd;
    VitesseAngulaire m_angular_speed, m_angular_speed_cmd;
    // long  m_elapsed;
    float m_goal_output[NB_NEURONS] = { 0. };
    float m_obstacles_output[NB_NEURONS] = { 0. };
    float m_lidar_output[NB_NEURONS] = { 0. };
    float m_angular_speed_vector[NB_NEURONS] = { 0. };
    float m_angular_landscape[NB_NEURONS] = { 0. };
    boost::optional<ros::Time> m_begin_match;
    Pose m_goal_pose;
    geometry_msgs::PoseStamped m_goal_pose_stamped;
    Distance m_distance_to_goal;
    bool m_reverse_gear_activated;
    float m_speed_inhibition_from_obstacle;
    krabi_msgs::strat_movement m_strat_movement_parameters;

    // ROS Params
    bool m_is_blue;

    // Publisher
    ros::Publisher m_motors_cmd_pub;
    ros::Publisher m_motors_enable_pub;
    ros::Publisher m_motors_parameters_pub;
    ros::Publisher m_motors_pwm_pub;
    ros::Publisher m_chrono_pub;
    ros::Publisher m_distance_asserv_pub;
    ros::Publisher m_target_orientation_pub;

    // Subscriber
    ros::Subscriber m_odometry_sub;
    ros::Subscriber m_goal_sub;
    ros::Subscriber m_lidar_sub;
    ros::Subscriber m_lidar_behind_sub;
    ros::Subscriber m_tirette_sub;
    ros::Subscriber m_reverse_gear_sub;
    ros::Subscriber m_strat_movement_sub;

    // Transform
    tf2_ros::Buffer m_tf_buffer;
    tf2_ros::TransformListener m_tf_listener;
    tf::TransformBroadcaster m_tf_broadcaster;
    Transform m_map_to_baselink;
    Transform m_baselink_to_map;
    Pose m_current_pose;

    ros::NodeHandle m_nh;

    // Arcuo
    void updateAruco(boost::shared_ptr<geometry_msgs::PoseStamped const> arucoPose, int id);
    void publishTf(const geometry_msgs::Pose& pose,
                   const std::string& frame_id,
                   const std::string& child_frame_id);
    std::map<int, ros::Subscriber> m_arucos_sub;
    std::array<geometry_msgs::PoseStamped, 10> m_arucos;

    // Buffer
    krabi_msgs::strat_movement m_buffer_strat_movement_parameters;
    Pose m_buffer_goal_pose;
    geometry_msgs::PoseStamped m_buffer_goal_pose_stamped;
    Angle m_previous_angle_to_goal;
    Pose m_previous_goal_pose;

};
