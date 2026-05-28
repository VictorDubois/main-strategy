#pragma once
/**
 * Main motion controller for the Krabi robot (ROS 2 node).
 *
 * Motion control uses a Dynamic Neural Field (DNF) approach (see https://theses.fr/2017CERG0898)
 *   - The full 360° heading space is discretised into NB_NEURONS bins (1°/bin).
 *   - An attractive potential (m_goal_output) is built around the desired heading using a
 *     log-cosh "hill" function (see target() in helpers_broker.c).
 *   - A repulsive potential (m_lidar_output) is built around the bearing of the nearest
 *     obstacle using a Gaussian bump (see gaussian() in helpers_broker.c). Currently deactivated
 *   - Both are summed into m_angular_landscape and differentiated (see differentiate()).
 *   - The derivative at the robot's own heading (neuron 0) gives the angular speed command:
 *     positive gradient → turn left, negative → turn right.
 *
 * The main loop runs at UPDATE_RATE Hz, driven by a ROS wall timer.
 * Topics and parameters are described in README.md.
 */
#include "rclcpp/node.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"

#include <thread>
#include <utility>
#include <vector>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <krabi_msgs/msg/motion_debug.hpp>
#include <krabi_msgs/msg/motors.hpp>
#include <krabi_msgs/msg/motors_cmd.hpp>
#include <krabi_msgs/msg/motors_distance_asserv.hpp>
#include <krabi_msgs/msg/motors_parameters.hpp>
#include <krabi_msgs/msg/odom_light.hpp>
#include <krabi_msgs/msg/strat_movement.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/bool.hpp>

#include "helpers_broker.h"
#include "krabilib/pose.h"

#include <diagnostic_updater/diagnostic_updater.hpp>

// Should remove this and put it in a global, "constant.h" file for the whole project
#define NB_NEURONS 360 // one neuron per degree of heading

#define DISABLE_LINEAR_SPEED false  // set true to freeze forward motion (debug)
#define DISABLE_ANGULAR_SPEED false // set true to freeze rotation (debug)

#define UPDATE_RATE                                                                                \
    10 // Hz — main loop frequency. @todo: increase to 20, 50, 100Hz => need to adapt
       // limitLinearSpeedCmdByGoal

// Eurobot matches last 100s, the last 15s are for the PAMIs. 84 s gives 1 s of safety margin to
// avoid the robot moving after the match has ended.
#define ENABLE_TIMEOUT_END_MATCH true
#define TIMEOUT_END_MATCH 84e3 // in ms

unsigned int angle_to_neuron_id(Angle a);

class Core : public rclcpp::Node
{

public:
    // States for the main loop state machine.
    // Normal progression: INIT_ODOM_TODO → WAIT_TIRETTE → NORMAL → EXIT
    enum class State
    {
        EXIT,           // match over (timeout), motors stopped permanently
        INIT_ODOM_TODO, // briefly resets encoders on startup so ICP/EKF can initialise
        WAIT_TIRETTE,   // waiting for the start signal (tirette pulled = match begins)
        NORMAL          // match running, full DNF motion control active
    };

    Core();
    ~Core();
    int Setup();
    State Loop();
    bool isOver();
    void produce_diagnostics(diagnostic_updater::DiagnosticStatusWrapper& stat);

private:
    void updateStratMovement(krabi_msgs::msg::StratMovement move);

    // Has the tirette been pulled?
    bool areWeGoForLaunch();

    // Update time + detect end of match
    // https://www.youtube.com/watch?v=2k0SmqbBIpQ
    bool isTimeToStop();

    // Maintain loop frequency @BROKER_FREQ Hz
    void maintainLoopTiming();

    // Set target speed and orientation
    void computeTargetSpeedOrientation(); // was superseded by limitLinearSpeedCmdByGoal()
    // and is no longer called — @todo delete both the declaration and the definition in core.cpp

    // Enforce limits on angular speed: absolute max and obstacle inhibition
    void limitAngularSpeedCmd(VitesseAngulaire& a_angular_speed);

    // Publish the time remaining on the match's clock
    void publishRemainingTime();

    void stopMotors();
    void brake();
    void setMotorsSpeed(Vitesse linearSpeed,
                        VitesseAngulaire angularSpeed,
                        bool enable,
                        bool resetEncoders);
    void setMotorsSpeed(Vitesse linearSpeed, VitesseAngulaire angularSpeed);
    void updateGoal(geometry_msgs::msg::PoseStamped goal_pose);
    void updateLidar(std::shared_ptr<geometry_msgs::msg::PoseStamped const> closest_obstacle,
                     bool front);
    void addObstacle(PolarPosition obstacle);
    void updateTirette(std_msgs::msg::Bool starting);
    void updateGear(std_msgs::msg::Bool a_reverse_gear_activated);
    void updateOdom(const nav_msgs::msg::Odometry& odometry);
    void updateCurrentPose();

    bool reverseGear();
    bool orienting();
    bool allowFineTuningLinear();
    bool recalage_bordure();
    bool clamp_mode();
    bool stop_angular();
    void chooseReverseGear(Angle diff_angle);

    double getReach(const std::string& end_point_frame_id);

    void create_publishers();
    void create_subscribers();
    void create_aruco_subscribers();

    /**
     * @brief getGoalAngle returns the relative angle in degres to the goal
     * @return float the angle
     */
    Angle getAngleToGoal();

    void updateCurrentSpeed();
    void limitLinearSpeedCmdByGoal();
    void fineTunePositionPID();
    void limitAcceleration();
    void limitLinearSpeedByAngularSpeed(VitesseAngulaire angular_speed);

    void plotAll();
    std::thread m_running;

    State m_state = State::INIT_ODOM_TODO;
    const Vitesse m_default_linear_speed = Vitesse(0.5f); // cap when no obstacle
    Angle m_target_orientation;
    Vitesse m_linear_speed, m_linear_speed_cmd;
    VitesseAngulaire m_angular_speed, m_angular_speed_cmd;

    // DNF neuron arrays — one float per degree of heading (index 0 = robot's current heading)
    float m_goal_output[NB_NEURONS] = { 0. };          // attractive potential toward goal bearing
    float m_lidar_output[NB_NEURONS] = { 0. };         // repulsive potential from nearest obstacle
    float m_angular_speed_vector[NB_NEURONS] = { 0. }; // gradient of m_angular_landscape
    float m_angular_landscape[NB_NEURONS] = { 0. };    // combined potential (goal + lidar)
    rclcpp::Time m_begin_match = rclcpp::Time(0, 0);
    Pose m_goal_pose;
    geometry_msgs::msg::PoseStamped m_goal_pose_stamped;
    Distance m_distance_to_goal;
    bool m_reverse_gear_activated;
    float m_speed_inhibition_from_obstacle;
    krabi_msgs::msg::StratMovement m_strat_movement_parameters;
    bool m_reverse_gear = false;
    bool m_fine_tuning_linear = false;

    // Position PID state — only active during m_fine_tuning_linear (within 2 cm of goal)
    float m_pid_position_integral = 0.0f;
    float m_pid_position_prev_error = 0.0f;

    // ROS Params
    bool m_is_blue;
    float m_maxAccel;
    AccelerationAngulaire m_maxAngularAccel = AccelerationAngulaire(3.0); // rad/s²
    JerkAngulaire m_maxAngularJerk = JerkAngulaire(30.0);                 // rad/s³

    // Tracked angular acceleration (for jerk limiting)
    AccelerationAngulaire m_angular_accel = AccelerationAngulaire(0.0);
    float m_tuning_spread;
    float m_tuning_offset;
    float m_max_current;

    // Publishers
    rclcpp::Publisher<krabi_msgs::msg::MotionDebug>::SharedPtr m_motion_debug_pub;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr m_motors_cmd_pub;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr m_motors_cmd_slash_pub;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr m_motors_enable_pub;
    rclcpp::Publisher<krabi_msgs::msg::MotorsParameters>::SharedPtr m_motors_parameters_pub;
    rclcpp::Publisher<krabi_msgs::msg::MotorsCmd>::SharedPtr m_motors_pwm_pub;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr m_target_orientation_pub;
    rclcpp::Publisher<builtin_interfaces::msg::Duration>::SharedPtr m_chrono_pub;
    rclcpp::Publisher<krabi_msgs::msg::MotorsDistanceAsserv>::SharedPtr m_distance_asserv_pub;

    // Subscribers
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr m_lidar_sub;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr m_lidar_behind_sub;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr m_tirette_sub;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr m_odometry_sub;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr m_odometry_slash_sub;
    rclcpp::Subscription<krabi_msgs::msg::StratMovement>::SharedPtr m_strat_movement_sub;

    // Transform
    std::shared_ptr<tf2_ros::TransformListener> m_tf_listener_{ nullptr };
    std::unique_ptr<tf2_ros::Buffer> m_tf_buffer_;
    // tf2_ros::TransformBroadcaster m_tf_broadcaster;
    std::unique_ptr<tf2_ros::TransformBroadcaster> m_tf_broadcaster;
    Transform m_map_to_baselink;
    Transform m_baselink_to_map;
    Pose m_current_pose;

    // Arcuo
    void updateAruco(std::shared_ptr<geometry_msgs::msg::PoseStamped const> arucoPose, int id);
    void publishTf(const geometry_msgs::msg::Pose& pose,
                   const std::string& frame_id,
                   const std::string& child_frame_id);
    std::map<int, rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr> m_arucos_sub;
    std::array<geometry_msgs::msg::PoseStamped, 10> m_arucos;

    // Kept for the abandoned buffering approach in updateStratMovement() — @todo delete
    krabi_msgs::msg::StratMovement m_buffer_strat_movement_parameters;
    Pose m_buffer_goal_pose;
    geometry_msgs::msg::PoseStamped m_buffer_goal_pose_stamped;
    Angle m_previous_angle_to_goal;
    Pose m_previous_goal_pose;

    rclcpp::Time m_end_init_odo;
    rclcpp::TimerBase::SharedPtr timer_;

    // Motion debug message accumulator (populated across multiple functions, published each loop)
    krabi_msgs::msg::MotionDebug m_motion_debug_msg;

    // Diagnostic
    bool m_transform_found = false;
    bool m_reach_transform_found = false;
    bool m_stopped_by_obstacle = false;
    bool m_stopped_by_goalstrat = false;
    bool m_match_ended = false;
};
