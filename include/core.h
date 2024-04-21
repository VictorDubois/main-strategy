#pragma once
/*********************************************
 *                  BROKER                   *
 **********************************************/
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/buffer.h"

#include <thread>
#include <utility>
#include <vector>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <krabi_msgs/msg/motors.hpp>
#include <krabi_msgs/msg/motors_parameters.hpp>
#include <krabi_msgs/msg/motors_distance_asserv.hpp>
#include <krabi_msgs/msg//odom_light.hpp>
#include <krabi_msgs/msg/strat_movement.hpp>
#include <krabi_msgs/msg/motors_cmd.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/bool.hpp>

#include "constants.h"
#include "helpers_broker.h"
#include "krabilib/pose.h"

// Should remove this and put it in a global, "constant.h" file for the whole project
#define NB_NEURONS 360

#define DISABLE_LINEAR_SPEED false
#define DISABLE_ANGULAR_SPEED false

#define UPDATE_RATE 10

#define ENABLE_TIMEOUT_END_MATCH true
#define TIMEOUT_END_MATCH 97e3 // in ms

unsigned int angle_to_neuron_id(Angle a);

class Core: public rclcpp::Node
{

public:
    // States ENUM for the main loop (m_state machine)
    enum class State
    {
        EXIT,
        INIT_ODOM_TODO,
        WAIT_TIRETTE,
        NORMAL
    };

    Core();
    ~Core();
    int Setup();
    State Loop();
    bool isOver();

private:
    void selectColor();

    void updateStratMovement(krabi_msgs::msg::StratMovement move);

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
    rclcpp::Time m_begin_match = rclcpp::Time(0, 0);
    Pose m_goal_pose;
    geometry_msgs::msg::PoseStamped m_goal_pose_stamped;
    Distance m_distance_to_goal;
    bool m_reverse_gear_activated;
    float m_speed_inhibition_from_obstacle;
    krabi_msgs::msg::StratMovement m_strat_movement_parameters;

    // ROS Params
    bool m_is_blue;

    // Publishers
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
    rclcpp::Subscription<krabi_msgs::msg::StratMovement>::SharedPtr m_strat_movement_sub;

    // Transform
    std::shared_ptr<tf2_ros::TransformListener> m_tf_listener_{nullptr};
    std::unique_ptr<tf2_ros::Buffer> m_tf_buffer_;
    //tf2_ros::TransformBroadcaster m_tf_broadcaster;
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

    // Buffer
    krabi_msgs::msg::StratMovement m_buffer_strat_movement_parameters;
    Pose m_buffer_goal_pose;
    geometry_msgs::msg::PoseStamped m_buffer_goal_pose_stamped;
    Angle m_previous_angle_to_goal;
    Pose m_previous_goal_pose;

    rclcpp::Time m_end_init_odo;
    rclcpp::TimerBase::SharedPtr timer_;
};
