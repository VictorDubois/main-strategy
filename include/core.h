#pragma once
/*********************************************
 *                  BROKER                   *
 **********************************************/
#include "ros/ros.h"
#include <tf/transform_broadcaster.h>
#include <utility>
#include <vector>

#include "krabilib/positionPlusAngle.h"
#include "constants.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Vector3.h"
#include "helpers_broker.h"
#include "nav_msgs/Odometry.h"
#include <krabi_msgs/motors.h>
#include <krabi_msgs/odom_light.h>
#include <std_msgs/Bool.h>

// Should remove this and put it in a global, "constant.h" file for the whole project
#define NB_NEURONS 360

#define DISABLE_LINEAR_SPEED false
#define DISABLE_ANGULAR_SPEED false

#define UPDATE_RATE 10

#define ENABLE_TIMEOUT_END_MATCH true
#define TIMEOUT_END_MATCH 99000 // in ms


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

    Core();
    ~Core();
    int Setup();
    State Loop();



private:
    void selectColor();

    /**
     * Convert a cartesian position to a polar one
     * @param posX the X position, in mm
     * @param posY the Y position, in mm
     * @param theta the angle, in degrees
     * @param posX the distance, in meters
     **/
    //#define DEBUG_cartToPolar
    void cartToPolar(int posX, int posY, float& theta, float& distance);

    // Update encoders + sanitize their inputs
    void updateEncoders(long& encoder1, long& encoder2);
    void updateOdometry(nav_msgs::Odometry odometry);

    // Has the tirette been pulled?
    bool areWeGoForLaunch();

    // Update time + detect end of match
    // https://www.youtube.com/watch?v=2k0SmqbBIpQ
    bool isTimeToStop();

    // Maintain loop frequency @BROKER_FREQ Hz
    void maintainLoopTiming();

    // Set target speed and orientation
    void computeTargetSpeedOrientation(const unsigned int orientation);

    // Enforce limits on angular speed: absolute max and obstacle inhibition
    void limitAngularSpeedCmd(float& m_angular_speed);

    void sendOdometry(const geometry_msgs::Pose& current_pose);

    // Publish the time remaining on the match's clock
    void publishRemainingTime();


    void stopMotors();
    void setMotorsSpeed(float linearSpeed, float angularSpeed, bool enable, bool resetEncoders);
    void setMotorsSpeed(float linearSpeed, float angularSpeed);
    void updateCurrentPose(krabi_msgs::encoders motors_state);
    void updateLightOdom(krabi_msgs::odom_light motors_state);
    void landscapeFromAngleAndStrength(std::vector<float> landscape, float angle, float strength);
    static float vector_to_angle(geometry_msgs::Vector3 vector);
    static float vector_to_angle(geometry_msgs::Point vector);
    static float vector_to_amplitude(geometry_msgs::Vector3 vector);
    static float vector_to_amplitude(geometry_msgs::Point vector);
    void updateGoal(geometry_msgs::PoseStamped goal_pose);
    void updateLidar(geometry_msgs::PoseStamped closest_obstacle);
    void updateLidarBehind(geometry_msgs::PoseStamped closest_obstacle);
    void addObstacle(float obstacle_distance, float closest_obstacle_angle);
    void updateTirette(std_msgs::Bool starting);
    void updateGear(std_msgs::Bool a_reverse_gear_activated);
    bool digitalRead(int);
    geometry_msgs::Pose updateCurrentPose(int32_t encoder1, int32_t encoder2);

    bool reverseGear();

    /**
     * @brief getGoalAngle returns the relative angle in degres to the goal
     * @return float the angle
     */
    float getAngleToGoal();

    void updateCurrentSpeed();
    void limitLinearSpeedCmdByGoal();

    // State of the broker loop
    State m_state = State::WAIT_TIRETTE;

    const float m_default_linear_speed
      = 0.5f; // Line speed to set when no positive valence strategy fires
    float m_target_orientation;
    float m_linear_speed, m_angular_speed, m_linear_speed_cmd;
    long m_encoder1, m_encoder2, m_last_encoder1, m_last_encoder2, m_elapsed;
    float m_goal_output[NB_NEURONS] = { 0. };
    float m_obstacles_output[NB_NEURONS] = { 0. };
    float m_lidar_output[NB_NEURONS] = { 0. };
    float m_angular_speed_vector[NB_NEURONS] = { 0. };
    float m_angular_landscape[NB_NEURONS] = { 0. };
    ros::Time m_begin_match;
    bool m_is_blue;
    ros::Publisher m_motors_cmd_pub;
    ros::Publisher m_motors_enable_pub;
    ros::Publisher m_current_pose_pub;
    ros::Publisher m_odom_pub;
    ros::Publisher m_chrono_pub;
    ros::Subscriber m_encoders_sub;
    ros::Subscriber m_odometry_sub;
    ros::Subscriber m_goal_sub;
    ros::Subscriber m_lidar_sub;
    ros::Subscriber m_lidar_behind_sub;
    ros::Subscriber m_tirette_sub;
    ros::Subscriber m_reverse_gear_sub;
    ros::Subscriber m_odom_light_sub;
    tf::TransformBroadcaster m_odom_broadcaster;
    float m_X;
    float m_starting_X;
    float m_Y;
    float m_starting_Y;
    float m_theta_zero;
    float m_current_theta;
    float m_last_theta;
    int32_t m_starting_encoder1;
    int32_t m_starting_encoder2;
    bool m_encoders_initialized;
    Position m_current_position;
    PositionPlusAngle m_goal_position;
    Position m_last_position;
    Position m_starting_position;
    float m_distance_to_goal;
    float m_current_linear_speed;
    float m_current_angular_speed;
    ros::Time m_last_speed_update_time;
    bool m_orienting;
    bool m_reverse_gear_activated;
    float m_speed_inhibition_from_obstacle;
};
