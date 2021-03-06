/*********************************************
 *                  BROKER                   *
 **********************************************/

#define STANDALONE_STRATEGIE 1
#include "ros/ros.h"
#include <tf/transform_broadcaster.h>
#include <utility>
#include <vector>

//#define RASPI
#ifdef RASPI
extern "C"
{
#include <wiringPi.h>
}
#endif
#include "Krabi/positionPlusAngle.h"
#include "constants.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Vector3.h"
#include "helpers_broker.h"
#include "nav_msgs/Odometry.h"
//#include "../../goal_strategy/motors.h"
#include <goal_strategy/motors.h>
#include <std_msgs/Bool.h>

/*#include "ct_scan.h"
//#include "getters_setters.h"

#include "constantes.h"
#include "dijkstra.h"
#include "etape.h"
#include "goal.h"
#include "goldo2018.h"
#include "joystick.h"
#include "lidarStrat.h"
#include "obstacles.h"
#include "positionPlusAngle.h"
#include "strategies.h"
*/
// Should remove this and put it in a global, "constant.h" file for the whole project
#define NB_NEURONS 360

// Whether to print the robot's final position or not
#define PRINT_FINAL_POS 1

// Define the frequency of the broker's main loop
#define BROKER_FREQ 10 //(in Hz)

#define FALSE 0
#define TRUE 1

#define DISABLE_LINEAR_SPEED FALSE
#define DISABLE_ANGULAR_SPEED FALSE

#define ENABLE_TIMEOUT_END_MATCH TRUE
#define TIMEOUT_END_MATCH 99000 // in ms

#define TIRETTE_ENABLED TRUE
#define PIN_TIRETTE 20
#define PIN_COLOR 21

/*
        Handles first CTRL + C by changing the state machine's state to EXIT
        and thus allow clean termination.
        It also arms the second signal handler to brutally exit if the user
        hits CTRL + C a second time.
*/

class Core
{
private:
    // States ENUM for the main loop (state machine)
    enum State
    {
        EXIT,
        WAIT_TIRETTE,
        NORMAL
    };

    // State of the broker loop
    enum State state = WAIT_TIRETTE;

    void select_color();

    /**
     * Convert a cartesian position to a polar one
     * @param posX the X position, in mm
     * @param posY the Y position, in mm
     * @param theta the angle, in degrees
     * @param posX the distance, in meters
     **/
    //#define DEBUG_cart_to_polar
    void cart_to_polar(int posX, int posY, float& theta, float& distance);

    // Update encoders + sanitize their inputs
    void update_encoders(long& encoder1, long& encoder2);
    void updateOdometry(nav_msgs::Odometry odometry);

    // Has the tirette been pulled?
    bool are_we_go_for_launch();

    // Update time + detect end of match
    // https://www.youtube.com/watch?v=2k0SmqbBIpQ
    bool is_time_to_stop();

    // Maintain loop frequency @BROKER_FREQ Hz
    void maintain_loop_timing();

    // Set target speed and orientation
    void compute_target_speed_orientation(const unsigned int orientation);

    // Enforce limits on angular speed: absolute max and obstacle inhibition
    void limit_angular_speed_cmd(float& angular_speed);

    void send_odometry(const geometry_msgs::Pose& current_pose);

    // Publish the time remaining on the match's clock
    void publish_remaining_time();

    const float default_linear_speed
      = 0.2f; // Line speed to set when no positive valence strategy fires
    float target_orientation;
    float linear_speed, angular_speed, linear_speed_cmd;
    long encoder1, encoder2, last_encoder1, last_encoder2, elapsed;
    float goal_output[NB_NEURONS] = { 0. };
    float obstacles_output[NB_NEURONS] = { 0. };
    float lidar_output[NB_NEURONS] = { 0. };
    float angular_speed_vector[NB_NEURONS] = { 0. };
    float angular_landscape[NB_NEURONS] = { 0. };
    ros::Time begin_match;
    bool is_blue;
    ros::Publisher motors_cmd_pub;
    ros::Publisher motors_enable_pub;
    ros::Publisher current_pose_pub;
    ros::Publisher odom_pub;
    ros::Publisher chrono_pub;
    ros::Subscriber encoders_sub;
    ros::Subscriber odometry_sub;
    ros::Subscriber goal_sub;
    ros::Subscriber lidar_sub;
    ros::Subscriber lidar_behind_sub;
    ros::Subscriber tirette_sub;
    ros::Subscriber reverse_gear_sub;
    tf::TransformBroadcaster odom_broadcaster;
    float X;
    float Y;
    float theta_zero;
    float current_theta;
    float last_theta;
    int32_t starting_encoder1;
    int32_t starting_encoder2;
    bool encoders_initialized;
    Position current_position;
    PositionPlusAngle goal_position;
    Position last_position;
    Position starting_position;
    float distance_to_goal;
    float current_linear_speed;
    float current_angular_speed;
    ros::Time last_speed_update_time;
    bool orienting;
    bool m_reverse_gear_activated;

    void stop_motors();
    void set_motors_speed(float linearSpeed, float angularSpeed, bool enable, bool resetEncoders);
    void set_motors_speed(float linearSpeed, float angularSpeed);
    void updateCurrentPose(goal_strategy::encoders motors_state);
    void landscapeFromAngleAndStrength(std::vector<float> landscape, float angle, float strength);
    float vector_to_angle(geometry_msgs::Vector3 vector);
    float vector_to_angle(geometry_msgs::Point vector);
    float vector_to_amplitude(geometry_msgs::Vector3 vector);
    float vector_to_amplitude(geometry_msgs::Point vector);
    void updateGoal(geometry_msgs::PoseStamped goal_pose);
    void updateLidar(geometry_msgs::PoseStamped closest_obstacle);
    void updateLidarBehind(geometry_msgs::PoseStamped closest_obstacle);
    void addObstacle(float obstacle_distance, float closest_obstacle_angle);
    void updateTirette(std_msgs::Bool starting);
    void updateGear(std_msgs::Bool a_reverse_gear_activated);
    bool digitalRead(int);
    geometry_msgs::Pose update_current_pose(int32_t encoder1, int32_t encoder2);

    bool reverseGear();

    /**
     * @brief getGoalAngle returns the relative angle in degres to the goal
     * @return float the angle
     */
    float getAngleToGoal();

    void update_current_speed();
    void limit_linear_speed_cmd_by_goal();

    float speed_inhibition_from_obstacle;

public:
    Core();
    ~Core();
    int Setup();
    int Loop();
};
