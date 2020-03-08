/*********************************************
 *                  BROKER                   *
**********************************************/

#define STANDALONE_STRATEGIE 1
#include "ros/ros.h"
#include <vector>
#include <utility>
#include <math.h>
#include <pthread.h>
#include <signal.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ipc.h>
#include <sys/shm.h>
#include <sys/types.h>
#include <time.h>
#include <unistd.h>
#include <sys/mman.h>
#include <sys/stat.h>        /* For mode constants */
#include <fcntl.h>

//#define RASPI
#ifdef RASPI
extern "C" {
#include <wiringPi.h>
}
#endif
#include "constants.h"
#include "helpers_broker.h"
#include <goal_strategy/motors.h>
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Twist.h"
#include <std_msgs/Bool.h>
#include "Krabi/positionPlusAngle.h"

/*#include "ct_scan.h"
//#include "getters_setters.h"

#include "goal.h"
#include "joystick.h"
#include "obstacles.h"
#include "lidarStrat.h"
#include "strategies.h"
#include "etape.h"
#include "dijkstra.h"
#include "goldo2018.h"
#include "positionPlusAngle.h"
#include "constantes.h"
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



class Core {
private:
	// States ENUM for the main loop (state machine)
	enum State {
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

	// Has the tirette been pulled?
	bool are_we_go_for_launch();
	
	// Update time + detect end of match
	// https://www.youtube.com/watch?v=2k0SmqbBIpQ
	bool is_time_to_stop();

	// Maintain loop frequency @BROKER_FREQ Hz
	void maintain_loop_timing();
	
	// Compute elapsed time since beginning of match in ms
	long compute_match_chrono();

	// Set target speed and orientation
	void compute_target_speed_orientation(const unsigned int orientation);

	// Enforce limits on angular speed: absolute max and obstacle inhibition
	void limit_angular_speed_cmd(int& angular_speed);
	
    const int default_linear_speed = 20; // Line speed to set when no positive valence strategy fires
	unsigned int nb_attractors;
	int target_orientation;
	int mode, linear_speed, angular_speed, linear_speed_cmd;
	int status;
	char cwd[1024];
	long encoder1, encoder2, last_encoder1, last_encoder2, elapsed, chrono;
	float goal_output[NB_NEURONS] = {0.};
	float obstacles_output[NB_NEURONS] = {0.};
	float lidar_output[NB_NEURONS] = {0.};
	float angular_speed_vector[NB_NEURONS] = {0.};
	float angular_landscape[NB_NEURONS] = {0.};
	struct timespec now, last, begin_match;
	bool is_tirette_msg_displayed = false;
	int is_blue;
	ros::Publisher motors_cmd_pub;
	ros::Publisher motors_enable_pub;
    ros::Publisher current_pose_pub;
	ros::Subscriber encoders_sub;
	ros::Subscriber goal_sub;
	ros::Subscriber lidar_sub;
	ros::Subscriber tirette_sub;
	ros::Subscriber color_sub;
	int32_t last_distance;
	float X;
	float Y;
	float theta_zero;
	float current_theta;
    int32_t starting_encoder1;
    int32_t starting_encoder2;
    bool encoders_initialized;
    Position current_position;
    Position goal_position;
    Position last_position;
    float distance_to_goal;
    float current_linear_speed;
    ros::Time last_speed_update_time;

	void stop_motors();
	void set_motors_speed(float linearSpeed, float angularSpeed, bool enable, bool resetEncoders);
	void set_motors_speed(float linearSpeed, float angularSpeed);
	void updateCurrentPose(goal_strategy::encoders motors_state);
	void landscapeFromAngleAndStrength(std::vector<float> landscape, float angle, float strength);
	float vector_to_angle(geometry_msgs::Vector3 vector);
	float vector_to_angle(geometry_msgs::Point vector);
	float vector_to_amplitude(geometry_msgs::Vector3 vector);
	float vector_to_amplitude(geometry_msgs::Point vector);
    void updateGoal(geometry_msgs::Pose goal_pose);
	void updateLidar(geometry_msgs::Vector3 closest_obstacle);
	void updateTeamColor(std_msgs::Bool new_color);
	void updateTirette(std_msgs::Bool starting);
	bool digitalRead(int);
	void update_current_pose(int32_t encoder1, int32_t encoder2);
    void updateRelativeGoal();
    void update_current_speed();
    void limit_linear_speed_cmd_by_goal();

public:
	Core();
	~Core();
	int Setup(int argc, char *argv[]);
	int Loop();	
};
