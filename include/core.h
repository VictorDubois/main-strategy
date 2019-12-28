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
#define BROKER_FREQ 50 //(in Hz)

#define FALSE 0
#define TRUE 1

#define DISABLE_LINEAR_SPEED FALSE
#define DISABLE_ANGULAR_SPEED FALSE

#define ENABLE_TIMEOUT_END_MATCH TRUE
#define TIMEOUT_END_MATCH 99000 // in ms

#define TIRETTE_ENABLED TRUE
//#define PIN_TIRETTE 16
//#define PIN_COLOR 26
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
	
	void setupGPIO();
	
	void select_color();
	
	/*
		Hardware setup of the tirette
	*/
	void setupTiretteRPI();
	
	/*
		Hardware setup of the color selection
	*/
	void setupColorRPI();
	
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

	// Updates "state" when the tirette is pulled
	void wait_for_tirette();
	
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
	unsigned int target_orientation, nb_attractors;
	int mode, linear_speed, angular_speed, theta, linear_speed_cmd;
	int status;
	char cwd[1024];
	long encoder1, encoder2, last_encoder1, last_encoder2, elapsed, chrono;
	float integration_field[NB_NEURONS] = {0.};
	float goal_output[NB_NEURONS] = {0.};
	float obstacles_output[NB_NEURONS] = {0.};
	float lidar_output[NB_NEURONS] = {0.};
	float angular_speed_vector[NB_NEURONS] = {0.};
	float angular_landscape[NB_NEURONS] = {0.};
	struct timespec now, last, begin_match;
	bool is_tirette_msg_displayed = false;
	int is_blue;
	ros::Publisher motors_cmd_pub;
	ros::Subscriber encoders_sub;

	void stop_motors();
	void set_motors_speed(float linearSpeed, float angularSpeed, bool enable, bool resetEncoders);
	void set_motors_speed(float linearSpeed, float angularSpeed);
	void updateCurrentPose(goal_strategy::motors motors_state);
public:
	Core();
	~Core();
	int Setup(int argc, char *argv[]);
	int Loop();	
};