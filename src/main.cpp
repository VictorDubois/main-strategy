#include "core.h"
#include "ros/ros.h"
#include <stdexcept>
#define MAX_ALLOWED_ANGULAR_SPEED 0.2

#ifndef MAX
#define MIN(a,b) (((a)<(b))?(a):(b))
#define MAX(a,b) (((a)>(b))?(a):(b))
#endif

#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Twist.h"
#include <goal_strategy/motors_cmd.h>

void Core::setupGPIO() {
#ifdef RASPI
	if (wiringPiSetupGpio() == -1) {
		fprintf (stderr, "WiringPI setup failed :'(\n");
		printf ("WiringPI setup failed :'(\n");
		fflush(stdout);
		exit(2);
		//return;// 1;
	}
#endif
}

/*
	Hardware setup of the tirette
*/
void Core::setupTiretteRPI() {
#ifdef RASPI
	printf("Setting up the tirette for the Raspi\n");
	fflush(stdout);

	pinMode(PIN_TIRETTE, INPUT);

	pullUpDnControl(PIN_TIRETTE, PUD_UP);
#endif // RASPI
}

/*
	Hardware setup of the color selection
*/
void Core::setupColorRPI() {
#ifdef RASPI
	printf("Setting up the color switch for the Raspi\n");
	fflush(stdout);

	pinMode(PIN_COLOR, INPUT);

	pullUpDnControl(PIN_COLOR, PUD_UP);
#endif // RASPI
}

/**
 * Convert a cartesian position to a polar one
 * @param posX the X position, in mm
 * @param posY the Y position, in mm
 * @param theta the angle, in degrees
 * @param posX the distance, in meters
 **/
//#define DEBUG_cart_to_polar
void Core::cart_to_polar(int posX, int posY, float& theta, float& distance) {
	if(posY == 0) {
		posY = 1;
	}
	theta = ((180./M_PI) * atan(((float)posX)/(float)posY));
	distance = sqrt((float)(posX * posX + posY * posY))/1000.f;

	// fix angular ambiguity
	if (posY < 0) {
		theta += 180;
	}

	#ifdef DEBUG_cart_to_polar
		std::cout << "posX = " << posX << "posY = " <<  posY<< "theta = " << theta << ", distance = " << distance << std::endl;
		int posXafter =(int)1000* distance*cos(theta * M_PI/180.f);
        	int posYafter =(int)1000* distance*sin(theta * M_PI/180.f);
	        std::cout << "posXafter = " << posXafter << ", posYafter = " << posYafter << std::endl;
	#endif
}


int main (int argc, char *argv[]) {
	ros::init(argc, argv, "mainStrat");
	Core* my_core = new Core();
	my_core->Setup(argc, argv);

	while(my_core->Loop() && ros::ok()) {
		ros::spinOnce();	
	}

	delete my_core;
}

void Core::select_color() {
// Color selection
	if (digitalRead(PIN_COLOR) == 1) {
		is_blue = 1; // BLUE
	}
	else {
		is_blue = 0;
	}

	if (is_blue == 1)
		printf("Starting match as BLUE\n");
	else
		printf("Starting match as YELLOW\n");
	fflush(stdout);
}
void Core::updateCurrentPose(goal_strategy::motors motors_state) {
	encoder1 = motors_state.encoders.encoder_right;
	encoder2 = motors_state.encoders.encoder_right;

	// low pass filter
	update_encoders(encoder1, encoder2);

}
Core::Core() {
	ros::NodeHandle n;
	motors_cmd_pub = n.advertise<geometry_msgs::Pose>("motors_cmd", 1000);
	encoders_sub = n.subscribe("encoders", 1000, &Core::updateCurrentPose, this);
	integration_field[NB_NEURONS] = {0.};
	goal_output[NB_NEURONS] = {0.};
	obstacles_output[NB_NEURONS] = {0.};
	lidar_output[NB_NEURONS] = {0.};
	angular_speed_vector[NB_NEURONS] = {0.};
	angular_landscape[NB_NEURONS] = {0.};
	is_tirette_msg_displayed = false;

	//@TODO Dont't start if arduino is running
	printf("done! Proceeding.\nStarting IA.\n");
	//@TODO Reset encoders (for now, the robot starts at its origin point)

	/**************************************
	 *      Variable initialization       *
	 **************************************/
	linear_speed = 0; // motor command for linear speed, in percentage (from -100 to 100)
	angular_speed = 0; // motor command for angular speed, in percentage (from -100 to 100)
	linear_speed_cmd = 0;

	select_color();

	std::vector<std::pair<int, int> > positionsCart;
	std::vector<std::pair<float, float> > positionsPolar;

	// first: longueur (vers l'autre couleur)
	// second: profondeur (vers le publique)
	std::pair<int, int> positionDepart = std::make_pair(300, 450);
	int angleDepart = -90;//degrees
	positionsCart.push_back(std::make_pair(800, 450));//OK          out of red
	positionsCart.push_back(std::make_pair(800, 750));//OK          out of green
	positionsCart.push_back(std::make_pair(300, 750));//OK          mid green
	positionsCart.push_back(std::make_pair(1700, 450));//           out of top of accel
	positionsCart.push_back(std::make_pair(1700, 200));//           in front of top of accel
	if (is_blue) { //BLUE
		positionsCart.push_back(std::make_pair(2250, 450));//            out of goldenium
		positionsCart.push_back(std::make_pair(2250, 200));//            in front of goldenium
	} else { // YELLOW
		positionsCart.push_back(std::make_pair(2230, 450));//            out of goldenium
		positionsCart.push_back(std::make_pair(2230, 200));//            in front of goldenium
	}
	positionsCart.push_back(std::make_pair(1500, 800));//          waypoint behind backhole


	// Create polar position from cartesian positions
	for (auto position: positionsCart) {
		float distance = 0;
		float angle = 0;
		cart_to_polar(position.first - positionDepart.first, position.second - positionDepart.second, angle, distance);
		angle += angleDepart;
		positionsPolar.push_back(std::make_pair(distance, angle));
	}

	// Revert for the other color
	if (is_blue) { // BLUE
		printf("Reversing position because of color choice\n");
		fflush(stdout);
		for (auto& position: positionsPolar) {
			std::cout << "position, before: " << position.second << std::endl;
			position.second = - position.second;
			std::cout << "after: " << position.second << std::endl;
		}
		fflush(stdout);
	}
	else {
		printf("Keeping positions\n");
		fflush(stdout);
	}


	std::cout << "There are " << positionsPolar.size() << " positionPolar, " << positionsCart.size() << " positionCart" << std::endl;
	fflush(stdout);

	chrono = 0;
	last_encoder1 = last_encoder2 = 0;
}

void Core::stop_motors() {
	set_motors_speed(0, 0, false, false);
}

void Core::set_motors_speed(float linearSpeed, float angularSpeed) {
	set_motors_speed(linearSpeed, angularSpeed, true, false);
}

void Core::set_motors_speed(float linearSpeed, float angularSpeed, bool enable, bool resetEncoders) {
	goal_strategy::motors_cmd new_motor_cmd;
	new_motor_cmd.speed_command.linear.x = linearSpeed;
	new_motor_cmd.speed_command.angular.z = angularSpeed;
	new_motor_cmd.enable = enable;
	new_motor_cmd.reset_encoders = resetEncoders;

	motors_cmd_pub.publish(new_motor_cmd);
}

int Core::Setup(int argc, char* argv[]) {
	/**************************************
	 *               Setup                *
	 **************************************/
	mode = 0;

	/**************************************
	 *         Broker Main Loop           *
	 **************************************/
	// Take time before entering the loop
	clock_gettime(CLOCK_MONOTONIC, &last);
	usleep(10000);// So the next now-last won't return 0

	/*************************************************************
	 *			   MAIN LOOP
	 ************************************************************/

	// old wait for tirette

	// Take time before entering the loop
	clock_gettime(CLOCK_MONOTONIC, &last);

	set_motors_speed(0, 0, false, false);
	printf("before while\n");
	fflush(stdout);


	return 0;
}

void Core::landscapeFromAngleAndStrength(float[]& landscape, float angle, float strenght) {
	for (int i = 0; i < NB_NEURONS; i++) {
		landscape[i] = cos(angle) * strength;
	}

}

Core::~Core() {
	// We broke out of the loop, stop everything
	stop_motors();

	printf ("Got out of the main loop, stopped everything.\n");

	// Print robot's final position
	if (PRINT_FINAL_POS) {
		float rho = integration_field[0];
		theta = 0;

		for (int i = 1; i < 360; i += 1) {
			if (integration_field[i] > rho) {
				rho = integration_field[i];
				theta = i;
			}
		}

		printf("Final position: dist = %.2f, angle = %d deg (%.2f, %.2f)\n", rho, theta, rho * cos((float) theta / 180. * M_PI), rho * sin((float) theta / 180. * M_PI));
	}
}

void Core::update_encoders(long& encoder1, long& encoder2) {
	/*
	 * We currently have a bug where sometimes the value of an encoder jumps to a
	 * very high value. We mitigate this by "filtering" these big jumps
	 */
	if (abs(encoder1 - last_encoder1) > 2000) {
		fprintf(stderr, "[WARNING] Jump in encoder1 value (%ld), skipping step!\n", encoder1 - last_encoder2);
		encoder1 = last_encoder1;
	} else {
		last_encoder1 = encoder1;
	}
	if (abs(encoder2 - last_encoder2) > 2000) {
		fprintf(stderr, "[WARNING] Jump in encoder2 value (%ld), skipping step!\n", encoder2 - last_encoder2);
		encoder2 = last_encoder2;
		}
	 else {
		last_encoder2 = encoder2;
	}
}

bool Core::are_we_go_for_launch() {
	return digitalRead(PIN_TIRETTE) == 1;
}

void Core::wait_for_tirette() {
	// Is the tirette is enabled, wait for it to be released
	if (TIRETTE_ENABLED == TRUE) {
		if (!is_tirette_msg_displayed) {
			printf("Waiting for tirette...\n");
			fflush(stdout);
			is_tirette_msg_displayed = true;
		}
		if (are_we_go_for_launch()) {
			printf("got it!\n Proceeding.\n");
			fflush(stdout);
			state = NORMAL;
			// Start counting the time
			clock_gettime(CLOCK_MONOTONIC, &begin_match);
		}
	} // otherwise, start immediately
	else {
		state = NORMAL;
	}
}

int Core::Loop() {

		if (is_time_to_stop()) {
			return state;
		}

		// Compute the robot's linear speed & orientation
		float speed = compute_linear_speed(encoder1, encoder2, elapsed);
		// the robot's orientation, in degrees (the robots is born at its 0 deg)
		unsigned int orientation = get_orientation(encoder1, encoder2);
		float coeff = speed / 1000. * (float) (elapsed / 1e9);

		// Compute activation for path integration for this current loop
		for (int i = 0; i < NB_NEURONS; i += 1) {
			integration_field[i] += coeff * cos((int)(i - orientation) * M_PI / 180.);
		}

		//i = get_idx_of_max(integration_field, NB_NEURONS);
		//printf("Rho = %.3f - Theta = %d deg\n", integration_field[i], i);

		if (state == WAIT_TIRETTE) {
			set_motors_speed(0, 0, false, false);
			wait_for_tirette();
		} else if (state == NORMAL) {
			chrono = compute_match_chrono();

			// Get the orientation we need to follow to reach the goal
			// TODO: assign priority integers to strategies, take the strategy that has the max of priority * strength

			compute_target_speed_orientation(orientation);

			// Compute repulsive vector from obstacles
			uint8_t closest_obstacle_id = get_idx_of_max(s_lidar.output->neural_field, NB_NEURONS);

			// Compute intensity of obstacle
			float peakValue = 175. * s_lidar.output->neural_field[(closest_obstacle_id) % 360];

			//printf("closest_obstacle_id = %d, peakValue = %f\n", closest_obstacle_id, peakValue);
			//fflush(stdout);
			// Then apply gaussian function centered on the sensor's angle
			for (int j = 0; j < NB_NEURONS; j += 1) {
				//obstacles_output[j] += - gaussian(50., a, (360 + 180 - idx) % 360, j);
				lidar_output[j] += - gaussian(50., peakValue, (360 + 180 + closest_obstacle_id) % 360, j);
			}


			// Inhibit linear speed if there are obstacles
			//TODO: handle negative values to be able to go reverse
			if (s_obstacles.output->strength != 0 && s_obstacles.output->speed_inhibition < linear_speed_cmd) {
				//linear_speed_cmd = s_obstacles.output->speed_inhibition;
			}
			//TODO: handle negative values to be able to go reverse
			if (s_lidar.output->strength != 0 && s_lidar.output->speed_inhibition < linear_speed_cmd) {
				linear_speed_cmd = s_lidar.output->speed_inhibition;
			}

			// Compute attractive vectors from positive valence strategies
			// TODO: choose the POSITIVE VALENCE STRATEGY!
			for (int i = 0; i < NB_NEURONS; i += 1) {
				goal_output[i] = target(107., 1.1, (360 + 180 - (target_orientation - orientation)) % 360, i);
			}

			// Sum positive and negative valence strategies
			for (int i = 0; i < NB_NEURONS; i += 1) {
				// Temporarily check if joystick is active (later: use weighted sum)
				if (s_joystick.output->strength == 0) {
					angular_landscape[i] = goal_output[i] + lidar_output[i];
				}
				else {
					printf("joystick is active\n");
					fflush(stdout);
					angular_landscape[i] = goal_output[i];
				}
			}

			// And finally: differentiate the angular_landscape vector to get drive
			differentiate(angular_landscape, angular_speed_vector, NB_NEURONS, 3000.);

			// Set linear speed according to the obstacles strategy & angular speed based on goal + obstacles
			// Robot's vision is now centered on 180 deg
			int angular_speed_cmd = (int) round(angular_speed_vector[180]);

			limit_angular_speed_cmd(angular_speed_cmd);
			
			update_speed(FALSE, &angular_speed, angular_speed_cmd);
			update_speed(FALSE, &linear_speed, linear_speed_cmd);

			if (DISABLE_LINEAR_SPEED)
				linear_speed = 0;

			if (DISABLE_ANGULAR_SPEED)
				angular_speed = 0;

			// Set motors speed according to values computed before
			set_motors_speed(linear_speed, angular_speed, true, false);
		} // End of state == NORMAL

		maintain_loop_timing();

		return state;
}

bool Core::is_time_to_stop() {
	clock_gettime(CLOCK_MONOTONIC, &now);

	// Exit and stop robot if we reached the end of the match
	if (ENABLE_TIMEOUT_END_MATCH == TRUE && chrono > TIMEOUT_END_MATCH) {
		state = EXIT;
		return true;
	}

	// At the beginning of a new loop, compute elapsed time since last loop
	elapsed = now.tv_nsec - last.tv_nsec; // elapsed time in ns
	// To support rollover
	if (elapsed <= 0) {
		elapsed += 1000000000L;
	}
	return false;
}

void Core::maintain_loop_timing() {
	// Update "last" values
	last = now;

	// Recompute current time at the end of this loop's execution
	clock_gettime (CLOCK_MONOTONIC, &now);
	// Compute elapsed time between the beginning and the end (now) of this loop's execution
	elapsed = now.tv_nsec - last.tv_nsec;

	if (elapsed <= 0)
		elapsed += 1000000000L;

	// Subtract this loop's execution time to the period we want (based on the FREQ)
	int delay = ((int) 1e6 / BROKER_FREQ) - (elapsed / 1000);

	// Sanitize input
	if (delay < 0 ) {
		printf("[WARNING] Computed delay < 0, setting to 0...\n");
		delay = 0;
	}

	// Print a warning message if this loop's execution took too much time compared with the desired frequency
	if (delay < (int) (0.10 * (int) 1e6 / BROKER_FREQ)) {
		fprintf (stderr, "[WARNING] Main loop consumes more than 90%% of the period.\n");
	}

	// Wait according to required freq, adjusted with the time this loop's execution took
	usleep (delay);
}

// Compute elapsed time since beginning of match in ms
long Core::compute_match_chrono() {
	long l_chrono = now.tv_nsec - begin_match.tv_nsec; // First compute nano seconds...
	if (l_chrono <= 0)                      // Then support rollover
		l_chrono += 1000000000L;
	l_chrono /= 1000000L;                   // Divide by 1e6 to convert to ms

	l_chrono += (now.tv_sec - begin_match.tv_sec) * 1000L; // And add seconds * 1000 (to convert to ms)
	
	return l_chrono;
}

void Core::compute_target_speed_orientation(const unsigned int orientation) {
	if (s_joystick.output->strength == 1) {
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
		 * If no positive valence strategy fires, set the target orientation to the robot's orientation
		 * which means the robot will go straight on
		 */
		target_orientation = orientation;
		linear_speed_cmd = default_linear_speed;
	}
}

void Core::limit_angular_speed_cmd(int& angular_speed) {
	// TODO: use the winning strategy with weights
	if (s_goal.output->strength == 1) {
		int speed_limited_by_obstacle = s_goal.output->angular_speed_inhibition * MAX_ALLOWED_ANGULAR_SPEED / 100;
		MIN(angular_speed, speed_limited_by_obstacle); 
		MAX(angular_speed, -speed_limited_by_obstacle); 
	}

	// Cap angular speed, so that the robot doesn't turn TOO FAST on itself
	MAX(angular_speed, -MAX_ALLOWED_ANGULAR_SPEED);
	MIN(angular_speed, MAX_ALLOWED_ANGULAR_SPEED);
}

