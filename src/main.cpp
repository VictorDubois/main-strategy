#include "core.h"
#include "ros/ros.h"
#include <stdexcept>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#define MAX_ALLOWED_ANGULAR_SPEED 0.2

#ifndef MAX
#define MIN(a,b) (((a)<(b))?(a):(b))
#define MAX(a,b) (((a)>(b))?(a):(b))
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
void Core::cart_to_polar(int posX, int posY, float& theta, float& distance) {
	theta = ((180./M_PI) * atan2((float) posX, (float) posY));
	distance = sqrt((float)(posX * posX + posY * posY));

	// fix angular ambiguity
	if (posY < 0) {
		theta += 180;
	}

	#ifdef DEBUG_cart_to_polar
		std::cout << "posX = " << posX << "posY = " <<  posY<< "theta = " << theta << ", distance = " << distance << std::endl;
		float posXafter = distance*cos(theta * M_PI/180.f);
        	float posYafter = distance*sin(theta * M_PI/180.f);
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

void Core::updateOdometry(nav_msgs::Odometry odometry) {
    last_position.setX(X);
    last_position.setY(Y);

    X = odometry.pose.pose.position.x;
    Y = odometry.pose.pose.position.y;
    current_position = Position(X, Y, false);

    double siny_cosp = 2 * (odometry.pose.pose.orientation.w * odometry.pose.pose.orientation.z + odometry.pose.pose.orientation.x * odometry.pose.pose.orientation.y);
    double cosy_cosp = 1 - 2 * (odometry.pose.pose.orientation.y * odometry.pose.pose.orientation.y + odometry.pose.pose.orientation.z * odometry.pose.pose.orientation.z);
    current_theta = std::atan2(siny_cosp, cosy_cosp) * 180.f/M_PI;

    current_pose_pub.publish(odometry.pose.pose);

    distance_to_goal = sqrt((X - goal_position.getX()) * (X - goal_position.getX()) + (Y - goal_position.getY()) * (Y - goal_position.getY()));
    std::cout << "distance to goal = " << distance_to_goal << std::endl;
}

void Core::updateCurrentPose(goal_strategy::encoders encoders) {
    last_position.setX(X);
    last_position.setY(Y);

	encoder1 = encoders.encoder_left;
	encoder2 = encoders.encoder_right;

    if (!encoders_initialized) {
        std::cout << "initializing encoders" << std::endl;
        starting_encoder1 = encoder1;
        starting_encoder2 = encoder2;
        encoders_initialized = true;
    }

    encoder1 -= starting_encoder1;
    encoder2 -= starting_encoder2;

	// low pass filter
    //update_encoders(encoder1, encoder2);
	//std::cout << "enc1: " << encoder1 << ",enc2: " << encoder2 << std::endl;
	//std::cout << get_orientation(encoder1, encoder2) << std::endl;

	geometry_msgs::Pose currentPose = update_current_pose(encoder1, encoder2);

	current_pose_pub.publish(currentPose);
    update_current_speed();
    send_odometry(currentPose);

    distance_to_goal = sqrt((X - goal_position.getX()) * (X - goal_position.getX()) + (Y - goal_position.getY()) * (Y - goal_position.getY()));
    std::cout << "distance to goal = " << distance_to_goal << std::endl;
}

float Core::vector_to_angle(geometry_msgs::Vector3 vector) {
	return atan2(vector.y, vector.x)*180/M_PI;
}

float Core::vector_to_angle(geometry_msgs::Point vector) {
	return atan2(vector.y, vector.x)*180/M_PI;
}

float Core::vector_to_amplitude(geometry_msgs::Vector3 vector) {
	return sqrt((vector.x * vector.x) + (vector.y * vector.y));
}

float Core::vector_to_amplitude(geometry_msgs::Point vector) {
	return sqrt((vector.x * vector.x) + (vector.y * vector.y));
}

void Core::updateRelativeGoal() {
    // Convert absolute position to relative position
    Position relative_goal_position = goal_position - current_position;

    // Orient to goal
    target_orientation = relative_goal_position.getAngle() * 180./M_PI;
    std::cout << "relative goal position = " << relative_goal_position.getX() << ", " << relative_goal_position.getY() << std::endl;
    std::cout << "target_orientation = " << target_orientation << std::endl;
}

void Core::updateGoal(geometry_msgs::Pose goal_pose) {
    goal_position = Position(goal_pose.position);

	//last_goal_max_speed = goal_out.max_speed;
}

void Core::updateTeamColor(std_msgs::Bool new_color) {
	is_blue = new_color.data;

	if (is_blue) {
		std::cout << "We are on the blue Team" << std::endl;
	}
	else {
		std::cout << "We are on the yellow Team" << std::endl;
	}
	
}

void Core::updateTirette(std_msgs::Bool starting) {
	if (starting.data && state == WAIT_TIRETTE) {
		std::cout << "Go for launch!" << std::endl;
		state = NORMAL;
		// Start counting the time
		clock_gettime(CLOCK_MONOTONIC, &begin_match);
	}
}

void Core::updateLidar(geometry_msgs::Vector3 closest_obstacle) {
	// Compute repulsive vector from obstacles
	int16_t closest_obstacle_id = vector_to_angle(closest_obstacle);

	// Compute intensity of obstacle
	float obstacle_dangerouseness = 175. * vector_to_amplitude(closest_obstacle);

	//printf("closest_obstacle_id = %d, peakValue = %f\n", closest_obstacle_id, peakValue);
	// Then apply gaussian function centered on the sensor's angle
	for (int j = 0; j < NB_NEURONS; j += 1) {
		//obstacles_output[j] += - gaussian(50., a, (360 + 180 - idx) % 360, j);
		lidar_output[j] += - gaussian(50., obstacle_dangerouseness, (fmod(360 + 180 + closest_obstacle_id, 360)), j);
	}
}

Core::Core() {
	ros::start();
    last_speed_update_time = ros::Time::now();
    current_linear_speed = 0;
    encoders_initialized = false;
    goal_position = Position();
	last_distance = 0;
    distance_to_goal = 0;
	geometry_msgs::Twist last_lidar_max_speed;
	last_lidar_max_speed.linear.x = 0;
	last_lidar_max_speed.linear.y = 0;
	last_lidar_max_speed.angular.z = 0;
	geometry_msgs::Twist last_goal_max_speed;
	last_goal_max_speed.linear.x = 0;
	last_goal_max_speed.linear.y = 0;
	last_goal_max_speed.angular.z = 0;
	ros::NodeHandle n;
    motors_cmd_pub = n.advertise<geometry_msgs::Twist>("cmd_vel_motor", 5);
    motors_enable_pub = n.advertise<std_msgs::Bool>("enable_motor", 5);
    current_pose_pub = n.advertise<geometry_msgs::Pose>("current_pose", 5);
    odom_pub = n.advertise<nav_msgs::Odometry>("odom", 5);
    encoders_sub = n.subscribe("encoders", 1000, &Core::updateCurrentPose, this);
    goal_sub = n.subscribe("goal_pose", 1000, &Core::updateGoal, this);
    odometry_sub = n.subscribe("odom_sub", 1000, &Core::updateOdometry, this);
	lidar_sub = n.subscribe("obstacle_lidar", 1000, &Core::updateLidar, this);
	color_sub = n.subscribe("team_color", 1000, &Core::updateTeamColor, this);
	tirette_sub = n.subscribe("tirette", 1000, &Core::updateTirette, this);
	goal_output[NB_NEURONS] = {0.};
	obstacles_output[NB_NEURONS] = {0.};
	lidar_output[NB_NEURONS] = {0.};
	angular_speed_vector[NB_NEURONS] = {0.};
	angular_landscape[NB_NEURONS] = {0.};
	is_tirette_msg_displayed = false;

	printf("done! Proceeding.\nStarting IA.\n");
	//@TODO Reset encoders (for now, the robot starts at its origin point)

	/**************************************
	 *      Variable initialization       *
	 **************************************/
	linear_speed = 0; // motor command for linear speed, in percentage (from -100 to 100)
	angular_speed = 0; // motor command for angular speed, in percentage (from -100 to 100)
	linear_speed_cmd = 0;

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

void Core::set_motors_speed(float linearSpeed, float angularSpeed, bool enable, bool /*resetEncoders*/) {
	geometry_msgs::Twist new_motor_cmd;
    new_motor_cmd.linear.x = linearSpeed;
	new_motor_cmd.angular.z = angularSpeed;

	//motors_cmd_pub.publish(new_motor_cmd);
	std_msgs::Bool new_enable_cmd;
    new_enable_cmd.data = enable;
	//motors_enable_pub.publish(new_enable_cmd);
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

	stop_motors();
	printf("before while\n");
	fflush(stdout);


	return 0;
}

void Core::landscapeFromAngleAndStrength(std::vector<float> landscape, float angle, float strength) {
	for (int i = 0; i < NB_NEURONS; i++) {
		landscape[i] = cos(angle-i) * strength;
	}

}

bool Core::digitalRead(int) {
	return true;
}

Core::~Core() {
	// We broke out of the loop, stop everything
	stop_motors();

	printf ("Got out of the main loop, stopped everything.\n");
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

geometry_msgs::Pose Core::update_current_pose(int32_t encoder1, int32_t encoder2) {
    float linear_dist = compute_linear_dist(encoder1, encoder2);
    current_theta = get_orientation_float(encoder1, encoder2);

    X += linear_dist * cos(current_theta * M_PI/180.f);
    Y += linear_dist * sin(current_theta * M_PI/180.f);
	
    std::cout << "X = " << X << ", Y = " << Y << ", theta = " << current_theta << ",linear_dist = " << linear_dist << std::endl;

    current_position = Position(X, Y, false);

    geometry_msgs::Pose currentPose;
    currentPose.position.x = X/1000;
    currentPose.position.y = Y/1000;
    //currentPose.orientation.z = current_theta * M_PI/180.f;

    tf2::Quaternion orientation_quat;
    orientation_quat.setRPY(0, 0, current_theta * M_PI/180.f);
    currentPose.orientation = tf2::toMsg(orientation_quat);
    return currentPose;
}

void Core::send_odometry(const geometry_msgs::Pose& currentPose) {
//first, we'll publish the transform over tf
	geometry_msgs::TransformStamped odom_trans;
	odom_trans.header.stamp = ros::Time::now();
      odom_trans.header.frame_id = "odom";
       odom_trans.child_frame_id = "base_link";
   
       odom_trans.transform.translation.x = currentPose.position.x;
       odom_trans.transform.translation.y = currentPose.position.y;
         odom_trans.transform.translation.z = 0.0;
         odom_trans.transform.rotation = currentPose.orientation;
    
        //send the transform
        odom_broadcaster.sendTransform(odom_trans);

	nav_msgs::Odometry odom_msg;
	
	//odom_msg.header.frame_id = "base_link";
	odom_msg.header.frame_id = "odom";
	odom_msg.header.stamp = ros::Time::now();
	/*odom_msg.header.stamp.nsec = 0;
	odom_msg.header.stamp.sec = 0;
	odom_msg.header.seq = 0;*/
	//odom_msg.child_frame_id = "odom";
	odom_msg.child_frame_id = "base_link";

	for (unsigned int i = 0; i < (sizeof(odom_msg.pose.covariance)/sizeof(odom_msg.pose.covariance[0])); i++){
		odom_msg.pose.covariance[i] = 0;
	}

	odom_msg.pose.pose = currentPose;
	/*odom_msg.pose.pose.position.x = 0;
	odom_msg.pose.pose.position.y = 0;
	odom_msg.pose.pose.position.z = 0;

	odom_msg.pose.pose.orientation.x = 0;
	odom_msg.pose.pose.orientation.y = 0;
	odom_msg.pose.pose.orientation.z = 0;*/
	odom_msg.pose.covariance[0] = 0.1;
	odom_msg.pose.covariance[7] = 0.1;
	odom_msg.pose.covariance[35] = 0.2;

	odom_msg.pose.covariance[14] = 0000; // set a non-zero covariance on unused
	odom_msg.pose.covariance[21] = 0000; // dimensions (z, pitch and roll); this
	odom_msg.pose.covariance[28] = 0000; // is a requirement of robot_pose_ekf
	//source: https://github.com/yujinrobot/kobuki/blob/0.6.6/kobuki_node/src/library/odometry.cpp#L145-L154

	for (unsigned int i = 0; i < (sizeof(odom_msg.twist.covariance)/sizeof(odom_msg.twist.covariance[0])); i++){
		odom_msg.twist.covariance[i] = 0;
	}

	odom_msg.twist.twist.linear.x = current_linear_speed;//(left_speed+right_speed)/2;
	odom_msg.twist.twist.linear.y = 0;
	odom_msg.twist.twist.linear.z = 0;

	odom_msg.twist.twist.angular.x = 0;
	odom_msg.twist.twist.angular.y = 0;
	odom_msg.twist.twist.angular.z = current_angular_speed;//(left_speed-right_speed)/2;
	odom_pub.publish(odom_msg);
}

void Core::update_current_speed() {
    ros::Time now = ros::Time::now();
    double time_since_last_speed_update = (last_speed_update_time - now).toSec();

    if (time_since_last_speed_update == 0) {
        std::cout << "Error: null time!" << std::endl;
    }

    int distance_moved = (current_position - last_position).getNorme();

    current_linear_speed = abs(distance_moved / time_since_last_speed_update);
    std::cout << "current_linear_speed = " << current_linear_speed << std::endl;

    current_angular_speed = (current_theta - last_theta)/time_since_last_speed_update;
    last_theta = current_theta;

    last_speed_update_time = now;
}

void Core::limit_linear_speed_cmd_by_goal() {
    float max_deceleration = 5;// mm/iter^2

    float desired_final_speed = 0;// mm/iter^2

    float distance_to_stop = (current_linear_speed-desired_final_speed)/max_deceleration;
    std::cout << "distance to stop = " << distance_to_stop << std::endl;

    float stopping_ratio = distance_to_stop/distance_to_goal;

    if(stopping_ratio > 0.5) {
        std::cout << "Approching target, slowing down!" << std::endl;
        linear_speed_cmd = default_linear_speed * MAX((1-stopping_ratio), 0.01);
    }
}

int Core::Loop() {

	if (is_time_to_stop()) {
		std::cout << "Time's up !" << std::endl;
		return state;
	}

	if (state == WAIT_TIRETTE) {
		set_motors_speed(0, 0, false, false);
	} else if (state == NORMAL) {
		chrono = compute_match_chrono();

		// Get the orientation we need to follow to reach the goal
		// TODO: assign priority integers to strategies, take the strategy that has the max of priority * strength

		//compute_target_speed_orientation(orientation);
        updateRelativeGoal();

		// Inhibit linear speed if there are obstacles

		// Compute attractive vectors from positive valence strategies
		// TODO: choose the POSITIVE VALENCE STRATEGY!
		for (int i = 0; i < NB_NEURONS; i += 1) {
			goal_output[i] = target(107., 1.1,fmod (360 + 180 - (target_orientation - current_theta), 360), i);
		}

        std::cout << "relative_target_orientation: " << (target_orientation - current_theta) << ", peak value: " << get_idx_of_max(goal_output, NB_NEURONS) << ", central value = " << goal_output[180];


		// Sum positive and negative valence strategies
		for (int i = 0; i < NB_NEURONS; i += 1) {
			// Temporarily check if joystick is active (later: use weighted sum)
			//if (s_joystick.output->strength == 0) {
				angular_landscape[i] = goal_output[i] + lidar_output[i];
			/*}
			else {
				printf("joystick is active\n");
				fflush(stdout);
				angular_landscape[i] = goal_output[i];
			}*/
		}

		// And finally: differentiate the angular_landscape vector to get drive
		differentiate(angular_landscape, angular_speed_vector, NB_NEURONS, 3000.);

		// Set linear speed according to the obstacles strategy & angular speed based on goal + obstacles
		// Robot's vision is now centered on 180 deg
        float angular_speed_cmd = -angular_speed_vector[180];// - as positive is towards the left in ros, while the derivation is left to right
        std::cout << ",angular_speed_cmd = " << angular_speed_cmd << std::endl;

        linear_speed_cmd = default_linear_speed;

        limit_linear_speed_cmd_by_goal();

		limit_angular_speed_cmd(angular_speed_cmd);
		
		update_speed(FALSE, &angular_speed, angular_speed_cmd);
		//update_speed(FALSE, &linear_speed, linear_speed_cmd);
		linear_speed = linear_speed_cmd;

		if (DISABLE_LINEAR_SPEED) {
			linear_speed = 0;
		}

		if (DISABLE_ANGULAR_SPEED) {
			angular_speed = 0;
		}

        //linear_speed = 0;
        if (angular_speed < -1 || angular_speed > 1) {
            linear_speed = 0;
        }

		// Set motors speed according to values computed before
        set_motors_speed((float) linear_speed/255.f, (float) angular_speed/20.f, true, false);
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
		 * If no positive valence strategy fires, set the target orientation to the robot's orientation
		 * which means the robot will go straight on
		 */
		//target_orientation = orientation;
		linear_speed_cmd = default_linear_speed;
	//}
}

void Core::limit_angular_speed_cmd(float& angular_speed) {
	// TODO: use the winning strategy with weights
	// TODO: restore speed limitations
	/*if (s_goal.output->strength == 1) {
		int speed_limited_by_obstacle = s_goal.output->angular_speed_inhibition * MAX_ALLOWED_ANGULAR_SPEED / 100;
		MIN(angular_speed, speed_limited_by_obstacle); 
		MAX(angular_speed, -speed_limited_by_obstacle); 
	}*/

	// Cap angular speed, so that the robot doesn't turn TOO FAST on itself
	MAX(angular_speed, -MAX_ALLOWED_ANGULAR_SPEED);
	MIN(angular_speed, MAX_ALLOWED_ANGULAR_SPEED);
}


