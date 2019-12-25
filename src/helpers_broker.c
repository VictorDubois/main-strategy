#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <unistd.h>
#include <math.h>

#include "helpers_broker.h"
#include "constants.h"

#define FALSE 0
#define TRUE 1

/*
	Return the Robot's orientation, in degrees, with respect to the last encoder reset.
*/
unsigned int get_orientation (long encoder1, long encoder2) {
	// encoder2 - encoder1 gives angles in trigonometric direction
	int absolute_orientation = (int) ((encoder2 - encoder1) / TICKS_PER_DEG) % 360;

	if (absolute_orientation >= 0)
		return(absolute_orientation);
	else
		return(360 + absolute_orientation); // reminder: abs_ori is < 0 here

}

/*
	Given current value of both encoders and the elapsed time (in ns) between last measurement,
	return the linear speed by approximating it as the average of both wheels' linear speeds.
	Static variables are used to keep last value of encoders.
*/
  	static long last_encoder1 = 0;
  	static long last_encoder2 = 0;
float compute_linear_speed (const long encoder1, const long encoder2, const long elapsed) {
  	last_encoder1 = encoder1;
  	last_encoder2 = encoder2;
	float speed1, speed2, speed;
	int diff_encoder1, diff_encoder2;

	// Compute difference in nb of ticks between last measurements and now
	diff_encoder1 = encoder1 - last_encoder1;
	diff_encoder2 = encoder2 - last_encoder2;

	// Compute each wheel's speed and approximate linear speed as their average
	speed1 = (DIST_PER_REVOLUTION * (float) diff_encoder1 / TICKS_PER_REVOLUTION) / (float) (elapsed / 1e9);
	speed2 = (DIST_PER_REVOLUTION * (float) diff_encoder2 / TICKS_PER_REVOLUTION) / (float) (elapsed / 1e9);
	speed = (speed1 + speed2) / 2.0;

	if (fabsf(speed) > 500.) {
		printf("\n/!\\ HIGH SPEED DETECTED: %.2f /!\\\n\n", speed);
		//exit(4);
	}

	// Update static variables' values (current encoder values become old ones)
	last_encoder1 = encoder1;
	last_encoder2 = encoder2;

	// Return the computed linear speed
	return speed;
}

/*
	Query the arduino for the distances read by the US and IR sensors.
	The 'msg_size' argument is the number of bytes to read.
	Returns 0 if it could read the corretc number of bytes, -1 otherwise.
*/
int get_sensor_values (int fd, void * buf, ssize_t msg_size) {
	if (read (fd, buf, msg_size) != msg_size)
		return -1;
	else
		return 0;
}
/*
 * Implement progressive ramp to update the motors' speed.
 * The 'immediate' parameter allow to override this ramp and set the target speed directly
 * (allows for emergency stop for instance)
 */
void update_speed (const int immediate, int *current_speed, const int new_speed) {
	// If immediate, do not use the ramp, set speed immediately
	if (immediate == TRUE) {
		*current_speed = new_speed;
		return;
	}
	// Otherwise, use the ramp's coefficient to smooth the transitions between speeds
	else {
		// If it's an acceleration, 'ceil' the value (to avoid asymptote and miss the target)
		if (new_speed > *current_speed) {
			*current_speed = ceil(SPEED_SMOOTH_FACTOR * new_speed + (1 - SPEED_SMOOTH_FACTOR) * (*current_speed));
			return;
		}
		// If it's a braking, 'floor' the value (to avoid asymptote and prevent stopping to 0)
		else {
			*current_speed = floor(SPEED_SMOOTH_FACTOR * new_speed + (1 - SPEED_SMOOTH_FACTOR) * (*current_speed));
			return;
		}
	}
}

/*
 * Apply a function with its input being circular
 * It obviously needs functions which are continuuous at their edges.
 * This function is designed to work with gaussian-like function (parameter u is the center)
 */
float apply_circular (float f (int), const int u, const int x) {
	// Version 1: approx by sum
	//return f(x - u) + f((x - u) - 360) + f ((x - u) + 360);
	
	// Version 2: find correct segment to evaluate from (the one nearest the center) and apply
	int newX = x;
	unsigned int dist = abs(newX - u);

	if (abs((x - 360) - u) < (int)dist) {
		newX = x - 360;
		dist = abs(newX - u);
	} else if (abs((x + 360) - u) < (int)dist) {
		newX = x + 360;
		dist = abs(newX - u);
	}

	//printf("For %d, evaluating in %d\n", x, newX);

	return f(newX - u);
}

/*
 * Applies a gaussian function with given sigma and mu value, in x
 */
float gaussian (const float sigma, const float a, const float mu, const float x) {
	int new_x = x;
	unsigned int dist = abs(new_x - mu);

	if ((unsigned int)abs((x - 360) - mu) < dist) {
		new_x = x - 360;
		dist = abs(new_x - mu);
	} else if ((unsigned int)abs((x + 360) - mu) < dist) {
		new_x = x + 360;
		dist = abs(new_x - mu);
	}

	return a / (sigma * sqrt(2. * M_PI)) * exp(-pow((new_x - mu) / sigma, 2) / 2.0);
}

/**
 * Given a neural field (float vector), return the derivative of this vector
 */
void differentiate (const float *vector, float *result, const size_t len, const float scale_factor) {
	unsigned int i, next, previous;

	for (i = 0; i < len; i += 1) {
		next = (len + i + 1) % len;
		previous = (len + i - 1) % len;
		result[i] = scale_factor * (vector[next] - vector[previous]) / 2.;
	}

}

float target (const float spread, const float offset, const float x0, const float x) {
	int new_x = x;
	unsigned int dist = abs(new_x - x0);

	if ((unsigned int)abs((x - 360) - x0) < dist) {
		new_x = x - 360;
		dist = abs(new_x - x0);
	} else if ((unsigned int)abs((x + 360) - x0) < dist) {
		new_x = x + 360;
		dist = abs(new_x - x0);
	}

	return - log(cosh((new_x - x0) / spread)) + offset;
}

float pseudo_gaussian_derivative (const int x) {
	/*
	 * Constant part: cap at the maximum of the gaussian bell
	 * This is because when an obstacle is far behinf (angularly),
	 * we end up in the tail of the gaussian bell, so we turn slowly.
	 */
	if (x > 50)
		return -24.2;
	if (x < -50)
		return 24.2;
	
	// Gaussian part: provides a nice (angular) desceleration when we reach the goal
	return -0.8 * x * exp(-pow(x, 2) / 5000.);
	//return -0.2 * x * exp(-pow(x, 2) / 5000.); // TEST
	//return -0.2 * x * exp(-pow(x, 2) / 1000.);
}

float obstacle_temporary_derivative (const int x) {
	return 4. * x * exp(-pow(x, 2) / 300.);
	
	// So that nearby uS sensors merge to repell obstacle
	//return 1. * x * exp(-pow(x, 2) / 650.);
}

unsigned int get_idx_of_max (const float vector[], const size_t len) {
	unsigned int curr_max = 0, i;

	for (i = 1; i < len; i += 1) {
		if (vector[i] > vector[curr_max])
			curr_max = i;
	}

	return curr_max;
}

void debug_vector (const char * title, const float * vector, const size_t len, const int display_index, const int display_max) {
	unsigned int i, idx;
	float max_value;

	printf("%s",title);

	for (i = 0; i < len; i += 10) {
		printf("%+.1f ", vector[i]);
	}
	printf("\n");

	if (display_index == TRUE) {
		for (i = 0; i < len; i += 10) {
		       printf("% 4d ", i);
		}

		printf("\n");
	}

	if (display_max == TRUE) {
		idx = 0;
		max_value = vector[0];

		for (i = 1; i < len; i += 1) {
			if (vector[i] > max_value) {
				idx = i;
				max_value = vector[i];
			}
		}

		printf("Max: %f @%d deg\n", max_value, idx);
	}

	printf("\n");
}
