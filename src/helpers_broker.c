#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#include "helpers_broker.h"

#define FALSE 0
#define TRUE 1

/*
        Query the arduino for the distances read by the US and IR sensors.
        The 'msg_size' argument is the number of bytes to read.
        Returns 0 if it could read the corretc number of bytes, -1 otherwise.
*/
int get_sensor_values(int fd, void* buf, ssize_t msg_size)
{
    if (read(fd, buf, msg_size) != msg_size)
        return -1;
    else
        return 0;
}
/*
 * Implement progressive ramp to update the motors' speed.
 * The 'immediate' parameter allow to override this ramp and set the target speed directly
 * (allows for emergency stop for instance)
 */
void update_speed(const float immediate, float* current_speed, const float new_speed)
{
    // If immediate, do not use the ramp, set speed immediately
    if (immediate == TRUE)
    {
        *current_speed = new_speed;
        return;
    }
    // Otherwise, use the ramp's coefficient to smooth the transitions between speeds
    else
    {
        // If it's an acceleration, 'ceil' the value (to avoid asymptote and miss the target)
        if (new_speed > *current_speed)
        {
            *current_speed = ceil(SPEED_SMOOTH_FACTOR * new_speed
                                  + (1 - SPEED_SMOOTH_FACTOR) * (*current_speed));
            return;
        }
        // If it's a braking, 'floor' the value (to avoid asymptote and prevent stopping to 0)
        else
        {
            *current_speed = floor(SPEED_SMOOTH_FACTOR * new_speed
                                   + (1 - SPEED_SMOOTH_FACTOR) * (*current_speed));
            return;
        }
    }
}

/*
 * Apply a function with its input being circular
 * It obviously needs functions which are continuuous at their edges.
 * This function is designed to work with gaussian-like function (parameter u is the center)
 */
float apply_circular(float f(int), const int u, const int x)
{
    // Version 1: approx by sum
    // return f(x - u) + f((x - u) - 360) + f ((x - u) + 360);

    // Version 2: find correct segment to evaluate from (the one nearest the center) and apply
    int newX = x;
    unsigned int dist = abs(newX - u);

    if (abs((x - 360) - u) < (int)dist)
    {
        newX = x - 360;
        dist = abs(newX - u);
    }
    else if (abs((x + 360) - u) < (int)dist)
    {
        newX = x + 360;
        dist = abs(newX - u);
    }

    // printf("For %d, evaluating in %d\n", x, newX);

    return f(newX - u);
}

/*
 * Circular Gaussian bump centred at mu, evaluated at x.
 * "Circular" means the 360-degree wrap-around is handled: x=361 is treated as x=1.
 * scaling_factor controls the peak amplitude; sigma controls the width.
 * Used in the DNF to build repulsive bumps from obstacle bearings (addObstacle).
 */
float gaussian(const float sigma, const float scaling_factor, const float mu, const float x)
{
    int new_x = x;
    unsigned int dist = fabs(new_x - mu);

    if ((unsigned int)fabs((x - 360) - mu) < dist)
    {
        new_x = x - 360;
        dist = fabs(new_x - mu);
    }
    else if ((unsigned int)fabs((x + 360) - mu) < dist)
    {
        new_x = x + 360;
        dist = fabs(new_x - mu);
    }

    return scaling_factor / (sigma * sqrt(2. * M_PI)) * exp(-pow((new_x - mu) / sigma, 2) / 2.0);
}

/*
 * Circular discrete derivative of a neural-field potential vector.
 * result[i] = scale_factor * (vector[i+1] - vector[i-1]) / 2  (indices wrap around).
 * When evaluated at the robot's heading neuron (i = angle_to_neuron_id(0)):
 *   positive result → field rises to the left  → command turn left
 *   negative result → field rises to the right → command turn right
 */
void differentiate(const float* vector, float* result, const size_t len, const float scale_factor)
{
    unsigned int i, next, previous;

    for (i = 0; i < len; i += 1)
    {
        next = (len + i + 1) % len;
        previous = (len + i - 1) % len;
        result[i] = scale_factor * (vector[next] - vector[previous]) / 2.;
    }
}

/*
 * Log-cosh attractive potential centred at x0 (circular, 360-wrapped).
 * Returns: -log(cosh((x - x0) / spread)) + offset
 * This creates a smooth hill peaking at x=x0 (value = offset) that falls off gently on both sides.
 * Used in the DNF (m_goal_output) to attract the robot toward the target heading.
 * - spread: width of the hill (tuned via ROS param "tuningSpread", default ~207°)
 * - offset: peak height (tuned via ROS param "tuningOffset", default ~1.1)
 */
float target(const float spread, const float offset, const float x0, const float x)
{
    float new_x = x;
    unsigned int dist = fabs(new_x - x0);

    if ((unsigned int)fabs((x - 360) - x0) < dist)
    {
        new_x = x - 360;
        dist = fabs(new_x - x0);
    }
    else if ((unsigned int)fabs((x + 360) - x0) < dist)
    {
        new_x = x + 360;
        dist = fabs(new_x - x0);
    }

    return -log(cosh((new_x - x0) / spread)) + offset;
}

// Legacy function — no longer called in main-strategy. Kept for reference.
float pseudo_gaussian_derivative(const int x)
{
    // Saturate in the tails of the bell so the robot still turns decisively when the goal
    // is far off to the side (in the Gaussian tail the gradient would be near zero otherwise).
    if (x > 50)
        return -24.2;
    if (x < -50)
        return 24.2;

    // Gaussian part: provides a nice (angular) desceleration when we reach the goal
    return -0.8 * x * exp(-pow(x, 2) / 5000.);
    // return -0.2 * x * exp(-pow(x, 2) / 5000.); // TEST
    // return -0.2 * x * exp(-pow(x, 2) / 1000.);
}

// Legacy function — no longer called in main-strategy. Kept for reference.
// Gaussian-derivative repulsion for fusing nearby ultrasonic sensor readings.
float obstacle_temporary_derivative(const int x)
{
    return 4. * x * exp(-pow(x, 2) / 300.);
    // So that nearby uS sensors merge to repell obstacle
    // return 1. * x * exp(-pow(x, 2) / 650.);
}

unsigned int get_idx_of_max(const float vector[], const size_t len)
{
    unsigned int curr_max = 0, i;

    for (i = 1; i < len; i += 1)
    {
        if (vector[i] > vector[curr_max])
            curr_max = i;
    }

    return curr_max;
}

void debug_vector(const char* title,
                  const float* vector,
                  const size_t len,
                  const int display_index,
                  const int display_max)
{
    unsigned int i, idx;
    float max_value;

    printf("%s", title);

    for (i = 0; i < len; i += 10)
    {
        printf("%+.1f ", vector[i]);
    }
    printf("\n");

    if (display_index == TRUE)
    {
        for (i = 0; i < len; i += 10)
        {
            printf("% 4d ", i);
        }

        printf("\n");
    }

    if (display_max == TRUE)
    {
        idx = 0;
        max_value = vector[0];

        for (i = 1; i < len; i += 1)
        {
            if (vector[i] > max_value)
            {
                idx = i;
                max_value = vector[i];
            }
        }

        printf("Max: %f @%d deg\n", max_value, idx);
    }

    printf("\n");
}
