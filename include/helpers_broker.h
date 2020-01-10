#ifndef _HELPERS_BROKER_H
#define _HELPERS_BROKER_H
#ifdef __cplusplus
extern "C" {
#endif
#define SPEED_SMOOTH_FACTOR 0.10 //0.05

#define TRUE 1
#define FALSE 0

int degs_to_ticks (int deg);
unsigned int get_orientation (long encoder1, long encoder2);
float compute_linear_speed (long encoder1, long encoder2, long elapsed);
int get_sensor_values (int fd, void * buf, ssize_t msg_size);
void update_speed (const int immediate, int *current_speed, const int new_speed);
float pseudo_gaussian_derivative(const int x);
float obstacle_temporary_derivative (const int x);
float apply_circular (float f (int), const int u, const int x);
float gaussian (const float sigma, const float a, const float mu, const float x);
unsigned int get_idx_of_max (const float vector[], const size_t len);
void differentiate (const float *vector, float *result, const size_t len, const float scale_factor);
float target (const float spread, const float offset, const float x0, const float x);
void debug_vector (const char * title, const float * vector, const size_t len, const int display_index, const int display_max);
#ifdef __cplusplus
}
#endif
#endif
