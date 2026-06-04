/**
 * Speed-control logic of the Core node.
 *
 * Grouped here so the speed-control pipeline can be read end-to-end without
 * scrolling through the rest of core.cpp.  Methods belong to the Core class —
 * this file is split for readability, not for class extraction (same pattern
 * as publishers_creator.cpp and subscribers_creator.cpp).
 *
 * Pipeline ordering inside Core::Loop():
 *   1. fineTunePositionPID()      ←─ used only within 2 cm of goal
 *      OR
 *      limitLinearSpeedCmdByGoal()←─ trapezoidal velocity profile to the goal
 *   2. limitLinearSpeedByAngularSpeed() ─ reduce linear cmd while turning
 *   3. limitAngularSpeedCmd()           ─ cap angular cmd to ROS-param max
 *   4. limitAcceleration()              ─ angular acceleration + jerk limits
 */

#include "core.h"

#define MAX_ALLOWED_ANGULAR_SPEED 5.f // rad/s
#define UPDATE_RATE 10                // Hz — must match the value in core.h

// Generic acceleration limiter.  Used by limitAcceleration() for angular speed.
template<typename velocity_t, typename acceleration_t>
static velocity_t limit_acceleration(velocity_t current_velocity,
                                     velocity_t cmd_velocity,
                                     acceleration_t max_acceleration,
                                     double dt)
{
    auto delta_vel = cmd_velocity - current_velocity;
    int delta_vel_sign = (delta_vel > 0) ? 1 : ((delta_vel < 0) ? -1 : 0);

    if (std::abs(delta_vel) > max_acceleration * dt)
    {
        delta_vel = delta_vel_sign * max_acceleration * dt;
    }
    return velocity_t(current_velocity + delta_vel);
}

// Reduce the linear-speed command when the robot is turning hard.
// Uses a Gaussian centred on angular_speed = 0 — full speed when going straight,
// near zero when turning at the sigma rate (0.1 rad/s).
void Core::limitLinearSpeedByAngularSpeed(VitesseAngulaire a_angular_speed)
{
    VitesseAngulaire l_sigma_angular_speed = VitesseAngulaire(0.1f); // rad/s
    float l_scale = 1.f / l_sigma_angular_speed;                     // 0.4f;
    // So that gaussian(0) = 1
    Vitesse linear_speed_limit
      = m_default_linear_speed * gaussian(l_sigma_angular_speed, l_scale, 0, a_angular_speed);

    m_linear_speed_cmd = std::min(m_linear_speed_cmd, linear_speed_limit);
    m_motion_debug_msg.linear_limit_by_angular_speed = static_cast<float>(linear_speed_limit);
    RCLCPP_DEBUG_STREAM(this->get_logger(),
                        "limit linear by angular speed : " << linear_speed_limit << std::endl);
}

// Apply angular-acceleration and angular-jerk caps to m_angular_speed_cmd.
void Core::limitAcceleration()
{
    // Only limit when speed magnitude is increasing; braking is handled elsewhere
    if (std::abs(double(m_angular_speed_cmd)) <= std::abs(double(m_angular_speed)))
    {
        m_angular_accel = m_angular_speed_cmd - m_angular_speed;
        return;
    }

    const double dt = 1.0 / double(UPDATE_RATE);

    // Limit angular acceleration: how fast angular speed can change per step
    m_angular_speed_cmd
      = limit_acceleration(m_angular_speed, m_angular_speed_cmd, m_maxAngularAccel, dt);

    // Limit angular jerk: how fast the acceleration itself can change per step
    AccelerationAngulaire target_accel
      = AccelerationAngulaire((double(m_angular_speed_cmd) - double(m_angular_speed)) / dt);
    double accel_delta = double(target_accel) - double(m_angular_accel);
    double max_accel_delta = double(m_maxAngularJerk) * dt;
    if (std::abs(accel_delta) > max_accel_delta)
    {
        target_accel = AccelerationAngulaire(double(m_angular_accel)
                                             + std::copysign(max_accel_delta, accel_delta));
        m_angular_speed_cmd = VitesseAngulaire(double(m_angular_speed) + double(target_accel) * dt);
    }
    m_angular_accel = target_accel;
}

// Cap angular speed to the absolute max and the per-move max from StratMovement.
void Core::limitAngularSpeedCmd(VitesseAngulaire& a_angular_speed_cmd)
{
    // TODO: use the winning strategy with weights
    // TODO: restore speed limitations
    /*if (s_goal.output->strength == 1) {
            int speed_limited_by_obstacle = s_goal.output->m_angular_speed_inhibition *
    MAX_ALLOWED_ANGULAR_SPEED / 100; MIN(m_angular_speed, speed_limited_by_obstacle);
            MAX(m_angular_speed, -speed_limited_by_obstacle);
    }*/

    // Cap angular speed, so that the robot doesn't turn TOO FAST on itself
    a_angular_speed_cmd
      = std::max(a_angular_speed_cmd, VitesseAngulaire(-MAX_ALLOWED_ANGULAR_SPEED));
    a_angular_speed_cmd
      = std::min(a_angular_speed_cmd, VitesseAngulaire(MAX_ALLOWED_ANGULAR_SPEED));
    a_angular_speed_cmd = std::max(
      a_angular_speed_cmd, VitesseAngulaire(-m_strat_movement_parameters.max_speed.angular.z));
    a_angular_speed_cmd = std::min(
      a_angular_speed_cmd, VitesseAngulaire(m_strat_movement_parameters.max_speed.angular.z));
}

// Trapezoidal velocity profile: accelerate toward max speed, then decelerate in time to arrive
// at m_strat_movement_parameters.max_speed_at_arrival (usually 0 for a precise stop).
// Warning: does not scale well when changing the update rate, as the robot needs time to reach the
// desired speed.
void Core::limitLinearSpeedCmdByGoal()
{
    Acceleration max_acceleration = Acceleration(0.35f); // m/s²
    Acceleration max_deceleration = Acceleration(0.35f); // m/s²

    // Allow faster robot when doing a low-precision movement (where we do not need to stop at the
    // end)
    /*if (m_strat_movement_parameters.max_speed_at_arrival > 0.01f)
    {
        max_acceleration = Acceleration(0.65f); // m*s-2
        max_deceleration = Acceleration(0.45f); // m*s-2
    }*/

    Vitesse new_speed_order = Vitesse(0); // m/s

    Vitesse desired_final_speed = Vitesse(m_strat_movement_parameters.max_speed_at_arrival); // m/s

    float time_to_stop = (m_linear_speed - desired_final_speed) / max_deceleration;
    time_to_stop = std::max(0.f, time_to_stop);
    RCLCPP_DEBUG_STREAM(this->get_logger(), "time to stop = " << time_to_stop << "s, ");

    Distance distance_to_stop
      = Distance(time_to_stop * (m_linear_speed - desired_final_speed) / 2.);
    RCLCPP_DEBUG_STREAM(this->get_logger(), ", distance to stop = " << distance_to_stop << "m, ");

    // Compute extra time if accelerating
    Vitesse average_extra_speed = Vitesse(
      2 * m_linear_speed + Vitesse((max_acceleration / 2. + max_deceleration / 2.) / UPDATE_RATE));
    Distance extra_distance = Distance(average_extra_speed / float(UPDATE_RATE));

    Distance l_distance_to_goal
      = Distance(m_distance_to_goal - getReach(m_strat_movement_parameters.endpoint_frame_id));

    if (l_distance_to_goal < distance_to_stop)
    {
        RCLCPP_DEBUG_STREAM(this->get_logger(), "decelerate");
        new_speed_order = m_linear_speed - max_deceleration / float(UPDATE_RATE);
        m_motion_debug_msg.speed_mode = krabi_msgs::msg::MotionDebug::DECELERATE;
    }
    else if (l_distance_to_goal < m_linear_speed / float(UPDATE_RATE))
    {
        RCLCPP_DEBUG_STREAM(this->get_logger(), "EMERGENCY BRAKE");
        new_speed_order = 0;
        m_motion_debug_msg.speed_mode = krabi_msgs::msg::MotionDebug::EMERGENCY_BRAKE;
    }
    else if (l_distance_to_goal > distance_to_stop + extra_distance)
    {
        RCLCPP_DEBUG_STREAM(this->get_logger(), "accelerate");
        new_speed_order = m_linear_speed + max_acceleration / float(UPDATE_RATE);
        m_motion_debug_msg.speed_mode = krabi_msgs::msg::MotionDebug::ACCELERATE;
    }
    else
    {
        RCLCPP_DEBUG_STREAM(this->get_logger(), "cruise speed");
        new_speed_order = m_linear_speed;
        m_motion_debug_msg.speed_mode = krabi_msgs::msg::MotionDebug::CRUISE;
    }
    new_speed_order
      = std::max(new_speed_order, Vitesse(-m_strat_movement_parameters.max_speed.linear.x));
    new_speed_order
      = std::min(new_speed_order, Vitesse(m_strat_movement_parameters.max_speed.linear.x));
    // m_linear_speed_cmd = MIN(m_default_linear_speed, new_speed_order);
    RCLCPP_DEBUG_STREAM(this->get_logger(),
                        "new speed: " << new_speed_order << " => " << m_linear_speed_cmd
                                      << std::endl);
    m_motion_debug_msg.time_to_stop = time_to_stop;
    m_motion_debug_msg.distance_to_stop = static_cast<float>(distance_to_stop);
    m_motion_debug_msg.new_speed_order = static_cast<float>(new_speed_order);
    m_motion_debug_msg.desired_final_speed = static_cast<float>(desired_final_speed);
    m_linear_speed_cmd = new_speed_order;
}

// Position PID, active only when the robot is within ~2 cm of the goal
// (m_fine_tuning_linear == true).  Replaces the trapezoidal profile for
// the final precise approach.  Currently P-only (KI=KD=0).
void Core::fineTunePositionPID()
{
    m_motion_debug_msg.speed_mode = krabi_msgs::msg::MotionDebug::FINE_TUNING;

    // Signed distance: project (goal - robot) onto robot heading.
    // Positive = goal is ahead, negative = goal is behind.
    // stop_angular() already zeroes rotation, so no 180° turn is possible.
    float dx
      = static_cast<float>(m_goal_pose.getPosition().getX() - m_current_pose.getPosition().getX());
    float dy
      = static_cast<float>(m_goal_pose.getPosition().getY() - m_current_pose.getPosition().getY());
    float heading = static_cast<float>(m_current_pose.getAngle());
    float signed_error = dx * std::cos(heading) + dy * std::sin(heading);

    constexpr float KP = 1.0f;
    constexpr float KI = 0.0f;
    constexpr float KD = 0.0f;
    constexpr float MAX_INTEGRAL = 1.0f; // anti-windup (m·s)
    constexpr float MAX_SPEED = 0.2f;    // m/s, lower than normal 0.5

    m_pid_position_integral += signed_error / static_cast<float>(UPDATE_RATE);
    m_pid_position_integral = std::clamp(m_pid_position_integral, -MAX_INTEGRAL, MAX_INTEGRAL);

    float derivative = (signed_error - m_pid_position_prev_error) * static_cast<float>(UPDATE_RATE);
    m_pid_position_prev_error = signed_error;

    float derivative_contribution = KD * derivative;
    float output = KP * signed_error + KI * m_pid_position_integral + derivative_contribution;
    output = std::clamp(output, -MAX_SPEED, MAX_SPEED);

    RCLCPP_DEBUG_STREAM(this->get_logger(),
                        "fineTunePositionPID: error="
                          << signed_error << " integral=" << m_pid_position_integral
                          << " derivative=" << derivative << " output=" << output);

    m_motion_debug_msg.pid_position_output = output;

    if (reverseGear())
        output = -output;
    m_linear_speed_cmd = Vitesse(output);
}
