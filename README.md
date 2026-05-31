# Main node of the robot [Krabi](https://github.com/VictorDubois/krabi)

Takes the inputs of the other nodes (Goal and Lidar), and send a speed command to the motors

From most important (top) to least important (bottom)
<img width="1346" height="723" alt="rosgraph_krabi (1)" src="https://github.com/user-attachments/assets/628c8b35-7644-413c-bbe8-3c72bf5db8b6" />

# Inputs

## strat_movement <= most important message
Custom message, from goal_strat, to detail where and how the robot should move (type: [krabi_msgs/msg/StratMovement](https://github.com/VictorDubois/krabi-msgs/blob/feature/ROS2/msg/StratMovement.msg))
- What the the goal's pose
- Should the robot go forward/backwards
- What is the absolute max speed
- What is the max speed when reaching the goal
- What should the mode be (rotation only, translatation only, recalage bordure... See full list in the message)
- Should the odometry be reset

## obstacle_pose_stamped
Most threatening obstacle in front of the robot (type: [PoseStamped](https://docs.ros2.org/foxy/api/geometry_msgs/msg/PoseStamped.html))

### obstacle_behind_pose_stamped
See [obstacle_pose_stamped](#obstacle_pose_stamped) (but behind the robot)

## odom
The odometry is used to know the current speed (both linear and angular) of the robot. (type: [Odometry](https://docs.ros2.org/foxy/api/nav_msgs/msg/Odometry.html))
The position is read from the TF transform

### /krabi_ns/odom
See [odom](#odom) (small hack: the two go to the same callback, one of the two is used for simulation, the other for the real robot)

## tirette
The signal to start the match (type: [Bool](https://docs.ros2.org/foxy/api/std_msgs/msg/Bool.html))
- False: the tirette is still inside the robot, therefore the match has not started yet
- True: the tirette is outside the robot, the match has started


# Outputs

## Motor outputs

### cmd_vel (=Command of Velocity)
Standard ROS/ROS2 message, 3D linear and angular speed order (type: [Twist](https://docs.ros2.org/foxy/api/geometry_msgs/msg/Twist.html))
=> Speed order for the Motor board

### /krabi_ns/cmd_vel
See [cmd_vel](#cmd_vel) 

### motors_cmd
Controls the mode in which the motors are (custom type: [MotorsCmd](https://github.com/VictorDubois/krabi-msgs/blob/feature/ROS2/msg/MotorsCmd.msg))
- enable/disable the motors
- enable/disable the PID (and override output values)
- reset the odometry

### motors_parameters
How much current are the motors allowed to consume (custom type: [MotorsParameters](https://github.com/VictorDubois/krabi-msgs/blob/feature/ROS2/msg/MotorsParameters.msg))

### enable_motor
Boolean to stop the motors (type: [Bool](https://docs.ros2.org/foxy/api/std_msgs/msg/Bool.html))
Note: @todo Should be delete, redondant with motors_cmd

## Diagnostics

### diagnostics
Used for this screen (type: [DiagnosticArray](https://docs.ros2.org/foxy/api/diagnostic_msgs/msg/DiagnosticArray.html))

<img width="100" height="50" alt="image" src="https://github.com/user-attachments/assets/dc3ff701-6853-42a4-b6e3-c8ad0786c019" />

### motion_debug
Custom debug message, used mostly for ploting (custom type: [MotionDebug](https://github.com/VictorDubois/krabi-msgs/blob/feature/ROS2/msg/MotionDebug.msg))

### target_orientation
Orientation that the robot targets. Centered on the robot (type: [PoseStamped](https://docs.ros2.org/foxy/api/geometry_msgs/msg/PoseStamped.html))
Used for displaying in the 3d view

## Others
### remaining_time
The number of seconds remaining until the end of the match (type: [Duration](https://docs.ros2.org/foxy/api/builtin_interfaces/msg/Duration.html))

# What to do for a new year?
Not much, this repo is completely generic year-wise
