# Main node of the robot [Krabi](https://github.com/VictorDubois/krabi)

Takes the inputs of the other nodes (Goal and Lidar), and send a speed command to the motors

From most important (top) to least important (bottom)
<img width="1346" height="723" alt="rosgraph_krabi (1)" src="https://github.com/user-attachments/assets/628c8b35-7644-413c-bbe8-3c72bf5db8b6" />

# Inputs

## strat_movement <= most important message
Custom message, from goal_strat, to detail where and how the robot should move (type: [krabi_msgs/msg/StratMovement](https://github.com/VictorDubois/krabi-msgs/blob/feature/ROS2/msg/StratMovement.msg))

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
Controls the mode in which the motors are
type: custom message

### motors_parameters
How much current are the motors allowed to consume
type: custom message

### Enable_motor
Boolean to stop the motors. Should be delete, redondant with motors_cmd
