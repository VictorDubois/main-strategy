# Main node of the robot [Krabi](https://github.com/VictorDubois/krabi)

Takes the inputs of the other nodes, and send a speed command to the motors

<img width="1273" height="1044" alt="rosgraph_mainstrat" src="https://github.com/user-attachments/assets/704167f1-8031-4cd5-bcc7-351a7a33ad57" />

## Inputs

### strat_movement <= most important message
Custom message, from goal_strat, to detail where and how the robot should move (type: [krabi_msgs/msg/StratMovement](https://github.com/VictorDubois/krabi-msgs/blob/feature/ROS2/msg/StratMovement.msg))

### obstacle_pose_stamped
Most threatening obstacle in front of the robot (type: [PoseStamped](https://docs.ros2.org/foxy/api/geometry_msgs/msg/PoseStamped.html))

### obstacle_behind_pose_stamped
See [obstacle_pose_stamped](#obstacle_pose_stamped) (but behind the robot)

### odom
The odometry is used to know the current speed (both linear and angular) of the robot. (type: [Odometry](https://docs.ros2.org/foxy/api/nav_msgs/msg/Odometry.html))
The position is read from the TF transform

### /krabi_ns/odom
See [odom](#odom) (small hack: the two go to the same callback, one of the two is used for simulation, the other for the real robot)

### tirette
The signal to start the match (type: [Bool](https://docs.ros2.org/foxy/api/std_msgs/msg/Bool.html))
- False: the tirette is still inside the robot, therefore the match has not started yet
- True: the tirette is outside the robot, the match has started


## Outputs
