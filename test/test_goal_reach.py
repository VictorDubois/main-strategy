"""
Functional simulation test: does the robot actually reach a goal?

This test starts a headless Gazebo simulation, bypasses goal_strategy, and
directly commands main_strategy_node to drive to a position 75 cm ahead of
the start pose. It passes if distance_to_goal < 5 cm within 30 seconds.

Run with:
    colcon test --packages-select main_strategy
Or manually (no Gazebo instance should be running):
    python3 -m pytest test/test_goal_reach.py -v -s

Architecture note: goal_strategy is intentionally excluded. The test publishes
StratMovement directly so it exercises main_strategy in isolation.

Launch sequencing note:
    spawn_world.py and spawn_and_bridge.launch.py both (transitively) declare a
    'world' launch argument and cannot share the same LaunchDescription context
    without conflicting. This file therefore starts Gazebo directly with
    ExecuteProcess (using the gz_world constant), then uses OnProcessExit on a
    'wait_for_gz' probe to gate the robot spawn — the same pattern as
    krabi_start_simu.py.
"""

import math
import os
import threading
import time
import unittest

import pytest
import rclpy
from ament_index_python.packages import get_package_share_directory
from krabi_msgs.msg import MotionDebug, StratMovement
from launch import LaunchDescription
from launch.actions import (ExecuteProcess, IncludeLaunchDescription,
                             RegisterEventHandler)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node as LaunchNode
from launch_ros.substitutions import FindPackageShare
from rclpy.node import Node
from std_msgs.msg import Bool

import launch_testing
import launch_testing.actions

# ---------------------------------------------------------------------------
# Robot start pose — must match the values passed to the launch description
# ---------------------------------------------------------------------------
START_X     = -1.25
START_Y     = -0.75
START_THETA = math.pi / 2   # robot faces +y

# Goal: 75 cm forward (+y direction at pi/2 orientation)
GOAL_X     = -1.25
GOAL_Y     = 0.0
GOAL_THETA = math.pi / 2

# Test parameters
GOAL_TOLERANCE_M         = 0.05   # 5 cm
GOAL_REACH_TIMEOUT_S     = 30.0
GAZEBO_STARTUP_TIMEOUT_S = 90.0   # wait up to 90 s for all nodes to initialise


# ---------------------------------------------------------------------------
# Launch description
# ---------------------------------------------------------------------------

@pytest.mark.launch_test
def generate_test_description():
    """
    Start headless Gazebo, wait for its world service, then spawn the robot
    and the control nodes.  No goal_strategy or lidar_strategy — the test
    injects StratMovement directly.
    """
    pkg = get_package_share_directory('krabi_description')
    os.environ['GZ_SIM_RESOURCE_PATH'] = pkg + '/models:' + pkg + '/worlds'

    # Kill any leftover Gazebo from a previous run, then launch fresh.
    kill_gz = ExecuteProcess(
        cmd=['bash', '-c', 'pkill -f "gz sim" 2>/dev/null; sleep 1'],
        output='screen'
    )

    gz_headless = ExecuteProcess(
        cmd=['gz', 'sim', 'table2026.world', '-v', '-r', '-s', '--headless-rendering'],
        output='screen'
    )

    # Polls every second until the Gazebo world service appears, then exits 0.
    # OnProcessExit fires on exit, so the robot launch runs only once Gazebo is up.
    wait_for_gz = ExecuteProcess(
        cmd=['bash', '-c',
             'until gz service -l 2>/dev/null | grep -q "^/world/"; do sleep 1; done'
             ' && echo "[test_goal_reach] Gazebo world service ready"'],
        output='screen'
    )

    # --- Nodes to start once Gazebo is ready ---

    # Robot URDF spawn + ROS-Gazebo bridge.
    # Uses 'zRobotOrientation' (the declared arg name in spawn_and_bridge.launch.py).
    spawn_robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('krabi_description'), 'launch', 'spawn_and_bridge.launch.py'
            ])
        ]),
        launch_arguments={
            'xRobotPos':         str(START_X),
            'yRobotPos':         str(START_Y),
            'zRobotOrientation': str(START_THETA),
            'isBlue':            'True',
        }.items()
    )

    # map → odom static transform (robot initial position in the map frame)
    map_to_odom_tf = LaunchNode(
        package='tf2_ros',
        executable='static_transform_publisher',
        namespace='krabi_ns',
        arguments=[
            '--x', str(START_X),
            '--y', str(START_Y),
            '--z', '0',
            '--roll', '0', '--pitch', '0', '--yaw', str(START_THETA),
            '--child-frame-id', 'odom',
            '--frame-id', 'map',
        ],
        parameters=[{'use_sim_time': True}]
    )

    # Static arm/tool transforms required by main_strategy (getReach calls)
    grabi_tf = LaunchNode(
        package='tf2_ros',
        executable='static_transform_publisher',
        namespace='krabi_ns',
        arguments=['--x', '0.17', '--y', '0', '--z', '0',
                   '--roll', '0', '--pitch', '0', '--yaw', '0',
                   '--child-frame-id', 'grabi', '--frame-id', 'base_link'],
        parameters=[{'use_sim_time': True}]
    )
    billig_tf = LaunchNode(
        package='tf2_ros',
        executable='static_transform_publisher',
        namespace='krabi_ns',
        arguments=['--x', '0', '--y', '-0.2', '--z', '0',
                   '--roll', '0', '--pitch', '0', '--yaw', '0',
                   '--child-frame-id', 'billig', '--frame-id', 'base_link'],
        parameters=[{'use_sim_time': True}]
    )

    # Odometry node: integrates Gazebo wheel odometry → base_link→odom TF + /krabi_ns/odom
    odom_node = LaunchNode(
        package='main_strategy',
        namespace='krabi_ns',
        executable='odometry_node',
        name='odom_tf',
        parameters=[
            {'init_pose/x':     START_X},
            {'init_pose/y':     START_Y},
            {'init_pose/theta': START_THETA},
            {'publish_tf_odom': True},
            {'use_sim_time':    True},
        ]
    )

    # Main strategy node — the system under test
    main_strat = LaunchNode(
        package='main_strategy',
        namespace='krabi_ns',
        executable='main_strategy_node',
        name='main_strat',
        parameters=[
            {'isBlue':          True},
            {'maxAccel':        0.1},
            {'maxAngularAccel': 3.0},
            {'maxAngularJerk':  5.0},
            {'tuningSpread':    220.0},
            {'tuningOffset':    1.1},
            {'maxCurrent':      3.0},
            {'use_sim_time':    True},
        ]
    )

    return LaunchDescription([
        kill_gz,
        RegisterEventHandler(
            OnProcessExit(
                target_action=kill_gz,
                on_exit=[
                    gz_headless,
                    wait_for_gz,
                    RegisterEventHandler(
                        OnProcessExit(
                            target_action=wait_for_gz,
                            on_exit=[
                                spawn_robot,
                                map_to_odom_tf,
                                grabi_tf,
                                billig_tf,
                                odom_node,
                                main_strat,
                                # Signal to launch_testing that test methods can start.
                                # The test still waits for cmd_vel before doing anything.
                                launch_testing.actions.ReadyToTest(),
                            ]
                        )
                    ),
                ]
            )
        ),
    ])


# ---------------------------------------------------------------------------
# Test class
# ---------------------------------------------------------------------------

class TestRobotReachesGoal(unittest.TestCase):
    """
    Integration smoke test: command the robot to drive 75 cm forward and
    verify it arrives within GOAL_REACH_TIMEOUT_S seconds.
    """

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = Node('test_goal_reach')

        cls.tirette_pub = cls.node.create_publisher(Bool, '/krabi_ns/tirette', 1)
        cls.strat_pub   = cls.node.create_publisher(
            StratMovement, '/krabi_ns/strat_movement', 1)

        cls._goal_reached = threading.Event()
        cls._last_distance = None

        cls.node.create_subscription(
            MotionDebug, '/krabi_ns/motion_debug', cls._on_motion_debug, 10)

        # Spin rclpy in a background thread so subscriptions are processed
        cls._spin_thread = threading.Thread(
            target=rclpy.spin, args=(cls.node,), daemon=True)
        cls._spin_thread.start()

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    @classmethod
    def _on_motion_debug(cls, msg: MotionDebug):
        cls._last_distance = msg.distance_to_goal
        if msg.distance_to_goal < GOAL_TOLERANCE_M:
            cls._goal_reached.set()

    # ------------------------------------------------------------------

    def _wait_for_system_ready(self):
        """
        Block until main_strategy_node publishes on cmd_vel.
        This confirms Gazebo is up, the bridge is running, and all nodes initialised.
        """
        deadline = time.time() + GAZEBO_STARTUP_TIMEOUT_S
        while time.time() < deadline:
            if self.node.count_publishers('/krabi_ns/cmd_vel') > 0:
                return True
            time.sleep(1.0)
        return False

    def _publish_goal(self):
        """Publish the StratMovement goal several times to ensure delivery."""
        msg = StratMovement()
        msg.orient       = StratMovement.GO_TO_GOALPOSE_POSITION
        msg.reverse_gear = StratMovement.FORWARD_OR_REVERSE
        msg.max_speed.linear.x   = 0.3   # m/s — moderate speed for stability
        msg.max_speed_at_arrival = 0.0   # full stop at goal

        msg.goal_pose.header.frame_id = 'map'
        msg.goal_pose.header.stamp    = self.node.get_clock().now().to_msg()
        msg.goal_pose.pose.position.x = GOAL_X
        msg.goal_pose.pose.position.y = GOAL_Y
        # Orientation as quaternion (only yaw matters for a ground robot)
        msg.goal_pose.pose.orientation.z = math.sin(GOAL_THETA / 2.0)
        msg.goal_pose.pose.orientation.w = math.cos(GOAL_THETA / 2.0)

        for _ in range(10):   # 10 × 200 ms = 2 s of repeated publishing
            self.strat_pub.publish(msg)
            time.sleep(0.2)

    def _publish_tirette(self):
        """Simulate the start signal (tirette pulled)."""
        msg = Bool(data=True)
        for _ in range(5):
            self.tirette_pub.publish(msg)
            time.sleep(0.1)

    # ------------------------------------------------------------------

    def test_robot_reaches_forward_goal(self):
        """
        Robot starts at (-1.25, -0.75, pi/2) and must reach (-1.25, 0.0)
        (75 cm forward in the +y direction) within 30 s.
        """
        # Step 1: wait for Gazebo + nodes to be ready
        ready = self._wait_for_system_ready()
        self.assertTrue(
            ready,
            f"System did not become ready within {GAZEBO_STARTUP_TIMEOUT_S} s. "
            "Is Gazebo installed and headless-rendering working?"
        )

        # Step 2: send the goal BEFORE the tirette so main_strategy
        #         already knows where to go the instant the match starts.
        self._publish_goal()

        # Step 3: start the match (tirette pulled)
        self._publish_tirette()

        # Step 4: keep re-sending the goal at ~1 Hz until arrival or timeout
        #         (main_strategy may drop the first messages during state transition)
        deadline = time.time() + GOAL_REACH_TIMEOUT_S
        while time.time() < deadline and not self._goal_reached.is_set():
            self._publish_goal()

        # Step 5: assert
        dist_str = (f"{self._last_distance:.3f} m"
                    if self._last_distance is not None else "unknown")
        self.assertTrue(
            self._goal_reached.is_set(),
            f"Robot did not reach goal ({GOAL_X}, {GOAL_Y}) within "
            f"{GOAL_REACH_TIMEOUT_S} s. "
            f"Last distance_to_goal: {dist_str}"
        )
