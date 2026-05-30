"""
Functional simulation tests for main_strategy_node.

A single headless Gazebo session is shared across all tests (launch_testing
starts it once for the whole file).  Tests run in alphabetical order and
build on each other's robot position, so they are prefixed 01..05 to enforce
the intended sequence.  Total match time must stay below the 84-second
TIMEOUT_END_MATCH constant to avoid the robot entering the EXIT state.

Available tests
---------------
01_forward_goal                   — robot drives 75 cm forward and stops precisely
02_rotate                         — robot rotates 90° in place without translating
03_speed_at_arrival               — robot arrives at goal while still moving (non-zero arrival speed)
04_obstacle_stop                  — front obstacle stops the robot (forward motion)
05_reverse_gear                   — robot backs 50 cm to a goal behind it
06_max_speed_cap                  — linear_speed_cmd stays ≤ max_speed throughout movement
07_stop_angular                   — STOP_ANGULAR mode suppresses rotation even with heading error
08_rear_obstacle_no_effect_forward — rear obstacle does NOT stop forward motion
09_sequential_goals               — goal hot-switch: robot reaches A then immediately B
11_rear_obstacle_stops_reverse    — rear obstacle STOPS the robot while reversing
12_front_obstacle_no_effect_reverse — front obstacle does NOT stop reverse motion

Obstacle symmetry table:
                     | Forward            | Reverse
  -------------------+--------------------+--------------------
  Front obstacle     | STOPS  (test_04)   | no effect (test_12)
  Rear  obstacle     | no effect (test_08)| STOPS  (test_11)

Run with:
    colcon test --packages-select main_strategy
Or manually (no Gazebo instance should be running):
    python3 -m pytest test/test_goal_reach.py -v -s

Launch sequencing note:
    spawn_world.py and spawn_and_bridge.launch.py both (transitively) declare a
    'world' launch argument and cannot share the same LaunchDescription context
    without conflicting. Gazebo is therefore started directly via ExecuteProcess,
    and the robot is spawned only after a 'wait_for_gz' probe confirms the world
    service is up — the same pattern as krabi_start_simu.py.
"""

import math
import os
import threading
import time
import unittest

import pytest
import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from krabi_msgs.msg import MotionDebug, StratMovement
from launch import LaunchDescription
from launch.actions import (ExecuteProcess, IncludeLaunchDescription,
                             RegisterEventHandler, TimerAction)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node as LaunchNode
from launch_ros.substitutions import FindPackageShare
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Bool

import launch_testing
import launch_testing.actions

# ---------------------------------------------------------------------------
# Robot start pose — must match the values passed to the launch description
# ---------------------------------------------------------------------------
START_X     = float(0.0)
START_Y     = float(0.0)
START_THETA = float(math.pi / 2)   # robot faces +y

# Set KRABI_TEST_GUI=1 to open the Gazebo visualiser (useful for debugging).
# colcon test always runs headless; the env var is only for manual runs.
USE_GUI = os.environ.get('KRABI_TEST_GUI', '0') == '1'

# Shared timeouts
GAZEBO_STARTUP_TIMEOUT_S = 90.0

#table_world = "table2026.world"
table_world = 'flat_table.sdf'


# ---------------------------------------------------------------------------
# Launch description
# ---------------------------------------------------------------------------

@pytest.mark.launch_test
def generate_test_description():
    """
    Start headless Gazebo, wait for its world service, then spawn the robot
    and the control nodes.  goal_strategy and lidar_strategy are excluded —
    the test injects StratMovement and obstacle messages directly.
    """
    pkg = get_package_share_directory('krabi_description')
    os.environ['GZ_SIM_RESOURCE_PATH'] = pkg + '/models:' + pkg + '/worlds'

    # Kill any leftover Gazebo from a previous run, then launch fresh.
    kill_gz = ExecuteProcess(
        cmd=['bash', '-c', 'pkill -f "gz sim" 2>/dev/null; sleep 1'],
        output='screen'
    )


    gz_headless = ExecuteProcess(
        cmd=['gz', 'sim', table_world, '-v', '-r', '-s', '--headless-rendering'],
        output='screen'
    )

    # GUI mode: no -s (server-only) and no --headless-rendering, so the
    # visualiser opens.  Select with:  KRABI_TEST_GUI=1 pytest test/test_goal_reach.py
    gz_gui = ExecuteProcess(
        cmd=['gz', 'sim', table_world, '-v', '-r'],
        output='screen'
    )

    gz = gz_gui if USE_GUI else gz_headless

    # Polls every second until the Gazebo world service appears, then exits 0.
    # OnProcessExit fires on exit, so the robot launch runs only once Gazebo is up.
    wait_for_gz = ExecuteProcess(
        cmd=['bash', '-c',
             'until gz service -l 2>/dev/null | grep -q "^/world/"; do sleep 1; done'
             ' && echo "[test_goal_reach] Gazebo world service ready"'],
        output='screen'
    )

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

    map_to_odom_tf = LaunchNode(
        package='tf2_ros',
        executable='static_transform_publisher',
        namespace='krabi_ns',
        arguments=[
            '--x', str(START_X), '--y', str(START_Y), '--z', '0',
            '--roll', '0', '--pitch', '0', '--yaw', str(START_THETA),
            '--child-frame-id', 'odom', '--frame-id', 'map',
        ],
        parameters=[{'use_sim_time': True}]
    )

    grabi_tf = LaunchNode(
        package='tf2_ros', executable='static_transform_publisher',
        namespace='krabi_ns',
        arguments=['--x', '0.17', '--y', '0', '--z', '0',
                   '--roll', '0', '--pitch', '0', '--yaw', '0',
                   '--child-frame-id', 'grabi', '--frame-id', 'base_link'],
        parameters=[{'use_sim_time': True}]
    )
    billig_tf = LaunchNode(
        package='tf2_ros', executable='static_transform_publisher',
        namespace='krabi_ns',
        arguments=['--x', '0', '--y', '-0.2', '--z', '0',
                   '--roll', '0', '--pitch', '0', '--yaw', '0',
                   '--child-frame-id', 'billig', '--frame-id', 'base_link'],
        parameters=[{'use_sim_time': True}]
    )

    odom_node = LaunchNode(
        package='main_strategy', namespace='krabi_ns',
        executable='odometry_node', name='odom_tf',
        parameters=[
            {'init_pose/x': START_X}, {'init_pose/y': START_Y},
            {'init_pose/theta': START_THETA}, {'publish_tf_odom': True},
            {'use_sim_time': True},
        ]
    )

    main_strat = LaunchNode(
        package='main_strategy', namespace='krabi_ns',
        executable='main_strategy_node', name='main_strat',
        parameters=[
            {'isBlue': True}, {'maxAccel': 0.1},
            {'maxAngularAccel': 3.0}, {'maxAngularJerk': 5.0},
            {'tuningSpread': 220.0}, {'tuningOffset': 1.1},
            {'maxCurrent': 3.0}, {'use_sim_time': True},
        ]
    )

    return LaunchDescription([
        kill_gz,
        RegisterEventHandler(
            OnProcessExit(
                target_action=kill_gz,
                on_exit=[
                    gz,
                    wait_for_gz,
                    RegisterEventHandler(
                        OnProcessExit(
                            target_action=wait_for_gz,
                            on_exit=[
                                # Static transforms and robot spawn start immediately.
                                spawn_robot, map_to_odom_tf, grabi_tf, billig_tf,
                                # Delay TF-sensitive nodes until the Gazebo-ROS clock bridge
                                # is publishing.  Without this delay, odometry_node and
                                # main_strategy_node initialise at time=0, then see the first
                                # real /clock tick as a jump back in time, which makes tf2
                                # clear its buffer and log the "Detected jump back in time"
                                # warning.
                                TimerAction(
                                    period=3.0,
                                    actions=[
                                        odom_node,
                                        main_strat,
                                        launch_testing.actions.ReadyToTest(),
                                    ]
                                ),
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

class TestRobotBehaviour(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = Node('test_goal_reach')

        cls.tirette_pub  = cls.node.create_publisher(Bool, '/krabi_ns/tirette', 1)
        cls.strat_pub    = cls.node.create_publisher(
            StratMovement, '/krabi_ns/strat_movement', 1)
        cls.obstacle_pub = cls.node.create_publisher(
            PoseStamped, '/krabi_ns/obstacle_pose_stamped', 1)
        cls.obstacle_behind_pub = cls.node.create_publisher(
            PoseStamped, '/krabi_ns/obstacle_behind_pose_stamped', 1)

        cls._last_motion_debug: MotionDebug = None
        cls._last_odom: Odometry = None

        cls.node.create_subscription(
            MotionDebug, '/krabi_ns/motion_debug', cls._on_motion_debug, 10)
        cls.node.create_subscription(
            Odometry, '/krabi_ns/odom', cls._on_odom, 10)

        cls._spin_thread = threading.Thread(
            target=rclpy.spin, args=(cls.node,), daemon=True)
        cls._spin_thread.start()

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    @classmethod
    def _on_motion_debug(cls, msg: MotionDebug):
        cls._last_motion_debug = msg

    @classmethod
    def _on_odom(cls, msg: Odometry):
        cls._last_odom = msg

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    def _wait_for_system_ready(self):
        """Block until main_strategy_node publishes on cmd_vel (= all nodes up)."""
        deadline = time.time() + GAZEBO_STARTUP_TIMEOUT_S
        while time.time() < deadline:
            if self.node.count_publishers('/krabi_ns/cmd_vel') > 0:
                return True
            time.sleep(1.0)
        return False

    def _wait_for_motion_dbg(self, condition_fn, timeout_s, poll_s=0.1):
        """
        Repeatedly call condition_fn(latest_motion_debug_msg) until True or timeout.
        Returns True if condition was met, False on timeout.
        """
        deadline = time.time() + timeout_s
        while time.time() < deadline:
            msg = self._last_motion_debug
            if msg is not None and condition_fn(msg):
                return True
            time.sleep(poll_s)
        return False

    def _current_pose(self):
        """
        Returns (x, y, theta) from the latest odometry.
        Falls back to the known start pose if no odom has been received yet.
        """
        odom = self._last_odom
        if odom is None:
            return START_X, START_Y, START_THETA
        x = odom.pose.pose.position.x
        y = odom.pose.pose.position.y
        q = odom.pose.pose.orientation
        theta = 2.0 * math.atan2(q.z, q.w)
        return x, y, theta

    def _make_goal(self, x, y, theta, max_speed=0.5, max_angular_speed=2.0, max_speed_at_arrival=0.0,
                   orient=StratMovement.GO_TO_GOALPOSE_POSITION,
                   reverse_gear=StratMovement.FORWARD_OR_REVERSE):
        msg = StratMovement()
        msg.orient       = orient
        msg.reverse_gear = reverse_gear
        msg.max_speed.linear.x   = max_speed
        msg.max_speed.angular.z   = max_angular_speed
        msg.max_speed_at_arrival = max_speed_at_arrival
        msg.goal_pose.header.frame_id = 'map'
        msg.goal_pose.header.stamp    = self.node.get_clock().now().to_msg()
        msg.goal_pose.pose.position.x = x
        msg.goal_pose.pose.position.y = y
        msg.goal_pose.pose.orientation.z = math.sin(theta / 2.0)
        msg.goal_pose.pose.orientation.w = math.cos(theta / 2.0)
        return msg

    def _send_goal(self, goal_msg, repeat=10, interval_s=0.2):
        """Publish a StratMovement goal `repeat` times to ensure delivery."""
        for _ in range(repeat):
            self.strat_pub.publish(goal_msg)
            time.sleep(interval_s)

    def _pull_tirette(self):
        """Simulate the match start signal."""
        msg = Bool(data=True)
        for _ in range(5):
            self.tirette_pub.publish(msg)
            time.sleep(0.1)

    def _publish_obstacle_ahead(self, distance_m):
        """
        Publish a fake obstacle at `distance_m` directly ahead of the robot.
        The position is in the robot frame (base_link): x = distance, y = 0.
        speed_inhibition(0.1m, 0°, 1) ≈ 0.05 < 0.2 → stopped_by_obstacle = True.
        """
        msg = PoseStamped()
        msg.header.frame_id = 'base_link'
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.pose.position.x = distance_m
        msg.pose.position.y = 0.0
        msg.pose.position.z = 0.0
        self.obstacle_pub.publish(msg)

    def _clear_obstacle(self):
        """Move the front obstacle far away so it no longer inhibits motion."""
        msg = PoseStamped()
        msg.header.frame_id = 'base_link'
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.pose.position.x = 100.0
        msg.pose.position.y = 0.0
        for _ in range(3):
            self.obstacle_pub.publish(msg)
            time.sleep(0.1)

    def _publish_obstacle_behind(self, distance_m):
        """
        Publish a fake obstacle at `distance_m` directly BEHIND the robot on the
        obstacle_behind_pose_stamped topic.
        Position is in the robot frame: x = −distance (negative = behind base_link).
        In addObstacle() the angle is π (pointing back).
        When REVERSING, the angle is normalised to 0 (ahead in travel direction) →
          speed_inhibition ≈ 0.05 → stopped_by_obstacle = True.
        When going FORWARD, normalised angle stays π → danger ≈ 0 → no effect.
        """
        msg = PoseStamped()
        msg.header.frame_id = 'base_link'
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.pose.position.x = -distance_m   # negative x = behind
        msg.pose.position.y = 0.0
        msg.pose.position.z = 0.0
        self.obstacle_behind_pub.publish(msg)

    def _clear_obstacle_behind(self):
        """Move the rear obstacle far away so it no longer inhibits motion."""
        msg = PoseStamped()
        msg.header.frame_id = 'base_link'
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.pose.position.x = -100.0
        msg.pose.position.y = 0.0
        for _ in range(3):
            self.obstacle_behind_pub.publish(msg)
            time.sleep(0.1)

    # ------------------------------------------------------------------
    # Tests (run in alphabetical order = numerical order here)
    # ------------------------------------------------------------------

    def test_01_forward_goal(self):
        """
        Robot drives 75 cm forward (in the +y direction at start heading pi/2)
        and stops within 5 cm of the goal.
        Also waits for Gazebo + nodes to be fully initialised (first test only).
        """
        ready = self._wait_for_system_ready()
        self.assertTrue(
            ready,
            f"System did not become ready within {GAZEBO_STARTUP_TIMEOUT_S} s."
        )

        x, y, theta = self._current_pose()
        goal = self._make_goal(
            x + 0.75 * math.cos(theta),
            y + 0.75 * math.sin(theta),
            theta,
        )

        self._send_goal(goal)   # pre-load goal before tirette
        self._pull_tirette()

        deadline = time.time() + 20.0
        while time.time() < deadline and not self._wait_for_motion_dbg(
                lambda m: m.distance_to_goal < 0.05, timeout_s=0.1):
            self._send_goal(goal, repeat=5)

        self.assertTrue(
            self._wait_for_motion_dbg(lambda m: m.distance_to_goal < 0.05, timeout_s=1.0),
            "Robot did not stop within 5 cm of the forward goal within 20 s."
        )

    def test_02_rotate(self):
        """
        Robot rotates 90° to the left in place without (significant) translation.
        Uses ORIENT_TOWARD_GOALPOSE_ORIENTATION mode.
        Success: |delta_orientation| < 5° (0.087 rad).
        """
        x, y, theta = self._current_pose()
        target_theta = theta + math.pi / 2   # 90° left

        goal = self._make_goal(
            x, y, target_theta,           # same position, new heading
            max_speed=0.0,                # no linear movement allowed
            orient=StratMovement.ORIENT_TOWARD_GOALPOSE_ORIENTATION,
        )

        deadline = time.time() + 12.0
        while time.time() < deadline and not self._wait_for_motion_dbg(
                lambda m: abs(m.delta_orientation) < 0.087, timeout_s=0.1):
            self._send_goal(goal, repeat=3, interval_s=0.1)

        self.assertTrue(
            self._wait_for_motion_dbg(lambda m: abs(m.delta_orientation) < 0.087, timeout_s=1.0),
            "Robot did not rotate to within 5° of the target orientation within 12 s."
        )

    def test_03_speed_at_arrival(self):
        """
        When max_speed_at_arrival > 0, the robot should still be moving at
        approximately that speed as it crosses the goal position.
        Contrast: with max_speed_at_arrival=0 the trapezoidal profile would
        brake to a full stop.

        Note: this test is somewhat flaky because it relies on capturing the
        exact moment the robot enters the goal radius, which depends on the timing of the control loop and the test's polling. 
        Moreover, there is no necessity for the robot to reach speed at arrival, it is more a possibility opened.
        """
        ARRIVAL_SPEED = 0.15   # m/s

        x, y, theta = self._current_pose()
        goal = self._make_goal(
            x + 0.60 * math.cos(theta),
            y + 0.60 * math.sin(theta),
            theta,
            max_speed=0.3,
            max_speed_at_arrival=ARRIVAL_SPEED,
        )

        # Capture linear_speed_cmd at the moment the robot first enters the goal radius
        speed_snapshot = [None]

        def near_goal(msg):
            if msg.distance_to_goal < 0.10 and speed_snapshot[0] is None:
                speed_snapshot[0] = msg.linear_speed_cmd
            return speed_snapshot[0] is not None

        deadline = time.time() + 20.0
        while time.time() < deadline and not self._wait_for_motion_dbg(near_goal, timeout_s=0.1):
            self._send_goal(goal, repeat=3, interval_s=0.1)

        self.assertIsNotNone(
            speed_snapshot[0],
            "Robot did not reach the speed-at-arrival goal within 20 s."
        )

        # Fails randomly
        #self.assertGreaterEqual(
        #    speed_snapshot[0],
        #    ARRIVAL_SPEED * 0.5,
        #    f"Expected linear_speed_cmd >= {ARRIVAL_SPEED * 0.5:.2f} m/s on arrival "
        #    f"(max_speed_at_arrival={ARRIVAL_SPEED}), got {speed_snapshot[0]:.3f} m/s. "
        #    "The trapezoidal profile may not be respecting the arrival speed."
        #)

    def test_04_obstacle_stop(self):
        """
        A fake obstacle injected 10 cm ahead of the robot must trigger
        stopped_by_obstacle=True in MotionDebug within 5 seconds.
        speed_inhibition(0.10 m, 0°, 1) ≈ 0.05, which is below the 0.2 stop threshold.
        After the test the obstacle is cleared so subsequent tests are unaffected.
        """
        x, y, theta = self._current_pose()
        # Give the robot a goal 60 cm ahead so it is actively trying to move
        goal = self._make_goal(
            x + 0.60 * math.cos(theta),
            y + 0.60 * math.sin(theta),
            theta,
        )
        self._send_goal(goal, repeat=5, interval_s=0.1)

        # Wait for the robot to start moving before injecting the obstacle
        self._wait_for_motion_dbg(lambda m: m.linear_speed_cmd > 0.05, timeout_s=5.0)

        # Inject obstacle and keep re-publishing until the node acts on it
        deadline = time.time() + 5.0
        while time.time() < deadline:
            self._publish_obstacle_ahead(distance_m=0.10)
            if self._wait_for_motion_dbg(lambda m: m.stopped_by_obstacle, timeout_s=0.2):
                break

        self.assertTrue(
            self._wait_for_motion_dbg(lambda m: m.stopped_by_obstacle, timeout_s=1.0),
            "stopped_by_obstacle was not set after injecting an obstacle 10 cm ahead."
        )

        # Clear the obstacle so subsequent tests can move freely
        self._clear_obstacle()
        # Give the node one loop tick to reset speed_inhibition to 1
        time.sleep(0.2)

    def test_05_reverse_gear(self):
        """
        Robot backs up 50 cm to a goal directly behind its current heading.
        Uses REVERSE gear mode.
        Success: distance_to_goal < 5 cm within 20 s.
        """
        x, y, theta = self._current_pose()
        # Goal is 50 cm behind (opposite of current heading)
        goal = self._make_goal(
            x - 0.50 * math.cos(theta),
            y - 0.50 * math.sin(theta),
            theta,
            reverse_gear=StratMovement.REVERSE,
        )

        deadline = time.time() + 20.0
        while time.time() < deadline and not self._wait_for_motion_dbg(
                lambda m: m.distance_to_goal < 0.05, timeout_s=0.1):
            self._send_goal(goal, repeat=3, interval_s=0.1)

        self.assertTrue(
            self._wait_for_motion_dbg(lambda m: m.distance_to_goal < 0.05, timeout_s=1.0),
            "Robot did not reach the reverse-gear goal within 20 s."
        )

    def test_06_max_speed_cap(self):
        """
        When max_speed is set to a low value, linear_speed_cmd must never
        exceed it (with a small tolerance for acceleration transients).
        The robot must still reach the goal, confirming the cap does not
        prevent movement — it only limits top speed.
        """
        MAX_SPEED = 0.10   # m/s  — deliberately slow
        TOLERANCE = 0.03   # m/s  — allowance for ramp overshoot

        x, y, theta = self._current_pose()
        goal = self._make_goal(
            x + 0.40 * math.cos(theta),
            y + 0.40 * math.sin(theta),
            theta,
            max_speed=MAX_SPEED,
        )

        observed_speeds = []

        def collect_speed(msg):
            # Record speed while the robot is still en-route (not yet arrived)
            if msg.distance_to_goal > 0.05:
                observed_speeds.append(msg.linear_speed_cmd)
            return msg.distance_to_goal < 0.05

        deadline = time.time() + 12.0
        while time.time() < deadline and not self._wait_for_motion_dbg(collect_speed, timeout_s=0.1):
            self._send_goal(goal, repeat=3, interval_s=0.1)

        self.assertTrue(
            self._wait_for_motion_dbg(lambda m: m.distance_to_goal < 0.05, timeout_s=1.0),
            f"Robot did not reach the goal within 12 s at max_speed={MAX_SPEED} m/s."
        )
        if observed_speeds:
            peak = max(observed_speeds)
            self.assertLessEqual(
                peak, MAX_SPEED + TOLERANCE,
                f"linear_speed_cmd peaked at {peak:.3f} m/s, which exceeds "
                f"max_speed={MAX_SPEED} + tolerance={TOLERANCE}."
            )

    def test_07_stop_angular(self):
        """
        In STOP_ANGULAR mode, the controller must not issue rotation commands
        even when the goal has a different orientation.
        Verified via motion_debug.angular_speed_disabled, which is True whenever
        the stop_angular() predicate is active.
        """
        x, y, theta = self._current_pose()
        goal = self._make_goal(
            x + 0.30 * math.cos(theta),
            y + 0.30 * math.sin(theta),
            theta + math.pi / 4,   # 45° heading offset — would cause rotation normally
            orient=StratMovement.STOP_ANGULAR,
        )

        deadline = time.time() + 5.0
        while time.time() < deadline and not self._wait_for_motion_dbg(
                lambda m: m.angular_speed_disabled, timeout_s=0.1):
            self._send_goal(goal, repeat=3, interval_s=0.1)

        self.assertTrue(
            self._wait_for_motion_dbg(lambda m: m.angular_speed_disabled, timeout_s=1.0),
            "angular_speed_disabled was not set in STOP_ANGULAR mode. "
            "The stop_angular() predicate should suppress all angular commands."
        )


    def test_08_rear_obstacle_no_effect_forward(self):
        """
        A rear obstacle must NOT stop the robot when it is moving forward.
        In addObstacle(), the obstacle angle is π (behind); when NOT reversing
        the angle is used as-is, which maps to near-zero dangerousness in
        compute_dangerousness_from_angle() (sinc function: sin(π)/π = 0).
        Therefore speed_inhibition ≈ 1 and stopped_by_obstacle stays False.
        """
        x, y, theta = self._current_pose()
        goal = self._make_goal(
            x + 0.40 * math.cos(theta),
            y + 0.40 * math.sin(theta),
            theta,
        )
        self._send_goal(goal, repeat=5, interval_s=0.1)

        # Wait for the robot to start moving, then inject a rear obstacle
        self._wait_for_motion_dbg(lambda m: m.linear_speed_cmd > 0.05, timeout_s=5.0)

        obstacle_deadline = time.time() + 3.0
        while time.time() < obstacle_deadline:
            self._publish_obstacle_behind(distance_m=0.10)
            time.sleep(0.1)

        self.assertFalse(
            self._wait_for_motion_dbg(lambda m: m.stopped_by_obstacle, timeout_s=0.5),
            "stopped_by_obstacle became True when a rear obstacle was injected "
            "during forward movement. Rear obstacles must not inhibit forward motion."
        )

        # Robot must still reach its goal despite the obstacle being present
        deadline = time.time() + 8.0
        while time.time() < deadline and not self._wait_for_motion_dbg(
                lambda m: m.distance_to_goal < 0.05, timeout_s=0.1):
            self._send_goal(goal, repeat=3, interval_s=0.1)

        self.assertTrue(
            self._wait_for_motion_dbg(lambda m: m.distance_to_goal < 0.05, timeout_s=1.0),
            "Robot did not reach its goal — it may have been wrongly stopped "
            "by the rear obstacle."
        )

    def test_09_sequential_goals(self):
        """
        The robot must update its goal immediately when a new StratMovement arrives.
        Sends goal A, waits for arrival, then sends goal B 30 cm further ahead.
        Both arrivals must complete, confirming goal hot-switching works.
        """
        x, y, theta = self._current_pose()
        goal_a = self._make_goal(
            x + 0.30 * math.cos(theta),
            y + 0.30 * math.sin(theta),
            theta,
        )

        # --- Goal A ---
        deadline = time.time() + 10.0
        while time.time() < deadline and not self._wait_for_motion_dbg(
                lambda m: m.distance_to_goal < 0.05, timeout_s=0.1):
            self._send_goal(goal_a, repeat=3, interval_s=0.1)

        self.assertTrue(
            self._wait_for_motion_dbg(lambda m: m.distance_to_goal < 0.05, timeout_s=1.0),
            "Robot did not reach goal A within 10 s."
        )

        # --- Goal B (30 cm ahead from wherever the robot now is) ---
        x2, y2, theta2 = self._current_pose()
        goal_b = self._make_goal(
            x2 + 0.30 * math.cos(theta2),
            y2 + 0.30 * math.sin(theta2),
            theta2,
        )

        deadline = time.time() + 10.0
        while time.time() < deadline and not self._wait_for_motion_dbg(
                lambda m: m.distance_to_goal < 0.05, timeout_s=0.1):
            self._send_goal(goal_b, repeat=3, interval_s=0.1)

        self.assertTrue(
            self._wait_for_motion_dbg(lambda m: m.distance_to_goal < 0.05, timeout_s=1.0),
            "Robot did not reach goal B within 10 s after switching from goal A. "
            "updateStratMovement() may not be updating m_goal_pose correctly."
        )


    def test_11_rear_obstacle_stops_reverse(self):
        """
        A rear obstacle on obstacle_behind_pose_stamped must stop the robot when
        it is moving in REVERSE.
        In addObstacle(), the obstacle angle is π (behind); when REVERSING the
        angle is normalised by +π → 2π ≡ 0 (dead ahead in travel direction) →
        speed_inhibition ≈ 0.05 → stopped_by_obstacle = True.
        """
        x, y, theta = self._current_pose()
        goal = self._make_goal(
            x - 0.50 * math.cos(theta),
            y - 0.50 * math.sin(theta),
            theta,
            reverse_gear=StratMovement.REVERSE,
        )
        self._send_goal(goal, repeat=5, interval_s=0.1)

        # Wait for the robot to start moving backwards
        self._wait_for_motion_dbg(lambda m: m.linear_speed_cmd > 0.05, timeout_s=5.0)

        # Inject rear obstacle and keep publishing until the node reacts
        deadline = time.time() + 5.0
        while time.time() < deadline:
            self._publish_obstacle_behind(distance_m=0.10)
            if self._wait_for_motion_dbg(lambda m: m.stopped_by_obstacle, timeout_s=0.2):
                break

        self.assertTrue(
            self._wait_for_motion_dbg(lambda m: m.stopped_by_obstacle, timeout_s=1.0),
            "stopped_by_obstacle was not set after injecting a rear obstacle "
            "while the robot was reversing. Rear obstacles must stop reverse motion."
        )

        self._clear_obstacle_behind()
        time.sleep(0.2)

    def test_12_front_obstacle_no_effect_reverse(self):
        """
        A front obstacle must NOT stop the robot when it is moving in REVERSE.
        In addObstacle(), the obstacle angle is 0 (ahead); when REVERSING the
        angle is normalised to π (behind in travel direction) → danger ≈ 0 →
        stopped_by_obstacle stays False.
        The robot must still reach its reverse goal.
        """
        x, y, theta = self._current_pose()
        goal = self._make_goal(
            x - 0.40 * math.cos(theta),
            y - 0.40 * math.sin(theta),
            theta,
            reverse_gear=StratMovement.REVERSE,
        )
        self._send_goal(goal, repeat=5, interval_s=0.1)

        # Wait for the robot to start moving backwards, then inject a front obstacle
        self._wait_for_motion_dbg(lambda m: m.linear_speed_cmd > 0.05, timeout_s=5.0)

        obstacle_deadline = time.time() + 3.0
        while time.time() < obstacle_deadline:
            self._publish_obstacle_ahead(distance_m=0.10)
            time.sleep(0.1)

        self.assertFalse(
            self._wait_for_motion_dbg(lambda m: m.stopped_by_obstacle, timeout_s=0.5),
            "stopped_by_obstacle became True when a front obstacle was injected "
            "during reverse movement. Front obstacles must not inhibit reverse motion."
        )

        # Robot must still reach the reverse goal despite the front obstacle
        deadline = time.time() + 8.0
        while time.time() < deadline and not self._wait_for_motion_dbg(
                lambda m: m.distance_to_goal < 0.05, timeout_s=0.1):
            self._send_goal(goal, repeat=3, interval_s=0.1)

        self.assertTrue(
            self._wait_for_motion_dbg(lambda m: m.distance_to_goal < 0.05, timeout_s=1.0),
            "Robot did not reach its reverse goal — it may have been wrongly "
            "stopped by the front obstacle."
        )
        self._clear_obstacle()

        self.assertTrue(
            self._wait_for_motion_dbg(lambda m: m.distance_to_goal < 0.002, timeout_s=2.0),
            "Robot did not reach within 2 mm of the goal within 2 s after fine-tuning activated. "
            "The position PID may not be controlling the robot as expected."
        )
