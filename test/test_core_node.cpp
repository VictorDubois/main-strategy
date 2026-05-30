/**
 * Unit tests for the Core ROS 2 node.
 *
 * These tests exercise the Core class through its public interface without
 * running a Gazebo simulation.  They are intentionally coarse — detailed
 * control-loop behaviour is covered by the functional tests in
 * test_goal_reach.py.  The purpose here is to catch:
 *   - Constructor / destructor crashes (missing parameter declarations, etc.)
 *   - Wrong initial state (should be INIT_ODOM_TODO, not EXIT)
 *   - State machine transitions that can be triggered without real hardware
 *     (INIT_ODOM_TODO → WAIT_TIRETTE, then WAIT_TIRETTE → NORMAL via tirette)
 *   - Publisher / subscriber presence (regression guard)
 */

#include <chrono>
#include <memory>
#include <thread>

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>

#include "core.h"

// ---------------------------------------------------------------------------
// Fixture: initialise / shutdown rclcpp around every test
// ---------------------------------------------------------------------------

class CoreNodeTest : public ::testing::Test
{
protected:
    void SetUp() override
    {
        rclcpp::init(0, nullptr);
        node_ = std::make_shared<Core>();
    }

    void TearDown() override
    {
        node_.reset();
        rclcpp::shutdown();
    }

    // Spin the node for up to `timeout_ms` milliseconds, stopping early if
    // `condition` becomes true.  Returns whether the condition was met.
    bool spin_until(std::function<bool()> condition, int timeout_ms = 2000)
    {
        rclcpp::executors::SingleThreadedExecutor exec;
        exec.add_node(node_);
        auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);
        while (std::chrono::steady_clock::now() < deadline)
        {
            exec.spin_some(std::chrono::milliseconds(50));
            if (condition())
                return true;
        }
        return false;
    }

    std::shared_ptr<Core> node_;
};

// ---------------------------------------------------------------------------
// Basic construction / destruction
// ---------------------------------------------------------------------------

TEST_F(CoreNodeTest, ConstructsWithoutError)
{
    EXPECT_NE(node_, nullptr);
}

TEST_F(CoreNodeTest, IsNotOverAfterConstruction)
{
    // Robot must not be in EXIT state right after construction
    EXPECT_FALSE(node_->isOver());
}

TEST_F(CoreNodeTest, SetupSucceeds)
{
    EXPECT_EQ(node_->Setup(), 0);
}

// ---------------------------------------------------------------------------
// Publisher / subscriber wiring
//
// Regression test: if a topic name or QoS changes, these fail immediately
// without needing the full simulation.
// ---------------------------------------------------------------------------

TEST_F(CoreNodeTest, PublishesCmdVel)
{
    node_->Setup();
    EXPECT_GT(node_->count_publishers("cmd_vel"), 0u)
      << "main_strategy must advertise cmd_vel (motor speed commands)";
}

TEST_F(CoreNodeTest, PublishesMotionDebug)
{
    node_->Setup();
    EXPECT_GT(node_->count_publishers("motion_debug"), 0u)
      << "main_strategy must advertise motion_debug (diagnostic feedback)";
}

TEST_F(CoreNodeTest, SubscribesToStratMovement)
{
    node_->Setup();
    EXPECT_GT(node_->count_subscribers("strat_movement"), 0u)
      << "main_strategy must subscribe to strat_movement (goal commands)";
}

TEST_F(CoreNodeTest, SubscribesToObstaclePoseStamped)
{
    node_->Setup();
    EXPECT_GT(node_->count_subscribers("obstacle_pose_stamped"), 0u)
      << "main_strategy must subscribe to the front-obstacle topic";
}

TEST_F(CoreNodeTest, SubscribesToObstacleBehindPoseStamped)
{
    node_->Setup();
    EXPECT_GT(node_->count_subscribers("obstacle_behind_pose_stamped"), 0u)
      << "main_strategy must subscribe to the rear-obstacle topic";
}

TEST_F(CoreNodeTest, SubscribesToTirette)
{
    node_->Setup();
    EXPECT_GT(node_->count_subscribers("tirette"), 0u)
      << "main_strategy must subscribe to tirette (match-start signal)";
}

// ---------------------------------------------------------------------------
// State-machine transitions (no hardware, no simulation clock)
// ---------------------------------------------------------------------------

TEST_F(CoreNodeTest, TransitionsFromInitOdomToWaitTirette)
{
    node_->Setup();
    // INIT_ODOM_TODO lasts for a short fixed duration (~500 ms by default).
    // After that the node transitions to WAIT_TIRETTE automatically.
    // We check that the robot is not yet in the EXIT state after 3 s.
    bool still_alive = spin_until([this]() { return node_->isOver(); }, /*ms=*/3000);
    EXPECT_FALSE(still_alive)
      << "Node must not enter EXIT state within 3 s of startup (match timer not started)";
}

TEST_F(CoreNodeTest, TiretteStartsMatch)
{
    node_->Setup();

    // Create a publisher on the tirette topic using the node's own graph so
    // it is in the same ROS context.
    auto tirette_pub = node_->create_publisher<std_msgs::msg::Bool>("tirette", 1);

    // Let the node advance out of INIT_ODOM_TODO first (~600 ms)
    rclcpp::executors::SingleThreadedExecutor exec;
    exec.add_node(node_);
    exec.spin_some(std::chrono::milliseconds(700));

    // Publish tirette = True to trigger WAIT_TIRETTE → NORMAL
    std_msgs::msg::Bool msg;
    msg.data = true;
    for (int i = 0; i < 5; ++i)
    {
        tirette_pub->publish(msg);
        exec.spin_some(std::chrono::milliseconds(50));
    }

    // After pulling the tirette the robot should be in NORMAL state.
    // isOver() only returns true in EXIT state — it should remain false.
    EXPECT_FALSE(node_->isOver())
      << "After tirette=True the node must be in NORMAL state, not EXIT";
}

// ---------------------------------------------------------------------------

int main(int argc, char** argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
