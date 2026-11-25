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
        // rclcpp is initialised once for the whole process in main().
        // Each test gets a fresh Core node.
        node_ = std::make_shared<Core>();
    }

    void TearDown() override
    {
        // Destroying the node will signal plotAll() to stop and join its thread
        // (see Core::~Core()).  The join takes at most ~100 ms (plotAll sleep interval).
        node_.reset();
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

// Publish `value` on the tirette topic for `duration_ms`, spinning the node meanwhile
static void publish_tirette(const std::shared_ptr<Core>& node, bool value, int duration_ms)
{
    auto pub = node->create_publisher<std_msgs::msg::Bool>("tirette", 5);
    rclcpp::executors::SingleThreadedExecutor exec;
    exec.add_node(node);
    std_msgs::msg::Bool msg;
    msg.data = value;
    const auto end = std::chrono::steady_clock::now() + std::chrono::milliseconds(duration_ms);
    while (std::chrono::steady_clock::now() < end)
    {
        pub->publish(msg);
        exec.spin_some(std::chrono::milliseconds(50));
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
}

TEST_F(CoreNodeTest, BootWithoutTiretteDoesNotStartMatch)
{
    node_->Setup();
    publish_tirette(node_, /*removed=*/true, 3000);
    EXPECT_EQ(node_->getState(), Core::State::WAIT_TIRETTE)
      << "tirette=True alone (never inserted) must not start the match";
}

TEST_F(CoreNodeTest, ShortInsertionDoesNotArmTheMatch)
{
    node_->Setup();
    publish_tirette(node_, /*removed=*/false, 300); // contact bounce
    publish_tirette(node_, /*removed=*/true, 1000);
    EXPECT_EQ(node_->getState(), Core::State::WAIT_TIRETTE);
}

// Pre-match motion (recalage): motors enabled on request, cut if goal_strat goes silent
TEST_F(CoreNodeTest, PreMatchMotionIsCutWhenStratMovementStops)
{
    node_->Setup();
    bool motors_enabled = false;
    auto sub = node_->create_subscription<std_msgs::msg::Bool>(
      "enable_motor", 5, [&](std_msgs::msg::Bool::SharedPtr m) { motors_enabled = m->data; });
    auto strat_pub = node_->create_publisher<krabi_msgs::msg::StratMovement>("strat_movement", 5);
    auto client = node_->create_client<std_srvs::srv::SetBool>("enable_pre_match_motion");

    ASSERT_TRUE(spin_until([this]() { return node_->getState() == Core::State::WAIT_TIRETTE; }));
    EXPECT_FALSE(motors_enabled) << "motors must be off before the match";

    auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
    request->data = true;
    auto future = client->async_send_request(request);

    // goal_strat alive: it keeps publishing, so the motors stay enabled well past the timeout
    krabi_msgs::msg::StratMovement move;
    move.orient = krabi_msgs::msg::StratMovement::RECALAGE_BORDURE;
    spin_until(
      [&]()
      {
          strat_pub->publish(move);
          return false;
      },
      2500);
    EXPECT_TRUE(motors_enabled);

    // goal_strat dies: motors cut within the timeout (1 s)
    EXPECT_TRUE(spin_until([&]() { return !motors_enabled; }, 2000))
      << "motors must be cut when strat_movement stops during pre-match motion";
    EXPECT_EQ(node_->getState(), Core::State::WAIT_TIRETTE);
}

TEST_F(CoreNodeTest, TiretteStartsMatch)
{
    node_->Setup();

    // The tirette must have been seen inserted first
    publish_tirette(node_, /*removed=*/false, 1500);
    EXPECT_EQ(node_->getState(), Core::State::WAIT_TIRETTE);

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
    EXPECT_EQ(node_->getState(), Core::State::NORMAL);
}

// ---------------------------------------------------------------------------

int main(int argc, char** argv)
{
    // Initialise rclcpp once for the whole process.
    // The fixture does NOT call init/shutdown per-test; doing so between tests
    // causes undefined behaviour in some rclcpp versions.
    rclcpp::init(argc, argv);
    testing::InitGoogleTest(&argc, argv);
    int result = RUN_ALL_TESTS();
    rclcpp::shutdown();
    return result;
}
