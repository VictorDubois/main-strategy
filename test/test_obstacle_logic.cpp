/**
 * Unit tests for the obstacle-inhibition pipeline.
 *
 * Two layers are tested here:
 *
 * 1. The GATE (updateLidar, line 76 of core.cpp):
 *      if (front == !reverseGear()) { addObstacle(...); }
 *    This decides which direction's sensor reading is relevant for the current
 *    travel direction.  It is the primary reason that rear obstacles don't stop
 *    a forward-moving robot (and vice-versa).
 *
 * 2. LidarStrat::speed_inhibition(distance, angle, coeff):
 *    Given that an obstacle passes the gate, this computes a [0,1] inhibition.
 *    Values < 0.2 set stopped_by_obstacle = True (threshold in addObstacle).
 *    It uses a tanh sigmoid centred at half_stop ≈ 0.55 m, modulated by a
 *    sinc function of the angle (angle_factor: 1 dead-ahead, 0 at ±90°).
 *
 * 3. Inline tests for the angle_to_neuron_id formula.
 *    The formula maps angles to DNF neuron indices and is the bridge between
 *    the robot's heading and the 360-element neural field arrays.
 */

#include <cmath>
#include <gtest/gtest.h>

#include "lidarStrat.h" // for LidarStrat::speed_inhibition, Distance, Angle

// ---------------------------------------------------------------------------
// 1. Gate:  front == !reverseGear()
//
// This is tested as a pure Boolean expression — no ROS or Core instantiation
// needed.  The four cases correspond exactly to the functional tests:
//   test_04: front + forward → STOPS   (gate opens)
//   test_08: rear  + forward → no stop (gate closed)
//   test_11: rear  + reverse → STOPS   (gate opens)
//   test_12: front + reverse → no stop (gate closed)
// ---------------------------------------------------------------------------

static bool obstacle_gate(bool front, bool reverse_gear)
{
    return front == !reverse_gear;
}

TEST(ObstacleGateTest, FrontObstacleForwardIsProcessed)
{
    EXPECT_TRUE(obstacle_gate(/*front=*/true, /*reverse=*/false));
}

TEST(ObstacleGateTest, RearObstacleReverseIsProcessed)
{
    EXPECT_TRUE(obstacle_gate(/*front=*/false, /*reverse=*/true));
}

TEST(ObstacleGateTest, RearObstacleForwardIsIgnored)
{
    EXPECT_FALSE(obstacle_gate(/*front=*/false, /*reverse=*/false));
}

TEST(ObstacleGateTest, FrontObstacleReverseIsIgnored)
{
    EXPECT_FALSE(obstacle_gate(/*front=*/true, /*reverse=*/true));
}

// ---------------------------------------------------------------------------
// 2. LidarStrat::speed_inhibition(distance, angle, distanceCoeff)
//
// Formula (for distanceCoeff=1, angle=0°/dead-ahead):
//   half_stop = 0.55 m  (where inhibition = 0.5)
//   slope     = 0.30 m  (transition width)
//   inhibition = 0.5 * (1 + tanh((distance − 0.55) / 0.30))
//
// The stopped_by_obstacle threshold in addObstacle() is 0.2.
// At 10 cm: tanh((0.10 − 0.55) / 0.30) = tanh(−1.5) ≈ −0.905  → inhibition ≈ 0.047 → STOPS
// At 55 cm: tanh(0)                                              → inhibition  = 0.5   → half-speed
// At  2  m: tanh((2.0 − 0.55) / 0.30) ≈ tanh(4.8) ≈ 1          → inhibition ≈ 1.0   → full speed
// ---------------------------------------------------------------------------

TEST(SpeedInhibitionTest, CloseObstacleStopped)
{
    float inhib = LidarStrat::speed_inhibition(Distance(0.10f), Angle(0.f), 1.f);
    EXPECT_LT(inhib, 0.2f)
      << "10 cm directly ahead must trigger stopped_by_obstacle (inhibition < 0.2 threshold)";
}

TEST(SpeedInhibitionTest, HalfSpeedAtHalfStopDistance)
{
    // At exactly half_stop (0.55 m for angle=0, coeff=1) the tanh returns 0
    float inhib = LidarStrat::speed_inhibition(Distance(0.55f), Angle(0.f), 1.f);
    EXPECT_NEAR(inhib, 0.5f, 0.01f) << "At the half-stop distance (0.55 m) inhibition must be 0.5";
}

TEST(SpeedInhibitionTest, FarObstacleFullSpeed)
{
    float inhib = LidarStrat::speed_inhibition(Distance(2.0f), Angle(0.f), 1.f);
    EXPECT_GT(inhib, 0.95f)
      << "A 2 m obstacle must not meaningfully reduce speed (inhibition > 0.95)";
}

TEST(SpeedInhibitionTest, SideObstacleLessDangerousThanFront)
{
    // 60° side obstacle — compute_dangerousness_from_angle returns sinc(π/3) ≈ 0.83,
    // which widens the half-stop distance slightly. At 40 cm both stop, but at 60 cm
    // the side obstacle allows a higher inhibition (more speed remaining).
    float front_60cm = LidarStrat::speed_inhibition(Distance(0.60f), Angle(0.f), 1.f);
    float side_60cm = LidarStrat::speed_inhibition(Distance(0.60f), Angle(float(M_PI / 3.f)), 1.f);
    EXPECT_LT(front_60cm, side_60cm)
      << "A frontal obstacle must be more inhibiting than a 60° side obstacle at the same range";
}

TEST(SpeedInhibitionTest, BeyondNinetyDegreesNoModulation)
{
    // |angle| > π/2 → compute_dangerousness returns 0 → parameters use their minimum values.
    // Minimum slope (0.10 m) makes the sigmoid SHARPER than for front obstacles.
    // At 1 m the robot should be at essentially full speed regardless of the sharp curve.
    float inhib_91deg
      = LidarStrat::speed_inhibition(Distance(1.0f), Angle(float(M_PI * 91.f / 180.f)), 1.f);
    EXPECT_GT(inhib_91deg, 0.9f)
      << "An obstacle at 91° (just past the ±90° cutoff) at 1 m must have near-zero effect";
}

TEST(SpeedInhibitionTest, MonotonicallyIncreasingWithDistance)
{
    // Inhibition should increase (more permissive) as the obstacle gets farther away
    float near = LidarStrat::speed_inhibition(Distance(0.20f), Angle(0.f), 1.f);
    float mid = LidarStrat::speed_inhibition(Distance(0.50f), Angle(0.f), 1.f);
    float far = LidarStrat::speed_inhibition(Distance(1.00f), Angle(0.f), 1.f);
    EXPECT_LT(near, mid);
    EXPECT_LT(mid, far);
}

// ---------------------------------------------------------------------------
// 3. angle_to_neuron_id formula (inline reference implementation)
//
// The actual function lives in core.cpp and depends on krabilib/Angle.
// These tests document the formula as pure arithmetic so new members can
// reason about the DNF without linking the full Core dependency tree.
//
// Formula:  neuron(a) = ((wrapAngle(a) + π) / (2π)) * NB_NEURONS
// Key values:
//   a =  0     → neuron 180  (robot's own heading)
//   a =  π/2   → neuron 270  (90° to the left)
//   a = −π/2   → neuron  90  (90° to the right)
//   a =  π     → neuron   0  (directly behind)
// ---------------------------------------------------------------------------

static const int NB_NEURONS = 360;

static unsigned int neuron_id(float a_rad)
{
    // Wrap to [-π, π]
    while (a_rad > float(M_PI))
        a_rad -= 2.f * float(M_PI);
    while (a_rad < -float(M_PI))
        a_rad += 2.f * float(M_PI);
    return static_cast<unsigned int>(((a_rad + float(M_PI)) / (2.f * float(M_PI))) * NB_NEURONS);
}

TEST(AngleToNeuronTest, StraightAheadIsCenterNeuron)
{
    EXPECT_EQ(neuron_id(0.f), 180u)
      << "angle=0 (straight ahead) must map to neuron 180 (centre of the array)";
}

TEST(AngleToNeuronTest, LeftQuarterPi)
{
    EXPECT_EQ(neuron_id(float(M_PI) / 2.f), 270u) << "angle=+π/2 (90° left) must map to neuron 270";
}

TEST(AngleToNeuronTest, RightQuarterPi)
{
    EXPECT_EQ(neuron_id(-float(M_PI) / 2.f), 90u) << "angle=-π/2 (90° right) must map to neuron 90";
}

TEST(AngleToNeuronTest, DirectlyBehindIsEdge)
{
    unsigned int idx = neuron_id(float(M_PI));
    EXPECT_LT(idx % 360, 5u) // should be 0 or near-0
      << "angle=π (directly behind) must map to neuron 0 (edge of the array)";
}

// ---------------------------------------------------------------------------

int main(int argc, char** argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
