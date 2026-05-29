/**
 * Unit tests for helpers_broker.c — the pure-C math layer of the DNF motion controller.
 *
 * No ROS, no krabilib: just <math.h> and gtest.
 * Run with:  colcon test --packages-select main_strategy
 *
 * Background on the DNF (Dynamic Neural Field) architecture:
 *   - The 360° heading space is discretised into NB_NEURONS bins (1°/bin).
 *   - A log-cosh attractive "hill" is built by target() around the desired heading.
 *   - Differentiating that hill gives the angular-speed command:
 *       positive gradient at the robot's heading neuron → turn left
 *       negative gradient                               → turn right
 *   The three DnfTest cases below make this concrete and regression-test it.
 */

#include <cmath>
#include <gtest/gtest.h>

#include "helpers_broker.h"

static const int NB_NEURONS = 360;

// ---------------------------------------------------------------------------
// gaussian
// ---------------------------------------------------------------------------

TEST(GaussianTest, PeaksAtCenter)
{
    float peak     = gaussian(30.f, 1.f, 90.f, 90.f);
    float off_peak = gaussian(30.f, 1.f, 90.f, 120.f);
    EXPECT_GT(peak, off_peak);
}

TEST(GaussianTest, NearZeroFarAway)
{
    // 180° from center → should be essentially 0
    EXPECT_NEAR(gaussian(30.f, 1.f, 90.f, 270.f), 0.f, 1e-5f);
}

TEST(GaussianTest, CircularSymmetry)
{
    // 350° is 20° below center at 10° (wrapping); 30° is 20° above it (no wrap).
    // Both should produce the same value.
    float wrap    = gaussian(30.f, 1.f, 10.f, 350.f);
    float no_wrap = gaussian(30.f, 1.f, 10.f,  30.f);
    EXPECT_NEAR(wrap, no_wrap, 1e-5f);
}

// ---------------------------------------------------------------------------
// target  (log-cosh attractive potential well)
// ---------------------------------------------------------------------------

TEST(TargetTest, PeaksAtX0)
{
    // -log(cosh(0)) + offset = 0 + offset
    EXPECT_FLOAT_EQ(target(207.f, 1.1f, 90.f, 90.f), 1.1f);
}

TEST(TargetTest, DecreasesMonotonically)
{
    float at_0   = target(207.f, 1.1f, 90.f,  90.f);
    float at_40  = target(207.f, 1.1f, 90.f, 130.f);
    float at_180 = target(207.f, 1.1f, 90.f, 270.f);
    EXPECT_GT(at_0, at_40);
    EXPECT_GT(at_40, at_180);
}

TEST(TargetTest, CircularSymmetry)
{
    // Center at 5°: 355° (10° away, wrapping) and 15° (10° away, no wrap) must be equal.
    float wrap    = target(50.f, 1.1f, 5.f, 355.f);
    float no_wrap = target(50.f, 1.1f, 5.f,  15.f);
    EXPECT_NEAR(wrap, no_wrap, 1e-5f);
}

// ---------------------------------------------------------------------------
// differentiate  (circular discrete gradient)
// ---------------------------------------------------------------------------

TEST(DifferentiateTest, FlatFieldZeroGradient)
{
    float field[NB_NEURONS] = {};
    float result[NB_NEURONS] = {};
    differentiate(field, result, NB_NEURONS, 1.f);
    for (int i = 0; i < NB_NEURONS; ++i)
        EXPECT_FLOAT_EQ(result[i], 0.f) << "at index " << i;
}

TEST(DifferentiateTest, PeakGradientSign)
{
    // Build a log-cosh hill centred at index 100.
    float field[NB_NEURONS], result[NB_NEURONS];
    for (int i = 0; i < NB_NEURONS; ++i)
        field[i] = target(50.f, 1.f, 100.f, static_cast<float>(i));
    differentiate(field, result, NB_NEURONS, 1.f);

    EXPECT_GT(result[90],  0.f);         // rising slope (left of peak) → positive
    EXPECT_LT(result[110], 0.f);         // falling slope (right of peak) → negative
    EXPECT_NEAR(result[100], 0.f, 1e-6f); // at the exact peak → gradient = 0
}

// ---------------------------------------------------------------------------
// get_idx_of_max
// ---------------------------------------------------------------------------

TEST(GetIdxOfMaxTest, CorrectIndex)
{
    float arr[] = { 1.f, 3.f, 7.f, 2.f, 4.f };
    EXPECT_EQ(get_idx_of_max(arr, 5), 2u);
}

TEST(GetIdxOfMaxTest, FirstElementWhenAllEqual)
{
    float arr[] = { 5.f, 5.f, 5.f, 5.f };
    EXPECT_EQ(get_idx_of_max(arr, 4), 0u);
}

// ---------------------------------------------------------------------------
// update_speed  (exponential ramp)
// ---------------------------------------------------------------------------

TEST(UpdateSpeedTest, ImmediateSetsDirect)
{
    float speed = 0.f;
    update_speed(/*immediate=*/1, &speed, /*new_speed=*/2.f);
    EXPECT_FLOAT_EQ(speed, 2.f);
}

TEST(UpdateSpeedTest, RampApproachesTarget)
{
    float speed = 0.f;
    update_speed(/*immediate=*/0, &speed, /*new_speed=*/10.f);
    EXPECT_GT(speed, 0.f);   // moved toward target
    EXPECT_LT(speed, 10.f);  // not there yet
}

// ---------------------------------------------------------------------------
// DNF integration tests
// ---------------------------------------------------------------------------
//
// In the robot's frame, the robot's own heading maps to neuron 180:
//   angle_to_neuron_id(0) = (0 + pi) / (2*pi) * 360 = 180
//
// A goal 45° to the left  → index (pi/4  + pi)/(2*pi)*360 = 225
// A goal 45° to the right → index (-pi/4 + pi)/(2*pi)*360 = 135
//
// The angular-speed command is the gradient value at neuron 180:
//   gradient > 0 → turn left   (positive = CCW in ROS)
//   gradient < 0 → turn right

static constexpr int HEADING_NEURON = 180;

TEST(DnfTest, FacingGoalNoTurn)
{
    float field[NB_NEURONS], gradient[NB_NEURONS];
    for (int i = 0; i < NB_NEURONS; ++i)
        field[i] = target(220.f, 1.1f, HEADING_NEURON, i);

    differentiate(field, gradient, NB_NEURONS, 3000.f);

    // Log-cosh is symmetric around 180, so field[181] == field[179] → gradient = 0
    EXPECT_NEAR(gradient[HEADING_NEURON], 0.f, 1e-4f);
}

TEST(DnfTest, GoalToLeftCommandsTurnLeft)
{
    float field[NB_NEURONS], gradient[NB_NEURONS];
    for (int i = 0; i < NB_NEURONS; ++i)
        field[i] = target(220.f, 1.1f, 225, i); // peak 45° left of heading

    differentiate(field, gradient, NB_NEURONS, 3000.f);

    // At neuron 180 we are on the rising slope toward neuron 225
    // → field[181] > field[179] → gradient > 0 → turn left
    EXPECT_GT(gradient[HEADING_NEURON], 0.f);
}

TEST(DnfTest, GoalToRightCommandsTurnRight)
{
    float field[NB_NEURONS], gradient[NB_NEURONS];
    for (int i = 0; i < NB_NEURONS; ++i)
        field[i] = target(220.f, 1.1f, 135, i); // peak 45° right of heading

    differentiate(field, gradient, NB_NEURONS, 3000.f);

    // At neuron 180 we are on the falling slope away from neuron 135
    // → field[181] < field[179] → gradient < 0 → turn right
    EXPECT_LT(gradient[HEADING_NEURON], 0.f);
}

// ---------------------------------------------------------------------------

int main(int argc, char** argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
