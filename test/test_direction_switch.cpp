#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include "frontier_exploration_ros2/direction_switch.hpp"

namespace frontier_exploration_ros2
{
namespace
{
constexpr int64_t second = 1'000'000'000;

TEST(DirectionSwitchTests, SmallSidewaysGoalChangesDoNotResetPhysicalStall)
{
  DirectionSwitch policy;
  EXPECT_FALSE(policy.observe({0., 0.}, {10., 0.}, 0));
  EXPECT_FALSE(policy.observe({0., .1}, {10., .2}, 5 * second));
  EXPECT_FALSE(policy.observe({0., -.1}, {10., -.2}, 10 * second));
  EXPECT_TRUE(policy.observe({0., 0.}, {10., 0.}, 15 * second));
  const std::vector<DirectionSwitch::XY> targets{{10., 0.}, {10., 1.}, {0., 8.}, {-8., 0.}, {-8., 4.}};
  EXPECT_EQ(policy.select(targets, {0., 0.}, 16 * second), (std::vector<std::size_t>{3, 4}));
}

TEST(DirectionSwitchTests, PhysicalProgressResetsTimeout)
{
  DirectionSwitch policy;
  for (int i = 0; i < 10; ++i) {
    EXPECT_FALSE(policy.observe({i * .6, 0.}, {20., 0.}, i * 10 * second));
  }
  EXPECT_EQ(policy.record_count(), 0U);
}

TEST(DirectionSwitchTests, NewOppositeGoalAfterWaitingGetsItsOwnTrialWindow)
{
  DirectionSwitch policy;
  policy.begin_goal({0., 0.}, {10., 0.}, 0);
  policy.observe({0., 0.}, {10., 0.}, 0);
  ASSERT_TRUE(policy.observe({0., 0.}, {10., 0.}, 15 * second));
  // The old progress timer is already expired after a long idle/reselection.
  policy.begin_goal({0., 0.}, {-10., 0.}, 45 * second);
  EXPECT_FALSE(policy.observe({0., 0.}, {-10., 0.}, 45 * second));
  EXPECT_FALSE(policy.observe({0., 0.}, {-10., 0.}, 59 * second));
  EXPECT_TRUE(policy.observe({0., 0.}, {-10., 0.}, 60 * second));
}

TEST(DirectionSwitchTests, NearbyAcceptedGoalStillCannotResetStall)
{
  DirectionSwitch policy;
  policy.begin_goal({0., 0.}, {10., 0.}, 0);
  policy.observe({0., 0.}, {10., 0.}, 0);
  policy.begin_goal({0., 0.}, {10., 1.}, 14 * second);
  EXPECT_TRUE(policy.observe({0., 0.}, {10., 1.}, 15 * second));
}

TEST(DirectionSwitchTests, ExpiredAvoidanceAllowsFreshTrialEvenForSameGoal)
{
  DirectionSwitch policy;
  policy.begin_goal({0., 0.}, {10., 0.}, 0);
  policy.observe({0., 0.}, {10., 0.}, 0);
  ASSERT_TRUE(policy.observe({0., 0.}, {10., 0.}, 15 * second));
  policy.begin_goal({0., 0.}, {10., 0.}, 105 * second);
  EXPECT_FALSE(policy.observe({0., 0.}, {10., 0.}, 105 * second));
}

TEST(DirectionSwitchTests, WithoutOppositeUsesGreatestDirectionChangeAndNeverSameSector)
{
  DirectionSwitch policy;
  policy.observe({0., 0.}, {10., 0.}, 0);
  ASSERT_TRUE(policy.observe({0., 0.}, {10., 0.}, 15 * second));
  EXPECT_EQ(policy.select({{10., 0.}, {1., 4.}, {-1., 4.}}, {0., 0.}, 16 * second),
    (std::vector<std::size_t>{2}));
  EXPECT_TRUE(policy.select({{10., 0.}, {10., 1.}}, {0., 0.}, 16 * second).empty());
}

TEST(DirectionSwitchTests, RemembersBothStalledSidesInsteadOfBouncingBack)
{
  DirectionSwitch policy;
  policy.observe({0., 0.}, {10., 0.}, 0);
  ASSERT_TRUE(policy.observe({0., 0.}, {10., 0.}, 15 * second));
  ASSERT_TRUE(policy.observe({0., 0.}, {-10., 0.}, 30 * second));
  EXPECT_EQ(policy.record_count(), 2U);
  EXPECT_EQ(policy.select({{10., 0.}, {-10., 0.}, {0., 10.}, {0., -10.}}, {0., 0.}, 31 * second),
    (std::vector<std::size_t>{2, 3}));
}

TEST(DirectionSwitchTests, ExpiryOrLeavingLocalAreaRestoresAllCandidates)
{
  for (bool travel : {false, true}) {
    DirectionSwitch policy;
    policy.observe({0., 0.}, {10., 0.}, 0);
    ASSERT_TRUE(policy.observe({0., 0.}, {10., 0.}, 15 * second));
    EXPECT_EQ(policy.select({{10., 0.}, {-10., 0.}}, travel ? DirectionSwitch::XY{0., 3.} : DirectionSwitch::XY{0., 0.},
      (travel ? 16 : 105) * second), (std::vector<std::size_t>{0, 1}));
    EXPECT_EQ(policy.record_count(), 0U);
  }
}

TEST(DirectionSwitchTests, TimeResetAndMissingPoseDoNotCreateFalseStalls)
{
  DirectionSwitch policy;
  policy.observe({0., 0.}, {10., 0.}, 0);
  policy.suspend_progress();
  EXPECT_FALSE(policy.observe({0., 0.}, {10., 0.}, 50 * second));
  ASSERT_TRUE(policy.observe({0., 0.}, {10., 0.}, 65 * second));
  EXPECT_FALSE(policy.observe({0., 0.}, {10., 0.}, second));
  EXPECT_EQ(policy.record_count(), 0U);
  policy.observe({std::numeric_limits<double>::quiet_NaN(), 0.}, {10., 0.}, 30 * second);
  EXPECT_FALSE(policy.observe({0., 0.}, {10., 0.}, 40 * second));
}

TEST(DirectionSwitchTests, MemoryIsBoundedAndSessionResetClearsIt)
{
  DirectionSwitch policy({1., .5, 90., 3.});
  policy.observe({0., 0.}, {10., 0.}, 0);
  for (int i = 1; i <= 10; ++i) {
    ASSERT_TRUE(policy.observe({0., 0.}, {10., double(i)}, i * second));
  }
  EXPECT_EQ(policy.record_count(), 4U);
  policy.reset();
  EXPECT_EQ(policy.record_count(), 0U);
  EXPECT_FALSE(policy.observe({0., 0.}, {10., 0.}, 100 * second));
}
}  // namespace
}  // namespace frontier_exploration_ros2
