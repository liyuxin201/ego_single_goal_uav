#include <autonomous_flight/simulation/flightBase.h>
#include <cmath>
#include <gtest/gtest.h>

namespace {

TEST(FlightBaseKinematics, FirstUpdateSeedsPreviousVelocityAndZeroAcceleration) {
  bool first_update = true;
  Eigen::Vector3d prev_vel = Eigen::Vector3d::Zero();
  Eigen::Vector3d curr_acc(9.0, 9.0, 9.0);
  ros::Time prev_state_time(0.0);
  const Eigen::Vector3d curr_vel(1.0, -2.0, 0.5);
  const ros::Time curr_time(10.0);

  AutoFlight::flightBase::updateKinematicState(
      curr_vel, curr_time, first_update, prev_vel, curr_acc, prev_state_time);

  EXPECT_FALSE(first_update);
  EXPECT_DOUBLE_EQ(curr_vel.x(), prev_vel.x());
  EXPECT_DOUBLE_EQ(curr_vel.y(), prev_vel.y());
  EXPECT_DOUBLE_EQ(curr_vel.z(), prev_vel.z());
  EXPECT_DOUBLE_EQ(0.0, curr_acc.x());
  EXPECT_DOUBLE_EQ(0.0, curr_acc.y());
  EXPECT_DOUBLE_EQ(0.0, curr_acc.z());
  EXPECT_DOUBLE_EQ(curr_time.toSec(), prev_state_time.toSec());
}

TEST(FlightBaseKinematics, SecondUpdateComputesFiniteAccelerationFromSeededVelocity) {
  bool first_update = true;
  Eigen::Vector3d prev_vel = Eigen::Vector3d::Zero();
  Eigen::Vector3d curr_acc = Eigen::Vector3d::Zero();
  ros::Time prev_state_time(0.0);

  AutoFlight::flightBase::updateKinematicState(
      Eigen::Vector3d(1.0, 2.0, 3.0), ros::Time(5.0), first_update, prev_vel, curr_acc, prev_state_time);
  AutoFlight::flightBase::updateKinematicState(
      Eigen::Vector3d(2.0, 4.0, 6.0), ros::Time(5.5), first_update, prev_vel, curr_acc, prev_state_time);

  EXPECT_TRUE(std::isfinite(curr_acc.x()));
  EXPECT_TRUE(std::isfinite(curr_acc.y()));
  EXPECT_TRUE(std::isfinite(curr_acc.z()));
  EXPECT_NEAR(2.0, curr_acc.x(), 1e-9);
  EXPECT_NEAR(4.0, curr_acc.y(), 1e-9);
  EXPECT_NEAR(6.0, curr_acc.z(), 1e-9);
}

TEST(FlightBaseKinematics, ExternalOffboardControlActiveWhileHeartbeatFresh) {
  const bool is_active = AutoFlight::flightBase::isExternalOffboardControlActive(
      true, ros::Time(10.0), ros::Time(10.2), 0.3);

  EXPECT_TRUE(is_active);
}

TEST(FlightBaseKinematics, ExternalOffboardControlInactiveAfterTimeout) {
  const bool is_active = AutoFlight::flightBase::isExternalOffboardControlActive(
      true, ros::Time(10.0), ros::Time(10.31), 0.3);

  EXPECT_FALSE(is_active);
}

TEST(FlightBaseKinematics, ExternalOffboardControlInactiveWithoutHeartbeat) {
  const bool is_active = AutoFlight::flightBase::isExternalOffboardControlActive(
      false, ros::Time(10.0), ros::Time(10.1), 0.3);

  EXPECT_FALSE(is_active);
}

TEST(FlightBaseKinematics, PointInsideLocalXYRangeIsAccepted) {
  EXPECT_TRUE(AutoFlight::flightBase::isPointInLocalXYRange(5.0, -3.0, 0.0, 0.0, 10.0, 10.0));
}

TEST(FlightBaseKinematics, PointOutsideLocalXYRangeIsRejected) {
  EXPECT_FALSE(AutoFlight::flightBase::isPointInLocalXYRange(10.1, 0.0, 0.0, 0.0, 10.0, 10.0));
  EXPECT_FALSE(AutoFlight::flightBase::isPointInLocalXYRange(0.0, -10.1, 0.0, 0.0, 10.0, 10.0));
}

}  // namespace

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
