#include <autonomous_flight/simulation/dynamicNavigation.h>
#include <trajectory_planner/bspline.h>
#include <uav_simulator/DynamicObstacleState.h>
#include <gtest/gtest.h>

namespace {

TEST(DynamicNavigationStartEndConditions, PlanarPlanningZerosVerticalConstraints) {
  const Eigen::Vector3d curr_vel(0.0, 0.0, -6.0);
  const Eigen::Vector3d curr_acc(0.0, 0.0, -50.0);
  const std::vector<Eigen::Vector3d> start_end_conditions =
      AutoFlight::dynamicNavigation::buildStartEndConditions(curr_vel, curr_acc, false);

  ASSERT_EQ(4u, start_end_conditions.size());
  EXPECT_DOUBLE_EQ(0.0, start_end_conditions[0](2));
  EXPECT_DOUBLE_EQ(0.0, start_end_conditions[1](2));
  EXPECT_DOUBLE_EQ(0.0, start_end_conditions[2](2));
  EXPECT_DOUBLE_EQ(0.0, start_end_conditions[3](2));

  const std::vector<Eigen::Vector3d> points = {
      Eigen::Vector3d(0.0, 0.0, 1.0),
      Eigen::Vector3d(1.0, 0.0, 1.0),
      Eigen::Vector3d(2.0, 0.0, 1.0),
      Eigen::Vector3d(3.0, 0.0, 1.0),
  };
  Eigen::MatrixXd control_points;
  trajPlanner::bspline::parameterizeToBspline(0.2, points, start_end_conditions, control_points);

  for (int i = 0; i < control_points.cols(); ++i) {
    EXPECT_GE(control_points(2, i), 0.7);
    EXPECT_LE(control_points(2, i), 1.3);
  }
}

TEST(DynamicNavigationStartEndConditions, ThreeDimensionalPlanningKeepsVerticalConstraints) {
  const Eigen::Vector3d curr_vel(0.3, -0.2, -1.5);
  const Eigen::Vector3d curr_acc(0.4, 0.1, -3.0);
  const std::vector<Eigen::Vector3d> start_end_conditions =
      AutoFlight::dynamicNavigation::buildStartEndConditions(curr_vel, curr_acc, true);

  ASSERT_EQ(4u, start_end_conditions.size());
  EXPECT_DOUBLE_EQ(curr_vel(0), start_end_conditions[0](0));
  EXPECT_DOUBLE_EQ(curr_vel(1), start_end_conditions[0](1));
  EXPECT_DOUBLE_EQ(curr_vel(2), start_end_conditions[0](2));
  EXPECT_DOUBLE_EQ(curr_acc(2), start_end_conditions[2](2));
}

TEST(DynamicNavigationStartEndConditions, PlannerOnlyModeSkipsPreTurnBeforePlanning) {
  EXPECT_FALSE(AutoFlight::dynamicNavigation::shouldTurnTowardGoalBeforePlanning(
      false, false, false));
}

TEST(DynamicNavigationStartEndConditions, ControllerDrivenModeKeepsPreTurnBehavior) {
  EXPECT_TRUE(AutoFlight::dynamicNavigation::shouldTurnTowardGoalBeforePlanning(
      false, false, true));
  EXPECT_FALSE(AutoFlight::dynamicNavigation::shouldTurnTowardGoalBeforePlanning(
      true, false, true));
  EXPECT_FALSE(AutoFlight::dynamicNavigation::shouldTurnTowardGoalBeforePlanning(
      false, true, true));
}

TEST(DynamicNavigationStartEndConditions, PlannerOnlyModeUsesLookaheadTimeForPlannerPoseTarget) {
  EXPECT_DOUBLE_EQ(1.35, AutoFlight::dynamicNavigation::computePlannerPoseTargetTime(
                             1.0, 2.0, false, 0.35));
}

TEST(DynamicNavigationStartEndConditions, ControllerDrivenModeUsesCurrentTrajectoryTime) {
  EXPECT_DOUBLE_EQ(1.0, AutoFlight::dynamicNavigation::computePlannerPoseTargetTime(
                            1.0, 2.0, true, 0.35));
}

TEST(DynamicNavigationStartEndConditions, PlannerPoseTargetTimeIsClampedToTrajectoryEnd) {
  EXPECT_DOUBLE_EQ(2.0, AutoFlight::dynamicNavigation::computePlannerPoseTargetTime(
                            1.9, 2.0, false, 0.35));
}

TEST(DynamicNavigationStartEndConditions, ControlTargetTimeUsesLookahead) {
  EXPECT_DOUBLE_EQ(1.35, AutoFlight::dynamicNavigation::computeControlTargetTime(
                             1.0, 2.0, 0.35));
}

TEST(DynamicNavigationStartEndConditions, ControlTargetTimeIsClampedToTrajectoryEnd) {
  EXPECT_DOUBLE_EQ(2.0, AutoFlight::dynamicNavigation::computeControlTargetTime(
                            1.9, 2.0, 0.35));
}

TEST(DynamicNavigationStartEndConditions, PlannerPoseTargetYawFollowsTrajectoryVelocity) {
  const Eigen::Vector3d velocity(0.0, 2.0, 0.0);
  EXPECT_NEAR(M_PI_2, AutoFlight::dynamicNavigation::computePlannerPoseTargetYaw(
                        velocity, 0.25, true, false), 1e-9);
}

TEST(DynamicNavigationStartEndConditions, PlannerPoseTargetYawKeepsFallbackWhenNearlyStopped) {
  const Eigen::Vector3d velocity(1e-5, 0.0, 0.0);
  EXPECT_DOUBLE_EQ(0.75, AutoFlight::dynamicNavigation::computePlannerPoseTargetYaw(
                             velocity, 0.75, true, false));
}

TEST(DynamicNavigationStartEndConditions, PlannerPoseTargetYawKeepsFallbackWhenYawControlDisabled) {
  const Eigen::Vector3d velocity(1.0, 1.0, 0.0);
  EXPECT_DOUBLE_EQ(-0.5, AutoFlight::dynamicNavigation::computePlannerPoseTargetYaw(
                             velocity, -0.5, false, false));
  EXPECT_DOUBLE_EQ(-0.5, AutoFlight::dynamicNavigation::computePlannerPoseTargetYaw(
                             velocity, -0.5, true, true));
}

TEST(DynamicNavigationStartEndConditions, Px4RawSetpointCarriesTrajectoryState) {
  const Eigen::Vector3d position(1.0, -2.0, 1.2);
  const Eigen::Vector3d velocity(0.8, 0.3, -0.1);
  const Eigen::Vector3d acceleration(0.2, -0.4, 0.0);
  const ros::Time stamp(123.0);
  const double yaw = 0.6;

  const mavros_msgs::PositionTarget setpoint =
      AutoFlight::dynamicNavigation::buildPx4RawSetpoint(
          position, velocity, acceleration, yaw, stamp);

  EXPECT_EQ(stamp, setpoint.header.stamp);
  EXPECT_EQ("map", setpoint.header.frame_id);
  EXPECT_EQ(mavros_msgs::PositionTarget::FRAME_LOCAL_NED, setpoint.coordinate_frame);
  EXPECT_EQ(mavros_msgs::PositionTarget::IGNORE_YAW_RATE, setpoint.type_mask);
  EXPECT_DOUBLE_EQ(position(0), setpoint.position.x);
  EXPECT_DOUBLE_EQ(position(1), setpoint.position.y);
  EXPECT_DOUBLE_EQ(position(2), setpoint.position.z);
  EXPECT_DOUBLE_EQ(velocity(0), setpoint.velocity.x);
  EXPECT_DOUBLE_EQ(velocity(1), setpoint.velocity.y);
  EXPECT_DOUBLE_EQ(velocity(2), setpoint.velocity.z);
  EXPECT_DOUBLE_EQ(acceleration(0), setpoint.acceleration_or_force.x);
  EXPECT_DOUBLE_EQ(acceleration(1), setpoint.acceleration_or_force.y);
  EXPECT_DOUBLE_EQ(acceleration(2), setpoint.acceleration_or_force.z);
  EXPECT_FLOAT_EQ(yaw, setpoint.yaw);
  EXPECT_FLOAT_EQ(0.0, setpoint.yaw_rate);
}

TEST(DynamicNavigationStartEndConditions, PredictedDynamicObstacleTriggersBeforeCurrentCollision) {
  const Eigen::Vector3d trajectory_point(2.0, 0.0, 1.0);
  const Eigen::Vector3d obstacle_pos(0.0, 0.0, 1.0);
  const Eigen::Vector3d obstacle_vel(2.0, 0.0, 0.0);
  const Eigen::Vector3d obstacle_size(0.4, 0.4, 1.0);

  EXPECT_TRUE(AutoFlight::dynamicNavigation::isPointInPredictedDynamicObstacle(
      trajectory_point, obstacle_pos, obstacle_vel, obstacle_size, 1.0, 0.3));
}

TEST(DynamicNavigationStartEndConditions, PredictedDynamicObstacleUsesSafetyMargin) {
  const Eigen::Vector3d trajectory_point(2.45, 0.0, 1.0);
  const Eigen::Vector3d obstacle_pos(0.0, 0.0, 1.0);
  const Eigen::Vector3d obstacle_vel(2.0, 0.0, 0.0);
  const Eigen::Vector3d obstacle_size(0.4, 0.4, 1.0);

  EXPECT_FALSE(AutoFlight::dynamicNavigation::isPointInPredictedDynamicObstacle(
      trajectory_point, obstacle_pos, obstacle_vel, obstacle_size, 1.0, 0.0));
  EXPECT_TRUE(AutoFlight::dynamicNavigation::isPointInPredictedDynamicObstacle(
      trajectory_point, obstacle_pos, obstacle_vel, obstacle_size, 1.0, 0.3));
}

TEST(DynamicNavigationStartEndConditions, ConvertsDynamicObstacleTruthToPlannerObstacle) {
  uav_simulator::DynamicObstacleState state;
  state.pose.position.x = 1.0;
  state.pose.position.y = -2.0;
  state.pose.position.z = 0.0;
  state.twist.linear.x = 0.4;
  state.twist.linear.y = -0.2;
  state.size = 1.6;
  state.height = 3.0;

  Eigen::Vector3d pos;
  Eigen::Vector3d vel;
  Eigen::Vector3d size;
  AutoFlight::dynamicNavigation::convertDynamicObstacleTruth(state, pos, vel, size);

  EXPECT_DOUBLE_EQ(1.0, pos(0));
  EXPECT_DOUBLE_EQ(-2.0, pos(1));
  EXPECT_DOUBLE_EQ(1.5, pos(2));
  EXPECT_DOUBLE_EQ(0.4, vel(0));
  EXPECT_DOUBLE_EQ(-0.2, vel(1));
  EXPECT_DOUBLE_EQ(0.0, vel(2));
  EXPECT_DOUBLE_EQ(1.6, size(0));
  EXPECT_DOUBLE_EQ(1.6, size(1));
  EXPECT_DOUBLE_EQ(3.0, size(2));
}

}  // namespace

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
