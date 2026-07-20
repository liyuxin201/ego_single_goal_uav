#include <gtest/gtest.h>
#include <tracking_controller/trackingController.h>

TEST(TrackingControllerRawSetpoint, CopiesTargetStateToPx4RawSetpoint) {
	tracking_controller::Target target;
	target.position.x = 1.0;
	target.position.y = 2.0;
	target.position.z = 3.0;
	target.velocity.x = 0.4;
	target.velocity.y = 0.5;
	target.velocity.z = 0.6;
	target.acceleration.x = 0.7;
	target.acceleration.y = 0.8;
	target.acceleration.z = 0.9;
	target.yaw = 1.2;

	const ros::Time stamp(12.3);
	const mavros_msgs::PositionTarget setpoint =
		controller::trackingController::buildRawSetpointFromTarget(target, stamp);

	EXPECT_EQ(stamp, setpoint.header.stamp);
	EXPECT_EQ(std::string("map"), setpoint.header.frame_id);
	EXPECT_EQ(mavros_msgs::PositionTarget::FRAME_LOCAL_NED, setpoint.coordinate_frame);
	EXPECT_EQ(mavros_msgs::PositionTarget::IGNORE_YAW_RATE, setpoint.type_mask);
	EXPECT_DOUBLE_EQ(1.0, setpoint.position.x);
	EXPECT_DOUBLE_EQ(2.0, setpoint.position.y);
	EXPECT_DOUBLE_EQ(3.0, setpoint.position.z);
	EXPECT_DOUBLE_EQ(0.4, setpoint.velocity.x);
	EXPECT_DOUBLE_EQ(0.5, setpoint.velocity.y);
	EXPECT_DOUBLE_EQ(0.6, setpoint.velocity.z);
	EXPECT_DOUBLE_EQ(0.7, setpoint.acceleration_or_force.x);
	EXPECT_DOUBLE_EQ(0.8, setpoint.acceleration_or_force.y);
	EXPECT_DOUBLE_EQ(0.9, setpoint.acceleration_or_force.z);
	EXPECT_FLOAT_EQ(1.2f, setpoint.yaw);
	EXPECT_DOUBLE_EQ(0.0, setpoint.yaw_rate);
}

TEST(TrackingControllerRawSetpoint, BuildsVelocityHeightSetpointWithPositionFeedbackAndLimits) {
	tracking_controller::Target target;
	target.position.x = 4.0;
	target.position.y = -2.0;
	target.position.z = 1.2;
	target.velocity.x = 0.5;
	target.velocity.y = -0.2;
	target.yaw = 0.7;

	geometry_msgs::Pose current_pose;
	current_pose.position.x = 1.0;
	current_pose.position.y = -1.0;
	current_pose.position.z = 0.9;

	const Eigen::Vector3d position_gain(0.8, 0.7, 1.0);
	const Eigen::Vector3d max_velocity(2.0, 0.6, 1.0);
	const ros::Time stamp(45.6);

	const mavros_msgs::PositionTarget setpoint =
		controller::trackingController::buildVelocityHeightSetpointFromTarget(
			target, current_pose, position_gain, max_velocity, stamp);

	EXPECT_EQ(stamp, setpoint.header.stamp);
	EXPECT_EQ(std::string("map"), setpoint.header.frame_id);
	EXPECT_EQ(mavros_msgs::PositionTarget::FRAME_LOCAL_NED, setpoint.coordinate_frame);
	EXPECT_EQ(
		mavros_msgs::PositionTarget::IGNORE_AFX |
		mavros_msgs::PositionTarget::IGNORE_AFY |
		mavros_msgs::PositionTarget::IGNORE_AFZ |
		mavros_msgs::PositionTarget::IGNORE_PX |
		mavros_msgs::PositionTarget::IGNORE_PY |
		mavros_msgs::PositionTarget::IGNORE_VZ |
		mavros_msgs::PositionTarget::IGNORE_YAW_RATE,
		setpoint.type_mask);
	EXPECT_DOUBLE_EQ(1.2, setpoint.position.z);
	EXPECT_DOUBLE_EQ(2.0, setpoint.velocity.x);
	EXPECT_DOUBLE_EQ(-0.6, setpoint.velocity.y);
	EXPECT_FLOAT_EQ(0.7f, setpoint.yaw);
	EXPECT_DOUBLE_EQ(0.0, setpoint.yaw_rate);
}

int main(int argc, char **argv) {
	testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}
