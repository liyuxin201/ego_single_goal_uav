#include <gtest/gtest.h>
#include <ros/time.h>
#include <uav_simulator/dynamic_pillar_state_aggregator.h>

namespace
{
uav_simulator::DynamicObstacleState makeState(
    const std::string& name,
    bool isDynamic,
    const ros::Time& stamp)
{
  uav_simulator::DynamicObstacleState state;
  state.header.stamp = stamp;
  state.header.frame_id = "world";
  state.name = name;
  state.obstacle_type = isDynamic ? "cylinder" : "square";
  state.size = 1.5;
  state.height = 3.0;
  state.is_dynamic = isDynamic;
  state.pose.position.x = 1.0;
  state.pose.position.y = 2.0;
  state.twist.linear.x = isDynamic ? 0.4 : 0.0;
  return state;
}
}

TEST(DynamicPillarStateAggregator, excludesStaticObstaclesByDefault)
{
  uav_simulator::DynamicPillarStateAggregator aggregator(false);

  aggregator.upsert(makeState("dynamic_01", true, ros::Time(10.0)));
  aggregator.upsert(makeState("static_01", false, ros::Time(10.0)));

  const auto array = aggregator.buildArray(ros::Time(11.0));

  ASSERT_EQ(array.obstacles.size(), 1u);
  EXPECT_EQ(array.obstacles[0].name, "dynamic_01");
  EXPECT_TRUE(array.obstacles[0].is_dynamic);
  EXPECT_EQ(array.header.stamp, ros::Time(11.0));
  EXPECT_EQ(array.header.frame_id, "world");
}

TEST(DynamicPillarStateAggregator, removesStaleObstacleStates)
{
  uav_simulator::DynamicPillarStateAggregator aggregator(false);

  aggregator.upsert(makeState("fresh_dynamic", true, ros::Time(9.0)));
  aggregator.upsert(makeState("stale_dynamic", true, ros::Time(1.0)));

  aggregator.removeStale(ros::Time(12.0), ros::Duration(5.0));
  const auto array = aggregator.buildArray(ros::Time(12.0));

  ASSERT_EQ(array.obstacles.size(), 1u);
  EXPECT_EQ(array.obstacles[0].name, "fresh_dynamic");
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
