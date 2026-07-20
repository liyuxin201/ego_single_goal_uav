#include <ros/ros.h>
#include <uav_simulator/DynamicObstacleArray.h>
#include <uav_simulator/DynamicObstacleState.h>
#include <uav_simulator/dynamic_pillar_state_aggregator.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "dynamic_pillar_state_node");

  ros::NodeHandle nh;
  ros::NodeHandle privateNh("~");

  std::string inputTopic;
  std::string outputTopic;
  bool publishStaticObstacles;
  double publishRate;
  double staleTimeout;

  privateNh.param<std::string>("input_topic", inputTopic, "/large_pillar/dynamic_obstacle_states");
  privateNh.param<std::string>("output_topic", outputTopic, "/large_pillar/dynamic_obstacles");
  privateNh.param("publish_static_obstacles", publishStaticObstacles, false);
  privateNh.param("rate", publishRate, 30.0);
  privateNh.param("stale_timeout", staleTimeout, 2.0);

  uav_simulator::DynamicPillarStateAggregator aggregator(publishStaticObstacles);

  ros::Subscriber stateSub = nh.subscribe<uav_simulator::DynamicObstacleState>(
      inputTopic, 100, [&aggregator](const uav_simulator::DynamicObstacleState::ConstPtr& msg) {
        aggregator.upsert(*msg);
      });

  ros::Publisher arrayPub =
      nh.advertise<uav_simulator::DynamicObstacleArray>(outputTopic, 10);

  ros::Rate rate(publishRate);
  const ros::Duration timeout(staleTimeout);

  while (ros::ok())
  {
    ros::spinOnce();

    const ros::Time now = ros::Time::now();
    aggregator.removeStale(now, timeout);
    arrayPub.publish(aggregator.buildArray(now));

    rate.sleep();
  }

  return 0;
}
