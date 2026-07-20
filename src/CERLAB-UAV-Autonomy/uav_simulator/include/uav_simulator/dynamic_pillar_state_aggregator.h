#ifndef UAV_SIMULATOR_DYNAMIC_PILLAR_STATE_AGGREGATOR_H
#define UAV_SIMULATOR_DYNAMIC_PILLAR_STATE_AGGREGATOR_H

#include <map>
#include <string>

#include <ros/duration.h>
#include <ros/time.h>
#include <uav_simulator/DynamicObstacleArray.h>
#include <uav_simulator/DynamicObstacleState.h>

namespace uav_simulator
{
class DynamicPillarStateAggregator
{
public:
  explicit DynamicPillarStateAggregator(bool publishStaticObstacles);

  void upsert(const DynamicObstacleState& state);
  void removeStale(const ros::Time& now, const ros::Duration& timeout);
  DynamicObstacleArray buildArray(const ros::Time& stamp) const;
  std::size_t size() const;

private:
  bool shouldPublish(const DynamicObstacleState& state) const;

  bool publishStaticObstacles_;
  std::map<std::string, DynamicObstacleState> states_;
};
}

#endif
