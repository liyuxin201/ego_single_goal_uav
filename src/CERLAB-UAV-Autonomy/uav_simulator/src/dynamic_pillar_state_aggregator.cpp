#include <uav_simulator/dynamic_pillar_state_aggregator.h>

namespace uav_simulator
{
DynamicPillarStateAggregator::DynamicPillarStateAggregator(bool publishStaticObstacles)
  : publishStaticObstacles_(publishStaticObstacles)
{
}

void DynamicPillarStateAggregator::upsert(const DynamicObstacleState& state)
{
  if (state.name.empty())
  {
    return;
  }

  states_[state.name] = state;
}

void DynamicPillarStateAggregator::removeStale(const ros::Time& now, const ros::Duration& timeout)
{
  for (auto it = states_.begin(); it != states_.end();)
  {
    if ((now - it->second.header.stamp) > timeout)
    {
      it = states_.erase(it);
    }
    else
    {
      ++it;
    }
  }
}

DynamicObstacleArray DynamicPillarStateAggregator::buildArray(const ros::Time& stamp) const
{
  DynamicObstacleArray array;
  array.header.stamp = stamp;
  array.header.frame_id = "world";

  for (const auto& item : states_)
  {
    if (shouldPublish(item.second))
    {
      array.obstacles.push_back(item.second);
    }
  }

  return array;
}

std::size_t DynamicPillarStateAggregator::size() const
{
  return states_.size();
}

bool DynamicPillarStateAggregator::shouldPublish(const DynamicObstacleState& state) const
{
  return publishStaticObstacles_ || state.is_dynamic;
}
}
