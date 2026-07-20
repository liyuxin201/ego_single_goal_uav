#ifndef UAV_SIMULATOR_SMOOTH_PILLAR_PATH_PLUGIN_HH
#define UAV_SIMULATOR_SMOOTH_PILLAR_PATH_PLUGIN_HH

#include <memory>
#include <string>
#include <vector>

#include <gazebo/common/common.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <ignition/math.hh>
#include <ros/ros.h>
#include <uav_simulator/DynamicObstacleState.h>

namespace gazebo
{
class SmoothPillarPathPlugin : public ModelPlugin
{
public:
  SmoothPillarPathPlugin();
  void Load(physics::ModelPtr parent, sdf::ElementPtr sdf) override;

private:
  void onUpdate();
  void loadParameters(sdf::ElementPtr sdf);
  void loadPath(sdf::ElementPtr sdf);
  void advance(double dt, double simTime);
  void advanceSegment();
  double currentSpeed(double simTime) const;
  ignition::math::Vector3d segmentDirection() const;
  double segmentDistance() const;
  void publishState(const ignition::math::Vector3d& velocity, const ros::Time& stamp);

  physics::ModelPtr model_;
  event::ConnectionPtr updateConnection_;
  std::unique_ptr<ros::NodeHandle> nodeHandle_;
  ros::Publisher statePublisher_;

  std::vector<ignition::math::Vector3d> waypoints_;
  ignition::math::Vector3d currentPosition_;

  std::string stateTopic_;
  std::string frameId_;
  std::string obstacleType_;
  bool loop_;
  bool orientation_;
  double velocityMin_;
  double velocityMax_;
  double speedPeriod_;
  double phase_;
  double size_;
  double height_;
  double publishRate_;
  double lastPublishTime_;

  std::size_t currentWaypoint_;
  std::size_t nextWaypoint_;
  int pathDirection_;
  bool active_;

  gazebo::common::Time lastUpdateTime_;
};
}

#endif
