#include <uav_simulator/smoothPillarPathPlugin.hh>

#include <algorithm>
#include <cmath>
#include <functional>

namespace gazebo
{
GZ_REGISTER_MODEL_PLUGIN(SmoothPillarPathPlugin)

SmoothPillarPathPlugin::SmoothPillarPathPlugin()
  : loop_(false)
  , orientation_(false)
  , velocityMin_(0.2)
  , velocityMax_(1.0)
  , speedPeriod_(8.0)
  , phase_(0.0)
  , size_(1.0)
  , height_(3.0)
  , publishRate_(30.0)
  , lastPublishTime_(-1.0)
  , currentWaypoint_(0)
  , nextWaypoint_(1)
  , pathDirection_(1)
  , active_(false)
{
}

void SmoothPillarPathPlugin::Load(physics::ModelPtr parent, sdf::ElementPtr sdf)
{
  model_ = parent;

  if (!ros::isInitialized())
  {
    ROS_FATAL_STREAM("smoothPillarPathPlugin requires gazebo_ros_init.");
    return;
  }

  loadParameters(sdf);
  loadPath(sdf);

  if (waypoints_.size() < 2)
  {
    ROS_ERROR_STREAM("smoothPillarPathPlugin on " << model_->GetName()
                                                  << " requires at least two waypoints.");
    return;
  }

  nodeHandle_.reset(new ros::NodeHandle());
  statePublisher_ = nodeHandle_->advertise<uav_simulator::DynamicObstacleState>(stateTopic_, 10);

  currentPosition_ = waypoints_.front();
  model_->SetWorldPose(ignition::math::Pose3d(currentPosition_, ignition::math::Quaterniond(0.0, 0.0, 0.0)));

  lastUpdateTime_ = model_->GetWorld()->SimTime();
  active_ = true;
  updateConnection_ = event::Events::ConnectWorldUpdateBegin(
      std::bind(&SmoothPillarPathPlugin::onUpdate, this));
}

void SmoothPillarPathPlugin::loadParameters(sdf::ElementPtr sdf)
{
  stateTopic_ = sdf->HasElement("state_topic")
                    ? sdf->Get<std::string>("state_topic")
                    : "/large_pillar/dynamic_obstacle_states";
  frameId_ = sdf->HasElement("frame_id") ? sdf->Get<std::string>("frame_id") : "world";
  obstacleType_ = sdf->HasElement("obstacle_type") ? sdf->Get<std::string>("obstacle_type") : "unknown";
  loop_ = sdf->HasElement("loop") ? sdf->Get<bool>("loop") : false;
  orientation_ = sdf->HasElement("orientation") ? sdf->Get<bool>("orientation") : false;
  velocityMin_ = sdf->HasElement("velocity_min") ? sdf->Get<double>("velocity_min") : 0.2;
  velocityMax_ = sdf->HasElement("velocity_max") ? sdf->Get<double>("velocity_max") : 1.0;
  speedPeriod_ = sdf->HasElement("speed_period") ? sdf->Get<double>("speed_period") : 8.0;
  phase_ = sdf->HasElement("phase") ? sdf->Get<double>("phase") : 0.0;
  size_ = sdf->HasElement("size") ? sdf->Get<double>("size") : 1.0;
  height_ = sdf->HasElement("height") ? sdf->Get<double>("height") : 3.0;
  publishRate_ = sdf->HasElement("publish_rate") ? sdf->Get<double>("publish_rate") : 30.0;

  if (velocityMax_ < velocityMin_)
  {
    std::swap(velocityMin_, velocityMax_);
  }
  if (speedPeriod_ <= 0.0)
  {
    speedPeriod_ = 8.0;
  }
  if (publishRate_ <= 0.0)
  {
    publishRate_ = 30.0;
  }
}

void SmoothPillarPathPlugin::loadPath(sdf::ElementPtr sdf)
{
  waypoints_.clear();

  if (!sdf->HasElement("path"))
  {
    return;
  }

  sdf::ElementPtr pathElem = sdf->GetElement("path");
  if (!pathElem->HasElement("waypoint"))
  {
    return;
  }

  sdf::ElementPtr waypointElem = pathElem->GetElement("waypoint");
  while (waypointElem)
  {
    waypoints_.push_back(waypointElem->Get<ignition::math::Vector3d>());
    waypointElem = waypointElem->GetNextElement("waypoint");
  }
}

void SmoothPillarPathPlugin::onUpdate()
{
  if (!active_)
  {
    return;
  }

  const gazebo::common::Time now = model_->GetWorld()->SimTime();
  const double dt = (now - lastUpdateTime_).Double();
  lastUpdateTime_ = now;

  if (dt <= 0.0)
  {
    return;
  }

  const double simTime = now.Double();
  const double speed = currentSpeed(simTime);
  advance(dt, simTime);

  const ignition::math::Vector3d velocity = segmentDirection() * speed;

  double yaw = 0.0;
  if (orientation_)
  {
    const ignition::math::Vector3d direction = segmentDirection();
    yaw = std::atan2(direction.Y(), direction.X());
  }

  model_->SetWorldPose(
      ignition::math::Pose3d(currentPosition_, ignition::math::Quaterniond(0.0, 0.0, yaw)));

  if ((lastPublishTime_ < 0.0) || (simTime - lastPublishTime_ >= 1.0 / publishRate_))
  {
    ros::Time stamp;
    stamp.fromSec(simTime);
    publishState(velocity, stamp);
    lastPublishTime_ = simTime;
  }
}

void SmoothPillarPathPlugin::advance(double dt, double simTime)
{
  double remainingStep = currentSpeed(simTime) * dt;

  while (remainingStep > 0.0)
  {
    const ignition::math::Vector3d target = waypoints_[nextWaypoint_];
    const ignition::math::Vector3d toTarget = target - currentPosition_;
    const double remainingDistance = toTarget.Length();

    if (remainingDistance < 1e-6)
    {
      advanceSegment();
      continue;
    }

    if (remainingStep < remainingDistance)
    {
      currentPosition_ += toTarget.Normalized() * remainingStep;
      break;
    }

    currentPosition_ = target;
    remainingStep -= remainingDistance;
    advanceSegment();
  }
}

void SmoothPillarPathPlugin::advanceSegment()
{
  currentWaypoint_ = nextWaypoint_;

  if (loop_)
  {
    nextWaypoint_ = (currentWaypoint_ + 1) % waypoints_.size();
    return;
  }

  if (currentWaypoint_ == waypoints_.size() - 1)
  {
    pathDirection_ = -1;
  }
  else if (currentWaypoint_ == 0)
  {
    pathDirection_ = 1;
  }

  nextWaypoint_ = static_cast<std::size_t>(static_cast<int>(currentWaypoint_) + pathDirection_);
}

double SmoothPillarPathPlugin::currentSpeed(double simTime) const
{
  const double mid = (velocityMin_ + velocityMax_) * 0.5;
  const double amp = (velocityMax_ - velocityMin_) * 0.5;
  return mid + amp * std::sin((2.0 * M_PI * simTime / speedPeriod_) + phase_);
}

ignition::math::Vector3d SmoothPillarPathPlugin::segmentDirection() const
{
  const ignition::math::Vector3d delta = waypoints_[nextWaypoint_] - currentPosition_;
  if (delta.Length() < 1e-6)
  {
    return ignition::math::Vector3d::Zero;
  }
  return delta.Normalized();
}

double SmoothPillarPathPlugin::segmentDistance() const
{
  return (waypoints_[nextWaypoint_] - currentPosition_).Length();
}

void SmoothPillarPathPlugin::publishState(const ignition::math::Vector3d& velocity, const ros::Time& stamp)
{
  uav_simulator::DynamicObstacleState state;
  state.header.stamp = stamp;
  state.header.frame_id = frameId_;
  state.name = model_->GetName();
  state.obstacle_type = obstacleType_;
  state.size = size_;
  state.height = height_;
  state.is_dynamic = true;
  state.pose.position.x = currentPosition_.X();
  state.pose.position.y = currentPosition_.Y();
  state.pose.position.z = currentPosition_.Z();
  state.pose.orientation.w = 1.0;
  state.twist.linear.x = velocity.X();
  state.twist.linear.y = velocity.Y();
  state.twist.linear.z = velocity.Z();

  statePublisher_.publish(state);
}
}
