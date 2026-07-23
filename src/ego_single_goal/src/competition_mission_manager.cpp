#include <actionlib/client/simple_action_client.h>
#include <across_gate_module/GatePlanAction.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/State.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <xmlrpcpp/XmlRpcValue.h>

#include <cmath>
#include <memory>
#include <string>
#include <vector>

namespace
{

struct Point3
{
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
};

double distance(const Point3& a, const Point3& b)
{
  const double dx = a.x - b.x;
  const double dy = a.y - b.y;
  const double dz = a.z - b.z;
  return std::sqrt(dx * dx + dy * dy + dz * dz);
}

Point3 pointFromPose(const geometry_msgs::PoseStamped& pose)
{
  Point3 point;
  point.x = pose.pose.position.x;
  point.y = pose.pose.position.y;
  point.z = pose.pose.position.z;
  return point;
}

bool xmlRpcToDouble(const XmlRpc::XmlRpcValue& value, double* output)
{
  if (value.getType() == XmlRpc::XmlRpcValue::TypeDouble)
  {
    *output = static_cast<double>(value);
    return true;
  }
  if (value.getType() == XmlRpc::XmlRpcValue::TypeInt)
  {
    *output = static_cast<int>(value);
    return true;
  }
  return false;
}

std::string boolText(bool value)
{
  return value ? "true" : "false";
}

}  // namespace

class CompetitionMissionManager
{
 public:
  CompetitionMissionManager()
    : private_nh_("~"), tf_listener_(tf_buffer_)
  {
    loadParams();

    goal_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(goal_topic_, 1, true);
    guide_path_pub_ = nh_.advertise<nav_msgs::Path>(guide_path_topic_, 1, true);
    task_a_goal_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(task_a_goal_input_topic_, 1, true);

    odom_sub_ = nh_.subscribe(odom_topic_, 1, &CompetitionMissionManager::odomCallback, this);
    state_sub_ = nh_.subscribe(mavros_state_topic_, 1, &CompetitionMissionManager::stateCallback, this);
    task_a_guide_sub_ = nh_.subscribe(task_a_guide_path_topic_, 1, &CompetitionMissionManager::taskAGuideCallback, this);
    task_a_candidate_sub_ =
        nh_.subscribe(task_a_candidate_guide_path_topic_, 1, &CompetitionMissionManager::taskACandidateCallback, this);

    if (task_d_enabled_)
    {
      gate_client_.reset(new GateClient(gate_action_name_, true));
    }

    timer_ = nh_.createTimer(ros::Duration(1.0 / std::max(loop_rate_hz_, 1.0)),
                             &CompetitionMissionManager::timerCallback, this);

    ROS_INFO("competition_mission_manager: frame=%s goal_topic=%s guide_path=%s odom=%s wait_offboard=%s",
             frame_id_.c_str(),
             goal_topic_.c_str(),
             guide_path_topic_.c_str(),
             odom_topic_.c_str(),
             boolText(wait_for_offboard_).c_str());
    ROS_INFO("competition_mission_manager: task_b_enabled=%s task_b_waypoints=%zu task_c_enabled=%s "
             "task_c_waypoints=%zu task_d_enabled=%s task_d_prepare_waypoints=%zu gate_action=%s",
             boolText(task_b_enabled_).c_str(),
             task_b_waypoints_.size(),
             boolText(task_c_enabled_).c_str(),
             task_c_waypoints_.size(),
             boolText(task_d_enabled_).c_str(),
             task_d_prepare_waypoints_.size(),
             gate_action_name_.c_str());
  }

  ~CompetitionMissionManager()
  {
    if (gate_client_ && gate_goal_sent_ && !gate_result_ready_ && !gate_failed_)
    {
      gate_client_->cancelGoal();
    }
  }

 private:
  using GateClient = actionlib::SimpleActionClient<across_gate_module::GatePlanAction>;

  enum class Phase
  {
    WAIT_READY,
    TAKEOFF,
    TASK_A_WAIT_PATH,
    TASK_A_FLY,
    TASK_B_SETUP,
    TASK_B_FLY,
    TASK_C_SETUP,
    TASK_C_FLY,
    TASK_D_PREPARE,
    TASK_D_START_ACTION,
    TASK_D_FLY_OBSERVATION,
    TASK_D_TRAVERSE,
    MISSION_DONE,
    MISSION_ABORTED
  };

  static const char* phaseName(Phase phase)
  {
    switch (phase)
    {
      case Phase::WAIT_READY:
        return "WAIT_READY";
      case Phase::TAKEOFF:
        return "TAKEOFF";
      case Phase::TASK_A_WAIT_PATH:
        return "TASK_A_WAIT_PATH";
      case Phase::TASK_A_FLY:
        return "TASK_A_FLY";
      case Phase::TASK_B_SETUP:
        return "TASK_B_SETUP";
      case Phase::TASK_B_FLY:
        return "TASK_B_FLY";
      case Phase::TASK_C_SETUP:
        return "TASK_C_SETUP";
      case Phase::TASK_C_FLY:
        return "TASK_C_FLY";
      case Phase::TASK_D_PREPARE:
        return "TASK_D_PREPARE";
      case Phase::TASK_D_START_ACTION:
        return "TASK_D_START_ACTION";
      case Phase::TASK_D_FLY_OBSERVATION:
        return "TASK_D_FLY_OBSERVATION";
      case Phase::TASK_D_TRAVERSE:
        return "TASK_D_TRAVERSE";
      case Phase::MISSION_DONE:
        return "MISSION_DONE";
      case Phase::MISSION_ABORTED:
        return "MISSION_ABORTED";
    }
    return "UNKNOWN";
  }

  void loadParams()
  {
    private_nh_.param<std::string>("frame_id", frame_id_, "map");
    private_nh_.param<std::string>("odom_topic", odom_topic_, "/mavros/local_position/odom");
    private_nh_.param<std::string>("mavros_state_topic", mavros_state_topic_, "/mavros/state");
    private_nh_.param<std::string>("goal_topic", goal_topic_, "/move_base_simple/goal");
    private_nh_.param<std::string>("guide_path_topic", guide_path_topic_, "/competition/guide_path");
    private_nh_.param<std::string>("task_a_guide_path_topic", task_a_guide_path_topic_, "/task_a/guide_path");
    private_nh_.param<std::string>("task_a_candidate_guide_path_topic",
                                   task_a_candidate_guide_path_topic_,
                                   "/task_a/candidate_guide_path");
    private_nh_.param<std::string>("task_a_goal_input_topic", task_a_goal_input_topic_, "/task_a/final_goal");
    private_nh_.param<std::string>("gate_action_name", gate_action_name_, "/gate_module/plan");

    private_nh_.param("loop_rate_hz", loop_rate_hz_, 10.0);
    private_nh_.param("wait_for_offboard", wait_for_offboard_, true);
    private_nh_.param("goal_republish_period", goal_republish_period_, 1.0);
    private_nh_.param("tf_timeout", tf_timeout_, 0.15);

    private_nh_.param("takeoff_enabled", takeoff_enabled_, true);
    private_nh_.param("takeoff_height", takeoff_height_, 1.0);
    private_nh_.param("takeoff_reach_radius", takeoff_reach_radius_, 0.15);

    private_nh_.param("task_a_send_goal_on_start", task_a_send_goal_on_start_, true);
    private_nh_.param("task_a_use_candidate_path", task_a_use_candidate_path_, false);
    private_nh_.param("task_a_reach_radius", task_a_reach_radius_, 0.35);
    private_nh_.param("task_a_goal_x", task_a_goal_.x, 3.120975971221924);
    private_nh_.param("task_a_goal_y", task_a_goal_.y, -5.956602573394775);
    private_nh_.param("task_a_goal_z", task_a_goal_.z, 1.0);

    private_nh_.param("task_b_enabled", task_b_enabled_, false);
    private_nh_.param("task_b_reach_radius", task_b_reach_radius_, 0.45);
    loadWaypointList("task_b_waypoints", task_b_waypoints_);

    private_nh_.param("task_c_enabled", task_c_enabled_, true);
    private_nh_.param("task_c_reach_radius", task_c_reach_radius_, 0.50);
    loadWaypointList("task_c_waypoints", task_c_waypoints_);

    private_nh_.param("task_d_enabled", task_d_enabled_, true);
    private_nh_.param("task_d_prepare_reach_radius", task_d_prepare_reach_radius_, 0.40);
    private_nh_.param("task_d_observation_reach_radius", task_d_observation_reach_radius_, 0.25);
    private_nh_.param("task_d_traverse_reach_radius", task_d_traverse_reach_radius_, 0.30);
    private_nh_.param("gate_action_wait_timeout", gate_action_wait_timeout_, 2.0);
    loadWaypointList("task_d_prepare_waypoints", task_d_prepare_waypoints_);

    if (task_d_prepare_waypoints_.empty())
    {
      Point3 transfer;
      private_nh_.param("transfer_x", transfer.x, 10.361700057983398);
      private_nh_.param("transfer_y", transfer.y, -5.367546081542969);
      private_nh_.param("transfer_z", transfer.z, 1.0);
      task_d_prepare_waypoints_.push_back(transfer);
    }
  }

  bool loadWaypointList(const std::string& param_name, std::vector<Point3>& waypoints)
  {
    waypoints.clear();
    XmlRpc::XmlRpcValue raw;
    if (!private_nh_.getParam(param_name, raw))
    {
      return false;
    }
    if (raw.getType() != XmlRpc::XmlRpcValue::TypeArray)
    {
      ROS_WARN("competition_mission_manager: parameter %s must be a list of [x, y, z] waypoints",
               param_name.c_str());
      return false;
    }

    for (int i = 0; i < raw.size(); ++i)
    {
      if (raw[i].getType() != XmlRpc::XmlRpcValue::TypeArray || raw[i].size() < 3)
      {
        ROS_WARN("competition_mission_manager: ignoring invalid waypoint %s[%d]", param_name.c_str(), i);
        continue;
      }
      Point3 point;
      if (!xmlRpcToDouble(raw[i][0], &point.x) || !xmlRpcToDouble(raw[i][1], &point.y) ||
          !xmlRpcToDouble(raw[i][2], &point.z))
      {
        ROS_WARN("competition_mission_manager: ignoring non-numeric waypoint %s[%d]", param_name.c_str(), i);
        continue;
      }
      waypoints.push_back(point);
    }
    return !waypoints.empty();
  }

  void transitionTo(Phase next, const std::string& reason)
  {
    if (phase_ == next)
    {
      return;
    }
    ROS_INFO("competition_mission_manager: %s -> %s (%s)",
             phaseName(phase_),
             phaseName(next),
             reason.c_str());
    phase_ = next;
    phase_initialized_ = false;
  }

  void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
  {
    current_position_.x = msg->pose.pose.position.x;
    current_position_.y = msg->pose.pose.position.y;
    current_position_.z = msg->pose.pose.position.z;
    odom_received_ = true;
  }

  void stateCallback(const mavros_msgs::State::ConstPtr& msg)
  {
    mavros_mode_ = msg->mode;
    mavros_state_received_ = true;
  }

  void taskAGuideCallback(const nav_msgs::Path::ConstPtr& msg)
  {
    if (msg->poses.size() < 2)
    {
      return;
    }
    task_a_guide_path_ = *msg;
    task_a_guide_received_ = true;
    ROS_INFO_ONCE("competition_mission_manager: received Task A guide path");
  }

  void taskACandidateCallback(const nav_msgs::Path::ConstPtr& msg)
  {
    if (msg->poses.size() < 2)
    {
      return;
    }
    task_a_candidate_path_ = *msg;
    task_a_candidate_received_ = true;
  }

  bool offboardReady() const
  {
    if (!wait_for_offboard_)
    {
      return true;
    }
    return mavros_state_received_ && mavros_mode_ == "OFFBOARD";
  }

  geometry_msgs::PoseStamped makePose(const Point3& point, const ros::Time& stamp) const
  {
    geometry_msgs::PoseStamped pose;
    pose.header.stamp = stamp;
    pose.header.frame_id = frame_id_;
    pose.pose.position.x = point.x;
    pose.pose.position.y = point.y;
    pose.pose.position.z = point.z;
    pose.pose.orientation.w = 1.0;
    return pose;
  }

  nav_msgs::Path makePathFromWaypoints(std::vector<Point3> waypoints,
                                       bool prepend_current_position,
                                       const ros::Time& stamp) const
  {
    if (prepend_current_position)
    {
      if (waypoints.empty() || distance(current_position_, waypoints.front()) > 0.05)
      {
        waypoints.insert(waypoints.begin(), current_position_);
      }
    }

    nav_msgs::Path path;
    path.header.stamp = stamp;
    path.header.frame_id = frame_id_;
    for (const Point3& point : waypoints)
    {
      path.poses.push_back(makePose(point, stamp));
    }
    return path;
  }

  bool transformPoseToMissionFrame(const geometry_msgs::PoseStamped& input,
                                   geometry_msgs::PoseStamped* output) const
  {
    geometry_msgs::PoseStamped pose = input;
    if (pose.header.frame_id.empty())
    {
      pose.header.frame_id = frame_id_;
    }

    if (pose.header.frame_id == frame_id_)
    {
      *output = pose;
      output->header.frame_id = frame_id_;
      return true;
    }

    try
    {
      tf_buffer_.transform(pose, *output, frame_id_, ros::Duration(tf_timeout_));
      return true;
    }
    catch (const tf2::TransformException& ex)
    {
      ROS_WARN_THROTTLE(1.0,
                        "competition_mission_manager: cannot transform pose from %s to %s: %s",
                        pose.header.frame_id.c_str(),
                        frame_id_.c_str(),
                        ex.what());
      return false;
    }
  }

  bool transformPathToMissionFrame(const nav_msgs::Path& input, nav_msgs::Path* output) const
  {
    if (input.poses.empty())
    {
      return false;
    }

    output->poses.clear();
    output->header = input.header;
    if (output->header.frame_id.empty())
    {
      output->header.frame_id = frame_id_;
    }
    output->header.frame_id = frame_id_;

    for (const geometry_msgs::PoseStamped& stamped : input.poses)
    {
      geometry_msgs::PoseStamped pose = stamped;
      if (pose.header.frame_id.empty())
      {
        pose.header = input.header;
      }
      if (pose.header.frame_id.empty())
      {
        pose.header.frame_id = frame_id_;
      }

      geometry_msgs::PoseStamped transformed;
      if (!transformPoseToMissionFrame(pose, &transformed))
      {
        return false;
      }
      output->poses.push_back(transformed);
    }
    return !output->poses.empty();
  }

  void setActivePath(const nav_msgs::Path& input_path, const std::string& label)
  {
    nav_msgs::Path path;
    if (!transformPathToMissionFrame(input_path, &path))
    {
      ROS_ERROR("competition_mission_manager: rejected %s path because it could not be transformed to %s",
                label.c_str(),
                frame_id_.c_str());
      transitionTo(Phase::MISSION_ABORTED, "path transform failed");
      return;
    }

    const ros::Time stamp = ros::Time::now();
    path.header.stamp = stamp;
    for (geometry_msgs::PoseStamped& pose : path.poses)
    {
      pose.header.stamp = stamp;
      pose.header.frame_id = frame_id_;
    }

    active_path_ = path;
    active_goal_ = active_path_.poses.back();
    active_goal_.header.stamp = stamp;
    active_goal_valid_ = true;
    last_goal_publish_time_ = ros::Time(0);

    guide_path_pub_.publish(active_path_);
    publishActiveGoal(true);

    ROS_INFO("competition_mission_manager: active %s path poses=%zu goal=(%.2f, %.2f, %.2f)",
             label.c_str(),
             active_path_.poses.size(),
             active_goal_.pose.position.x,
             active_goal_.pose.position.y,
             active_goal_.pose.position.z);
  }

  void publishActiveGoal(bool force = false)
  {
    if (!active_goal_valid_)
    {
      return;
    }

    const ros::Time now = ros::Time::now();
    if (!force && goal_republish_period_ > 0.0 && !last_goal_publish_time_.isZero() &&
        (now - last_goal_publish_time_).toSec() < goal_republish_period_)
    {
      return;
    }

    active_goal_.header.stamp = now;
    goal_pub_.publish(active_goal_);
    last_goal_publish_time_ = now;
  }

  bool activeGoalReached(double radius) const
  {
    if (!active_goal_valid_ || !odom_received_)
    {
      return false;
    }
    return distance(current_position_, pointFromPose(active_goal_)) <= radius;
  }

  void publishTaskAGoal()
  {
    if (!task_a_send_goal_on_start_ || task_a_goal_sent_)
    {
      return;
    }
    const geometry_msgs::PoseStamped goal = makePose(task_a_goal_, ros::Time::now());
    task_a_goal_pub_.publish(goal);
    task_a_goal_sent_ = true;
    ROS_INFO("competition_mission_manager: sent Task A final goal to %s: (%.2f, %.2f, %.2f)",
             task_a_goal_input_topic_.c_str(),
             task_a_goal_.x,
             task_a_goal_.y,
             task_a_goal_.z);
  }

  bool chooseTaskAPath(nav_msgs::Path* output) const
  {
    if (task_a_guide_received_)
    {
      *output = task_a_guide_path_;
      return true;
    }
    if (task_a_use_candidate_path_ && task_a_candidate_received_)
    {
      *output = task_a_candidate_path_;
      return true;
    }
    return false;
  }

  void sendGateGoal()
  {
    if (!task_d_enabled_ || !gate_client_)
    {
      transitionTo(Phase::MISSION_DONE, "Task D disabled");
      return;
    }

    if (!gate_client_->waitForServer(ros::Duration(gate_action_wait_timeout_)))
    {
      ROS_WARN_THROTTLE(2.0,
                        "competition_mission_manager: waiting for gate action server %s",
                        gate_action_name_.c_str());
      return;
    }

    across_gate_module::GatePlanGoal goal;
    goal.request_id = ++gate_request_id_;
    gate_goal_sent_ = true;
    gate_result_ready_ = false;
    gate_failed_ = false;
    gate_observation_received_ = false;

    gate_client_->sendGoal(goal,
                           boost::bind(&CompetitionMissionManager::gateDoneCallback, this, _1, _2),
                           GateClient::SimpleActiveCallback(),
                           boost::bind(&CompetitionMissionManager::gateFeedbackCallback, this, _1));

    ROS_INFO("competition_mission_manager: sent gate action request_id=%u", gate_request_id_);
    transitionTo(Phase::TASK_D_FLY_OBSERVATION, "gate action started");
  }

  void gateFeedbackCallback(const across_gate_module::GatePlanFeedbackConstPtr& feedback)
  {
    if (!feedback || feedback->observation_pose.header.frame_id.empty())
    {
      return;
    }

    geometry_msgs::PoseStamped observation;
    if (!transformPoseToMissionFrame(feedback->observation_pose, &observation))
    {
      return;
    }

    if (!gate_observation_received_ ||
        distance(pointFromPose(observation), pointFromPose(gate_observation_pose_)) > 0.05)
    {
      gate_observation_pose_ = observation;
      gate_observation_received_ = true;
      gate_observation_path_initialized_ = false;
      ROS_INFO("competition_mission_manager: gate observation pose %s=(%.2f, %.2f, %.2f), state=%s",
               frame_id_.c_str(),
               observation.pose.position.x,
               observation.pose.position.y,
               observation.pose.position.z,
               feedback->state_name.c_str());
    }
  }

  void gateDoneCallback(const actionlib::SimpleClientGoalState& state,
                        const across_gate_module::GatePlanResultConstPtr& result)
  {
    gate_result_ready_ = true;

    if (!result || !result->success)
    {
      gate_failed_ = true;
      ROS_ERROR("competition_mission_manager: gate action failed state=%s code=%d message=%s",
                state.toString().c_str(),
                result ? static_cast<int>(result->error_code) : -1,
                result ? result->message.c_str() : "no result");
      return;
    }

    if (!transformPathToMissionFrame(result->traversal_path, &gate_traversal_path_))
    {
      gate_failed_ = true;
      ROS_ERROR("competition_mission_manager: gate traversal path could not be transformed to %s", frame_id_.c_str());
      return;
    }

    ROS_INFO("competition_mission_manager: gate traversal path ready poses=%zu message=%s",
             gate_traversal_path_.poses.size(),
             result->message.c_str());
  }

  void timerCallback(const ros::TimerEvent&)
  {
    if (phase_ != Phase::WAIT_READY && wait_for_offboard_ && !offboardReady())
    {
      ROS_WARN_THROTTLE(1.0,
                        "competition_mission_manager: waiting for OFFBOARD, current mode=%s",
                        mavros_mode_.c_str());
      return;
    }

    switch (phase_)
    {
      case Phase::WAIT_READY:
        runWaitReady();
        break;
      case Phase::TAKEOFF:
        runTakeoff();
        break;
      case Phase::TASK_A_WAIT_PATH:
        runTaskAWaitPath();
        break;
      case Phase::TASK_A_FLY:
        runTaskAFly();
        break;
      case Phase::TASK_B_SETUP:
        runTaskBSetup();
        break;
      case Phase::TASK_B_FLY:
        runTaskBFly();
        break;
      case Phase::TASK_C_SETUP:
        runTaskCSetup();
        break;
      case Phase::TASK_C_FLY:
        runTaskCFly();
        break;
      case Phase::TASK_D_PREPARE:
        runTaskDPrepare();
        break;
      case Phase::TASK_D_START_ACTION:
        sendGateGoal();
        break;
      case Phase::TASK_D_FLY_OBSERVATION:
        runTaskDFlyObservation();
        break;
      case Phase::TASK_D_TRAVERSE:
        runTaskDTraverse();
        break;
      case Phase::MISSION_DONE:
        ROS_INFO_THROTTLE(2.0, "competition_mission_manager: mission done, holding final goal");
        publishActiveGoal(false);
        break;
      case Phase::MISSION_ABORTED:
        ROS_ERROR_THROTTLE(2.0, "competition_mission_manager: mission aborted");
        break;
    }
  }

  void runWaitReady()
  {
    if (!odom_received_)
    {
      ROS_INFO_THROTTLE(1.0, "competition_mission_manager: waiting for odometry on %s", odom_topic_.c_str());
      return;
    }
    if (!offboardReady())
    {
      ROS_INFO_THROTTLE(1.0,
                        "competition_mission_manager: waiting for OFFBOARD on %s, current mode=%s",
                        mavros_state_topic_.c_str(),
                        mavros_mode_.c_str());
      return;
    }

    transitionTo(takeoff_enabled_ ? Phase::TAKEOFF : Phase::TASK_A_WAIT_PATH, "inputs ready");
  }

  void runTakeoff()
  {
    if (!phase_initialized_)
    {
      std::vector<Point3> waypoints;
      waypoints.push_back(current_position_);
      Point3 target = current_position_;
      target.z = takeoff_height_;
      waypoints.push_back(target);
      setActivePath(makePathFromWaypoints(waypoints, false, ros::Time::now()), "takeoff");
      phase_initialized_ = true;
    }

    publishActiveGoal(false);
    if (activeGoalReached(takeoff_reach_radius_))
    {
      transitionTo(Phase::TASK_A_WAIT_PATH, "takeoff reached");
    }
  }

  void runTaskAWaitPath()
  {
    publishTaskAGoal();

    nav_msgs::Path task_a_path;
    if (!chooseTaskAPath(&task_a_path))
    {
      ROS_INFO_THROTTLE(1.0,
                        "competition_mission_manager: waiting for Task A guide path on %s",
                        task_a_guide_path_topic_.c_str());
      return;
    }

    setActivePath(task_a_path, "Task A");
    transitionTo(Phase::TASK_A_FLY, "Task A path ready");
  }

  void runTaskAFly()
  {
    publishActiveGoal(false);
    if (activeGoalReached(task_a_reach_radius_))
    {
      transitionTo(Phase::TASK_B_SETUP, "Task A reached");
    }
  }

  void runTaskBSetup()
  {
    if (!task_b_enabled_ || task_b_waypoints_.empty())
    {
      transitionTo(Phase::TASK_C_SETUP, "Task B placeholder skipped");
      return;
    }

    setActivePath(makePathFromWaypoints(task_b_waypoints_, true, ros::Time::now()), "Task B placeholder");
    phase_initialized_ = true;
    transitionTo(Phase::TASK_B_FLY, "Task B placeholder path ready");
  }

  void runTaskBFly()
  {
    publishActiveGoal(false);
    if (activeGoalReached(task_b_reach_radius_))
    {
      transitionTo(Phase::TASK_C_SETUP, "Task B placeholder reached");
    }
  }

  void runTaskCSetup()
  {
    if (!task_c_enabled_ || task_c_waypoints_.empty())
    {
      ROS_WARN("competition_mission_manager: Task C has no waypoints; skipping to Task D");
      transitionTo(Phase::TASK_D_PREPARE, "Task C skipped");
      return;
    }

    setActivePath(makePathFromWaypoints(task_c_waypoints_, true, ros::Time::now()), "Task C");
    phase_initialized_ = true;
    transitionTo(Phase::TASK_C_FLY, "Task C path ready");
  }

  void runTaskCFly()
  {
    publishActiveGoal(false);
    if (activeGoalReached(task_c_reach_radius_))
    {
      transitionTo(Phase::TASK_D_PREPARE, "Task C reached");
    }
  }

  void runTaskDPrepare()
  {
    if (!task_d_enabled_)
    {
      transitionTo(Phase::MISSION_DONE, "Task D disabled");
      return;
    }

    if (task_d_prepare_waypoints_.empty())
    {
      transitionTo(Phase::TASK_D_START_ACTION, "no Task D prepare point");
      return;
    }

    if (!phase_initialized_)
    {
      setActivePath(makePathFromWaypoints(task_d_prepare_waypoints_, true, ros::Time::now()), "Task D prepare");
      phase_initialized_ = true;
    }

    publishActiveGoal(false);
    if (activeGoalReached(task_d_prepare_reach_radius_))
    {
      transitionTo(Phase::TASK_D_START_ACTION, "Task D prepare reached");
    }
  }

  void runTaskDFlyObservation()
  {
    if (gate_failed_)
    {
      transitionTo(Phase::MISSION_ABORTED, "gate action failed");
      return;
    }

    if (gate_result_ready_)
    {
      setActivePath(gate_traversal_path_, "Task D traversal");
      transitionTo(Phase::TASK_D_TRAVERSE, "gate traversal path ready");
      return;
    }

    if (!gate_observation_received_)
    {
      ROS_INFO_THROTTLE(1.0,
                        "competition_mission_manager: waiting for gate observation pose from %s",
                        gate_action_name_.c_str());
      return;
    }

    if (!gate_observation_path_initialized_)
    {
      std::vector<Point3> waypoints;
      waypoints.push_back(pointFromPose(gate_observation_pose_));
      setActivePath(makePathFromWaypoints(waypoints, true, ros::Time::now()), "Task D observation");
      gate_observation_path_initialized_ = true;
    }

    publishActiveGoal(false);
    if (activeGoalReached(task_d_observation_reach_radius_))
    {
      ROS_INFO_THROTTLE(1.0,
                        "competition_mission_manager: gate observation reached; waiting for final traversal path");
    }
  }

  void runTaskDTraverse()
  {
    publishActiveGoal(false);
    if (activeGoalReached(task_d_traverse_reach_radius_))
    {
      transitionTo(Phase::MISSION_DONE, "Task D traversal reached");
    }
  }

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  ros::Publisher goal_pub_;
  ros::Publisher guide_path_pub_;
  ros::Publisher task_a_goal_pub_;
  ros::Subscriber odom_sub_;
  ros::Subscriber state_sub_;
  ros::Subscriber task_a_guide_sub_;
  ros::Subscriber task_a_candidate_sub_;
  ros::Timer timer_;

  std::unique_ptr<GateClient> gate_client_;

  std::string frame_id_;
  std::string odom_topic_;
  std::string mavros_state_topic_;
  std::string goal_topic_;
  std::string guide_path_topic_;
  std::string task_a_guide_path_topic_;
  std::string task_a_candidate_guide_path_topic_;
  std::string task_a_goal_input_topic_;
  std::string gate_action_name_;

  double loop_rate_hz_ = 10.0;
  bool wait_for_offboard_ = true;
  double goal_republish_period_ = 1.0;
  double tf_timeout_ = 0.15;

  bool takeoff_enabled_ = true;
  double takeoff_height_ = 1.0;
  double takeoff_reach_radius_ = 0.15;

  bool task_a_send_goal_on_start_ = true;
  bool task_a_use_candidate_path_ = false;
  double task_a_reach_radius_ = 0.35;
  Point3 task_a_goal_;

  bool task_b_enabled_ = false;
  double task_b_reach_radius_ = 0.45;
  std::vector<Point3> task_b_waypoints_;

  bool task_c_enabled_ = true;
  double task_c_reach_radius_ = 0.50;
  std::vector<Point3> task_c_waypoints_;

  bool task_d_enabled_ = true;
  double task_d_prepare_reach_radius_ = 0.40;
  double task_d_observation_reach_radius_ = 0.25;
  double task_d_traverse_reach_radius_ = 0.30;
  double gate_action_wait_timeout_ = 2.0;
  std::vector<Point3> task_d_prepare_waypoints_;

  Phase phase_ = Phase::WAIT_READY;
  bool phase_initialized_ = false;

  bool odom_received_ = false;
  bool mavros_state_received_ = false;
  std::string mavros_mode_;
  Point3 current_position_;

  bool active_goal_valid_ = false;
  geometry_msgs::PoseStamped active_goal_;
  nav_msgs::Path active_path_;
  ros::Time last_goal_publish_time_;

  bool task_a_goal_sent_ = false;
  bool task_a_guide_received_ = false;
  bool task_a_candidate_received_ = false;
  nav_msgs::Path task_a_guide_path_;
  nav_msgs::Path task_a_candidate_path_;

  uint32_t gate_request_id_ = 0;
  bool gate_goal_sent_ = false;
  bool gate_observation_received_ = false;
  bool gate_observation_path_initialized_ = false;
  bool gate_result_ready_ = false;
  bool gate_failed_ = false;
  geometry_msgs::PoseStamped gate_observation_pose_;
  nav_msgs::Path gate_traversal_path_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "competition_mission_manager");
  CompetitionMissionManager manager;
  ros::spin();
  return 0;
}
