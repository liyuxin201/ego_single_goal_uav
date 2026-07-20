/*
	FILE: dynamicNavigation.h
	------------------------
	dynamic navigation header file in simulation
*/

#ifndef AUTOFLIGHT_DYNAMIC_NAVIGATION_H
#define AUTOFLIGHT_DYNAMIC_NAVIGATION_H

#include <autonomous_flight/simulation/flightBase.h>
#include <map_manager/dynamicMap.h>
#include <onboard_detector/fakeDetector.h>
#include <global_planner/rrtOccMap.h>
#include <trajectory_planner/polyTrajOccMap.h>
#include <trajectory_planner/piecewiseLinearTraj.h>
#include <trajectory_planner/bsplineTraj.h>
#include <uav_simulator/DynamicObstacleArray.h>
#include <uav_simulator/DynamicObstacleState.h>
#include <visualization_msgs/MarkerArray.h>

namespace AutoFlight{
	class dynamicNavigation : public flightBase{
	private:
		std::shared_ptr<mapManager::dynamicMap> map_;
		std::shared_ptr<onboardDetector::fakeDetector> detector_;
		std::shared_ptr<globalPlanner::rrtOccMap<3>> rrtPlanner_;
		std::shared_ptr<trajPlanner::polyTrajOccMap> polyTraj_;
		std::shared_ptr<trajPlanner::pwlTraj> pwlTraj_;
		std::shared_ptr<trajPlanner::bsplineTraj> bsplineTraj_;

		ros::Timer plannerTimer_;
		ros::Timer replanCheckTimer_;
		ros::Timer trajExeTimer_;
		ros::Timer visTimer_;
		ros::Timer freeMapTimer_;
		ros::Subscriber dynamicObstacleTruthSub_;
		ros::Subscriber externalGuidePathSub_;

		ros::Publisher rrtPathPub_;
		ros::Publisher polyTrajPub_;
		ros::Publisher pwlTrajPub_;
		ros::Publisher bsplineTrajPub_;
		ros::Publisher inputTrajPub_;
		ros::Publisher plannerPoseTargetPub_;
		ros::Publisher px4RawSetpointPub_;
		ros::Publisher dynamicObstacleTruthMarkerPub_;

		// parameters
		bool useFakeDetector_;
		bool useGlobalPlanner_;
		bool noYawTurning_;
		bool useYawControl_;
		double desiredVel_;
		double desiredAcc_;
		double desiredAngularVel_;
		double replanTimeForDynamicObstacle_;
		double dynamicCollisionCheckStep_ = 0.05;
		double dynamicCollisionPredictionTime_ = 1.5;
		double dynamicCollisionSafetyMargin_ = 0.4;
		double dynamicCollisionReplanDistance_ = 0.0;
		std::string trajSavePath_;
		bool useDynamicObstacleTruthTopic_ = false;
		std::string dynamicObstacleTruthTopic_;
		std::string dynamicObstacleTruthMarkerTopic_;
		bool useExternalGuidePath_ = false;
		std::string externalGuidePathTopic_;

		// navigation data
		bool replan_ = false;
		bool needGlobalPlan_ = false;
		bool globalPlanReady_ = false;
		nav_msgs::Path rrtPathMsg_;
		nav_msgs::Path polyTrajMsg_;
		nav_msgs::Path pwlTrajMsg_;
		nav_msgs::Path bsplineTrajMsg_;
		nav_msgs::Path inputTrajMsg_;
		nav_msgs::Path externalGuidePath_;
		bool externalGuidePathReceived_ = false;
		bool trajectoryReady_ = false;
		ros::Time trajStartTime_;
		double trajTime_; // current trajectory time
		double prevInputTrajTime_ = 0.0;
		trajPlanner::bspline trajectory_; // trajectory data for tracking
		double facingYaw_;
		bool firstTimeSave_ = false;
		bool lastDynamicObstacle_ = false;
		ros::Time lastDynamicObstacleTime_;
		std::string plannerPoseTargetTopic_;
		std::string px4RawSetpointTopic_;
		bool autoTakeoff_ = true;
		double plannerPoseTargetLookahead_ = 0.35;
			double controlTargetLookahead_ = 0.0;
			bool publishPx4RawSetpoint_ = false;
			bool requireOffboardBeforePlannerTarget_ = false;
			bool plannerPoseTargetYawInitialized_ = false;
		double lastPlannerPoseTargetYaw_ = 0.0;
		std::vector<uav_simulator::DynamicObstacleState> dynamicObstacleTruth_;
		



	public:
		dynamicNavigation(const ros::NodeHandle& nh);
		void initParam();
		void initModules();
		void registerPub();
		void registerCallback();

		void plannerCB(const ros::TimerEvent&);
		void replanCheckCB(const ros::TimerEvent&);
		void trajExeCB(const ros::TimerEvent&);
		void visCB(const ros::TimerEvent&);
		void freeMapCB(const ros::TimerEvent&); // using fake detector
		void dynamicObstacleTruthCB(const uav_simulator::DynamicObstacleArray::ConstPtr& msg);
		void externalGuidePathCB(const nav_msgs::Path::ConstPtr& msg);

		void run();	
		void getStartEndConditions(std::vector<Eigen::Vector3d>& startEndConditions);	
		static std::vector<Eigen::Vector3d> buildStartEndConditions(const Eigen::Vector3d& currVel,
																	const Eigen::Vector3d& currAcc,
																	bool planInZAxis);
		static bool shouldTurnTowardGoalBeforePlanning(bool noYawTurning,
														 bool useYawControl,
														 bool enableControlOutput);
			static double computePlannerPoseTargetTime(double trajTime,
														 double trajDuration,
														 bool enableControlOutput,
														 double plannerPoseTargetLookahead);
			static double computeControlTargetTime(double trajTime,
												   double trajDuration,
												   double controlTargetLookahead);
			static double computePlannerPoseTargetYaw(const Eigen::Vector3d& poseTargetVel,
													  double fallbackYaw,
													  bool useYawControl,
													  bool noYawTurning);
			static mavros_msgs::PositionTarget buildPx4RawSetpoint(const Eigen::Vector3d& position,
																  const Eigen::Vector3d& velocity,
																  const Eigen::Vector3d& acceleration,
																  double yaw,
																  const ros::Time& stamp);
			static bool isPointInPredictedDynamicObstacle(const Eigen::Vector3d& point,
														  const Eigen::Vector3d& obstaclePos,
														  const Eigen::Vector3d& obstacleVel,
														  const Eigen::Vector3d& obstacleSize,
														  double predictionTime,
														  double safetyMargin);
			static void convertDynamicObstacleTruth(const uav_simulator::DynamicObstacleState& obstacle,
													Eigen::Vector3d& pos,
													Eigen::Vector3d& vel,
													Eigen::Vector3d& size);
		bool hasCollision();
		bool hasDynamicCollision();
		double computeExecutionDistance();
		bool replanForDynamicObstacle();
		nav_msgs::Path getCurrentTraj(double dt);
		nav_msgs::Path getRestGlobalPath();
		void getDynamicObstacles(std::vector<Eigen::Vector3d>& obstaclesPos, std::vector<Eigen::Vector3d>& obstaclesVel, std::vector<Eigen::Vector3d>& obstaclesSize);
		void getDynamicObstacleTruth(std::vector<Eigen::Vector3d>& obstaclesPos, std::vector<Eigen::Vector3d>& obstaclesVel, std::vector<Eigen::Vector3d>& obstaclesSize);
		void publishDynamicObstacleTruthMarkers();
		nav_msgs::Path buildGuidedPath(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal);
	};
}

#endif
