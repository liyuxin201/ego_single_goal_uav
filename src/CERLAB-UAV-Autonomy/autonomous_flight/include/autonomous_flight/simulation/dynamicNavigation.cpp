/*
	FILE: dynamicNavigation.cpp
	------------------------
	dynamic navigation implementation file in simulation
*/
#include <autonomous_flight/simulation/dynamicNavigation.h>

namespace AutoFlight{
	dynamicNavigation::dynamicNavigation(const ros::NodeHandle& nh) : flightBase(nh){
		this->initParam();
		this->initModules();
		this->registerPub();
		if (this->useFakeDetector_ and not this->useDynamicObstacleTruthTopic_){
			// free map callback
			this->freeMapTimer_ = this->nh_.createTimer(ros::Duration(0.01), &dynamicNavigation::freeMapCB, this);
		}
		
	}

	void dynamicNavigation::initParam(){
    	// parameters    
    	// use simulation detector	
		if (not this->nh_.getParam("autonomous_flight/use_fake_detector", this->useFakeDetector_)){
			this->useFakeDetector_ = false;
			cout << "[AutoFlight]: No use fake detector param found. Use default: false." << endl;
		}
		else{
			cout << "[AutoFlight]: Use fake detector is set to: " << this->useFakeDetector_ << "." << endl;
		}


    	// use global planner or not	
		if (not this->nh_.getParam("autonomous_flight/use_global_planner", this->useGlobalPlanner_)){
			this->useGlobalPlanner_ = false;
			cout << "[AutoFlight]: No use global planner param found. Use default: false." << endl;
		}
		else{
			cout << "[AutoFlight]: Global planner use is set to: " << this->useGlobalPlanner_ << "." << endl;
		}

		// No turning of yaw
		if (not this->nh_.getParam("autonomous_flight/no_yaw_turning", this->noYawTurning_)){
			this->noYawTurning_ = false;
			cout << "[AutoFlight]: No yaw turning param found. Use default: false." << endl;
		}
		else{
			cout << "[AutoFlight]: Yaw turning use is set to: " << this->noYawTurning_ << "." << endl;
		}	

		// full state control (yaw)
		if (not this->nh_.getParam("autonomous_flight/use_yaw_control", this->useYawControl_)){
			this->useYawControl_ = false;
			cout << "[AutoFlight]: No yaw control param found. Use default: false." << endl;
		}
		else{
			cout << "[AutoFlight]: Yaw control use is set to: " << this->useYawControl_ << "." << endl;
		}		

    	// desired linear velocity    	
		if (not this->nh_.getParam("autonomous_flight/desired_velocity", this->desiredVel_)){
			this->desiredVel_ = 1.0;
			cout << "[AutoFlight]: No desired velocity param found. Use default: 1.0 m/s." << endl;
		}
		else{
			cout << "[AutoFlight]: Desired velocity is set to: " << this->desiredVel_ << "m/s." << endl;
		}

		// desired acceleration
		if (not this->nh_.getParam("autonomous_flight/desired_acceleration", this->desiredAcc_)){
			this->desiredAcc_ = 1.0;
			cout << "[AutoFlight]: No desired acceleration param found. Use default: 1.0 m/s^2." << endl;
		}
		else{
			cout << "[AutoFlight]: Desired acceleration is set to: " << this->desiredAcc_ << "m/s^2." << endl;
		}


    	// desired angular velocity    	
		if (not this->nh_.getParam("autonomous_flight/desired_angular_velocity", this->desiredAngularVel_)){
			this->desiredAngularVel_ = 1.0;
			cout << "[AutoFlight]: No desired angular velocity param found. Use default: 0.5 rad/s." << endl;
		}
		else{
			cout << "[AutoFlight]: Desired angular velocity is set to: " << this->desiredAngularVel_ << "rad/s." << endl;
		}	

    	// replan time for dynamic obstacle
		if (not this->nh_.getParam("autonomous_flight/replan_time_for_dynamic_obstacles", this->replanTimeForDynamicObstacle_)){
			this->replanTimeForDynamicObstacle_ = 0.2;
			cout << "[AutoFlight]: No dynamic obstacle replan time param found. Use default: 0.2s." << endl;
		}
		else{
			cout << "[AutoFlight]: Dynamic obstacle replan time is set to: " << this->replanTimeForDynamicObstacle_ << "s." << endl;
		}	

		if (not this->nh_.getParam("autonomous_flight/dynamic_collision_check_step", this->dynamicCollisionCheckStep_)){
			this->dynamicCollisionCheckStep_ = 0.05;
		}
		cout << "[AutoFlight]: Dynamic collision check step is set to: " << this->dynamicCollisionCheckStep_ << "s." << endl;

		if (not this->nh_.getParam("autonomous_flight/dynamic_collision_prediction_time", this->dynamicCollisionPredictionTime_)){
			this->dynamicCollisionPredictionTime_ = 1.5;
		}
		cout << "[AutoFlight]: Dynamic collision prediction time is set to: " << this->dynamicCollisionPredictionTime_ << "s." << endl;

		if (not this->nh_.getParam("autonomous_flight/dynamic_collision_safety_margin", this->dynamicCollisionSafetyMargin_)){
			this->dynamicCollisionSafetyMargin_ = 0.4;
		}
		cout << "[AutoFlight]: Dynamic collision safety margin is set to: " << this->dynamicCollisionSafetyMargin_ << "m." << endl;

		if (not this->nh_.getParam("autonomous_flight/dynamic_collision_replan_distance", this->dynamicCollisionReplanDistance_)){
			this->dynamicCollisionReplanDistance_ = 0.0;
		}
		cout << "[AutoFlight]: Dynamic collision replan distance is set to: " << this->dynamicCollisionReplanDistance_ << "m." << endl;

    	// trajectory data save path   	
		if (not this->nh_.getParam("autonomous_flight/trajectory_info_save_path", this->trajSavePath_)){
			this->trajSavePath_ = "No";
			cout << "[AutoFlight]: No trajectory info save path param found. Use current directory." << endl;
		}
		else{
			cout << "[AutoFlight]: Trajectory info save path is set to: " << this->trajSavePath_ << "." << endl;
		}
		if (not this->nh_.getParam("autonomous_flight/planner_pose_target_topic", this->plannerPoseTargetTopic_)){
			this->plannerPoseTargetTopic_ = "/autonomous_flight/planner_pose_target";
		}
		cout << "[AutoFlight]: Planner pose target topic is set to: " << this->plannerPoseTargetTopic_ << "." << endl;

		if (not this->nh_.getParam("autonomous_flight/auto_takeoff", this->autoTakeoff_)){
			this->autoTakeoff_ = true;
		}
		cout << "[AutoFlight]: Auto takeoff is set to: " << this->autoTakeoff_ << "." << endl;

		if (not this->nh_.getParam("autonomous_flight/planner_pose_target_lookahead", this->plannerPoseTargetLookahead_)){
			this->plannerPoseTargetLookahead_ = 0.35;
		}
		cout << "[AutoFlight]: Planner pose target lookahead is set to: " << this->plannerPoseTargetLookahead_ << "s." << endl;

		if (not this->nh_.getParam("autonomous_flight/control_target_lookahead", this->controlTargetLookahead_)){
			this->controlTargetLookahead_ = 0.0;
		}
		cout << "[AutoFlight]: Control target lookahead is set to: " << this->controlTargetLookahead_ << "s." << endl;

			if (not this->nh_.getParam("autonomous_flight/publish_px4_raw_setpoint", this->publishPx4RawSetpoint_)){
				this->publishPx4RawSetpoint_ = false;
			}
			cout << "[AutoFlight]: PX4 raw setpoint output is set to: " << this->publishPx4RawSetpoint_ << "." << endl;

			if (not this->nh_.getParam("autonomous_flight/require_offboard_before_planner_target", this->requireOffboardBeforePlannerTarget_)){
				this->requireOffboardBeforePlannerTarget_ = false;
			}
			cout << "[AutoFlight]: Require OFFBOARD before planner target output is set to: "
				 << this->requireOffboardBeforePlannerTarget_ << "." << endl;

			if (not this->nh_.getParam("autonomous_flight/px4_raw_setpoint_topic", this->px4RawSetpointTopic_)){
				this->px4RawSetpointTopic_ = "/mavros/setpoint_raw/local";
		}
		cout << "[AutoFlight]: PX4 raw setpoint topic is set to: " << this->px4RawSetpointTopic_ << "." << endl;

		if (not this->nh_.getParam("autonomous_flight/use_dynamic_obstacle_truth_topic", this->useDynamicObstacleTruthTopic_)){
			this->useDynamicObstacleTruthTopic_ = false;
		}
		cout << "[AutoFlight]: Dynamic obstacle truth topic use is set to: " << this->useDynamicObstacleTruthTopic_ << "." << endl;

		if (not this->nh_.getParam("autonomous_flight/dynamic_obstacle_truth_topic", this->dynamicObstacleTruthTopic_)){
			this->dynamicObstacleTruthTopic_ = "/large_pillar/dynamic_obstacles";
		}
		cout << "[AutoFlight]: Dynamic obstacle truth topic is set to: " << this->dynamicObstacleTruthTopic_ << "." << endl;

		if (not this->nh_.getParam("autonomous_flight/dynamic_obstacle_truth_marker_topic", this->dynamicObstacleTruthMarkerTopic_)){
			this->dynamicObstacleTruthMarkerTopic_ = "/onboard_detector/GT_obstacle_bbox";
		}
		cout << "[AutoFlight]: Dynamic obstacle truth marker topic is set to: " << this->dynamicObstacleTruthMarkerTopic_ << "." << endl;

		if (not this->nh_.getParam("autonomous_flight/use_external_guide_path", this->useExternalGuidePath_)){
			this->useExternalGuidePath_ = false;
		}
		cout << "[AutoFlight]: External guide path use is set to: " << this->useExternalGuidePath_ << "." << endl;

		if (not this->nh_.getParam("autonomous_flight/external_guide_path_topic", this->externalGuidePathTopic_)){
			this->externalGuidePathTopic_ = "/task_a/fixed_path";
		}
		cout << "[AutoFlight]: External guide path topic is set to: " << this->externalGuidePathTopic_ << "." << endl;
	}

	void dynamicNavigation::initModules(){
		// initialize map
		if (this->useFakeDetector_){
			if (not this->useDynamicObstacleTruthTopic_){
				// initialize fake detector
				this->detector_.reset(new onboardDetector::fakeDetector (this->nh_));
			}
			this->map_.reset(new mapManager::dynamicMap (this->nh_, false));
		}
		else{
			this->map_.reset(new mapManager::dynamicMap (this->nh_));
		}
		// initialize rrt planner
		this->rrtPlanner_.reset(new globalPlanner::rrtOccMap<3> (this->nh_));
		this->rrtPlanner_->setMap(this->map_);

		// initialize polynomial trajectory planner
		this->polyTraj_.reset(new trajPlanner::polyTrajOccMap (this->nh_));
		this->polyTraj_->setMap(this->map_);
		this->polyTraj_->updateDesiredVel(this->desiredVel_);
		this->polyTraj_->updateDesiredAcc(this->desiredAcc_);

		// initialize piecewise linear trajectory planner
		this->pwlTraj_.reset(new trajPlanner::pwlTraj (this->nh_));

		// initialize bspline trajectory planner
		this->bsplineTraj_.reset(new trajPlanner::bsplineTraj (this->nh_));
		this->bsplineTraj_->setMap(this->map_);
		this->bsplineTraj_->updateMaxVel(this->desiredVel_);
		this->bsplineTraj_->updateMaxAcc(this->desiredAcc_);
	}

	void dynamicNavigation::registerPub(){
		this->rrtPathPub_ = this->nh_.advertise<nav_msgs::Path>("dynamicNavigation/rrt_path", 10);
		this->polyTrajPub_ = this->nh_.advertise<nav_msgs::Path>("dynamicNavigation/poly_traj", 10);
		this->pwlTrajPub_ = this->nh_.advertise<nav_msgs::Path>("dynamicNavigation/pwl_trajectory", 10);
		this->bsplineTrajPub_ = this->nh_.advertise<nav_msgs::Path>("dynamicNavigation/bspline_trajectory", 10);
		this->inputTrajPub_ = this->nh_.advertise<nav_msgs::Path>("dynamicNavigation/input_trajectory", 10);
		this->plannerPoseTargetPub_ = this->nh_.advertise<geometry_msgs::PoseStamped>(this->plannerPoseTargetTopic_, 10);
		if (this->publishPx4RawSetpoint_){
			this->px4RawSetpointPub_ = this->nh_.advertise<mavros_msgs::PositionTarget>(this->px4RawSetpointTopic_, 10);
		}
		if (this->useDynamicObstacleTruthTopic_){
			this->dynamicObstacleTruthMarkerPub_ = this->nh_.advertise<visualization_msgs::MarkerArray>(this->dynamicObstacleTruthMarkerTopic_, 10);
		}
	}

	void dynamicNavigation::registerCallback(){
		// planner callback
		this->plannerTimer_ = this->nh_.createTimer(ros::Duration(0.02), &dynamicNavigation::plannerCB, this);

		// collision check callback
		this->replanCheckTimer_ = this->nh_.createTimer(ros::Duration(0.01), &dynamicNavigation::replanCheckCB, this);

		// trajectory execution callback
		this->trajExeTimer_ = this->nh_.createTimer(ros::Duration(0.01), &dynamicNavigation::trajExeCB, this);

		// visualization callback
		this->visTimer_ = this->nh_.createTimer(ros::Duration(0.033), &dynamicNavigation::visCB, this);

		if (this->useDynamicObstacleTruthTopic_){
			this->dynamicObstacleTruthSub_ = this->nh_.subscribe(
				this->dynamicObstacleTruthTopic_, 1000, &dynamicNavigation::dynamicObstacleTruthCB, this);
		}
		if (this->useExternalGuidePath_){
			this->externalGuidePathSub_ = this->nh_.subscribe(
				this->externalGuidePathTopic_, 1, &dynamicNavigation::externalGuidePathCB, this);
		}
	}

	void dynamicNavigation::plannerCB(const ros::TimerEvent&){
		if (not this->firstGoal_) return;

		if (this->replan_){
			std::vector<Eigen::Vector3d> obstaclesPos, obstaclesVel, obstaclesSize;
			if (this->useDynamicObstacleTruthTopic_){
				this->getDynamicObstacleTruth(obstaclesPos, obstaclesVel, obstaclesSize);
			}
			else if (this->useFakeDetector_){
				this->getDynamicObstacles(obstaclesPos, obstaclesVel, obstaclesSize);
			}
			else{ 
				this->map_->getDynamicObstacles(obstaclesPos, obstaclesVel, obstaclesSize);
			}
			// get start and end condition for trajectory generation (the end condition is the final zero condition)
			std::vector<Eigen::Vector3d> startEndConditions;
			this->getStartEndConditions(startEndConditions); 
			nav_msgs::Path inputTraj;
			// bspline trajectory generation
			double finalTime; // final time for bspline trajectory
			double initTs = this->bsplineTraj_->getInitTs();
			if (this->useGlobalPlanner_){
				if (this->needGlobalPlan_){
					this->rrtPlanner_->updateStart(this->odom_.pose.pose);
					this->rrtPlanner_->updateGoal(this->goal_.pose);
					nav_msgs::Path rrtPathMsgTemp;
					this->rrtPlanner_->makePlan(rrtPathMsgTemp);
					if (rrtPathMsgTemp.poses.size() >= 2){
						this->rrtPathMsg_ = rrtPathMsgTemp;
						this->globalPlanReady_ = true;
					}
					this->needGlobalPlan_ = false;
					return;
				}
				else{
					if (this->globalPlanReady_){
						// get rest of global plan
						nav_msgs::Path restPath = this->getRestGlobalPath();
						this->polyTraj_->updatePath(restPath, startEndConditions);
						this->polyTraj_->makePlan(this->polyTrajMsg_); // no corridor constraint		
						nav_msgs::Path adjustedInputPolyTraj;
						bool satisfyDistanceCheck = false;
						double dtTemp = initTs;
						double finalTimeTemp;
						ros::Time startTime = ros::Time::now();
						ros::Time currTime;
						while (ros::ok()){
							currTime = ros::Time::now();
							if ((currTime - startTime).toSec() >= 0.05){
								cout << "[AutoFlight]: Exceed path check time. Use the best." << endl;
								break;
							}
							nav_msgs::Path inputPolyTraj = this->polyTraj_->getTrajectory(dtTemp);
							satisfyDistanceCheck = this->bsplineTraj_->inputPathCheck(inputPolyTraj, adjustedInputPolyTraj, dtTemp, finalTimeTemp);
							if (satisfyDistanceCheck) break;
							dtTemp *= 0.8;
						}

						inputTraj = adjustedInputPolyTraj;
						finalTime = finalTimeTemp;
						startEndConditions[1] = this->polyTraj_->getVel(finalTime);
						startEndConditions[3] = this->polyTraj_->getAcc(finalTime);	

					}
					else{
						cout << "[AutoFlight]: Global planner fails. Check goal and map." << endl;
					}		
				}				
			}
			else{
				if (obstaclesPos.size() == 0){ // use prev planned trajectory if there is no dynamic obstacle
					if (not this->trajectoryReady_){ // use polynomial trajectory as input
						nav_msgs::Path waypoints, polyTrajTemp;
						geometry_msgs::PoseStamped start, goal;
						start.pose = this->odom_.pose.pose; goal = this->goal_;
						waypoints = this->buildGuidedPath(start, goal);
						
						this->polyTraj_->updatePath(waypoints, startEndConditions);
						this->polyTraj_->makePlan(false); // no corridor constraint
						
						nav_msgs::Path adjustedInputPolyTraj;
						bool satisfyDistanceCheck = false;
						double dtTemp = initTs;
						double finalTimeTemp;
						ros::Time startTime = ros::Time::now();
						ros::Time currTime;
						while (ros::ok()){
							currTime = ros::Time::now();
							if ((currTime - startTime).toSec() >= 0.05){
								cout << "[AutoFlight]: Exceed path check time. Use the best." << endl;
								break;
							}
							nav_msgs::Path inputPolyTraj = this->polyTraj_->getTrajectory(dtTemp);
							satisfyDistanceCheck = this->bsplineTraj_->inputPathCheck(inputPolyTraj, adjustedInputPolyTraj, dtTemp, finalTimeTemp);
							if (satisfyDistanceCheck) break;
							
							dtTemp *= 0.8;
						}

						inputTraj = adjustedInputPolyTraj;
						finalTime = finalTimeTemp;
						startEndConditions[1] = this->polyTraj_->getVel(finalTime);
						startEndConditions[3] = this->polyTraj_->getAcc(finalTime);
					}
					else{
						Eigen::Vector3d bsplineLastPos = this->trajectory_.at(this->trajectory_.getDuration());
						geometry_msgs::PoseStamped lastPs; lastPs.pose.position.x = bsplineLastPos(0); lastPs.pose.position.y = bsplineLastPos(1); lastPs.pose.position.z = bsplineLastPos(2);
						Eigen::Vector3d goalPos (this->goal_.pose.position.x, this->goal_.pose.position.y, this->goal_.pose.position.z);
						// check the distance between last point and the goal position
						if ((bsplineLastPos - goalPos).norm() >= 0.2){ // use polynomial trajectory to make the rest of the trajectory
							nav_msgs::Path waypoints, polyTrajTemp;
							waypoints = this->buildGuidedPath(lastPs, this->goal_);
							std::vector<Eigen::Vector3d> polyStartEndConditions;
							Eigen::Vector3d polyStartVel = this->trajectory_.getDerivative().at(this->trajectory_.getDuration());
							Eigen::Vector3d polyEndVel (0.0, 0.0, 0.0);
							Eigen::Vector3d polyStartAcc = this->trajectory_.getDerivative().getDerivative().at(this->trajectory_.getDuration());
							Eigen::Vector3d polyEndAcc (0.0, 0.0, 0.0);
							polyStartEndConditions.push_back(polyStartVel);
							polyStartEndConditions.push_back(polyEndVel);
							polyStartEndConditions.push_back(polyStartAcc);
							polyStartEndConditions.push_back(polyEndAcc);
							this->polyTraj_->updatePath(waypoints, polyStartEndConditions);
							this->polyTraj_->makePlan(false); // no corridor constraint
							
							nav_msgs::Path adjustedInputCombinedTraj;
							bool satisfyDistanceCheck = false;
							double dtTemp = initTs;
							double finalTimeTemp;
							ros::Time startTime = ros::Time::now();
							ros::Time currTime;
							while (ros::ok()){
								currTime = ros::Time::now();
								if ((currTime - startTime).toSec() >= 0.05){
									cout << "[AutoFlight]: Exceed path check time. Use the best." << endl;
									break;
								}							
								nav_msgs::Path inputRestTraj = this->getCurrentTraj(dtTemp);
								nav_msgs::Path inputPolyTraj = this->polyTraj_->getTrajectory(dtTemp);
								nav_msgs::Path inputCombinedTraj;
								inputCombinedTraj.poses = inputRestTraj.poses;
								for (size_t i=1; i<inputPolyTraj.poses.size(); ++i){
									inputCombinedTraj.poses.push_back(inputPolyTraj.poses[i]);
								}
								
								satisfyDistanceCheck = this->bsplineTraj_->inputPathCheck(inputCombinedTraj, adjustedInputCombinedTraj, dtTemp, finalTimeTemp);
								if (satisfyDistanceCheck) break;
								
								dtTemp *= 0.8; // magic number 0.8
							}
							inputTraj = adjustedInputCombinedTraj;
							finalTime = finalTimeTemp - this->trajectory_.getDuration(); // need to subtract prev time since it is combined trajectory
							startEndConditions[1] = this->polyTraj_->getVel(finalTime);
							startEndConditions[3] = this->polyTraj_->getAcc(finalTime);
						}
						else{
							nav_msgs::Path adjustedInputRestTraj;
							bool satisfyDistanceCheck = false;
							double dtTemp = initTs;
							double finalTimeTemp;
							ros::Time startTime = ros::Time::now();
							ros::Time currTime;
							while (ros::ok()){
								currTime = ros::Time::now();
								if ((currTime - startTime).toSec() >= 0.05){
									cout << "[AutoFlight]: Exceed path check time. Use the best." << endl;
									break;
								}
								nav_msgs::Path inputRestTraj = this->getCurrentTraj(dtTemp);
								satisfyDistanceCheck = this->bsplineTraj_->inputPathCheck(inputRestTraj, adjustedInputRestTraj, dtTemp, finalTimeTemp);
								if (satisfyDistanceCheck) break;
								
								dtTemp *= 0.8;
							}
							inputTraj = adjustedInputRestTraj;
						}
					}
				}
				else{
					nav_msgs::Path simplePath;
					geometry_msgs::PoseStamped pStart, pGoal;
					pStart.pose = this->odom_.pose.pose;
					pGoal = this->goal_;
					simplePath = this->buildGuidedPath(pStart, pGoal);
					this->pwlTraj_->updatePath(simplePath, 1.0, false);
					this->pwlTraj_->makePlan(inputTraj, this->bsplineTraj_->getControlPointDist());
				}
			}
			

			this->inputTrajMsg_ = inputTraj;
			bool updateSuccess = this->bsplineTraj_->updatePath(inputTraj, startEndConditions);
			if (obstaclesPos.size() != 0 and updateSuccess){
				this->bsplineTraj_->updateDynamicObstacles(obstaclesPos, obstaclesVel, obstaclesSize);
			}
			if (updateSuccess){
				nav_msgs::Path bsplineTrajMsgTemp;
				bool planSuccess = this->bsplineTraj_->makePlan(bsplineTrajMsgTemp);
				if (planSuccess){
					this->bsplineTrajMsg_ = bsplineTrajMsgTemp;
					this->trajStartTime_ = ros::Time::now();
					this->trajTime_ = 0.0; // reset trajectory time
					this->trajectory_ = this->bsplineTraj_->getTrajectory();

					// optimize time
					// ros::Time timeOptStartTime = ros::Time::now();
					// this->timeOptimizer_->optimize(this->trajectory_, this->desiredVel_, this->desiredAcc_, 0.1);
					// ros::Time timeOptEndTime = ros::Time::now();
					// cout << "[AutoFlight]: Time optimizatoin spends: " << (timeOptEndTime - timeOptStartTime).toSec() << "s." << endl;

					this->trajectoryReady_ = true;
					this->replan_ = false;
					cout << "\033[1;32m[AutoFlight]: Trajectory generated successfully.\033[0m " << endl;
				}
				else{
					// if the current trajectory is still valid, then just ignore this iteration
					// if the current trajectory/or new goal point is assigned is not valid, then just stop
					if (this->hasCollision()){
						this->trajectoryReady_ = false;
						this->stop();
						cout << "[AutoFlight]: Stop!!! Trajectory generation fails." << endl;
						this->replan_ = false;
					}
					else if (this->hasDynamicCollision()){
						this->trajectoryReady_ = false;
						this->stop();
						cout << "[AutoFlight]: Stop!!! Trajectory generation fails. Replan for dynamic obstacles." << endl;
						this->replan_ = true;
					}
					else{
						if (this->trajectoryReady_){
							cout << "[AutoFlight]: Trajectory fail. Use trajectory from previous iteration." << endl;
							this->replan_ = false;
						}
						else{
							cout << "[AutoFlight]: Unable to generate a feasible trajectory. Please provide a new goal." << endl;
							this->replan_ = false;
						}
					}
				}
			}
			else{
				this->trajectoryReady_ = false;
				this->stop();
				this->replan_ = false;
				cout << "[AutoFlight]: Goal is not valid. Stop." << endl;
			}
		}
	}

	void dynamicNavigation::replanCheckCB(const ros::TimerEvent&){
		/*
			Replan if
			1. collision detected
			2. new goal point assigned
			3. fixed distance
		*/
		if (this->goalReceived_){
			this->replan_ = false;
			this->trajectoryReady_ = false;
			if (dynamicNavigation::shouldTurnTowardGoalBeforePlanning(
					this->noYawTurning_, this->useYawControl_, this->enableControlOutput_)){
				double yaw = atan2(this->goal_.pose.position.y - this->odom_.pose.pose.position.y, this->goal_.pose.position.x - this->odom_.pose.pose.position.x);
				this->facingYaw_ = yaw;
				this->moveToOrientation(yaw, this->desiredAngularVel_);
			}
			this->firstTimeSave_ = true;
			this->replan_ = true;
			this->goalReceived_ = false;
			if (this->useGlobalPlanner_){
				cout << "[AutoFlight]: Start global planning." << endl;
				this->needGlobalPlan_ = true;
				this->globalPlanReady_ = false;
			}

			cout << "[AutoFlight]: Replan for new goal position." << endl; 
			return;
		}

		if (this->trajectoryReady_){
			if (this->hasCollision()){ // if trajectory not ready, do not replan
				this->replan_ = true;
				cout << "[AutoFlight]: Replan for collision." << endl;
				return;
			}

			// replan for dynamic obstacles
			if (this->computeExecutionDistance() >= this->dynamicCollisionReplanDistance_ and this->hasDynamicCollision()){
			// if (this->hasDynamicObstacle()){
				this->replan_ = true;
				cout << "[AutoFlight]: Replan for dynamic obstacles." << endl;
				return;
			}

			if (this->computeExecutionDistance() >= 1.5 and AutoFlight::getPoseDistance(this->odom_.pose.pose, this->goal_.pose) >= 3){
				this->replan_ = true;
				cout << "[AutoFlight]: Regular replan." << endl;
				return;
			}

			if (this->computeExecutionDistance() >= this->dynamicCollisionReplanDistance_ and this->replanForDynamicObstacle()){
				this->replan_ = true;
				cout << "[AutoFlight]: Regular replan for dynamic obstacles." << endl;
				return;
			}
		}
	}

	void dynamicNavigation::trajExeCB(const ros::TimerEvent&){
		if (this->trajectoryReady_){
			ros::Time currTime = ros::Time::now();
			double realTime = (currTime - this->trajStartTime_).toSec();
			this->trajTime_ = this->bsplineTraj_->getLinearReparamTime(realTime);
			double linearReparamFactor = this->bsplineTraj_->getLinearFactor();
			const double controlTargetTime = dynamicNavigation::computeControlTargetTime(
				this->trajTime_, this->trajectory_.getDuration(), this->controlTargetLookahead_);
			Eigen::Vector3d pos = this->trajectory_.at(controlTargetTime);
			Eigen::Vector3d vel = this->trajectory_.getDerivative().at(controlTargetTime) * linearReparamFactor;
			Eigen::Vector3d acc = this->trajectory_.getDerivative().getDerivative().at(controlTargetTime) * pow(linearReparamFactor, 2);
			double endTime = this->trajectory_.getDuration()/linearReparamFactor;

			double leftTime = endTime - realTime; 
			tracking_controller::Target target;
			if (leftTime <= 0.0){ // zero vel and zero acc if close to
				target.position.x = pos(0);
				target.position.y = pos(1);
				target.position.z = pos(2);
				target.velocity.x = 0.0;
				target.velocity.y = 0.0;
				target.velocity.z = 0.0;
				target.acceleration.x = 0.0;
				target.acceleration.y = 0.0;
				target.acceleration.z = 0.0;
				target.yaw = AutoFlight::rpy_from_quaternion(this->odom_.pose.pose.orientation);
				this->updateTargetWithState(target);						
			}
			else{
				if (not this->useYawControl_){
					target.yaw = this->facingYaw_;
				}
				else if (this->noYawTurning_){
					target.yaw = AutoFlight::rpy_from_quaternion(this->odom_.pose.pose.orientation);
				}
				else{
					target.yaw = atan2(vel(1), vel(0));
				}				
				target.position.x = pos(0);
				target.position.y = pos(1);
				target.position.z = pos(2);
				target.velocity.x = vel(0);
				target.velocity.y = vel(1);
				target.velocity.z = vel(2);
				target.acceleration.x = acc(0);
				target.acceleration.y = acc(1);
				target.acceleration.z = acc(2);
				this->updateTargetWithState(target);						
			}

				const bool plannerTargetOutputAllowed =
					!this->requireOffboardBeforePlannerTarget_ ||
					!this->usePx4Offboard_ ||
					(this->mavrosStateReceived_ && this->mavrosState_.mode == "OFFBOARD");

				if (this->publishPx4RawSetpoint_ && plannerTargetOutputAllowed){
					const Eigen::Vector3d setpointPos(
						target.position.x, target.position.y, target.position.z);
					const Eigen::Vector3d setpointVel(
					target.velocity.x, target.velocity.y, target.velocity.z);
				const Eigen::Vector3d setpointAcc(
					target.acceleration.x, target.acceleration.y, target.acceleration.z);
					this->px4RawSetpointPub_.publish(dynamicNavigation::buildPx4RawSetpoint(
						setpointPos, setpointVel, setpointAcc, target.yaw, currTime));
				}
				else if (this->publishPx4RawSetpoint_ && !plannerTargetOutputAllowed){
					ROS_WARN_THROTTLE(1.0, "[AutoFlight]: Waiting for OFFBOARD before publishing PX4 raw setpoint.");
				}

				if (!plannerTargetOutputAllowed){
					ROS_WARN_THROTTLE(1.0, "[AutoFlight]: Waiting for OFFBOARD before publishing planner pose target.");
					return;
				}

				const double poseTargetTime = dynamicNavigation::computePlannerPoseTargetTime(
					this->trajTime_, this->trajectory_.getDuration(), this->enableControlOutput_, this->plannerPoseTargetLookahead_);
			const Eigen::Vector3d poseTargetPos = this->trajectory_.at(poseTargetTime);
			Eigen::Vector3d poseTargetVel = this->trajectory_.getDerivative().at(poseTargetTime) * linearReparamFactor;

			geometry_msgs::PoseStamped poseTarget;
			poseTarget.header.stamp = currTime;
			poseTarget.header.frame_id = "map";
			poseTarget.pose.position.x = poseTargetPos(0);
			poseTarget.pose.position.y = poseTargetPos(1);
			poseTarget.pose.position.z = poseTargetPos(2);
			double poseTargetYaw = dynamicNavigation::computePlannerPoseTargetYaw(
				poseTargetVel,
				this->plannerPoseTargetYawInitialized_ ? this->lastPlannerPoseTargetYaw_ : target.yaw,
				this->useYawControl_,
				this->noYawTurning_);
			this->lastPlannerPoseTargetYaw_ = poseTargetYaw;
			this->plannerPoseTargetYawInitialized_ = true;
			poseTarget.pose.orientation = AutoFlight::quaternion_from_rpy(0, 0, poseTargetYaw);
			this->plannerPoseTargetPub_.publish(poseTarget);
		}
	}

	void dynamicNavigation::visCB(const ros::TimerEvent&){
		if (this->rrtPathMsg_.poses.size() != 0){
			this->rrtPathPub_.publish(this->rrtPathMsg_);
		}
		if (this->polyTrajMsg_.poses.size() != 0){
			this->polyTrajPub_.publish(this->polyTrajMsg_);
		}
		if (this->pwlTrajMsg_.poses.size() != 0){
			this->pwlTrajPub_.publish(this->pwlTrajMsg_);
		}
		if (this->bsplineTrajMsg_.poses.size() != 0){
			this->bsplineTrajPub_.publish(this->bsplineTrajMsg_);
		}
		if (this->inputTrajMsg_.poses.size() != 0){
			this->inputTrajPub_.publish(this->inputTrajMsg_);
		}
		if (this->useDynamicObstacleTruthTopic_){
			this->publishDynamicObstacleTruthMarkers();
		}
	}

	void dynamicNavigation::freeMapCB(const ros::TimerEvent&){
		std::vector<onboardDetector::box3D> obstacles;
		std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> freeRegions;
		this->detector_->getObstacles(obstacles);
		double fov = 1.57;
		for (onboardDetector::box3D ob: obstacles){
			if (this->detector_->isObstacleInSensorRange(ob, fov)){
				Eigen::Vector3d lowerBound (ob.x-ob.x_width/2-0.3, ob.y-ob.y_width/2-0.3, ob.z);
				Eigen::Vector3d upperBound (ob.x+ob.x_width/2+0.3, ob.y+ob.y_width/2+0.3, ob.z+ob.z_width+0.3);
				freeRegions.push_back(std::make_pair(lowerBound, upperBound));
			}
		}
		this->map_->updateFreeRegions(freeRegions);
		this->map_->freeRegions(freeRegions);
	}

	void dynamicNavigation::externalGuidePathCB(const nav_msgs::Path::ConstPtr& msg){
		if (msg->poses.size() < 2){
			return;
		}
		this->externalGuidePath_ = *msg;
		this->externalGuidePathReceived_ = true;
	}

	void dynamicNavigation::run(){
		if (this->autoTakeoff_){
			this->takeoff();
		}

		// register timer callback
		this->registerCallback();
	}

	void dynamicNavigation::getStartEndConditions(std::vector<Eigen::Vector3d>& startEndConditions){
		/*	
			1. start velocity
			2. start acceleration (set to zero)
			3. end velocity
			4. end acceleration (set to zero) 
		*/

		startEndConditions = dynamicNavigation::buildStartEndConditions(
			this->currVel_, this->currAcc_, this->bsplineTraj_->getPlanInZAxis());
	}

	std::vector<Eigen::Vector3d> dynamicNavigation::buildStartEndConditions(const Eigen::Vector3d& currVel,
																			 const Eigen::Vector3d& currAcc,
																			 bool planInZAxis){
		Eigen::Vector3d startVel = currVel;
		Eigen::Vector3d endVel (0.0, 0.0, 0.0);
		Eigen::Vector3d startAcc = currAcc;
		Eigen::Vector3d endAcc (0.0, 0.0, 0.0);

		if (!planInZAxis){
			startVel(2) = 0.0;
			startAcc(2) = 0.0;
		}

		return std::vector<Eigen::Vector3d>{startVel, endVel, startAcc, endAcc};
	}

	nav_msgs::Path dynamicNavigation::buildGuidedPath(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal){
		nav_msgs::Path path;
		path.header = this->externalGuidePath_.header;
		const double minDist = 0.15;
		const geometry_msgs::Point& startP = start.pose.position;
		const geometry_msgs::Point& goalP = goal.pose.position;
		const double guideDx = goalP.x - startP.x;
		const double guideDy = goalP.y - startP.y;
		const double guideDz = goalP.z - startP.z;
		const double guideLen2 = guideDx * guideDx + guideDy * guideDy + guideDz * guideDz;
		auto appendIfFar = [&](const geometry_msgs::PoseStamped& pose){
			if (path.poses.empty()){
				path.poses.push_back(pose);
				return;
			}
			const geometry_msgs::Point& prev = path.poses.back().pose.position;
			const geometry_msgs::Point& curr = pose.pose.position;
			const double dx = curr.x - prev.x;
			const double dy = curr.y - prev.y;
			const double dz = curr.z - prev.z;
			if (sqrt(dx * dx + dy * dy + dz * dz) >= minDist){
				path.poses.push_back(pose);
			}
		};

		appendIfFar(start);
		if (this->useExternalGuidePath_ and this->externalGuidePathReceived_){
			for (size_t i = 0; i < this->externalGuidePath_.poses.size(); ++i){
				const geometry_msgs::PoseStamped& guide = this->externalGuidePath_.poses[i];
				if (i == 0){
					continue;
				}
				const geometry_msgs::Point& gp = guide.pose.position;
				const double dxGoal = gp.x - goalP.x;
				const double dyGoal = gp.y - goalP.y;
				const double dzGoal = gp.z - goalP.z;
				if (sqrt(dxGoal * dxGoal + dyGoal * dyGoal + dzGoal * dzGoal) < minDist){
					continue;
				}
				if (guideLen2 > minDist * minDist){
					const double dxStart = gp.x - startP.x;
					const double dyStart = gp.y - startP.y;
					const double dzStart = gp.z - startP.z;
					const double projection = dxStart * guideDx + dyStart * guideDy + dzStart * guideDz;
					if (projection <= minDist || projection >= guideLen2 - minDist){
						continue;
					}
				}
				appendIfFar(guide);
			}
		}
		appendIfFar(goal);
		if (path.poses.size() < 2){
			path.poses = std::vector<geometry_msgs::PoseStamped>{start, goal};
		}
		return path;
	}

	bool dynamicNavigation::shouldTurnTowardGoalBeforePlanning(bool noYawTurning,
															 bool useYawControl,
															 bool enableControlOutput){
		return !noYawTurning && !useYawControl && enableControlOutput;
	}

	double dynamicNavigation::computePlannerPoseTargetTime(double trajTime,
														  double trajDuration,
														  bool enableControlOutput,
														  double plannerPoseTargetLookahead){
		if (enableControlOutput){
			return trajTime;
		}

		return std::min(trajDuration, trajTime + std::max(0.0, plannerPoseTargetLookahead));
	}

	double dynamicNavigation::computeControlTargetTime(double trajTime,
													   double trajDuration,
													   double controlTargetLookahead){
		return std::min(trajDuration, trajTime + std::max(0.0, controlTargetLookahead));
	}

	double dynamicNavigation::computePlannerPoseTargetYaw(const Eigen::Vector3d& poseTargetVel,
													  double fallbackYaw,
													  bool useYawControl,
													  bool noYawTurning){
		if (useYawControl && !noYawTurning && poseTargetVel.head<2>().norm() > 1e-3){
			return atan2(poseTargetVel(1), poseTargetVel(0));
		}

		return fallbackYaw;
	}

	mavros_msgs::PositionTarget dynamicNavigation::buildPx4RawSetpoint(const Eigen::Vector3d& position,
																	  const Eigen::Vector3d& velocity,
																	  const Eigen::Vector3d& acceleration,
																	  double yaw,
																	  const ros::Time& stamp){
		mavros_msgs::PositionTarget setpoint;
		setpoint.header.stamp = stamp;
		setpoint.header.frame_id = "map";
		setpoint.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
		setpoint.type_mask = mavros_msgs::PositionTarget::IGNORE_YAW_RATE;
		setpoint.position.x = position(0);
		setpoint.position.y = position(1);
		setpoint.position.z = position(2);
		setpoint.velocity.x = velocity(0);
		setpoint.velocity.y = velocity(1);
		setpoint.velocity.z = velocity(2);
		setpoint.acceleration_or_force.x = acceleration(0);
		setpoint.acceleration_or_force.y = acceleration(1);
		setpoint.acceleration_or_force.z = acceleration(2);
		setpoint.yaw = yaw;
		setpoint.yaw_rate = 0.0;
		return setpoint;
	}

	bool dynamicNavigation::isPointInPredictedDynamicObstacle(const Eigen::Vector3d& point,
															 const Eigen::Vector3d& obstaclePos,
															 const Eigen::Vector3d& obstacleVel,
															 const Eigen::Vector3d& obstacleSize,
															 double predictionTime,
															 double safetyMargin){
		Eigen::Vector3d predictedObstaclePos = obstaclePos + std::max(0.0, predictionTime) * obstacleVel;
		Eigen::Vector3d halfSize = obstacleSize / 2.0;
		halfSize(0) += std::max(0.0, safetyMargin);
		halfSize(1) += std::max(0.0, safetyMargin);

		return point(0) >= predictedObstaclePos(0) - halfSize(0) &&
			   point(0) <= predictedObstaclePos(0) + halfSize(0) &&
			   point(1) >= predictedObstaclePos(1) - halfSize(1) &&
			   point(1) <= predictedObstaclePos(1) + halfSize(1) &&
			   point(2) >= predictedObstaclePos(2) - halfSize(2) &&
			   point(2) <= predictedObstaclePos(2) + halfSize(2);
	}

	bool dynamicNavigation::hasCollision(){
		if (this->trajectoryReady_){
			for (double t=this->trajTime_; t<=this->trajectory_.getDuration(); t+=0.1){
				Eigen::Vector3d p = this->trajectory_.at(t);
				bool hasCollision = this->map_->isInflatedOccupied(p);
				if (hasCollision){
					return true;
				}
			}
		}
		return false;
	}

	bool dynamicNavigation::hasDynamicCollision(){
		if (this->trajectoryReady_){
			std::vector<Eigen::Vector3d> obstaclesPos, obstaclesVel, obstaclesSize;
			if (this->useDynamicObstacleTruthTopic_){
				this->getDynamicObstacleTruth(obstaclesPos, obstaclesVel, obstaclesSize);
			}
			else if (this->useFakeDetector_){
				this->getDynamicObstacles(obstaclesPos, obstaclesVel, obstaclesSize);
			}
			else{ 
				this->map_->getDynamicObstacles(obstaclesPos, obstaclesVel, obstaclesSize);
			}

			double checkStep = std::max(0.01, this->dynamicCollisionCheckStep_);
			double maxPredictionTime = std::max(0.0, this->dynamicCollisionPredictionTime_);
			for (double t=this->trajTime_; t<=this->trajectory_.getDuration(); t+=checkStep){
				double predictionTime = std::min(std::max(0.0, t - this->trajTime_), maxPredictionTime);
				Eigen::Vector3d p = this->trajectory_.at(t);
				
				for (size_t i=0; i<obstaclesPos.size(); ++i){
					if (dynamicNavigation::isPointInPredictedDynamicObstacle(
							p, obstaclesPos[i], obstaclesVel[i], obstaclesSize[i], predictionTime, this->dynamicCollisionSafetyMargin_)){
						return true;
					}					
				}
			}
		}
		return false;
	}

	double dynamicNavigation::computeExecutionDistance(){
		if (this->trajectoryReady_ and not this->replan_){
			Eigen::Vector3d prevP, currP;
			bool firstTime = true;
			double totalDistance = 0.0;
			for (double t=0.0; t<=this->trajTime_; t+=0.1){
				currP = this->trajectory_.at(t);
				if (firstTime){
					firstTime = false;
				}
				else{
					totalDistance += (currP - prevP).norm();
				}
				prevP = currP;
			}
			return totalDistance;
		}
		return -1.0;
	}

	bool dynamicNavigation::replanForDynamicObstacle(){
		ros::Time currTime = ros::Time::now();
		std::vector<Eigen::Vector3d> obstaclesPos, obstaclesVel, obstaclesSize;
		if (this->useDynamicObstacleTruthTopic_){
			this->getDynamicObstacleTruth(obstaclesPos, obstaclesVel, obstaclesSize);
		}
		else if (this->useFakeDetector_){
			this->getDynamicObstacles(obstaclesPos, obstaclesVel, obstaclesSize);
		}
		else{ 
			this->map_->getDynamicObstacles(obstaclesPos, obstaclesVel, obstaclesSize);
		}

		bool replan = false;
		bool hasDynamicObstacle = (obstaclesPos.size() != 0);
		if (hasDynamicObstacle){
			double timePassed = (currTime - this->lastDynamicObstacleTime_).toSec();
			if (this->lastDynamicObstacle_ == false or timePassed >= this->replanTimeForDynamicObstacle_){
				replan = true;
				this->lastDynamicObstacleTime_ = currTime;
			}
			this->lastDynamicObstacle_ = true;
		}
		else{
			this->lastDynamicObstacle_ = false;
		}

		return replan;
	}

	nav_msgs::Path dynamicNavigation::getCurrentTraj(double dt){
		nav_msgs::Path currentTraj;
		currentTraj.header.frame_id = "map";
		currentTraj.header.stamp = ros::Time::now();
	
		if (this->trajectoryReady_){
			// include the current pose
			// geometry_msgs::PoseStamped psCurr;
			// psCurr.pose = this->odom_.pose.pose;
			// currentTraj.poses.push_back(psCurr);
			for (double t=this->trajTime_; t<=this->trajectory_.getDuration(); t+=dt){
				Eigen::Vector3d pos = this->trajectory_.at(t);
				geometry_msgs::PoseStamped ps;
				ps.pose.position.x = pos(0);
				ps.pose.position.y = pos(1);
				ps.pose.position.z = pos(2);
				currentTraj.poses.push_back(ps);
			}		
		}
		return currentTraj;
	}


	nav_msgs::Path dynamicNavigation::getRestGlobalPath(){
		nav_msgs::Path currPath;

		int nextIdx = this->rrtPathMsg_.poses.size()-1;
		Eigen::Vector3d pCurr (this->odom_.pose.pose.position.x, this->odom_.pose.pose.position.y, this->odom_.pose.pose.position.z);
		double minDist = std::numeric_limits<double>::infinity();
		for (size_t i=0; i<this->rrtPathMsg_.poses.size()-1; ++i){
			geometry_msgs::PoseStamped ps = this->rrtPathMsg_.poses[i];
			Eigen::Vector3d pEig (ps.pose.position.x, ps.pose.position.y, ps.pose.position.z);
			Eigen::Vector3d pDiff = pCurr - pEig;

			geometry_msgs::PoseStamped psNext = this->rrtPathMsg_.poses[i+1];
			Eigen::Vector3d pEigNext (psNext.pose.position.x, psNext.pose.position.y, psNext.pose.position.z);
			Eigen::Vector3d diffToNext = pEigNext - pEig;
			double dist = (pEig - pCurr).norm();
			if (trajPlanner::angleBetweenVectors(diffToNext, pDiff) > PI_const*3.0/4.0){
				if (dist < minDist){
					nextIdx = i;
					minDist = dist;
				}
			}
		}


		geometry_msgs::PoseStamped psCurr;
		psCurr.pose = this->odom_.pose.pose;
		currPath.poses.push_back(psCurr);
		for (size_t i=nextIdx; i<this->rrtPathMsg_.poses.size(); ++i){
			currPath.poses.push_back(this->rrtPathMsg_.poses[i]);
		}
		return currPath;		
	}

	void dynamicNavigation::getDynamicObstacles(std::vector<Eigen::Vector3d>& obstaclesPos, std::vector<Eigen::Vector3d>& obstaclesVel, std::vector<Eigen::Vector3d>& obstaclesSize){
		std::vector<onboardDetector::box3D> obstacles;
		this->detector_->getObstaclesInSensorRange(PI_const, obstacles);
		for (onboardDetector::box3D ob : obstacles){
			Eigen::Vector3d pos (ob.x, ob.y, ob.z);
			Eigen::Vector3d vel (ob.Vx, ob.Vy, 0.0);
			Eigen::Vector3d size (ob.x_width, ob.y_width, ob.z_width);
			obstaclesPos.push_back(pos);
			obstaclesVel.push_back(vel);
			obstaclesSize.push_back(size);
		}
	}

	void dynamicNavigation::dynamicObstacleTruthCB(const uav_simulator::DynamicObstacleArray::ConstPtr& msg){
		this->dynamicObstacleTruth_ = msg->obstacles;
	}

	void dynamicNavigation::convertDynamicObstacleTruth(const uav_simulator::DynamicObstacleState& obstacle,
														Eigen::Vector3d& pos,
														Eigen::Vector3d& vel,
														Eigen::Vector3d& size){
		const double obstacleSize = std::max(0.0, obstacle.size);
		const double obstacleHeight = std::max(0.0, obstacle.height);
		pos = Eigen::Vector3d(
			obstacle.pose.position.x,
			obstacle.pose.position.y,
			obstacle.pose.position.z + obstacleHeight / 2.0);
		vel = Eigen::Vector3d(
			obstacle.twist.linear.x,
			obstacle.twist.linear.y,
			obstacle.twist.linear.z);
		size = Eigen::Vector3d(obstacleSize, obstacleSize, obstacleHeight);
	}

	void dynamicNavigation::getDynamicObstacleTruth(std::vector<Eigen::Vector3d>& obstaclesPos,
													std::vector<Eigen::Vector3d>& obstaclesVel,
													std::vector<Eigen::Vector3d>& obstaclesSize){
		for (const uav_simulator::DynamicObstacleState& obstacle : this->dynamicObstacleTruth_){
			Eigen::Vector3d pos, vel, size;
			dynamicNavigation::convertDynamicObstacleTruth(obstacle, pos, vel, size);
			obstaclesPos.push_back(pos);
			obstaclesVel.push_back(vel);
			obstaclesSize.push_back(size);
		}
	}

	void dynamicNavigation::publishDynamicObstacleTruthMarkers(){
		if (!this->dynamicObstacleTruthMarkerPub_){
			return;
		}

		visualization_msgs::MarkerArray markerArray;
		int markerId = 0;
		const ros::Time now = ros::Time::now();
		for (const uav_simulator::DynamicObstacleState& obstacle : this->dynamicObstacleTruth_){
			Eigen::Vector3d pos, vel, size;
			dynamicNavigation::convertDynamicObstacleTruth(obstacle, pos, vel, size);

			visualization_msgs::Marker marker;
			marker.header.frame_id = "map";
			marker.header.stamp = now;
			marker.ns = "dynamic_obstacle_truth";
			marker.id = markerId++;
			marker.type = visualization_msgs::Marker::CUBE;
			marker.action = visualization_msgs::Marker::ADD;
			marker.pose.position.x = pos(0);
			marker.pose.position.y = pos(1);
			marker.pose.position.z = pos(2);
			marker.pose.orientation.w = 1.0;
			marker.scale.x = size(0);
			marker.scale.y = size(1);
			marker.scale.z = size(2);
			marker.color.a = 0.55;
			marker.color.r = 1.0;
			marker.color.g = 0.25;
			marker.color.b = 0.0;
			marker.lifetime = ros::Duration(0.2);
			markerArray.markers.push_back(marker);

			visualization_msgs::Marker velocityMarker;
			velocityMarker.header.frame_id = "map";
			velocityMarker.header.stamp = now;
			velocityMarker.ns = "dynamic_obstacle_truth_velocity";
			velocityMarker.id = markerId++;
			velocityMarker.type = visualization_msgs::Marker::ARROW;
			velocityMarker.action = visualization_msgs::Marker::ADD;
			geometry_msgs::Point start;
			start.x = pos(0);
			start.y = pos(1);
			start.z = pos(2);
			geometry_msgs::Point end;
			end.x = pos(0) + vel(0);
			end.y = pos(1) + vel(1);
			end.z = pos(2) + vel(2);
			velocityMarker.points.push_back(start);
			velocityMarker.points.push_back(end);
			velocityMarker.scale.x = 0.08;
			velocityMarker.scale.y = 0.18;
			velocityMarker.scale.z = 0.18;
			velocityMarker.color.a = 0.9;
			velocityMarker.color.r = 1.0;
			velocityMarker.color.g = 1.0;
			velocityMarker.color.b = 0.0;
			velocityMarker.lifetime = ros::Duration(0.2);
			markerArray.markers.push_back(velocityMarker);
		}

		this->dynamicObstacleTruthMarkerPub_.publish(markerArray);
	}
}
