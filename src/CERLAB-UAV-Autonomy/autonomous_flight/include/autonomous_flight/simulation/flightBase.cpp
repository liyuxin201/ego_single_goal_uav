/*
	FILE: flightBase.h
	-------------------------
	implementation of flight base
*/
#include <autonomous_flight/simulation/flightBase.h>

namespace AutoFlight{
	flightBase::flightBase(const ros::NodeHandle& nh) : nh_(nh){
    	// parameters    	
		if (not this->nh_.getParam("autonomous_flight/takeoff_height", this->takeoffHgt_)){
			this->takeoffHgt_ = 1.0;
			cout << "[AutoFlight]: No takeoff height param found. Use default: 1.0 m." << endl;
		}
		else{
			cout << "[AutoFlight]: Takeoff Height: " << this->takeoffHgt_ << "m." << endl;
		}

		if (not this->nh_.getParam("autonomous_flight/use_px4_offboard", this->usePx4Offboard_)){
			this->usePx4Offboard_ = false;
			cout << "[AutoFlight]: No PX4 offboard param found. Use default: false." << endl;
		}
		else{
			cout << "[AutoFlight]: PX4 offboard control is set to: " << this->usePx4Offboard_ << endl;
		}

		if (not this->nh_.getParam("autonomous_flight/release_control_on_goal", this->releaseControlOnGoal_)){
			this->releaseControlOnGoal_ = false;
			cout << "[AutoFlight]: No release-control-on-goal param found. Use default: false." << endl;
		}
		else{
			cout << "[AutoFlight]: Release control on goal is set to: " << this->releaseControlOnGoal_ << endl;
		}

		if (not this->nh_.getParam("autonomous_flight/enable_control_output", this->enableControlOutput_)){
			this->enableControlOutput_ = true;
		}
		cout << "[AutoFlight]: Control output is set to: " << this->enableControlOutput_ << endl;

		if (not this->nh_.getParam("autonomous_flight/listen_click_goal", this->listenClickGoal_)){
			this->listenClickGoal_ = true;
		}
		cout << "[AutoFlight]: Click goal input is set to: " << this->listenClickGoal_ << endl;

		if (not this->nh_.getParam("autonomous_flight/enforce_click_goal_range", this->enforceClickGoalRange_)){
			this->enforceClickGoalRange_ = false;
		}
		cout << "[AutoFlight]: Click goal range enforcement is set to: " << this->enforceClickGoalRange_ << endl;

		if (not this->nh_.getParam("autonomous_flight/click_goal_range_x", this->clickGoalRangeX_)){
			this->clickGoalRangeX_ = 10.0;
		}
		if (not this->nh_.getParam("autonomous_flight/click_goal_range_y", this->clickGoalRangeY_)){
			this->clickGoalRangeY_ = 10.0;
		}
		cout << "[AutoFlight]: Click goal valid XY range is set to: +/-"
			 << this->clickGoalRangeX_ << "m, +/-" << this->clickGoalRangeY_ << "m." << endl;

		if (not this->nh_.getParam("autonomous_flight/goal_topic", this->goalTopic_)){
			this->goalTopic_ = "/move_base_simple/goal";
		}
		cout << "[AutoFlight]: Goal topic is set to: " << this->goalTopic_ << endl;

		if (this->nh_.getParam("autonomous_flight/external_pose_target_topic", this->externalPoseTargetTopic_)){
			cout << "[AutoFlight]: External pose target topic is set to: " << this->externalPoseTargetTopic_ << endl;
		}
		else{
			this->externalPoseTargetTopic_.clear();
		}

		if (this->nh_.getParam("autonomous_flight/external_offboard_setpoint_topic", this->externalOffboardSetpointTopic_)){
			cout << "[AutoFlight]: External offboard setpoint topic is set to: " << this->externalOffboardSetpointTopic_ << endl;
		}
		else{
			this->externalOffboardSetpointTopic_.clear();
		}

		if (not this->nh_.getParam("autonomous_flight/external_offboard_timeout", this->externalOffboardTimeout_)){
			this->externalOffboardTimeout_ = 0.0;
		}
		cout << "[AutoFlight]: External offboard timeout is set to: " << this->externalOffboardTimeout_ << "s." << endl;

		// subscriber
		if (this->usePx4Offboard_){
			this->stateSub_ = this->nh_.subscribe("/mavros/state", 1000, &flightBase::stateCB, this);
			this->odomSub_ = this->nh_.subscribe("/mavros/local_position/odom", 1000, &flightBase::odomCB, this);
			this->armClient_ = this->nh_.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
			this->setModeClient_ = this->nh_.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
		}
		else{
			this->odomSub_ = this->nh_.subscribe("/CERLAB/quadcopter/odom", 1000, &flightBase::odomCB, this);
		}
		if (this->listenClickGoal_){
			this->clickSub_ = this->nh_.subscribe(this->goalTopic_, 1000, &flightBase::clickCB, this);
		}
		if (!this->externalPoseTargetTopic_.empty()){
			this->externalPoseTargetSub_ = this->nh_.subscribe(this->externalPoseTargetTopic_, 1000, &flightBase::externalPoseTargetCB, this);
		}
		if (this->usePx4Offboard_ and !this->externalOffboardSetpointTopic_.empty() and this->externalOffboardTimeout_ > 0.0){
			this->externalOffboardSetpointSub_ = this->nh_.subscribe(
				this->externalOffboardSetpointTopic_, 1000, &flightBase::externalOffboardSetpointCB, this);
		}
		

		// publisher
		if (this->usePx4Offboard_){
			this->posePub_ = this->nh_.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local", 1000);
		}
		else{
			this->posePub_ = this->nh_.advertise<geometry_msgs::PoseStamped>("/CERLAB/quadcopter/setpoint_pose", 1000);
		}
		this->statePub_ = this->nh_.advertise<tracking_controller::Target>("/autonomous_flight/target_state", 1000);

		ros::Rate r (10);
		while (ros::ok() and not this->odomReceived_){
			if (this->usePx4Offboard_ and (not this->mavrosStateReceived_ or not this->mavrosState_.connected)){
				ros::spinOnce();
				r.sleep();
				continue;
			}
			ros::spinOnce();
			r.sleep();
		}

		this->poseTgt_.pose = this->odom_.pose.pose;
		this->poseTgt_.header.frame_id = "map";
		if (this->poseTgt_.pose.orientation.w == 0.0 &&
			this->poseTgt_.pose.orientation.x == 0.0 &&
			this->poseTgt_.pose.orientation.y == 0.0 &&
			this->poseTgt_.pose.orientation.z == 0.0){
			this->poseTgt_.pose.orientation.w = 1.0;
		}
		cout << "[AutoFlight]: Control topics are ready." << endl;
	
    	// Tareget publish thread
		this->targetPubWorker_ = std::thread(&flightBase::publishTarget, this);
		this->targetPubWorker_.detach();


		// state update callback
		this->stateUpdateTimer_ = this->nh_.createTimer(ros::Duration(0.033), &flightBase::stateUpdateCB, this);
	}

	void flightBase::stateCB(const mavros_msgs::State::ConstPtr& state){
		this->mavrosState_ = *state;
		this->mavrosStateReceived_ = true;
	}

	void flightBase::publishTarget(){
		ros::Rate r (200);
		if (this->usePx4Offboard_ && this->enableControlOutput_){
			for(int i = 100; ros::ok() && i > 0; --i){
				this->poseTgt_.header.frame_id = "map";
				this->poseTgt_.header.stamp = ros::Time::now();
				this->posePub_.publish(this->poseTgt_);
				r.sleep();
			}
		}
		while (ros::ok()){
			if (!this->enableControlOutput_){
				r.sleep();
				continue;
			}

			if (this->releaseControlOnGoal_ && this->goalReceived_){
				this->relinquishedControl_ = true;
			}

			if (this->usePx4Offboard_ and this->externalOffboardTimeout_ > 0.0){
				const bool externalOffboardControlActive = flightBase::isExternalOffboardControlActive(
					this->externalOffboardSetpointReceived_,
					this->lastExternalOffboardSetpointTime_,
					ros::Time::now(),
					this->externalOffboardTimeout_);
				if (externalOffboardControlActive != this->externalOffboardControlActive_){
					this->externalOffboardControlActive_ = externalOffboardControlActive;
					if (this->externalOffboardControlActive_){
						cout << "[AutoFlight]: External offboard control detected. Suspend hover output." << endl;
					}
					else{
						this->holdCurrentPose();
						cout << "[AutoFlight]: External offboard control timeout. Resume hover hold." << endl;
					}
				}
			}

			if (this->usePx4Offboard_){
				mavros_msgs::SetMode offboardMode;
				offboardMode.request.custom_mode = "OFFBOARD";
				mavros_msgs::CommandBool armCmd;
				armCmd.request.value = true;
				static ros::Time lastRequest = ros::Time(0);
				if (this->mavrosState_.mode != "OFFBOARD" && (ros::Time::now() - lastRequest > ros::Duration(0.5))){
					if (this->setModeClient_.call(offboardMode) && offboardMode.response.mode_sent){
						cout << "[AutoFlight]: Offboard mode enabled." << endl;
					}
					lastRequest = ros::Time::now();
				}
				else if (!this->mavrosState_.armed && (ros::Time::now() - lastRequest > ros::Duration(0.5))){
					if (this->armClient_.call(armCmd) && armCmd.response.success){
						cout << "[AutoFlight]: Vehicle armed." << endl;
					}
					lastRequest = ros::Time::now();
				}
			}
	        if (this->relinquishedControl_){
	        	r.sleep();
	        	continue;
	        }
	        if (this->externalOffboardControlActive_){
	        	r.sleep();
	        	continue;
	        }
	        if (this->poseControl_){
	        	this->poseTgt_.header.stamp = ros::Time::now();
	        	this->posePub_.publish(this->poseTgt_);
	        }
	        else{
				this->statePub_.publish(this->stateTgt_);
			}
			r.sleep();
		}			
	}


	void flightBase::clickCB(const geometry_msgs::PoseStamped::ConstPtr& cp){
		if (this->enforceClickGoalRange_ &&
			!flightBase::isPointInLocalXYRange(
				cp->pose.position.x,
				cp->pose.position.y,
				this->odom_.pose.pose.position.x,
				this->odom_.pose.pose.position.y,
				this->clickGoalRangeX_,
				this->clickGoalRangeY_)){
			ROS_WARN_STREAM("[AutoFlight]: Reject click goal outside local valid range. "
							<< "goal=(" << cp->pose.position.x << ", " << cp->pose.position.y << "), "
							<< "uav=(" << this->odom_.pose.pose.position.x << ", " << this->odom_.pose.pose.position.y << "), "
							<< "range=(+/-" << this->clickGoalRangeX_ << ", +/-" << this->clickGoalRangeY_ << ").");
			return;
		}
		this->goal_ = *cp;
		this->goal_.pose.position.z = this->takeoffHgt_;
		if (not this->firstGoal_){
			this->firstGoal_ = true;
		}

		if (not this->goalReceived_){
			this->goalReceived_ = true;
		}
	}

	void flightBase::externalPoseTargetCB(const geometry_msgs::PoseStamped::ConstPtr& ps){
		geometry_msgs::PoseStamped poseTarget = *ps;
		if (poseTarget.pose.orientation.w == 0.0 &&
			poseTarget.pose.orientation.x == 0.0 &&
			poseTarget.pose.orientation.y == 0.0 &&
			poseTarget.pose.orientation.z == 0.0){
			poseTarget.pose.orientation.w = 1.0;
		}
		this->updateTarget(poseTarget);
	}

	void flightBase::externalOffboardSetpointCB(const mavros_msgs::PositionTarget::ConstPtr&){
		this->externalOffboardSetpointReceived_ = true;
		this->lastExternalOffboardSetpointTime_ = ros::Time::now();
	}


	void flightBase::odomCB(const nav_msgs::OdometryConstPtr& odom){
		this->odom_ = *odom;
		this->currPos_(0) = this->odom_.pose.pose.position.x;
		this->currPos_(1) = this->odom_.pose.pose.position.y;
		this->currPos_(2) = this->odom_.pose.pose.position.z;
		if (this->odomReceived_ == false){
			this->odomReceived_ = true;
		}
	}

	void flightBase::stateUpdateCB(const ros::TimerEvent&){
		Eigen::Vector3d currVelBody (this->odom_.twist.twist.linear.x, this->odom_.twist.twist.linear.y, this->odom_.twist.twist.linear.z);
		Eigen::Vector4d orientationQuat (this->odom_.pose.pose.orientation.w, this->odom_.pose.pose.orientation.x, this->odom_.pose.pose.orientation.y, this->odom_.pose.pose.orientation.z);
		Eigen::Matrix3d orientationRot = AutoFlight::quat2RotMatrix(orientationQuat);
		this->currVel_ = orientationRot * currVelBody;	
		ros::Time currTime = ros::Time::now();	
		flightBase::updateKinematicState(this->currVel_, currTime, this->stateUpdateFirstTime_, this->prevVel_, this->currAcc_, this->prevStateTime_);
	}

	void flightBase::updateKinematicState(const Eigen::Vector3d& currVel,
										  const ros::Time& currTime,
										  bool& stateUpdateFirstTime,
										  Eigen::Vector3d& prevVel,
										  Eigen::Vector3d& currAcc,
										  ros::Time& prevStateTime){
		if (stateUpdateFirstTime){
			currAcc = Eigen::Vector3d::Zero();
			prevVel = currVel;
			prevStateTime = currTime;
			stateUpdateFirstTime = false;
			return;
		}

		double dt = (currTime - prevStateTime).toSec();
		if (dt <= 1e-6){
			currAcc = Eigen::Vector3d::Zero();
		}
		else{
			currAcc = (currVel - prevVel)/dt;
		}

		prevVel = currVel;
		prevStateTime = currTime;
	}

	bool flightBase::isExternalOffboardControlActive(bool externalOffboardSetpointReceived,
													 const ros::Time& lastExternalOffboardSetpointTime,
													 const ros::Time& now,
													 double timeoutSec){
		if (!externalOffboardSetpointReceived || timeoutSec <= 0.0){
			return false;
		}

		return (now - lastExternalOffboardSetpointTime).toSec() <= timeoutSec;
	}

	bool flightBase::isPointInLocalXYRange(double pointX,
										  double pointY,
										  double centerX,
										  double centerY,
										  double rangeX,
										  double rangeY){
		return std::abs(pointX - centerX) <= std::abs(rangeX) &&
			   std::abs(pointY - centerY) <= std::abs(rangeY);
	}


	void flightBase::takeoff(){
		if (this->odom_.pose.pose.position.z >= 0.2){
			this->hasTakeoff_ = true;
		}

		geometry_msgs::PoseStamped ps;
		ps.pose = this->odom_.pose.pose;
		ps.pose.position.z = this->takeoffHgt_;
		if (ps.pose.orientation.w == 0.0 &&
			ps.pose.orientation.x == 0.0 &&
			ps.pose.orientation.y == 0.0 &&
			ps.pose.orientation.z == 0.0){
			ps.pose.orientation.w = 1.0;
		}
		this->updateTarget(ps);
		ros::Rate r (30);
		while (ros::ok() and std::abs(this->odom_.pose.pose.position.z - this->takeoffHgt_) >= 0.1 and not this->hasTakeoff_){
			ros::spinOnce();
			r.sleep();
		}
		this->hasTakeoff_ = true;
	}



	void flightBase::run(){
		// flight test with circle
		double r; // radius
		double v; // circle velocity
    	
    	// track circle radius parameters    	
		if (not this->nh_.getParam("autonomous_flight/radius", r)){
			r = 2.0;
			cout << "[AutoFlight]: No circle radius param found. Use default: 2.0 m." << endl;
		}
		else{
			cout << "[AutoFlight]: Circle radius: " << r << "m." << endl;
		}

    	// track circle velocity parameters    	
		if (not this->nh_.getParam("autonomous_flight/circle_velocity", v)){
			v = 1.0;
			cout << "[AutoFlight]: No circle velocity param found. Use default: 1.0 m/s." << endl;
		}
		else{
			cout << "[AutoFlight]: Circle velocity: " << v << "m/s." << endl;
		}

		double z = this->odom_.pose.pose.position.z;
		geometry_msgs::PoseStamped startPs;
		startPs.pose.position.x = r;
		startPs.pose.position.y = 0.0;
		startPs.pose.position.z = z;
		this->updateTarget(startPs);
		
		cout << "[AutoFlight]: Go to target point..." << endl;
		ros::Rate rate (30);
		while (ros::ok() and std::abs(this->odom_.pose.pose.position.x - startPs.pose.position.x) >= 0.1){
			ros::spinOnce();
			rate.sleep();
		}
		cout << "[AutoFlight]: Reach target point." << endl;

		ros::Time startTime = ros::Time::now();
		while (ros::ok()){
			ros::Time currTime = ros::Time::now();
			double t = (currTime - startTime).toSec();
			double rad = v * t / r;
			double x = r * cos(rad);
			double y = r * sin(rad);
			double vx = -v * sin(rad);
			double vy = v * cos(rad);
			double vz = 0.0;
			double aNorm = v*v/r;
			Eigen::Vector3d accVec (x, y, 0);
			accVec = -aNorm * accVec / accVec.norm();
			double ax = accVec(0);
			double ay = accVec(1);
			double az = 0.0;

			// state target message
			tracking_controller::Target target;
			target.position.x = x;
			target.position.y = y;
			target.position.z = z;
			target.velocity.x = vx;
			target.velocity.y = vy;
			target.velocity.z = vz;
			target.acceleration.x = ax;
			target.acceleration.y = ay;
			target.acceleration.z = az;
			this->updateTargetWithState(target);
			ros::spinOnce();
			rate.sleep();
		}
	}

	void flightBase::stop(){
		geometry_msgs::PoseStamped ps;
		ps.pose = this->odom_.pose.pose;
		this->updateTarget(ps);		
	}

	void flightBase::moveToOrientation(double yaw, double desiredAngularVel){
		double yawTgt = yaw;
		geometry_msgs::Quaternion orientation = AutoFlight::quaternion_from_rpy(0, 0, yaw);
		double yawCurr = AutoFlight::rpy_from_quaternion(this->odom_.pose.pose.orientation);		
		geometry_msgs::PoseStamped ps;
		ps.pose = this->odom_.pose.pose;
		ps.pose.orientation = orientation;

		double yawDiff = yawTgt - yawCurr; // difference between yaw
		double direction = 0;
		double yawDiffAbs = std::abs(yawDiff);
		if ((yawDiffAbs <= PI_const) and (yawDiff>0)){
			direction = 1.0; // counter clockwise
		} 
		else if ((yawDiffAbs <= PI_const) and (yawDiff<0)){
			direction = -1.0; // clockwise
		}
		else if ((yawDiffAbs > PI_const) and (yawDiff>0)){
			direction = -1.0; // rotate in clockwise direction
			yawDiffAbs = 2 * PI_const - yawDiffAbs;
		}
		else if ((yawDiffAbs > PI_const) and (yawDiff<0)){
			direction = 1.0; // counter clockwise
			yawDiffAbs = 2 * PI_const - yawDiffAbs;
		}

		double endTime = yawDiffAbs/desiredAngularVel;
		tracking_controller::Target target;
		geometry_msgs::PoseStamped psT;
		psT.pose = ps.pose;
		ros::Time startTime = ros::Time::now();
		ros::Time currTime = ros::Time::now();
		ros::Rate r (200);
		while (ros::ok() and not this->isReach(ps)){
			currTime = ros::Time::now();
			double t = (currTime - startTime).toSec();

			if (t >= endTime){ 
				psT = ps;
			}
			else{
				double currYawTgt = yawCurr + (double) direction * t/endTime * yawDiffAbs;
				geometry_msgs::Quaternion quatT = AutoFlight::quaternion_from_rpy(0, 0, currYawTgt);
				psT.pose.orientation = quatT;
				
			}

			// this->updateTarget(psT);
			target.position.x = psT.pose.position.x;
			target.position.y = psT.pose.position.y;
			target.position.z = psT.pose.position.z;
			target.yaw = AutoFlight::rpy_from_quaternion(psT.pose.orientation);
			this->updateTargetWithState(target);
			// cout << "here" << endl;
			ros::spinOnce();
			r.sleep();
		}
	}

	void flightBase::updateTarget(const geometry_msgs::PoseStamped& ps){ // global frame
		this->poseTgt_ = ps;
		this->poseTgt_.header.frame_id = "map";
		this->poseControl_ = true;
	}

	void flightBase::updateTargetWithState(const tracking_controller::Target& target){
		this->stateTgt_ = target;
		this->poseControl_ = false;
	}

	void flightBase::holdCurrentPose(){
		geometry_msgs::PoseStamped ps;
		ps.pose = this->odom_.pose.pose;
		if (ps.pose.orientation.w == 0.0 &&
			ps.pose.orientation.x == 0.0 &&
			ps.pose.orientation.y == 0.0 &&
			ps.pose.orientation.z == 0.0){
			ps.pose.orientation.w = 1.0;
		}
		this->updateTarget(ps);
	}

	bool flightBase::isReach(const geometry_msgs::PoseStamped& poseTgt, bool useYaw){
		double targetX, targetY, targetZ, targetYaw, currX, currY, currZ, currYaw;
		targetX = poseTgt.pose.position.x;
		targetY = poseTgt.pose.position.y;
		targetZ = poseTgt.pose.position.z;
		targetYaw = AutoFlight::rpy_from_quaternion(poseTgt.pose.orientation);
		currX = this->odom_.pose.pose.position.x;
		currY = this->odom_.pose.pose.position.y;
		currZ = this->odom_.pose.pose.position.z;
		currYaw = AutoFlight::rpy_from_quaternion(this->odom_.pose.pose.orientation);
		
		bool reachX, reachY, reachZ, reachYaw;
		reachX = std::abs(targetX - currX) < 0.1;
		reachY = std::abs(targetY - currY) < 0.1;
		reachZ = std::abs(targetZ - currZ) < 0.15;
		if (useYaw){
			reachYaw = std::abs(targetYaw - currYaw) < 0.1;
		}
		else{
			reachYaw = true;
		}
		// cout << reachX << reachY << reachZ << reachYaw << endl;
		if (reachX and reachY and reachZ and reachYaw){
			return true;
		}
		else{
			return false;
		}
	}

	bool flightBase::isReach(const geometry_msgs::PoseStamped& poseTgt, double dist, bool useYaw){
		double targetX, targetY, targetZ, targetYaw, currX, currY, currZ, currYaw;
		targetX = poseTgt.pose.position.x;
		targetY = poseTgt.pose.position.y;
		targetZ = poseTgt.pose.position.z;
		targetYaw = AutoFlight::rpy_from_quaternion(poseTgt.pose.orientation);
		currX = this->odom_.pose.pose.position.x;
		currY = this->odom_.pose.pose.position.y;
		currZ = this->odom_.pose.pose.position.z;
		currYaw = AutoFlight::rpy_from_quaternion(this->odom_.pose.pose.orientation);
		
		bool reachX, reachY, reachZ, reachYaw;
		reachX = std::abs(targetX - currX) < dist;
		reachY = std::abs(targetY - currY) < dist;
		reachZ = std::abs(targetZ - currZ) < dist;
		if (useYaw){
			reachYaw = std::abs(targetYaw - currYaw) < 0.1;
		}
		else{
			reachYaw = true;
		}

		if (reachX and reachY and reachZ and reachYaw){
			return true;
		}
		else{
			return false;
		}
	}
}
