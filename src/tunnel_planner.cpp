#include "robospect_planner/tunnel_planner.h"

TunnelPlanner::TunnelPlanner(ros::NodeHandle h) : nh_(h), private_nh_("~"), desired_freq_(100.0),
						  Component(desired_freq_),
						  action_server_goto(nh_, ros::this_node::getName(), false)
						  // boost::bind(&purepursuit_planner_node::executeCB, this, _1), false)
{
  bRunning = false;
  
  ROSSetup();
  
  dLookAhead_ = d_lookahead_min_;
  dLinearSpeed_ = 0;
  approach_state_=false;
  pose2d_robot.x = pose2d_robot.y = pose2d_robot.theta = 0.0;
  bEnabled = true;
  bCancel = false;
  
  sComponentName.assign("tunnel_planner");
  iState = INIT_STATE;

  tm_=new TunnelMap("tunnel_map");

  ROS_INFO("TunnelPlanner initialized");
}

/*! \fn void TunnelPlanner::reconfigure_callback(robospect_planner::PlannerConfig &config, uint32_t level)
 *  \brief callback called when dynamic parameters are modified
 * 
*/
void TunnelPlanner::reconfigureCallback(robospect_planner::PlannerConfig &config, uint32_t level) {
  //
  if(level == 0){
	  ROS_INFO("Reconfigure Request: %.3f %.3f %.3lf", config.d_lookahead_min, config.d_lookahead_max, config.desired_distance);
	  this->d_lookahead_min_ = config.d_lookahead_min;
	  this->d_lookahead_max_ = config.d_lookahead_max;
	  this->desired_distance_ = config.desired_distance;
	  this->Kr = config.kr;
  }else if(level == 1){
	//ROS_INFO("Reconfigure Request: %s", config.preferred_wall_side.c_str());	
	tm_->setParams(config.footprint_width, config.crane_length, config.footprint_length, config.lateral_clearance, config.obstacle_range, config.lookahead_distance,
	config.preferred_wall_side, config.ransac_threshold, config.max_y_dist);
  }
  
}



void TunnelPlanner::ROSSetup(){
  string s_command_type;
		
  private_nh_.param<string>("odom_topic", odom_topic_, "/odom");
  private_nh_.param<string>("cmd_topic_vel", cmd_topic_vel_,"/cmd_vel");
  private_nh_.param<string>("position_source", position_source_, "ODOM");
  private_nh_.param("desired_freq", desired_freq_, desired_freq_);
  private_nh_.param<string>("target_frame", target_frame_, "/base_footprint");
  private_nh_.param<string>("command_type", s_command_type, COMMAND_ACKERMANN_STRING);
  private_nh_.param("d_lookahead_min", d_lookahead_min_, D_LOOKAHEAD_MIN);
  private_nh_.param("d_lookahead_max", d_lookahead_max_, D_LOOKAHEAD_MAX);
  private_nh_.param("d_dist_wheel_to_center", d_dist_wheel_to_center_, D_WHEEL_ROBOT_CENTER);
  private_nh_.param("desired_distance", desired_distance_, DEFAULT_DESIRED_DISTANCE);
  private_nh_.param("max_speed", max_speed_, MAX_SPEED);
  private_nh_.param("kr", Kr, DEFAULT_KR);

  ROS_INFO("%s::ROSSetup(): odom_topic = %s, command_topic_vel = %s, position source = %s, desired_hz=%.1lf, min_lookahead = %.1lf, max_lookahead = %.1lf, kr = %.2lf, command_type = %s", sComponentName.c_str(), odom_topic_.c_str(), cmd_topic_vel_.c_str(), position_source_.c_str(), desired_freq_, d_lookahead_min_, d_lookahead_max_, Kr, s_command_type.c_str());
		
  // From Component class
  threadData.dDesiredHz = desired_freq_;
  
  if(s_command_type.compare(COMMAND_ACKERMANN_STRING) == 0){
    command_type_ = COMMAND_ACKERMANN;
  }else if(s_command_type.compare(COMMAND_TWIST_STRING) == 0){
    command_type_ = COMMAND_TWIST;
    d_dist_wheel_to_center_ = 1.0;
  }else{
    command_type_ = COMMAND_TWIST;  // default value
    d_dist_wheel_to_center_ = 1.0;
  }
         
  if(command_type_ == COMMAND_ACKERMANN){
    // Publish through the node handle Ackerman type messages to the command vel topic
    vel_pub_ = private_nh_.advertise<ackermann_msgs::AckermannDriveStamped>(cmd_topic_vel_, 1);			
  }else{
    // Publish through the node handle Twist type messages to the command vel topic
    vel_pub_ = private_nh_.advertise<geometry_msgs::Twist>(cmd_topic_vel_, 1);	
  }
		
  if(position_source_ == "MAP"){
    ui_position_source = MAP_SOURCE;
    tranform_map_pub_ = private_nh_.advertise<geometry_msgs::TransformStamped>("map_localization", 100);
  }else 
    ui_position_source = ODOM_SOURCE;
    
  odom_sub_ = private_nh_.subscribe<nav_msgs::Odometry>(odom_topic_, 1, &TunnelPlanner::OdomCallback, this);

  pub_next_point_ = private_nh_.advertise<geometry_msgs::PointStamped>("next_point", 100);
  pub_next_pose_ = private_nh_.advertise<geometry_msgs::PoseStamped>("next_pose", 100);
  state_pub_ = private_nh_.advertise<robospect_planner::State>("state", 1);

	         	
  // Action server 
  action_server_goto.registerGoalCallback(boost::bind(&TunnelPlanner::GoalCB, this)); //DO NOTHING!!!
  action_server_goto.registerPreemptCallback(boost::bind(&TunnelPlanner::PreemptCB, this));
  
  // Reconfigure server
  r_callback = boost::bind(&TunnelPlanner::reconfigureCallback, this, _1, _2);
  reconfigure_server.setCallback(r_callback);
  
}

void TunnelPlanner::OdomCallback(const nav_msgs::Odometry::ConstPtr& odom_msg)
{
  last_odom_time_ = ros::Time::now();
  if(ui_position_source == ODOM_SOURCE){		
    // converts the odom to pose 2d only if the robot position source is set to odometry
    // when using MAP_SOURCE, the robot pose update is done in AllState()
    pose2d_robot.x = odom_msg->pose.pose.position.x;
    pose2d_robot.y = odom_msg->pose.pose.position.y;
    pose2d_robot.theta = tf::getYaw(odom_msg->pose.pose.orientation);
  }
		
  // Get the linear speed
  dLinearSpeed_= odom_msg->twist.twist.linear.x;
  //ROS_INFO("Received odom message!!");
}

void TunnelPlanner::ControlThread()
{
  ROS_INFO("TunnelPlanner::ControlThread(): Init");
  ros::Rate r(desired_freq_); 
  			
  while(ros::ok()) {			
    switch(iState){
    case INIT_STATE:
      InitState();
      break;
      
    case STANDBY_STATE:
      StandbyState();
      break;
      
    case READY_STATE:
      ReadyState();
      break;
      
    case SHUTDOWN_STATE:
      ShutDownState();
      break;
      
    case EMERGENCY_STATE:
      EmergencyState();
      break;
      
    case FAILURE_STATE:
      FailureState();
      break;     
    }
    
    AllState();
    
    ros::spinOnce();
    r.sleep();
  }
  ShutDownState();
  ROS_INFO("TunnelPlanner::ControlThread(): End");
}

void TunnelPlanner::InitState(){
  // ROS_INFO("TunnelPlanner::InitState");
  if(bInitialized && bRunning){ //Flag from the Component class
    if(CheckOdomReceive() == 0)
      SwitchToState(STANDBY_STATE);
  }else{
    if(!bInitialized)
      Setup();
    if(!bRunning)
      Start();
  }
  //ROS_INFO("bInitialized: %u, bRunning: %u", bInitialized, bRunning);
}

ReturnValue TunnelPlanner::Setup(){
  // Checks if has been initialized
  if(bInitialized){
    ROS_INFO("TunnelPlanner::Setup: Already initialized");	
    return INITIALIZED;
  }
  // Starts action server 
  action_server_goto.start();
  bInitialized = true;  
  return OK;
}

ReturnValue TunnelPlanner::Start(){
  if(bRunning){
    ROS_INFO("TunnelPlanner::Start: the component's thread is already running");
    return THREAD_RUNNING;
  }
  bRunning = true;
  return OK;
}

int TunnelPlanner::CheckOdomReceive(){		
  if((ros::Time::now() - last_odom_time_).toSec() > ODOM_TIMEOUT_ERROR)
    return -1;
  else{
    if((ui_position_source == MAP_SOURCE) && ((ros::Time::now() - last_map_time_).toSec() > MAP_TIMEOUT_ERROR))
      return -1;
    else return 0;
  }
}

void TunnelPlanner::StandbyState(){
  //ROS_INFO("TunnelPlanner::StandbyState, isFree: %u, bCancel: %u, bEnabled: %u",);
  if(CheckOdomReceive() == -1){
	ROS_ERROR("%s::StandbyState: Not receiving robot's odometry at the desired frequency", sComponentName.c_str());
    SwitchToState(EMERGENCY_STATE);
  }else{
    if(bEnabled && !bCancel && tm_->isFree()){
      //If we have one or more paths and they were correctly merged, switch to Ready
      if(pathCurrent_.Size() > 0 || MergePath() == OK){    //In MergePath we check that pathCurrent_.Size is >2. Not understand the sense of the first check!!! 
		ROS_INFO("%s::StandbyState: route available, switching to ReadyState", sComponentName.c_str());
		SwitchToState(READY_STATE);
      }else{
		  if(pathCurrent_.Size() > 0 && tm_->isFree()){
			  	ROS_WARN("%s::StandbyState: route available but obstacle detected", sComponentName.c_str());

		  }
	  }
    }
    
    //ROS_INFO("%s::StandbyState: Enabled? %s, Cancel? %s, Free? %s",sComponentName.c_str(), bEnabled?"True":"False", bCancel?"True":"False", tm_->isFree()?"True":"False");
  }
}

/*! \fn int TunnelPlanner::CalculateDirectionSpeed(geometry_msgs::Pose2D robot_position, geometry_msgs::Pose2D target_position)
*	\brief Calcula el sentido de movimiento de una ruta, dependiendo de la posición inicial y el ángulo del robot
*	\return 1 si el sentido es positivo
*	\return -1 si el sentido es negativo
*/
int TunnelPlanner::CalculateDirectionSpeed(geometry_msgs::Pose2D robot_position, geometry_msgs::Pose2D target_position){
	int ret = 1;
	double alpha = robot_position.theta;
	double x =	robot_position.x, y = robot_position.y;
	double ux, uy, vx, vy;
	double beta = 0.0;
	static int last_direction = 0;
	static double pi_medios = M_PI / 2.0, max_diff = 5.0 * M_PI / 180.0;
	int iCase = 0;

	//
	// si la posicion objetivo es la misma del robot, devolvemos 0
	if( (target_position.x == x) && (target_position.y == y) ){
		return 0;
	}
	// Cálculo del vector director del robot respecto al sistema de coordenadas del robot
	ux = cos(alpha);
	uy = sin(alpha);
	// Cálculo del vector entre el punto objetivo y el robot
	vx = target_position.x - x;
	vy = target_position.y - y;

	// Cálculo del ángulo entre el vector director y el vector al punto objetivo
	beta = acos( (ux * vx + uy * vy) / ( sqrt(ux*ux + uy*uy) * sqrt(vx*vx + vy*vy) ) );

	// Devolvemos valor dependiendo del ángulo entre la orientación del robot y la posición objetivo (radianes)
	// Tendremos en cuenta el valor del sentido de avance de la última ruta.
	if(fabs(beta) <= pi_medios){
		// Calculo inicial de direccion
		if(last_direction == 0)
			ret = 1;
		else {
			ret = 1;

			if( fabs(beta - pi_medios) <= max_diff){
				if(last_direction != ret){
					iCase = 1;
					ret = -1;

				}else {
					iCase = 2;
				}
			}

		}
	}else{
		// Calculo inicial de direccion
		if(last_direction == 0)
			ret = -1;
		else {
			ret = -1;
			if(fabs(beta - pi_medios) <= max_diff){
				if(last_direction != ret){
					ret = 1;
					iCase = 3;
				}else{
					iCase = 4;
				}
			}

		}
	}

	ROS_INFO("%s:CalculateDirectionSpeed:  case %d. Beta = %.2lf. Diff = %.2lf. Last direction = %d, new direction = %d", sComponentName.c_str(),
			 iCase, beta*180.0/M_PI, (beta - pi_medios)*180.0/M_PI, last_direction, ret);

	last_direction = ret;
	return ret;
}

ReturnValue TunnelPlanner::MergePath(){  //TODO: why do we need a queue of paths?
  Waypoint new_waypoint, wFirst, wLast;
  Path aux;
  int direction = 0;
		
  if(action_server_goto.isNewGoalAvailable()){
    goto_goal.target = action_server_goto.acceptNewGoal()->target; // Reads points from action server
    if(goto_goal.target.size() > 0){
		if(goto_goal.target.size() > 1){	// Tries to use the second point of the route
			direction = CalculateDirectionSpeed(this->pose2d_robot, goto_goal.target[1].pose);
		  }else{
			direction = CalculateDirectionSpeed(this->pose2d_robot, goto_goal.target[0].pose);
		}
		ROS_INFO("%s::MergePath: Direction speed = %d", sComponentName.c_str(), direction);
							
		for(int i = 0; i < goto_goal.target.size(); i++){
			ROS_INFO("%s::MergePath: Point %d (%.3lf, %.3lf, %.3lf) speed = %.3lf", sComponentName.c_str(), i,  goto_goal.target[i].pose.x, 
							goto_goal.target[i].pose.y, goto_goal.target[i].pose.theta, goto_goal.target[i].speed);
							
			new_waypoint.dX = goto_goal.target[i].pose.x;
			new_waypoint.dY = goto_goal.target[i].pose.y;
			new_waypoint.dA = goto_goal.target[i].pose.theta;
			// Depending on the calculated motion direction, applies positive or negative speed
			if(direction == 1){ 
				new_waypoint.dSpeed = fabs(goto_goal.target[i].speed);
			}else{
			  new_waypoint.dSpeed = -fabs(goto_goal.target[i].speed);
			}
							
			pathFilling_.AddWaypoint(new_waypoint);							
		}
		  
		if(pathFilling_.Optimize(TURN_RADIUS) != OK) 
			ROS_ERROR("%s::MergePath: Error optimizing the path", sComponentName.c_str());

		ROS_INFO("Printing the path after the optimization");
		pathFilling_.Print();

		// Adds the new path to the queue
		qPath.push(pathFilling_);
		// Clears temporary path object
		pathFilling_.Clear();
					
		goto_feedback.percent_complete = 0.0;  // Inits the feedback percentage
		goto_result.route_result = 100;			// Inits the result value
					
		  // Only if exists any path into the queue
		if(qPath.size() > 0){					
			aux = qPath.front();
			aux.GetWaypoint(0, &wFirst);
			aux.BackWaypoint(&wLast);

			ROS_INFO("%s::MergePath: Adding new %d points from (%.2lf, %.2lf) to (%.2lf, %.2lf) and %d magnets",
				 sComponentName.c_str(), aux.NumOfWaypoints() ,wFirst.dX, wFirst.dY, wLast.dX, wLast.dY, aux.NumOfMagnets());
			ROS_INFO("%s::MergePath: Current number of points = %d and magnets = %d",
				 sComponentName.c_str(), pathCurrent_.NumOfWaypoints() , pathCurrent_.NumOfMagnets());
							
			// Adds the first path in the queue to the current path
			pathCurrent_+=qPath.front();
							
			// Needs at least two points
			if(pathCurrent_.NumOfWaypoints() < 2){		
			  if(pathCurrent_.CreateInterpolatedWaypoint(this->pose2d_robot) == ERROR){
				ROS_ERROR("%s::MergePath: Error adding an extra point", sComponentName.c_str());
			  }
			}
							
			ROS_INFO("%s::MergePath: New number of points in current path (AFTER ADDING NEW POINTS AND INTERPOLATING THEM)= %d and magnets = %d", sComponentName.c_str(), pathCurrent_.NumOfWaypoints() , pathCurrent_.NumOfMagnets());
			pathCurrent_.Print();
			qPath.pop(); // Pops the extracted path
			goto_goal.target.clear();	// removes current goals		
			return OK;
		}
    }
  }
  return ERROR;
}

void TunnelPlanner::ReadyState(){
  //SECURITY CHECKS
  if(CheckOdomReceive() == -1){
    SetRobotSpeed(0.0, 0.0);
    SwitchToState(EMERGENCY_STATE);
    return;
  }
  if(!bEnabled){
    ROS_INFO("%s::ReadyState: Motion is disabled", sComponentName.c_str());
    SetRobotSpeed(0.0, 0.0);
    SwitchToState(STANDBY_STATE);
    return;
  }
  if(bCancel){
    ROS_INFO("%s::ReadyState: Cancel requested", sComponentName.c_str());
    SetRobotSpeed(0.0, 0.0);
    SwitchToState(STANDBY_STATE);
    return;
  }
  if(!tm_->isFree()){
    ROS_INFO("%s::ReadyState: Obstacle detected, cancel requested", sComponentName.c_str());
    SetRobotSpeed(0.0, 0.0);
    SwitchToState(STANDBY_STATE);
    return;
  }

  //PLANNER CALL
  int ret = PurePursuit();
  
  if(ret == -1){
    ROS_ERROR("%s::ReadyState: Error on PurePursuit", sComponentName.c_str());
    bCancel = true;	//Activates the flag to cancel the mision
    SetRobotSpeed(0.0, 0.0);
    goto_result.route_result = -1;
    goto_feedback.percent_complete = 100.0;	// Set the percent to 100% to complete the action
    SwitchToState(STANDBY_STATE);
    
  }else if(ret == 1){
    ROS_INFO("%s::ReadyState: Route correctly finished", sComponentName.c_str());
    SetRobotSpeed(0.0, 0.0);
    goto_result.route_result = 0;
    goto_feedback.percent_complete = 100.0;	// Set the percent to 100% to complete the action
    SwitchToState(STANDBY_STATE);
  }
  // TODO: update the percent while the mision is ongoing
}
	
void TunnelPlanner::SetRobotSpeed(double speed, double angle){
  if(command_type_ == COMMAND_ACKERMANN){
    ackermann_msgs::AckermannDriveStamped ref_msg;
    
    ref_msg.header.stamp = ros::Time::now();
    ref_msg.drive.jerk = 0.0; 
    ref_msg.drive.acceleration = 0.0; 
    ref_msg.drive.steering_angle_velocity = 0.0;
    ref_msg.drive.steering_angle = angle;
    ref_msg.drive.speed = speed;
    
    vel_pub_.publish(ref_msg);
  }else{
    geometry_msgs::Twist ref_msg;
    ref_msg.angular.x = 0.0;  ref_msg.angular.y = 0.0; ref_msg.angular.z = angle;
    ref_msg.linear.x = speed;   ref_msg.linear.y = 0.0; ref_msg.linear.z = 0.0;
    vel_pub_.publish(ref_msg);
  }
}

void TunnelPlanner::ShutDownState(){
  if(bRunning)
    Stop();
  else if(bInitialized)
    ShutDown();
}

ReturnValue TunnelPlanner::Stop(){
  if(!bRunning){
    ROS_INFO("TunnelPlanner::Stop: Thread not running");
    return THREAD_NOT_RUNNING;
  }		
  bRunning = false;
  return OK;
}
	
void TunnelPlanner::EmergencyState(){
  if(CheckOdomReceive() == 0){
    SwitchToState(STANDBY_STATE);
    return;
  }
}
	
void TunnelPlanner::AllState(){
  // Only if we use the map as source for localization
  if(ui_position_source == MAP_SOURCE){		
    try{
      listener.lookupTransform("/map", target_frame_, ros::Time(0), transform_);
      geometry_msgs::TransformStamped msg;
      tf::transformStampedTFToMsg(transform_, msg);
      pose2d_robot.x = msg.transform.translation.x;
      pose2d_robot.y = msg.transform.translation.y;
      pose2d_robot.theta = tf::getYaw(msg.transform.rotation);  
      
      last_map_time_ = ros::Time::now();
      msg.header.stamp = last_map_time_;
      tranform_map_pub_.publish(msg);
    }catch (tf::TransformException ex){
      //ROS_ERROR("%s::AllState: %s", sComponentName.c_str(), ex.what());
    }
  }
  
  robospect_planner::State state_msg;
  state_msg.state.state = stateToPlannerState(iState);
  state_msg.state.desired_freq = desired_freq_;
  state_msg.state.state_description = GetStateString(iState);
  
  state_pub_.publish(state_msg);
  
  AnalyseCB();	// Checks action server state
  
  if(bCancel)	// Cancel current path if required
    CancelPath();
}

void TunnelPlanner::AnalyseCB(){
  if (!action_server_goto.isActive()){
    //ROS_INFO("%s::AnalyseCB: Action server not active", sComponentName.c_str());
    return;
  }
  
  //TODO: update goto_feedback value here	
  action_server_goto.publishFeedback(goto_feedback);
  
  if(goto_result.route_result == -1){
    action_server_goto.setAborted(goto_result);
    ROS_INFO("%s::AnalyseCB: Action finished (aborted)", sComponentName.c_str());
  }else if(goto_result.route_result == 0 and goto_feedback.percent_complete == 100.0){
    //action_server_goto.setSucceeded(goto_result);
    action_server_goto.setSucceeded(goto_result);
    ROS_INFO("%s::AnalyseCB: Action finished", sComponentName.c_str());
  }
}

void TunnelPlanner::CancelPath(){		
  pathCurrent_.Clear();	// Clears current path
  pathFilling_.Clear();	// Clears the auxiliary path
  while(!qPath.empty())	// Clears the queue of paths
    qPath.pop();
		
  bCancel = false;
  // Cancels current action
  ROS_INFO("%s::CancelPath: action server preempted", sComponentName.c_str());
  action_server_goto.setPreempted();
}

int TunnelPlanner::PurePursuit(){
  Waypoint last_waypoint, next_waypoint;
  double dAuxSpeed, dAuxDist, wref;
  double yaw;
  double dx1,dy1,x1,y1;
  double desired_yaw_,actual_distance_;
  double curv;
  double dth;
  double direction = 1;
  
  geometry_msgs::Pose2D current_position = this->pose2d_robot;		
  geometry_msgs::Pose2D next_position;
 
  yaw = current_position.theta;

  if(pathCurrent_.GetNextWaypoint(&next_waypoint) == ERROR){
    ROS_ERROR("%s::PurePursuit: Error getting next waypoint in the path", sComponentName.c_str());
    return -1;
  }
  if(next_waypoint.dSpeed > 0.0)
	direction = 1;
  else
	direction = -1;
  
  if ((next_waypoint.dSpeed > 0.0 and current_position.x>=next_waypoint.dX) or ((next_waypoint.dSpeed < 0.0 and current_position.x<=next_waypoint.dX))){  
    //TODO: improve this check (it does not work when the robot is moving in the negative-x direction)	
    pathCurrent_.NextWaypoint();
    if(pathCurrent_.GetCurrentWaypointIndex() == ERROR){
      ROS_ERROR("%s::PurePursuit: Error setting next waypoint", sComponentName.c_str());
      return ERROR;
    }
  }

  UpdateLookAhead();
  
  double desired_yaw=tm_->getWallOrientation();
  //! If true, the robot is in the approach phase (i.e., the robot is approaching the wall)
  if (fabs(current_position.theta-desired_yaw)>0.25 && !approach_state_){
	tm_->setState(APPROACH_STATE);
	ROS_INFO("PurePursuit(): Approaching the wall");
	approach_state_=true;
  }else if (fabs(current_position.theta-desired_yaw)<0.25 && approach_state_){
	tm_->setState(NAVIGATION_STATE);
	ROS_INFO("PurePursuit(): Wall reached, start navigation");
	approach_state_=false;
  }

  double actual_distance=tm_->getWallDistance();
  //TODO: check tf: these values are referred to the base_footprint frame (laser scans needs to be expressed in base_footprint frame)
  ROS_INFO("PurePursuit: Distance to wall = %.3lf(desired = %.3lf), desired_yaw = %.3lf, lookahead = %.3lf", actual_distance, desired_distance_, desired_yaw, dLookAhead_);
  
  // We need to correct the distance instead of aligning the orientation
  dx1=(-direction)*(actual_distance-desired_distance_)*sin(desired_yaw);
  dy1=(actual_distance-desired_distance_)*cos(desired_yaw);
  ROS_INFO("PurePursuit: dx1 = %.3lf, dy1 = %.3lf", dx1, dy1);
  //Already in the robot frame
  next_position.x=cos(desired_yaw)*dLookAhead_+dx1;
  next_position.y=sin(desired_yaw)*dLookAhead_+dy1;
  next_position.theta=desired_yaw;

  geometry_msgs::PointStamped next_point;
  geometry_msgs::PoseStamped next_pose;

  next_point.header.frame_id="base_footprint";
  next_point.header.stamp=ros::Time::now();
  next_point.point.x=next_position.x;
  next_point.point.y=next_position.y;
  next_point.point.z=0.0;

  next_pose.header.frame_id="base_footprint";
  next_pose.header.stamp=ros::Time::now();
  next_pose.pose.position.x=next_position.x;
  next_pose.pose.position.y=next_position.y;
  next_pose.pose.position.z=0.0;
  next_pose.pose.orientation = tf::createQuaternionMsgFromYaw(next_position.theta);

  pub_next_point_.publish(next_point);
  pub_next_pose_.publish(next_pose);

  //ROS_INFO("Next position in robot frame. x: %5.5f, y: %5.5f theta: %5.5f", next_position.x, next_position.y, next_position.theta);

  x1 = next_position.x;
  y1 = next_position.y;
  if ((x1*x1 + y1*y1) == 0)
    curv = 0;
  else
    curv = (2.0 / (x1*x1 + y1*y1)) * y1; //Original

  wref = atan(d_dist_wheel_to_center_/(1.0/curv));
  
  if(pathCurrent_.BackWaypoint(&last_waypoint) == ERROR){
    ROS_ERROR("%s::PurePursuit: Error getting the last point in the path", sComponentName.c_str());
    return -1;
  }

  dAuxDist = Dist(current_position.x, current_position.y, last_waypoint.dX, last_waypoint.dY);
  
  dAuxSpeed = next_waypoint.dSpeed;

  if (dAuxSpeed >= 0)
    dth = next_position.theta - current_position.theta;
  else
    dth = -(next_position.theta + Pi - current_position.theta);

  // normalize
  radnorm(&dth);
  double aux_wref = wref;
  wref += Kr * dth;
		
  //Check speeds limits depending of distance or speed restrictions 
  if(fabs(dAuxSpeed) > max_speed_){ 
    if(dAuxSpeed > 0)
      dAuxSpeed = max_speed_;
    else
      dAuxSpeed = -max_speed_;
  }
  	
  if(dAuxDist <= FIRST_DECELERATION_DISTANCE){
    if( (dAuxSpeed < 0.0) && (dAuxSpeed < FIRST_DECELERATION_MAXSPEED))
      dAuxSpeed = -FIRST_DECELERATION_MAXSPEED;
    else if( (dAuxSpeed > 0.0) && (dAuxSpeed > FIRST_DECELERATION_MAXSPEED) )
      dAuxSpeed = FIRST_DECELERATION_MAXSPEED;
    
  }else if(dAuxDist <= SECOND_DECELERATION_DISTANCE){
    if( (dAuxSpeed < 0.0) && (dAuxSpeed < -SECOND_DECELERATION_MAXSPEED) )
      dAuxSpeed = -SECOND_DECELERATION_MAXSPEED;
    else if( (dAuxSpeed > 0.0) && (dAuxSpeed > SECOND_DECELERATION_MAXSPEED) )
      dAuxSpeed = SECOND_DECELERATION_MAXSPEED;
  }
  
  SetRobotSpeed(dAuxSpeed, wref);
    
  // When the robot is on the way towards the last waypoint, checks the distance to the end
  if( pathCurrent_.GetCurrentWaypointIndex() >= (pathCurrent_.NumOfWaypoints() - 2) ){
    pathCurrent_.BackWaypoint(&last_waypoint);
    double ddist2 = fabs(current_position.x-last_waypoint.dX);
    if (ddist2 < WAYPOINT_POP_DISTANCE_M) {
      SetRobotSpeed(0.0, 0.0);
      ROS_INFO("%s::PurePursuit: target position reached (%lf, %lf, %lf). Ending current path",
	       sComponentName.c_str(), current_position.x, current_position.x, current_position.theta*180.0/Pi);
      pathCurrent_.Clear();
      return 1;
    }
  }
  return 0;
}

void TunnelPlanner::UpdateLookAhead(){
  double aux_lookahead = fabs(dLinearSpeed_);
  double desired_lookahead = 0.0;
  double inc = 0.01;	// lookahead increment
  
  if(aux_lookahead < d_lookahead_min_)
    desired_lookahead = d_lookahead_min_;
  else if(aux_lookahead > d_lookahead_max_)
    desired_lookahead = d_lookahead_max_;
  else{
    desired_lookahead = aux_lookahead;
  }
  
  if((desired_lookahead - 0.001) > dLookAhead_){
    dLookAhead_+= inc;
  }else if((desired_lookahead + 0.001) < dLookAhead_)
    dLookAhead_-= inc;
}

double TunnelPlanner::Dist(double x1, double y1, double x2, double y2) {
  double diff_x = (x2 - x1);
  double diff_y = (y2 - y1);
  return sqrt( diff_x*diff_x + diff_y*diff_y );
}

int TunnelPlanner::stateToPlannerState(int state){
	switch(state){
		case INIT_STATE:
			return robotnik_msgs::State::INIT_STATE;
		break;

		case STANDBY_STATE:
			return robotnik_msgs::State::STANDBY_STATE;
		break;

		case READY_STATE:
			return robotnik_msgs::State::READY_STATE;
		break;

		case SHUTDOWN_STATE:
			return robotnik_msgs::State::SHUTDOWN_STATE;
		break;

		case EMERGENCY_STATE:
			return robotnik_msgs::State::EMERGENCY_STATE;
		break;

		case FAILURE_STATE:
			return robotnik_msgs::State::EMERGENCY_STATE;
		break;    
	}
	return robotnik_msgs::State::UNKOWN_STATE;
};
