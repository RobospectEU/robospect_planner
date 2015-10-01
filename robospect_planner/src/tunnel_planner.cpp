TunnelPlanner::TunnelPlanner(ros::NodeHandle h) : nh_(h), private_nh_("~"), desired_freq_(100.0),
						  Component(desired_freq_),
						  action_server_goto(node_handle_, ros::this_node::getName(), false)
						  // boost::bind(&purepursuit_planner_node::executeCB, this, _1), false)
{
  bRunning = false;
  
  ROSSetup();
  
  dLookAhead_ = d_lookahear_min_;
  dLinearSpeed_ = 0;
  pose2d_robot.x = pose2d_robot.y = pose2d_robot.theta = 0.0;
  bEnabled = true;
  bCancel = false;
  bObstacle = true;
  
  sComponentName.assign("tunnel_planner");
  iState = INIT_STATE;
}

void TunnelPlanner::ROSSetup(){
  string s_command_type;
		
  private_nh_.param<string>("odom_topic", odom_topic_, "/odom");
  private_nh_.param<string>("pcl_topic", pcl_topic_, "/scan_cloud");
  private_nh_.param<string>("cmd_topic_vel", cmd_topic_vel_,"/cmd_vel");
  private_nh_.param<string>("position_source", position_source_, "ODOM");
  private_nh_.param("desired_freq", desired_freq_, desired_freq_);
  private_nh_.param<string>("target_frame", target_frame_, "/base_footprint");
  private_nh_.param<string>("command_type", s_command_type, COMMAND_ACKERMANN_STRING);
  private_nh_.param("d_lookahear_min", d_lookahear_min_, D_LOOKAHEAD_MIN);
  private_nh_.param("d_lookahear_max", d_lookahear_max_, D_LOOKAHEAD_MAX);
  private_nh_.param("d_dist_wheel_to_center", d_dist_wheel_to_center_, D_WHEEL_ROBOT_CENTER);
  private_nh_.param("max_speed", max_speed_, MAX_SPEED); 
  private_nh_.param("kr", Kr, AGVS_DEFAULT_KR); //TODO

  ROS_INFO("%s::ROSSetup(): odom_topic = %s, command_topic_vel = %s, position source = %s, desired_hz=%.1lf, min_lookahead = %.1lf, max_lookahead = %.1lf, kr = %.2lf, command_type = %s", sComponentName.c_str(), odom_topic_.c_str(), cmd_topic_vel_.c_str(), position_source_.c_str(), desired_freq_, d_lookahear_min_, d_lookahear_max_, Kr, s_command_type.c_str());
		
  // From Component class
  threadData.dDesiredHz = desired_freq_;
  
  if(s_command_type.compare(COMMAND_ACKERMANN_STRING) == 0){
    command_type_ = COMMAND_ACKERMANN;
  }else if(s_command_type.compare(COMMAND_TWIST_STRING) == 0){
    command_type_ = COMMAND_TWIST;
  }else{
    command_type_ = COMMAND_TWIST;  // default value
    d_dist_wheel_to_center_ = 1.0;
  }
         
  if(command_type == COMMAND_ACKERMANN){
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
  pcl_sub_ = private_nh_.subscribe<sensor_msgs::PointCloud2>(pcl_topic_, 1, &TunnelPlanner::pclCallback, this);
    
	         	
  // Action server 
  action_server_goto.registerGoalCallback(boost::bind(&TunnelPlanner::GoalCB, this)); //DO NOTHING!!!
  action_server_goto.registerPreemptCallback(boost::bind(&TunnelPlanner::PreemptCB, this));
}

void TunnelPlanner::OdomCallback(const nav_msgs::Odometry::ConstPtr& odom_msg)
{
  last_command_time_ = ros::Time::now();
  if(ui_position_source == ODOM_SOURCE){		
    // converts the odom to pose 2d only if the robot position source is set to odometry
    // when using MAP_SOURCE, the robot pose update is done in AllState()
    pose2d_robot.x = odom_msg->pose.pose.position.x;
    pose2d_robot.y = odom_msg->pose.pose.position.y;
    pose2d_robot.theta = tf::getYaw(odom_msg->pose.pose.orientation);
  }
		
  // Get the linear speed
  dLinearSpeed_= odometry_robot.twist.twist.linear.x;
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
  if(bInitialized && bRunning){ //Flag from the Component class
    if(CheckOdomReceive() == 0)
      SwitchToState(STANDBY_STATE);
  }else{
    if(!bInitialized)
      Setup();
    if(!bRunning)
      Start();
  }		  
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
  if(CheckOdomReceive() == -1)
    SwitchToState(EMERGENCY_STATE);
  else{
    if(bEnabled && !bCancel && !t_map.bObstacle){
      //If we have one or more paths and they were correctly merged, switch to Ready
      if(pathCurrent_.Size() > 0 || MergePath() == OK){    //In MergePath we check that pathCurrent_.Size is >2. Not understand the sense of the first check!!! 
	ROS_INFO("%s::StandbyState: route available, switching to ReadyState", sComponentName.c_str());
	SwitchToState(READY_STATE);
      }
    }
  }
}

ReturnValue TunnelPlanner::MergePath(){  //TODO: Is this a sort of GoalCB??? NOT COMPLEATELY CLEAR THE ROLE OF THE QUEUE OF PATHS!!
  Waypoint new_waypoint, wFirst, wLast;
  Path aux;
		
  if(action_server_goto.isNewGoalAvailable()){
    goto_goal.target = action_server_goto.acceptNewGoal()->target; // Reads points from action server
    if(goto_goal.target.size() > 0){
      /*      if(goto_goal.target.size() > 1){	// Tries to use the second point of the route
	direction = CalculateDirectionSpeed(goto_goal.target[1].pose);
      }else{
	direction = CalculateDirectionSpeed(goto_goal.target[0].pose);
	}*/ //TODO:: for now not used directional indicator!!
						
      for(int i = 0; i < goto_goal.target.size(); i++){
	ROS_INFO("%s::MergePath: Point %d (%.3lf, %.3lf, %.3lf) speed = %.3lf", sComponentName.c_str(), i,  goto_goal.target[i].pose.x, 
					goto_goal.target[i].pose.y, goto_goal.target[i].pose.theta, goto_goal.target[i].speed);
					
	new_waypoint.dX = goto_goal.target[i].pose.x;
	new_waypoint.dY = goto_goal.target[i].pose.y;
	new_waypoint.dA = goto_goal.target[i].pose.theta;
	// Depending on the calculated motion direction, applies positive or negative speed
	//if(direction == 1){ TODO!!! Does it have to turn??
	new_waypoint.dSpeed = fabs(goto_goal.target[i].speed);
	  /*}else{
	  new_waypoint.dSpeed = -fabs(goto_goal.target[i].speed);
	  }*/
					
	pathFilling_.AddWaypoint(new_waypoint);							
      }
      
      if(pathFilling_.Optimize(AGVS_TURN_RADIUS) != OK) //TODO!!! SOLVE!!!!
	ROS_ERROR("%s::MergePath: Error optimizing the path", sComponentName.c_str());

      ROS_INFO("Printing the path after the optimization");
      pathFilling_.Print();
      
      // Adds the new path to the queue
      qPath.push(pathFilling_);
      // Clears temporary path object
      pathFilling_.Clear();
    				
      goto_feedback.percent_complete = 0.0;  // Inits the feedback percentage
				
      // Only if exists any path into the queue
      if(qPath.size() > 0){					
	aux = qPath.front();
	aux.GetWaypoint(0, &wFirst);
	aux.BackWaypoint(&wLast);

	ROS_INFO("%s::MergePath: Adding new %d points from (%.2lf, %.2lf) to (%.2lf, %.2lf) and %d magnets", sComponentName.c_str(), aux.NumOfWaypoints() ,wFirst.dX, wFirst.dY, wLast.dX, wLast.dY, aux.NumOfMagnets());
	ROS_INFO("%s::MergePath: Current number of points = %d and magnets = %d", sComponentName.c_str(), pathCurrent_.NumOfWaypoints() , pathCurrent_.NumOfMagnets());
					
	// Adds the first path in the queue to the current path
	pathCurrent_+=qPath.front();
					
	// Needs at least two points
	if(pathCurrent_.NumOfWaypoints() < 2){		
	  if(pathCurrent.CreateInterpolatedWaypoint(this->pose2d_robot) == ERROR){
	    ROS_ERROR("%s::MergePath: Error adding an extra point", sComponentName.c_str());
	  }
	}
					
	ROS_INFO("%s::MergePath: New number of points in current path (AFTER ADDING NEW POINTS AND INTERPOLATING THEM)= %d and magnets = %d", sComponentName.c_str(), pathCurrent_.NumOfWaypoints() , pathCurrent_.NumOfMagnets());
	
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
  if(t_map.bObstacle){
    ROS_INFO("%s::ReadyState: Obstacle detected, cancel requested", sComponentName.c_str());
    SetRobotSpeed(0.0, 0.0);
    SwitchToState(STANDBY_STATE);
    return;
  }

  //REAL PLANNER CALL
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
  // We have to update the percent while the mision is ongoing TODO!!!
}
	
void TunnelPlanner::SetRobotSpeed(double speed, double angle){
  if(command_type == COMMAND_ACKERMANN){
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
  
  AnalyseCB();	// Checks action server state
  
  if(bCancel)	// Cancel current path if required
    CancelPath();
}

void TunnelPlanner::AnalyseCB(){
  if (!action_server_goto.isActive()){
    ROS_INFO("%s::AnalyseCB: Action server not active", sComponentName.c_str());
    return;
  }
  
  //TODO: update goto_feedback value here!!! Otherwise it always publishes 0.0 or 100.0		
  action_server_goto.publishFeedback(goto_feedback);
  
  if(goto_feedback.percent_complete == 100.0){
    //action_server_goto.setSucceeded(goto_result);
    action_server_goto.setAborted(goto_result);
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
  Waypoint last_waypoint;
  double dAuxSpeed, dAuxDist, wref;


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