#ifndef TUNNEL_PLANNER_H
#define TUNNEL_PLANNER_H

#include <robospect_planner/Path.h>
#include <robospect_planner/Component.h>
#include <robospect_planner/tunnel_map.h>

#include <ros/ros.h>
#include <math.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <actionlib/server/simple_action_server.h>
#include <robotnik_pp_msgs/GoToAction.h>
#include <robotnik_pp_msgs/goal.h>

#include <nav_msgs/Odometry.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose2D.h>

//TODO: Add diagnostic and understand how they work

#define COMMAND_ACKERMANN         100 
#define COMMAND_TWIST	          200 
#define COMMAND_ACKERMANN_STRING  "Ackermann" 
#define COMMAND_TWIST_STRING      "Twist"

#define D_LOOKAHEAD_MIN  0.3	// Minimum distance to the current sub-goal (PurePursuit with dynamic lookahead)
#define D_LOOKAHEAD_MAX	 1.1	// Maxiumum distance to the current sub-goal
#define DEFAULT_DESIRED_DISTANCE 1.0				  

#define ODOM_TIMEOUT_ERROR   0.2 // max num. of seconds without receiving odom values TODO: MAKE PARAM
#define MAP_TIMEOUT_ERROR    0.2 

#define WAYPOINT_POP_DISTANCE_M  0.10 // Maximum error in distance to the last waypoint to end the mission
#define MAX_SPEED 1.2
#define D_WHEEL_ROBOT_CENTER 0.3

#define DEFAULT_KR 0.20
#define TURN_RADIUS 0.20 //TODO!!!

#define FIRST_DECELERATION_DISTANCE   0.5   // [m] When the vehicle is reaching the goal, it has to decelarate starting from this distance
#define FIRST_DECELERATION_MAXSPEED   0.15  // [m/s]
#define SECOND_DECELERATION_DISTANCE  0.25  // [m] When the vehicle is reaching the goal, it has to increase the deceleration starting from this distance
#define SECOND_DECELERATION_MAXSPEED  0.1   // [m/s]

enum{
  ODOM_SOURCE = 1,
  MAP_SOURCE = 2
};

using namespace std;

class TunnelPlanner: public Component
{
 public:
  TunnelPlanner(ros::NodeHandle h);
  ~TunnelPlanner(){};

  //! Setup ROS stuffs (inside constructor)
  void ROSSetup();
  inline void GoalCB(){};
  inline void PreemptCB(){ bCancel=true; }
  
  //! ControlThread()
  void ControlThread();
  
  void InitState();
  void StandbyState();
  void ReadyState();
  void ShutDownState();
  void EmergencyState();
  inline void FailureState(){};

  //Stuffs that needs to be executed at every iteration, independently from the current state
  void AllState();
  void CancelPath();
  void AnalyseCB();
  ReturnValue MergePath();

  ReturnValue Setup();
  ReturnValue Start();
  ReturnValue Stop();
 
  void OdomCallback(const nav_msgs::Odometry::ConstPtr& odom_msg);
  int CheckOdomReceive();
  void SetRobotSpeed(double speed, double angle);

  int PurePursuit();
  double Dist(double x1, double y1, double x2, double y2);
  void UpdateLookAhead();
  
 private:

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  double desired_freq_;

  //! Distance from the robot center to the wheel's center
  double d_dist_wheel_to_center_;
  //! Sets the type of command to send to the robot (Twist or ackermann)
  int command_type_;
  //! Target frame for the transform from /map to it
  string target_frame_;

  //! Mode for reading the position of the robot ("ODOM", "MAP")
  string position_source_;
  //! Mode in numeric format
  unsigned int ui_position_source;
  
  //! Flag to enable/disable the motion
  bool bEnabled;  //TODO: ACTUALLY SETS TO TRUE AT THE BEGINNING AND NOT CHANGED ANYMORE!!!
  //! Flag to cancel the current  path
  bool bCancel;

  //! Current robot's position 
  geometry_msgs::Pose2D pose2d_robot;
  //! Object with the current path that the robot is following
  Path pathCurrent_;
  //! Object with the path that is being filled
  Path pathFilling_;
  //! Vector with next paths to follow
  queue <Path> qPath;
  //! Current robot's linear speed
  double dLinearSpeed_;
  //! Lookahead distance
  double dLookAhead_;
  //! Max allowed speed
  double max_speed_;
  //! constant for Purepursuit
  double Kr;

  double d_lookahead_min_;
  double d_lookahead_max_;
  double desired_distance_;

  TunnelMap* tm_;
  
  //ROS
  //! Publisher into command velocity (for the robot)
  ros::Publisher vel_pub_;
  //! Subscriber to /odom
  ros::Subscriber odom_sub_;
  //! Odom topic name
  string odom_topic_;
  //! Topic name to publish the vel & pos commands
  string cmd_topic_vel_;
  //! Publish the transformation between map->target_frame_ (ONLY IN MAP MODE)
  ros::Publisher tranform_map_pub_;
  //! Saves the time whenever receives an odom msg and a transform between map and base link (if configured)
  ros::Time last_odom_time_, last_map_time_;

  // ACTIONLIB
  actionlib::SimpleActionServer<robotnik_pp_msgs::GoToAction> action_server_goto;
  robotnik_pp_msgs::GoToFeedback goto_feedback;
  robotnik_pp_msgs::GoToResult goto_result;
  robotnik_pp_msgs::GoToGoal goto_goal;

  //TFs (used only in MAP_SOURCE mode)
  tf::TransformListener listener;
  tf::StampedTransform transform_;

  //JUST FOR DEBUG!!!
  ros::Publisher pub_next_point_;
  ros::Publisher pub_next_pose_;
  
};

#endif
