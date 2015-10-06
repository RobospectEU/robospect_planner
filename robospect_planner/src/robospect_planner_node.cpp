#include <ros/ros.h>
#include <robospect_planner/tunnel_planner.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "robospect_planner_node");
	
  ros::NodeHandle nh;		
  TunnelPlanner planner(nh);
  
  planner.ControlThread();
  
  return (0);
}
