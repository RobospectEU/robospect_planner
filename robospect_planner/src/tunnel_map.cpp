#include<robospect_planner/tunnel_map.h>

using namespace std;

TunnelMap::TunnelMap(ros::NodeHandle h): private_nh_("~"), nh_(h) {
  
  bObstacle=true; 
  private_nh_.param("obstacle_range", obstacle_range_, DEFAULT_OBSTACLE_RANGE);
  private_nh_.param("lookahead_distance",lookahead_dist_,DEFAULT_LOOKAHEAD_DISTANCE);

  input_cloud_.reset(new  pcl::PointCloud<pcl::PointXYZ>());
  obstacle_cloud_.reset(new  pcl::PointCloud<pcl::PointXYZ>());
  wall_cloud_.reset(new  pcl::PointCloud<pcl::PointXYZ>());
}

TunnelMap::~TunnelMap(){}

void TunnelMap::pclCallback(const sensor_msgs::PointCloud2::ConstPtr& pcl_msg){
  pcl::PCLPointCloud2 temp_pcl_;
  pcl_conversions::toPCL(input_cloud_, temp_pcl_);
  pcl::fromPCLPointCloud2(temp_pcl_,input_cloud_);

  /****** DO OBSTACLE DETECTION FIRST *******/
  pcl::PassThrough<pcl::PointXYZ> pass;
  pass.setInputCloud (input_cloud_);
  pass.setFilterFieldName ("x");
  pass.setFilterLimits (x_low_lim_,x_high_lim_);
  pass.filter (*obstacle_cloud_); 

  pass.setInputCloud (obstacle_cloud_);
  pass.setFilterFieldName ("y");
  pass.setFilterLimits (y_low_lim_,y_high_lim_);
  pass.filter (*obstacle_cloud_);

  bObstacle = (obstacle_cloud ) ? true : false;

  /****** NOW TURN TO WALL DETECTION *******/

}
