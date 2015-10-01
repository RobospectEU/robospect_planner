#include<ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/passthrough.h>

using namespace std;

#define DEFAULT_OBSTACLE_RANGE 3
#define DEFAULT_LOOKAHEAD_DISTANCE 3

class TunnelMap{
 public:
  TunnelMap();
  ~TunnelMap();

  //TODO: now all incoming pcl are processed in the callback for obstacle and wall detection, should we create a buffer or a queue?
  void pclCallback(const sensor_msgs::PointCloud2::ConstPtr& pcl_msg);

 public:
  ros::NodeHandle nh_;
  //!Flag variable to memorize if obstacles were detected
  bool bObstacle;
  //!Input cloud
  pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud_;
  //!Obstacle cloud
  pcl::PointCloud<pcl::PointXYZ>::Ptr obstacle_cloud_;
  //!Wall cloud
  pcl::PointCloud<pcl::PointXYZ>::Ptr wall_cloud_;

  //!Obstacle range
  double obstacle_range_;
  //!Lookahead distance for wall detection
  double lookahead_dist_;
  
 private:
  ros::NodeHandle private_nh_;
  
};
