#ifndef TUNNEL_MAP_H
#define TUNNEL_MAP_H

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseStamped.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/passthrough.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/sac_model_line.h>
#include <pcl/sample_consensus/sac_model.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/point_types.h>
#include <pcl/io/io.h>

using namespace std;

#define DEFAULT_OBSTACLE_RANGE 3.0
#define DEFAULT_LOOKAHEAD_DISTANCE 3.0
#define DEFAULT_FOOTPRINT_WIDTH 0.6
#define DEFAULT_FOOTPRINT_LENGTH 1.0
#define DEFAULT_CRANE_LENGTH 3.0
#define DEFAULT_LATERAL_CLEARANCE 0.5

enum{
  WALL_LEFT_SIDE = 1,
  WALL_RIGHT_SIDE = -1
};

class TunnelMap{
 public:
  TunnelMap(string name);
  ~TunnelMap();

  void pclCallback(const sensor_msgs::PointCloud2& pcl_msg);

  bool getWallLine(const pcl::PointCloud<pcl::PointXYZ>::Ptr& input_pcl, int side, geometry_msgs::PoseStamped& line_pose, double& distance);
  inline bool isFree() { return !bObstacle; }
  inline double getWallOrientation(){ return yaw_; }
  inline double getWallDistance(){ return distance_; }
  void setState(bool state);

 private:
  ros::NodeHandle private_nh_;
  ros::Subscriber pcl_sub_;

  //! For visualization purposes
  ros::Publisher pub_wall_line_;
  ros::Publisher pub_wall_pcl_;
  //!Whether to publish the detected wall line
  string publish_wall_line_;
  //! Wall line pose
  geometry_msgs::PoseStamped wall_line_pose_;

  //!PCL topic name
  string pcl_topic_;
  //!Input cloud
  pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud_;
  //!Obstacle cloud
  pcl::PointCloud<pcl::PointXYZ>::Ptr obstacle_cloud_;
  //!PointCloud2 header
  std_msgs::Header header_;

  //!Flag variable to memorize if obstacles were detected
  bool bObstacle;
  //!Flag variable to memorize if the wall line was found
  bool bLineFound;

  //!Obstacle range
  double nav_obstacle_range_;
  double obstacle_range_;
  //!Lookahead distance for wall detection
  double lookahead_dist_;
  //!Select on which side first check for wall line
  string preferred_wall_side_;
  int first_wall_side_;

  double ransac_threshold_;
  double distance_;
  double yaw_;

  double obs_x_low_, obs_x_high_,obs_y_low_,obs_y_high_;
  double max_y_dist_;
  double nav_lateral_clearance_;
  double lateral_clearance_;
  double footprint_width_;
  double footprint_length_;    
  double crane_length_;
};

#endif
