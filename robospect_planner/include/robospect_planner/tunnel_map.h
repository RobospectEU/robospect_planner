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
#define DEFAULT_LATERAL_CLEARANCE 0.5

enum{
  WALL_LEFT_SIDE = 1,
  WALL_RIGHT_SIDE = -1
};

class TunnelMap{
 public:
  TunnelMap(string name);
  ~TunnelMap();

  //TODO: now all incoming pcl are processed in the callback for obstacle and wall detection, should we create a buffer or a queue?
  void pclCallback(const sensor_msgs::PointCloud2& pcl_msg);

  bool getWallLine(const pcl::PointCloud<pcl::PointXYZ>::Ptr& input_pcl, int side, geometry_msgs::PoseStamped& line_pose, double& distance);
  inline bool isFree() { return !bObstacle; }
  inline double getWallOrientation(){ return yaw_; }
  inline double getWallDistance(){ return distance_; }

 public:
  ros::Publisher pub_wall_line_;
  ros::Subscriber pcl_sub_;
  string pcl_topic_;
  //!Flag variable to memorize if obstacles were detected
  bool bObstacle;
  //!Input cloud
  pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud_;
  //!Obstacle cloud
  pcl::PointCloud<pcl::PointXYZ>::Ptr obstacle_cloud_;

  std_msgs::Header header_;

  //!Obstacle range
  double obstacle_range_;
  //!Lookahead distance for wall detection
  double lookahead_dist_;
  //!Select on which side first check for wall line
  string preferred_wall_side_;
  int first_wall_side_;
  int second_wall_side_;
  geometry_msgs::PoseStamped wall_line_pose_;

  double ransac_threshold_;
  double distance_;
  double yaw_;

  double obs_x_low_, obs_x_high_,obs_y_low_,obs_y_high_;
  double lateral_clearance_;
  double footprint_width_;
  double footprint_length_;
  double max_y_dist_;

  bool bFirstLineFound;
  bool bSecondLineFound;
  string publish_wall_line_;

  //! Mutex to control the access
  boost::mutex mutex_;

 private:
  ros::NodeHandle private_nh_;
    
};

#endif
