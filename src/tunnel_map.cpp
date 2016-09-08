#include<robospect_planner/tunnel_map.h>

using namespace std;

void TunnelMap::readParams(){
	private_nh_.param("footprint_width",footprint_width_,DEFAULT_FOOTPRINT_WIDTH);
	private_nh_.param("crane_length",crane_length_,DEFAULT_CRANE_LENGTH);
	private_nh_.param("footprint_length",footprint_length_,DEFAULT_FOOTPRINT_LENGTH);
	private_nh_.param("lateral_clearance",nav_lateral_clearance_,DEFAULT_LATERAL_CLEARANCE);
	private_nh_.param("obstacle_range", nav_obstacle_range_, DEFAULT_OBSTACLE_RANGE);

	private_nh_.param("lookahead_distance",lookahead_dist_,DEFAULT_LOOKAHEAD_DISTANCE);
	private_nh_.param<string>("preferred_wall_side", preferred_wall_side_, "LEFT");
	private_nh_.param("ransac_threshold",ransac_threshold_,0.1);
	private_nh_.param("max_y_dist",max_y_dist_,10.0);   
	private_nh_.param<string>("publish_wall_line", publish_wall_line_, "true");
	private_nh_.param<string>("pcl_topic", pcl_topic_, "/scan_cloud");

	if(preferred_wall_side_ == "RIGHT"){
		first_wall_side_ = WALL_RIGHT_SIDE;
	}else{ 
		first_wall_side_ = WALL_LEFT_SIDE;
	}
	
	//ROS_INFO("TunnelMap::readParams: %lf, wall_side = (%s) %d",lookahead_dist_, preferred_wall_side_.c_str(), first_wall_side_);
	
}

void TunnelMap::setParams(double footprint_width, double crane_length, double footprint_length, double lateral_clearance, double obstacle_range, double lookahead_dist,
	string wall_side, double ransac_threshold, double max_y_dist){
	
	this->footprint_width_ = footprint_width;
	this->crane_length_ = crane_length;
	this->footprint_length_ = footprint_length;
	this->nav_lateral_clearance_ = lateral_clearance;
	this->nav_obstacle_range_ = obstacle_range;
	this->lookahead_dist_ = lookahead_dist;
	this->preferred_wall_side_ = wall_side;
	this->ransac_threshold_ = ransac_threshold;
	this->max_y_dist_ = max_y_dist;
	
	if(preferred_wall_side_ == "RIGHT"){
		first_wall_side_ = WALL_RIGHT_SIDE;
	}else{ 
		first_wall_side_ = WALL_LEFT_SIDE;
	}	
}

TunnelMap::TunnelMap(string name): private_nh_("~"){
 
  readParams();

  input_cloud_.reset(new  pcl::PointCloud<pcl::PointXYZ>());
  obstacle_cloud_.reset(new  pcl::PointCloud<pcl::PointXYZ>());
  
  if(publish_wall_line_=="true")
    pub_wall_line_ = private_nh_.advertise<geometry_msgs::PoseStamped> ("/wall_reference_line", 1);
  
  pub_wall_pcl_ = private_nh_.advertise<sensor_msgs::PointCloud2> ("/wall_pcl", 1);
  pub_obstacle_pcl_ = private_nh_.advertise<sensor_msgs::PointCloud2> ("/obstacle_pcl", 1);
  
  pcl_sub_=private_nh_.subscribe(pcl_topic_, 1, &TunnelMap::pclCallback, this);    

  yaw_=0.0;
  distance_=max_y_dist_;
  obstacle_range_=nav_obstacle_range_;
  lateral_clearance_=nav_lateral_clearance_;
  
  bLineFound=false;
  bObstacle=true;  
  obs_x_low_=-footprint_length_/2;
  obs_x_high_=obstacle_range_+footprint_length_/2+crane_length_;
  obs_y_low_=-(footprint_width_/2+lateral_clearance_);
  obs_y_high_=(footprint_width_/2+lateral_clearance_);

  ROS_INFO("TunnelMap initialized");
  ROS_INFO("TunnelMap(): pcl_topic = %s",pcl_topic_.c_str());
}

TunnelMap::~TunnelMap(){}

void TunnelMap::setState(bool state){
  if(state){
     obstacle_range_=0.40;
     lateral_clearance_=0.05;
  }else{	
     obstacle_range_=nav_obstacle_range_;
     lateral_clearance_=nav_lateral_clearance_;
  }
}

void TunnelMap::pclCallback(const sensor_msgs::PointCloud2& pcl_msg){
  pcl::PCLPointCloud2 temp_pcl_;
  pcl_conversions::toPCL(pcl_msg, temp_pcl_);
  pcl::fromPCLPointCloud2(temp_pcl_,*input_cloud_);
  //ROS_INFO("Received a tunnel cloud of %u points",(uint32_t)(input_cloud_->size()));
  
  header_=pcl_msg.header;

  /****** DO OBSTACLE DETECTION FIRST *******/
  pcl::PassThrough<pcl::PointXYZ> pass;
  pass.setInputCloud (input_cloud_);
  pass.setFilterFieldName ("x");
  pass.setFilterLimits (obs_x_low_,obs_x_high_);
  pass.filter (*obstacle_cloud_); 

  pass.setInputCloud (obstacle_cloud_);
  pass.setFilterFieldName ("y");
  pass.setFilterLimits (obs_y_low_,obs_y_high_);
  pass.filter (*obstacle_cloud_);

  //ROS_INFO("Get an obstacle cloud of %u points",(uint32_t)(obstacle_cloud_->size()));
  bObstacle = (obstacle_cloud_->points.size()>0) ? true : false;
  
  pcl::PCLPointCloud2 temp_pcl2_;
  sensor_msgs::PointCloud2 cloud_out;
  pcl::toPCLPointCloud2(*obstacle_cloud_, temp_pcl2_);
  pcl_conversions::fromPCL(temp_pcl2_, cloud_out);
  cloud_out.header=header_;
  pub_obstacle_pcl_.publish(cloud_out);
  

  /****** NOW TURN TO WALL DETECTION *******/
  //!First get the first point cloud on which interpolate the tunnel line 
  bLineFound=getWallLine(input_cloud_,first_wall_side_,wall_line_pose_,distance_);
  yaw_=tf::getYaw(wall_line_pose_.pose.orientation);
  
  if((publish_wall_line_=="true") && bLineFound)
    pub_wall_line_.publish(wall_line_pose_); 
}

bool TunnelMap::getWallLine(const pcl::PointCloud<pcl::PointXYZ>::Ptr& input_pcl, int side, geometry_msgs::PoseStamped& line_pose, double& distance){
  
  pcl::PointCloud<pcl::PointXYZ>::Ptr wall_cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PassThrough<pcl::PointXYZ> pass, pass2;
  
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  
  pcl::IndicesPtr indices_x (new std::vector <int>);
  pcl::IndicesPtr indices_y (new std::vector <int>);

  distance=-100.0;

	double y_filter_min = side*(footprint_width_/2+lateral_clearance_);
	double y_filter_max = side*max_y_dist_;
	double x_filter_min = footprint_length_/2;// + crane_length_-0.5;
	double x_filter_max = footprint_length_/2 + crane_length_+lookahead_dist_;
	
	// In case of negative ranges
	if(y_filter_max < y_filter_min){
		double aux = y_filter_max;
		
		y_filter_max = y_filter_min;
		y_filter_min = aux;
	}
	if(x_filter_max < x_filter_min){
		double aux = x_filter_max;
		
		x_filter_max = x_filter_min;
		x_filter_min = aux;
	}
	
  pass.setInputCloud (input_pcl);
  pass.setFilterFieldName ("y");
  pass.setFilterLimits (y_filter_min, y_filter_max);

  pass.filter(*indices_y);
  //ROS_INFO("getWallLine: side = %d, y filter between %lf and %lf", side, y_filter_min, y_filter_max);

  //pass.setInputCloud (wall_cloud);
  pass.setIndices (indices_y);
  pass.setFilterFieldName ("x");
  pass.setFilterLimits (x_filter_min, x_filter_max); 
  //ROS_INFO("getWallLine: x filter between %lf and %lf", x_filter_min, x_filter_max);
  //TODO make another param? (lookback_dist_?)
  pass.filter(*wall_cloud);

  //ONLY FOR DEBUG!!
  pcl::PCLPointCloud2 temp_pcl_;
  sensor_msgs::PointCloud2 cloud_out;
  pcl::toPCLPointCloud2(*wall_cloud, temp_pcl_);
  pcl_conversions::fromPCL(temp_pcl_, cloud_out);
  cloud_out.header=header_;
  pub_wall_pcl_.publish(cloud_out);
  
  if (wall_cloud->points.size ()>2){
    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_LINE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setDistanceThreshold (ransac_threshold_);			
    
    seg.setInputCloud(wall_cloud);
    seg.segment (*inliers, *coefficients);
    //ROS_INFO("Get %u inliers in a cloud of %u points",
    //	     (uint32_t)(inliers->indices.size()),(uint32_t)(wall_cloud->size()));
  }else{
     ROS_WARN("The filtered cloud has less than 2 points, could not interpolate the line!");
     return false;
  }
    
  // Obtain the line point and direction
  Eigen::Vector4f line_pt  (coefficients->values[0], coefficients->values[1], coefficients->values[2], 1);
  Eigen::Vector4f line_dir (coefficients->values[3], coefficients->values[4], coefficients->values[5], 0);
  Eigen::Vector4f origin_pt (0.0,0.0,coefficients->values[2],1);
  line_dir.normalize ();
 
  // Calculate the distance from the base_footprint to the line
  // D = ||(P2-P1) x (P1-P0)|| / ||P2-P1|| = norm (cross (p2-p1, p2-p0)) / norm(p2-p1)
  distance = sqrt ((line_pt - origin_pt).cross3 (line_dir).squaredNorm ());
//ROS_INFO("The distance of the robot from the line is: %5.5f m",distance);

  //Now fill in the line_pose message (PoseStamped)
  line_pose.header=header_;

  line_pose.pose.position.x = footprint_length_/2+crane_length_;    
  line_pose.pose.position.y =line_pt[1]+(line_dir[1]/line_dir[0])*(footprint_length_/2+crane_length_-line_pt[0]);
  line_pose.pose.position.z = 0.0;
  line_pose.pose.orientation = tf::createQuaternionMsgFromYaw(atan(line_dir[1]/line_dir[0]));
  
  //TODO Add some checks
  return true;
}
