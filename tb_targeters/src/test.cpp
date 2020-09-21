#include <ros/ros.h>
#include <octomap_msgs/conversions.h>
#include <octomap_msgs/Octomap.h>
#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <octomap/OcTreeBase.h>
#include <octomap/octomap_types.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/PointStamped.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Range.h>
#include <sensor_msgs/Joy.h>
#include <queue>
#include <vector>
#include <string>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_listener.h>
#include "tf2_ros/message_filter.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include <tf/transform_datatypes.h>
#include "message_filters/subscriber.h"
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Point32.h>
#include <nav_msgs/Path.h>
#include <actionlib_msgs/GoalID.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <image_transport/image_transport.h>
#include <dynamicEDT3D/dynamicEDTOctomap.h>
#include <nav_msgs/OccupancyGrid.h>
#include <map_msgs/OccupancyGridUpdate.h>
#include <chrono>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/UInt8.h>
#include <visualization_msgs/MarkerArray.h>
#include <eigen3/Eigen/Core>
#include <geometry_msgs/Vector3Stamped.h>
#include <cv_bridge/cv_bridge.h>
#include <nav_msgs/Odometry.h>


tf2_ros::Buffer tfBuffer;
ros::Publisher pub_target,pub_pnt,pub_small_pnt,pub_elevate,pub_target_path,pub_setp_xyz;

cv::Mat img_ff_bld(1000,1000,CV_8U,cv::Scalar(0)); //create image, set encoding and size, init pixels to default val
cv::Mat img_ff_bld_backup(1000,1000,CV_8U,cv::Scalar(0)); //create image, set encoding and size, init pixels to default val
cv::Mat img_ff_bld_copy(1000,1000,CV_8U,cv::Scalar(0)); //create image, set encoding and size, init pixels to default val
cv::Mat img_blank_mono(1000,1000,CV_8U,cv::Scalar(0)); //create image, set encoding and size, init pixels to default val
cv::Mat img_mono(1000,1000,CV_8U,cv::Scalar(0)); //create image, set encoding and size, init pixels to default val
cv::Mat img_paths(1000,1000,CV_8U,cv::Scalar(0)); //create image, set encoding and size, init pixels to default val
cv::Mat img(1000,1000,CV_8UC3,cv::Scalar(0,0,0)); //create image, set encoding and size, init pixels to default val
cv::Mat img_blank(1000,1000,CV_8UC3,cv::Scalar(0,0,0)); //create image, set encoding and size, init pixels to default val
cv::Mat img_perm(1000,1000,CV_8UC3,cv::Scalar(0,0,0)); //create image, set encoding and size, init pixels to default val
cv::Mat img_copy(1000,1000,CV_8UC3,cv::Scalar(0,0,0)); //create image, set encoding and size, init pixels to default val
cv::Mat imgdeb(1000,1000,CV_8UC3,cv::Scalar(0,0,0)); //create image, set encoding and size, init pixels to default val
cv::Mat img_height(1000,1000,CV_8UC3,cv::Scalar(0,0,0)); //create image, set encoding and size, init pixels to default val
geometry_msgs::PointStamped target_xyz;
geometry_msgs::PolygonStamped poly_heading;
float target_heading;
std::vector<cv::Mat> mapimgs_at_zlvlz;
geometry_msgs::Point pos,pnt_ref,last_pos,cmd_pos;
std::vector<float> z_lvls;
nav_msgs::Path path_visited;
std::string state_building = "looking_for_building";
std::string state_building_scan = "initial_vertical";
float state_target_heading = 0;
ros::Time state_building_change,state_building_scan_change;
geometry_msgs::PointStamped closest_obstacle_2d,target_point,current_midpoint,building_centroid;
int par_scouting_zn_start,par_scouting_zn_end;
double par_maprad,par_dz;
int cnt = 0;
int cnt2 = 0;
int cnt3 = 0;
nav_msgs::Path path_side,last_path,gridpath;
std::vector<nav_msgs::Path> paths_vec;
geometry_msgs::PolygonStamped poly_building_active,poly_cleared_extended,building_obstacle_points,building_polygon,building_polygon_min,poly_cleared,poly_obstacles_received,poly_legal,poly_illegal;
int last_i = 0;
ros::Time last_rosinfo;
float pos_yaw = 0;
float last_pose_yaw = 0;
float last_yaw;
ros::Time last_scan;
geometry_msgs::PoseStamped last_pose,target_pose,target_pose_last,current_midpose;
std::vector<geometry_msgs::PolygonStamped> buildings_polygons;
std::vector<geometry_msgs::PolygonStamped> polys_heightimage_clustered;
std::vector<geometry_msgs::PolygonStamped> polys_obstacles_received;
std::vector<geometry_msgs::PolygonStamped> polys_scanin_obstacles;
std::vector<geometry_msgs::PolygonStamped> polys_scanin_cleared;

std::vector<geometry_msgs::PointStamped> building_centroids;
std::vector<nav_msgs::Path> building_paths_complete;
std::vector<nav_msgs::Path> building_paths_available;
std::vector<std::vector<nav_msgs::Path>> buildings_paths_complete;
std::vector<std::vector<geometry_msgs::PointStamped>> buildings_centroids;
nav_msgs::Path path_active_rightway,path_active_wrongway;

ros::Time last_sec;
double par_dst_target;
int zlvl_max;
///*TARGETER SIDE*///
std::vector<std::vector<float>>paths_side_bboxes;
std::vector<nav_msgs::Path> paths_side;
std::vector<geometry_msgs::Point> paths_side_centers;
std::vector<int> targetindexes_sent;
std::vector<int> blacklist;
std::vector<int> blacklist_paths_side;
float target_distance,cmd_pos_yaw;
///*TARGETER SIDE*///
nav_msgs::Path path_unknown,path_building_above,path_building_below,path_building_active;
nav_msgs::Path path_side_norm,path_sideauto_small,path_sideauto_big;
nav_msgs::Path path_organize_1,path_organize_2,path_organize_raw;
nav_msgs::Path path_active_reduced,path_candidates,path_building_targets,path_spiral,path_targets,path_targets_sent;
std::vector<int> targets_complete;
int path_targets_i = 0;
geometry_msgs::Point info_point;
float info_interval = 20;
int building_v0 = 0;
float rad2deg = 180.0/M_PI;
int path_spiral_i = 0;
float current_rad;
std::vector<geometry_msgs::PolygonStamped> poly_clusters;

geometry_msgs::Point get_ave_pnt_poly(geometry_msgs::PolygonStamped polyin){
	geometry_msgs::Point pnt;
	if(polyin.polygon.points.size() == 0)
		return pnt;
	for(int i = 0; i < polyin.polygon.points.size(); i++){
		pnt.x += polyin.polygon.points[i].x;
		pnt.y += polyin.polygon.points[i].y;
		pnt.z += polyin.polygon.points[i].z;
	}
  pnt.x /= polyin.polygon.points.size();
  pnt.y /= polyin.polygon.points.size();
  pnt.z /= polyin.polygon.points.size();
  return pnt;
}

geometry_msgs::Point get_ave_pnt(nav_msgs::Path pathin){
  geometry_msgs::Point pnt;
  for(int i = 0; i < pathin.poses.size(); i++){
    pnt.x += pathin.poses[i].pose.position.x;
    pnt.y += pathin.poses[i].pose.position.y;
    pnt.z += pathin.poses[i].pose.position.z;
  }
  pnt.x /= pathin.poses.size();
  pnt.y /= pathin.poses.size();
  pnt.z /= pathin.poses.size();
  return pnt;
}
std::vector<int> vec_to_min_max_ave(std::vector<int> vec_in){
  std::vector<int> min_max_ave;
  min_max_ave.push_back(143131);
  min_max_ave.push_back(-143131);
  min_max_ave.push_back(0);
  if(vec_in.size() == 0){
    min_max_ave[0] = 0;
    min_max_ave[1] = 0;
    return min_max_ave;
  }

  float vec_sum = 0;
  for(int i = 0; i < vec_in.size(); i++){
    vec_sum += vec_in[i];
    if(vec_in[i] > min_max_ave[1])
     min_max_ave[1] = vec_in[i];
    if(vec_in[i] < min_max_ave[0])
     min_max_ave[0] = vec_in[i];
  }
    min_max_ave[2] = vec_sum / vec_in.size();
  return min_max_ave;
}
void state_building_end(){

}
void set_state_building_scan(std::string newstate){
  if(newstate != state_building_scan){
    float dt = (ros::Time::now() - state_building_scan_change).toSec();
    ROS_INFO("state_building_scan: %s -> %s (%.3f seconds in state)",state_building_scan.c_str(),newstate.c_str(),dt);
    state_building_scan_change  = ros::Time::now();
    state_building_scan  = newstate;
  }
}

void set_state_building(std::string newstate){
  if(newstate != state_building){
    float dt = (ros::Time::now() - state_building_change).toSec();
    ROS_INFO("state_building: %s -> %s (%.3f seconds in state)",state_building.c_str(),newstate.c_str(),dt);
    state_building_change  = ros::Time::now();
		if(state_building == "approaching_building" && newstate == "inspecting_building"){
			path_building_targets.poses.resize(0);
			path_building_targets.poses.push_back(target_pose);
			building_v0 = path_visited.poses.size();
		}
		state_building = newstate;
  }
}

float get_dst2d(geometry_msgs::Point p2, geometry_msgs::Point p1){
  return(sqrt(pow(p1.x-p2.x,2)+pow(p1.y-p2.y,2)));
}
float get_dst2d32(geometry_msgs::Point32 p2, geometry_msgs::Point32 p1){
  return(sqrt(pow(p1.x-p2.x,2)+pow(p1.y-p2.y,2)));
}
float get_dst3d32(geometry_msgs::Point32 p2, geometry_msgs::Point32 p1){
	return(sqrt(pow(p1.x-p2.x,2)+pow(p1.y-p2.y,2)+pow(p1.z-p2.z,2)));
}
float get_dst3d32d(geometry_msgs::Point p2, geometry_msgs::Point32 p1){
	return(sqrt(pow(p1.x-p2.x,2)+pow(p1.y-p2.y,2)+pow(p1.z-p2.z,2)));
}
float get_dst2d32d(geometry_msgs::Point p2, geometry_msgs::Point32 p1){
  return(sqrt(pow(p1.x-p2.x,2)+pow(p1.y-p2.y,2)));
}
float get_dst3d(geometry_msgs::Point p2, geometry_msgs::Point p1){
  return(sqrt(pow(p1.x-p2.x,2)+pow(p1.y-p2.y,2)+pow(p1.z-p2.z,2)));
}
float get_hdng(geometry_msgs::Point p1,geometry_msgs::Point p0){
  float dx = p1.x - p0.x;
  float dy = p1.y - p0.y;
  return atan2(dy,dx);
}
float get_hdng32d(geometry_msgs::Point32 p1,geometry_msgs::Point p0){
  float dx = p1.x - p0.x;
  float dy = p1.y - p0.y;
  return atan2(dy,dx);
}
float get_hdng32(geometry_msgs::Point32 p1,geometry_msgs::Point32 p0){
  float dx = p1.x - p0.x;
  float dy = p1.y - p0.y;
  return atan2(dy,dx);
}
double get_shortest(double target_hdng,double actual_hdng){
  double a = target_hdng - actual_hdng;
  if(a > M_PI)a -= M_PI*2;
  else if(a < -M_PI)a += M_PI*2;
  return a;
}
std_msgs::Header hdr(){
  std_msgs::Header header;
  header.frame_id = "map";
  header.stamp    = ros::Time::now();
  return header;
}
float y2r(float y){
  return (img.rows / 2 - y);
}
float x2c(float x){
  return (x + img.cols/2);
}
int r2y(float r){
  return int((img.rows / 2 - r));
}
int c2x(float c){
  return int((c - img.cols / 2));
}
cv::Point pnt322cv(geometry_msgs::Point32 pin){
	int c = x2c(pin.x);
	int r = y2r(pin.y);
	return cv::Point(c,r);
}
cv::Point pnt2cv(geometry_msgs::Point pin){
	int c = x2c(pin.x);
	int r = y2r(pin.y);
	return cv::Point(c,r);
}

bool is_point_in_path(nav_msgs::Path path_to_check,geometry_msgs::Point pin,float lim, bool use_3d){
  float res,dst;
  if(path_to_check.poses.size() == 0)
    return false;
  for(int i = 0; i < path_to_check.poses.size(); i++){
		if(use_3d && get_dst3d(path_to_check.poses[i].pose.position,pin) < lim)
      return true;
		else if(!use_3d && get_dst3d(path_to_check.poses[i].pose.position,pin) < lim)
	     return true;
	}
  return false;
}

bool is_point32_in_poly(geometry_msgs::PolygonStamped poly_to_check,geometry_msgs::Point32 pin,float lim){
  float res,dst;
  if(poly_to_check.polygon.points.size() == 0)
    return false;
  for(int i = 0; i < poly_to_check.polygon.points.size(); i++){
     if(get_dst3d32(poly_to_check.polygon.points[i],pin) < lim)
        return true;
  }
  return false;
}
bool is_point32_in_path(nav_msgs::Path path_to_check,geometry_msgs::Point32 pin,float lim){
  float res,dst;
  if(path_to_check.poses.size() == 0)
	return false;
  for(int i = 0; i < path_to_check.poses.size(); i++){
     if(get_dst2d32d(path_to_check.poses[i].pose.position,pin) < lim)
		 return true;
  }
	return false;
}
nav_msgs::Path get_new_path(nav_msgs::Path path_to_check,nav_msgs::Path pathin,float cutoff_dst,bool use_3d){
	nav_msgs::Path pathout;
	pathout.header = hdr();
	for(int i = 0; i < pathin.poses.size(); i++){
		if(!is_point_in_path(path_to_check,pathin.poses[i].pose.position,cutoff_dst,use_3d)){
			pathout.poses.push_back(pathin.poses[i]);
		}
	}
	ROS_INFO("GET NEW PATH: %i -> %i",pathin.poses.size(),pathout.poses.size());
	return pathout;
}
geometry_msgs::PolygonStamped remove_path_overlap_from_poly(nav_msgs::Path path_to_check,geometry_msgs::PolygonStamped polyin, float visited_dst_cutoff, bool use_3d){
	geometry_msgs::PolygonStamped polyout;
	polyout.header = hdr();
	for(int i = 0; i < polyin.polygon.points.size(); i++){
		if(is_point32_in_path(path_to_check,polyin.polygon.points[i],visited_dst_cutoff)){
			polyout.polygon.points.push_back(polyin.polygon.points[i]);
		}
	}
	return polyout;
}
double constrainAngle(double x){
  if(x > M_PI)
    return (x - M_PI*2);
  else if(x < -M_PI)
    return (x + M_PI*2);
  else
    return x;
}
float get_path_yaw(nav_msgs::Path pathin){
	if(pathin.poses.size() == 0)
		return 0.0;
	float yaw_sum = 0;
	for(int i = 0; i < pathin.poses.size(); i++){
		yaw_sum += tf::getYaw(pathin.poses[0].pose.orientation);
	}
	return yaw_sum / pathin.poses.size();
}
bool in_vec(std::vector<int> vec,int k){
	if(vec.size() == 0)
		return false;
	//	ROS_INFO("in vec: %i",k);
	for(int i = 0; i < vec.size(); i++){
		if(vec[i] == k)
			return true;
	}
	return false;
}
float get_zmax(nav_msgs::Path pathin){
  float zmx = 0;
  for(int i = 0; i < pathin.poses.size(); i++){
    if(pathin.poses[i].pose.position.z > zmx){
      zmx = pathin.poses[i].pose.position.z;
    }
  }
  return zmx;
}
int get_closest_i(nav_msgs::Path pathin,geometry_msgs::Point pin){
  float res,dst;
  int closest_i = 0;
  float closest_dst = 100;
  if(pathin.poses.size() == 0)
    return closest_i;
  for(int i = 0; i < pathin.poses.size(); i++){
    float dst = get_dst2d(pathin.poses[i].pose.position,pin);
     if(dst < closest_dst){
       closest_dst = dst;
       closest_i = i;
     }
  }
  return closest_i;
}
int getinpath_closestindex2d_blacklist(nav_msgs::Path pathin,geometry_msgs::Point pnt){
  int lowest_dist_i = 0;
  float lowest_dist = 1000;
  for(int i = 0; i < pathin.poses.size(); i++){
		if(!in_vec(blacklist,i)){
			float dst = get_dst2d(pathin.poses[i].pose.position,pnt);
	    if(dst < lowest_dist){
	      lowest_dist_i = i;
	      lowest_dist = dst;
			}
		}
	}
	return lowest_dist_i;
}
int getinpath_closestindex2d_poly(geometry_msgs::PolygonStamped polyin,geometry_msgs::Point pnt){
	if(polyin.polygon.points.size() == 0)
		return -1;
  int lowest_dist_i = 0;
  float lowest_dist = 1000;
  for(int i = 0; i < polyin.polygon.points.size(); i++){
    float dst = get_dst2d32d(pnt,polyin.polygon.points[i]);
    if(dst < lowest_dist){
      lowest_dist_i = i;
      lowest_dist = dst;
		}
	}
	return lowest_dist_i;
}
int getinpath_closestindex2d(nav_msgs::Path pathin,geometry_msgs::Point pnt){
  int lowest_dist_i = 0;
  float lowest_dist = 1000;
  for(int i = 0; i < pathin.poses.size(); i++){
    float dst = get_dst2d(pathin.poses[i].pose.position,pnt);
    if(dst < lowest_dist){
      lowest_dist_i = i;
      lowest_dist = dst;
		}
	}
	return lowest_dist_i;
}
int getinpath_farthestindex2d(nav_msgs::Path pathin,geometry_msgs::Point pnt){
  int highest_dist_i = 0;
  float highest_dist = 0;
  for(int i = 0; i < pathin.poses.size(); i++){
    float dst = get_dst2d(pathin.poses[i].pose.position,pnt);
    if(dst > highest_dist){
      highest_dist_i = i;
      highest_dist = dst;
		}
	}
	return highest_dist_i;
}
nav_msgs::Path remove_nth_first(nav_msgs::Path pathin,int i0){
  nav_msgs::Path pathout;
  pathout.header = hdr();
  if(i0 < 0)
    return pathout;
  for(int i = i0; i < pathin.poses.size(); i++){
    pathout.poses.push_back(pathin.poses[i]);
  }
  return pathout;
}
int get_indexes_within_rad(nav_msgs::Path pathin,float max_dst,geometry_msgs::Point centroid){
  for(int i = 0; i < pathin.poses.size(); i++){
    if(get_dst3d(pathin.poses[i].pose.position,centroid) > max_dst)
      return i;
  }
  return pathin.poses.size();
}

////////////////**********CLUSTERS**************///////////////////////
////////////////**********CLUSTERS**************///////////////////////


std::vector<int> getinpath_indexes_inrad(nav_msgs::Path pathin,int i_to_check,float radius){
  std::vector<int> vec_out;
  if(pathin.poses.size() == 0){
    //ROS_INFO("PathAdmin: pathin is empty");
    return vec_out;
  }
  float yaw0 = tf::getYaw(pathin.poses[i_to_check].pose.orientation);
  for(int i = 0; i < pathin.poses.size(); i++){
    float dist = get_dst3d(pathin.poses[i].pose.position,pathin.poses[i_to_check].pose.position);
    float dyaw = get_shortest(tf::getYaw(pathin.poses[i].pose.orientation),yaw0);
    float dz   = 0;//pathin.poses[i].pose.position.z-pathin.poses[i_to_check].pose.position.z;
    if(dyaw < 0)
      dyaw *= -1;

    if(dist <= radius && dist > 0 && dyaw < M_PI/4 && dz == 0)
      vec_out.push_back(i);
  }
  return vec_out;
}

std::vector<std::vector<int>> getinpath_neighbours(nav_msgs::Path pathin,float radius){
	std::vector<std::vector<int>> neighbours_at_index;
  if(pathin.poses.size() == 0){
    //ROS_INFO("PathAdmin: pathin is empty");
    return neighbours_at_index;
  }
  for(int i = 0; i < pathin.poses.size(); i++){
		neighbours_at_index.push_back(getinpath_indexes_inrad(pathin,i,radius));
  //  ROS_INFO("neighbours_at_index[%i]: %i",i,neighbours_at_index[i].size());
  }
  return neighbours_at_index;
}

std::vector<int> add_neighbours_index(std::vector<std::vector<int>> neighbours_at_indexes,std::vector<int> neighbours_in_cluster,std::vector<int> indexes_to_add){
  std::vector<int> new_neighbours;
  for(int k = 0; k < indexes_to_add.size(); k++){
    if(!in_vec(neighbours_in_cluster,indexes_to_add[k])
    && !in_vec(new_neighbours,       indexes_to_add[k]))
      new_neighbours.push_back(indexes_to_add[k]);
    for(int i = 0; i < neighbours_at_indexes[indexes_to_add[k]].size(); i++){
      if(!in_vec(neighbours_in_cluster,neighbours_at_indexes[indexes_to_add[k]][i])
      && !in_vec(new_neighbours,       neighbours_at_indexes[indexes_to_add[k]][i])){
        new_neighbours.push_back(neighbours_at_indexes[indexes_to_add[k]][i]);
      }
    }
  }
//  if(new_neighbours.size() > 2)
  //  ROS_INFO("new neighbours: count %i 0: %i N: %i",new_neighbours.size(),new_neighbours[0],new_neighbours[new_neighbours.size()-1]);
  return new_neighbours;
}

std::vector<int> get_neighbour_cluster(nav_msgs::Path pathin,float radius,int start_index){
  std::vector<std::vector<int>> neighbours_at_index;
  neighbours_at_index = getinpath_neighbours(pathin,radius);
  std::vector<int> neighbours_in_cluster;
  std::vector<int> indexes_to_add;
  indexes_to_add.push_back(start_index);
  while(indexes_to_add.size() > 0){
    for(int i = 0; i < indexes_to_add.size(); i++){
      neighbours_in_cluster.push_back(indexes_to_add[i]);
    }
    indexes_to_add = add_neighbours_index(neighbours_at_index,neighbours_in_cluster,indexes_to_add);
  }
  return neighbours_in_cluster;
}

std::vector<int> update_neighbours_clustered(std::vector<int> neighbours_clustered,std::vector<int> neighbours_in_cluster){
  for(int i = 0; i < neighbours_in_cluster.size(); i++){
    neighbours_clustered.push_back(neighbours_in_cluster[i]);
  }
  return neighbours_clustered;
}
std::vector<int> get_neighbours_not_clustered(std::vector<int> neighbours_clustered,int path_size){
  std::vector<int> not_clustered;
  for(int i = 0; i < path_size; i++){
    if(!in_vec(neighbours_clustered,i))
      not_clustered.push_back(i);
  }
  return not_clustered;
}

std::vector<std::vector<int>> get_neighbour_clusters(nav_msgs::Path pathin,float radius){
  std::vector<int> neighbours_not_clustered;
  std::vector<int> neighbours_in_cluster;
  std::vector<int> neighbours_clustered;
  std::vector<std::vector<int>> neighbour_clusters;
  while(neighbours_clustered.size() < pathin.poses.size()){
    neighbours_not_clustered = get_neighbours_not_clustered(neighbours_clustered,pathin.poses.size());
    neighbours_in_cluster    = get_neighbour_cluster(pathin,radius,neighbours_not_clustered[0]);
    neighbours_clustered     = update_neighbours_clustered(neighbours_clustered,neighbours_in_cluster);
    neighbour_clusters.push_back(neighbours_in_cluster);
  //  ROS_INFO("Neighbours cluster: %i / %i, neighbours_in_cluster: %i, neighbour_clusters: %i",neighbours_clustered.size(),neighbours_not_clustered.size(),neighbours_in_cluster.size(),neighbour_clusters.size());
  }
  return neighbour_clusters;
}
std::vector<nav_msgs::Path> paths_from_clusters(nav_msgs::Path pathin,std::vector<std::vector<int>> clusters_in){
//  ROS_INFO("Clusters_in: %i, pathposes_in: %i",clusters_in.size(),pathin.poses.size());
  std::vector<nav_msgs::Path> path_clusters;
  for(int i = 0; i < clusters_in.size(); i++){
    if(clusters_in[i].size() > 1){
      nav_msgs::Path path_cluster;
      path_cluster.header = pathin.header;
      for(int k = 0; k < clusters_in[i].size(); k++){
        path_cluster.poses.push_back(pathin.poses[clusters_in[i][k]]);
      }
  //    ROS_INFO("Cluster: %i path_size: %i",i,path_cluster.poses.size());
      path_clusters.push_back(path_cluster);
    }
  }
  return path_clusters;
}
////////////////**********CLUSTERS**************///////////////////////

int getinpath_neighbours(nav_msgs::Path pathin,int i0,float radius){
  int count = 0;
  for(int i = 0; i < pathin.poses.size(); i++){
    if(get_dst2d(pathin.poses[i].pose.position,pathin.poses[i0].pose.position) <= radius)
      count++;
  }
  return count;
}


geometry_msgs::Point get_poly_centroidarea(geometry_msgs::PolygonStamped polyin){
    geometry_msgs::Point centroid;
    double signedArea = 0.0;
    double x0 = 0.0; // Current vertex X
    double y0 = 0.0; // Current vertex Y
    double x1 = 0.0; // Next vertex X
    double y1 = 0.0; // Next vertex Y
    double a = 0.0;  // Partial signed area

    // For all vertices except last
    int i=0;

    if(polyin.polygon.points.size() < 2){
      return centroid;
    }
    for (i=0; i<polyin.polygon.points.size()-1; ++i)
    {
        x0 = polyin.polygon.points[i].x;
        y0 = polyin.polygon.points[i].y;
        x1 = polyin.polygon.points[i+1].x;
        y1 = polyin.polygon.points[i+1].y;
        a = x0*y1 - x1*y0;
        signedArea += a;
        centroid.x += (x0 + x1)*a;
        centroid.y += (y0 + y1)*a;
    }

    // Do last vertex separately to avoid performing an expensive
    // modulus operation in each iteration.
    x0 = polyin.polygon.points[i].x;
    y0 = polyin.polygon.points[i].y;
    x1 = polyin.polygon.points[0].x;
    y1 = polyin.polygon.points[0].y;
    a = x0*y1 - x1*y0;
    signedArea += a;
    centroid.x += (x0 + x1)*a;
    centroid.y += (y0 + y1)*a;

    signedArea *= 0.5;
    centroid.x /= (6.0*signedArea);
    centroid.y /= (6.0*signedArea);
    centroid.z = signedArea;

    return centroid;
}

bool sort_dst_pair(const std::tuple<int,float>& a,
               const std::tuple<int,float>& b)
{
    return (std::get<1>(a) < std::get<1>(b));
}
geometry_msgs::PolygonStamped sort_poly_around_z(geometry_msgs::PolygonStamped polyin){
  geometry_msgs::PolygonStamped polyout;
  if(polyin.polygon.points.size() <= 1)
    return polyin;
  polyout.header = polyin.header;
  std::vector<std::tuple<int,float>>i_dst;
  for(int i = 0; i < polyin.polygon.points.size(); i++){
    i_dst.push_back(std::make_tuple(i,-polyin.polygon.points[i].z));
  }
  sort(i_dst.begin(),i_dst.end(),sort_dst_pair);
	for(int i = 0; i < i_dst.size(); i++){
    polyout.polygon.points.push_back(polyin.polygon.points[std::get<0>(i_dst[i])]);
  }
	return polyout;
}
geometry_msgs::PolygonStamped sort_poly_around_dst(geometry_msgs::PolygonStamped polyin,geometry_msgs::Point centroid){
  geometry_msgs::PolygonStamped polyout;
  if(polyin.polygon.points.size() <= 1)
    return polyin;
  polyout.header = polyin.header;
  std::vector<std::tuple<int,float>>i_dst;
  for(int i = 0; i < polyin.polygon.points.size(); i++){
		i_dst.push_back(std::make_tuple(i,get_dst3d32d(pos,polyin.polygon.points[i])));
  }
  sort(i_dst.begin(),i_dst.end(),sort_dst_pair);
	for(int i = 0; i < i_dst.size(); i++){
    polyout.polygon.points.push_back(polyin.polygon.points[std::get<0>(i_dst[i])]);
  }
	return polyout;
}
geometry_msgs::Point32 offset32(geometry_msgs::Point32 pin, float rad,float offset){
  geometry_msgs::Point32 t;
  t.z = pin.z;
  t.x = pin.x + offset * cos(rad);
  t.y = pin.y + offset * sin(rad);
  return t;
}
geometry_msgs::PolygonStamped project_poly(geometry_msgs::PolygonStamped polyin, float offset){
	if(polyin.polygon.points.size() < 3)
		return polyin;
	geometry_msgs::Point centroid;
	centroid = get_poly_centroidarea(polyin);
	centroid.z = polyin.polygon.points[0].z;

	for(int i = 0; i < polyin.polygon.points.size(); i++){
		float rad = get_hdng32d(polyin.polygon.points[i],centroid);
		polyin.polygon.points[i] = offset32(polyin.polygon.points[i],rad,offset);
	}
	return polyin;
}

geometry_msgs::PolygonStamped sort_poly_around_pnt(geometry_msgs::PolygonStamped polyin,geometry_msgs::Point pnt){
  geometry_msgs::PolygonStamped polyout;
  if(polyin.polygon.points.size() <= 1)
    return polyin;
  polyout.header = polyin.header;
  std::vector<std::tuple<int,float>>i_dst;
  for(int i = 0; i < polyin.polygon.points.size(); i++){
    i_dst.push_back(std::make_tuple(i,get_hdng32d(polyin.polygon.points[i],pnt)));
  }
  sort(i_dst.begin(),i_dst.end(),sort_dst_pair);
	for(int i = 0; i < i_dst.size(); i++){
    polyout.polygon.points.push_back(polyin.polygon.points[std::get<0>(i_dst[i])]);
  }
	return polyout;
}
nav_msgs::Path sort_path(nav_msgs::Path pathin,std::string sort_by){
  nav_msgs::Path pathout;
  if(pathin.poses.size() <= 1)
    return pathin;
  pathout.header = pathin.header;
  std::vector<std::tuple<int,float>>i_dst;
  for(int i = 0; i < pathin.poses.size(); i++){
		if(sort_by == "dst_2d")
      i_dst.push_back(std::make_tuple(i,get_dst2d(pos,pathin.poses[i].pose.position)));
		else if(sort_by == "dst_3d_ref")
			i_dst.push_back(std::make_tuple(i,get_dst3d(pnt_ref,pathin.poses[i].pose.position)));
		else if(sort_by == "hdng_3d_ref")
			i_dst.push_back(std::make_tuple(i,get_hdng(pathin.poses[i].pose.position,pnt_ref)));
		else if(sort_by == "dst_3d")
      i_dst.push_back(std::make_tuple(i,get_dst3d(pos,pathin.poses[i].pose.position)));
		else if(sort_by == "hdng_rel")
			i_dst.push_back(std::make_tuple(i,get_shortest(get_hdng(pathin.poses[i].pose.position,pos),pos_yaw)));
    else if(sort_by == "hdng")
      i_dst.push_back(std::make_tuple(i,get_hdng(pathin.poses[i].pose.position,pos)));
		else if(sort_by == "building_rad")
			i_dst.push_back(std::make_tuple(i,get_hdng(pathin.poses[i].pose.position,building_centroid.point)));
	    else if(sort_by == "z")
      i_dst.push_back(std::make_tuple(i,pathin.poses[i].pose.position.z));
  }
  sort(i_dst.begin(),i_dst.end(),sort_dst_pair);
	for(int i = 0; i < i_dst.size(); i++){
    pathout.poses.push_back(pathin.poses[std::get<0>(i_dst[i])]);
  }
	return pathout;
}

bool in_poly(geometry_msgs::PolygonStamped polyin, float x, float y)
{
  int cross = 0;

  for(int i = 0,j = polyin.polygon.points.size()-1; i < polyin.polygon.points.size(); j=i++){
//  for (int i = 0, j = vertices_.size() - 1; i < vertices_.size(); j = i++) {
    if ( ((polyin.polygon.points[i].y > y) != (polyin.polygon.points[j].y > y))
           && (x < (polyin.polygon.points[j].x - polyin.polygon.points[i].x) * (y - polyin.polygon.points[i].y) /
            (polyin.polygon.points[j].y - polyin.polygon.points[i].y) + polyin.polygon.points[i].x) )
    {
      cross++;
    }
  }
  return bool(cross % 2);
}
nav_msgs::Path constrain_path_bbpoly(nav_msgs::Path pathin,geometry_msgs::PolygonStamped poly_bb,bool get_in_poly){
  nav_msgs::Path path_inpoly,path_outpoly;
  path_outpoly.header = path_inpoly.header = hdr();
  if(pathin.poses.size() == 0 || poly_bb.polygon.points.size() == 0)
    return pathin;
  for(int i = 0; i < pathin.poses.size(); i++){
    if(in_poly(poly_bb,pathin.poses[i].pose.position.x,pathin.poses[i].pose.position.y))
      path_inpoly.poses.push_back(pathin.poses[i]);
		else
			path_outpoly.poses.push_back(pathin.poses[i]);
  }
	if(get_in_poly)
  	return path_inpoly;
	else
		return path_outpoly;
}
cv::Scalar get_color(int base_blue,int base_green, int base_red){
	cv::Scalar color;
	color[0] = base_blue;
	color[1] = base_green;
	color[2] = base_red;
	return color;
}

void draw_rectangle(geometry_msgs::Point midpoint,float size, cv::Scalar color){
  geometry_msgs::Point p0,p1;
  p0.x = midpoint.x-size; p1.x = midpoint.x+size*2;
  p0.y = midpoint.y-size; p1.y = midpoint.y+size*2;
  cv::rectangle(img, pnt2cv(p0),pnt2cv(p1),color,1,8,0);
}
void draw_line(geometry_msgs::Point p0,float a,float r,cv::Scalar color){
  geometry_msgs::Point pyaw;
  pyaw.x = p0.x + r * cos(a);
  pyaw.y = p0.y + r * sin(a);
  cv::line (img, pnt2cv(p0), pnt2cv(pyaw),color,1,cv::LINE_8,0);
}

void draw_pnt(geometry_msgs::Point pnt, int rectangle_size, int circle_size, int yaw_size, bool pixel,float yaw, cv::Scalar color){
	if(circle_size > 0)
		cv::circle(img,pnt2cv(pnt),circle_size,color,1);
	if(rectangle_size > 0)
		draw_rectangle(pnt,rectangle_size,color);
	if(yaw_size > 0)
		draw_line(pnt,yaw,yaw_size,color);
	if(pixel){
		if(color[0] > 0)
			img.at<cv::Vec3b>( y2r(pnt.y),x2c(pnt.x) )[0] = color[0];
		if(color[1] > 0)
			img.at<cv::Vec3b>( y2r(pnt.y),x2c(pnt.x) )[1] = color[1];
		if(color[2] > 0)
			img.at<cv::Vec3b>( y2r(pnt.y),x2c(pnt.x) )[2] = color[2];
	}
}
void draw_info(std::string info, int rectangle_size, int circle_size, int yaw_size, bool pixel,float yaw, cv::Scalar color){
	info_point.y -= info_interval;
	geometry_msgs::Point info_point_beside;
	info_point_beside.x = info_point.x - info_interval / 2;
	info_point_beside.y = info_point.y + info_interval / 2;
	draw_pnt(info_point_beside,rectangle_size,circle_size,yaw_size,pixel,yaw,color);
	putText(img,info,pnt2cv(info_point),
			cv::FONT_HERSHEY_COMPLEX_SMALL, 0.8, color, 1, CV_AA);
}
void draw_path(std::string info,nav_msgs::Path pathin, int rectangle_size, int circle_size, int yaw_size, bool pixel, cv::Scalar color, bool draw_line){
	ROS_INFO("DRAWING PATH: %s %i",info.c_str(),pathin.poses.size());
	if(pathin.poses.size() == 0)
		return;
	for(int i = 0; i < pathin.poses.size(); i++){
		draw_pnt(pathin.poses[i].pose.position,rectangle_size,circle_size,yaw_size,pixel,tf::getYaw(pathin.poses[i].pose.orientation),color);
		if(draw_line && i > 0)
			cv::line (img, pnt2cv(pathin.poses[i-1].pose.position), pnt2cv(pathin.poses[i].pose.position),color,1,cv::LINE_8,0);
	}
	draw_info(info,rectangle_size,circle_size,yaw_size,pixel,0.0,color);
}
void draw_path_old(nav_msgs::Path pathin,cv::Scalar color, int size){
	if(pathin.poses.size() == 0)
		return;
	for(int i = 0; i < pathin.poses.size(); i++){
		geometry_msgs::Point pnt = pathin.poses[i].pose.position;
    if(size == 0){
			if(color[0] > 0)
      	img.at<cv::Vec3b>( y2r(pnt.y),x2c(pnt.x) )[0] = color[0];
			if(color[1] > 0)
      	img.at<cv::Vec3b>( y2r(pnt.y),x2c(pnt.x) )[1] = color[1];
			if(color[2] > 0)
				img.at<cv::Vec3b>( y2r(pnt.y),x2c(pnt.x) )[2] = color[2];
    }
    else
      cv::circle(img,pnt2cv(pnt),size,color,1);
		draw_line(pnt,tf::getYaw(pathin.poses[i].pose.orientation),3,color);
	}
}





void draw_poly(std::string info, geometry_msgs::PolygonStamped poly, int rectangle_size, int circle_size, int yaw_size, bool pixel, cv::Scalar color, bool draw_lines){
	ROS_INFO("DRAWING PATH: %s %i",info.c_str(),poly.polygon.points.size());
	if(poly.polygon.points.size() < 3)
		return;
	for(int i = 0; i < poly.polygon.points.size(); i++){
		geometry_msgs::Point p;
		p.x = poly.polygon.points[i].x;
		p.y = poly.polygon.points[i].y;
		draw_pnt(p,rectangle_size,circle_size,0,pixel,0,color);
		if(draw_lines && i > 0)
				cv::line (img, pnt322cv(poly.polygon.points[i-1]), pnt322cv(poly.polygon.points[i]),color,1,cv::LINE_8,0);
	}
	draw_info(info,rectangle_size,circle_size,yaw_size,pixel,0.0,color);
}
void draw_pose(std::string info,geometry_msgs::Point pnt, int rectangle_size, int circle_size, int yaw_size, bool pixel, float yaw,cv::Scalar color){
	draw_pnt(pnt,rectangle_size,circle_size,yaw_size,pixel,yaw,color);
	draw_info(info,rectangle_size,circle_size,yaw_size,pixel,yaw,color);
	if(info == "pos"){
		draw_line(pos,constrainAngle(pos_yaw + M_PI/3),50,color);
		draw_line(pos,constrainAngle(pos_yaw - M_PI/3),50,color);
	}
}
std::vector<geometry_msgs::PolygonStamped> find_clusters(geometry_msgs::PolygonStamped polyin, float cluster_spacing_cutoff){
	ROS_INFO("FIND CLUSTERS");
	std::vector<geometry_msgs::PolygonStamped> clustersout;
	geometry_msgs::PolygonStamped polyout;
	for(int i = 0; i < polyin.polygon.points.size(); i++){
		if(get_dst3d32(polyin.polygon.points[i],polyin.polygon.points[i-1]) < cluster_spacing_cutoff)
			polyout.polygon.points.push_back(polyin.polygon.points[i]);
		else if(polyout.polygon.points.size() > 0){
			clustersout.push_back(polyout);
			polyout.polygon.points.resize(0);
		}
	}
	return clustersout;
}
geometry_msgs::PolygonStamped reorganize_poly(geometry_msgs::PolygonStamped polyin){
	return sort_poly_around_pnt(polyin,get_ave_pnt_poly(polyin));
}
std::vector<geometry_msgs::PolygonStamped> reorganize_polys(std::vector<geometry_msgs::PolygonStamped> polysin){
	for(int i = 0; i < polysin.size(); i++){
		polysin[i] = reorganize_poly(polysin[i]);
	}
	return polysin;
}
std::vector<geometry_msgs::Point> getinpath_boundingbox(nav_msgs::Path pathin){
  std::vector<geometry_msgs::Point> bbmnbbmx;

  geometry_msgs::Point bbtotmax,bbtotmin;
  bbtotmax.x = bbtotmax.y = bbtotmax.z = -100;
  bbtotmin.x = bbtotmin.y = bbtotmin.z = 100;
  for(int i = 0; i < pathin.poses.size(); i++){
    if(pathin.poses[i].pose.position.x > bbtotmax.x)bbtotmax.x = pathin.poses[i].pose.position.x;
    if(pathin.poses[i].pose.position.y > bbtotmax.y)bbtotmax.y = pathin.poses[i].pose.position.y;
    if(pathin.poses[i].pose.position.z > bbtotmax.z)bbtotmax.z = pathin.poses[i].pose.position.z;
    if(pathin.poses[i].pose.position.x < bbtotmin.x)bbtotmin.x = pathin.poses[i].pose.position.x;
    if(pathin.poses[i].pose.position.y < bbtotmin.y)bbtotmin.y = pathin.poses[i].pose.position.y;
    if(pathin.poses[i].pose.position.z < bbtotmin.z)bbtotmin.z = pathin.poses[i].pose.position.z;
  }
  bbmnbbmx.push_back(bbtotmin);
  bbmnbbmx.push_back(bbtotmax);

  float diag = sqrt(pow(bbtotmax.x-bbtotmin.x,2)+pow(bbtotmax.y-bbtotmin.y,2)+pow(bbtotmax.z-bbtotmin.z,2));
  //ROS_INFO("GENERICNODE: LIMITS: diagonal: %.2f,max(%.2f %.2f %.2f) min(%.2f %.2f %.2f)",diag,bbtotmax.x,bbtotmax.y,bbtotmax.z,bbtotmin.x,bbtotmin.y,bbtotmin.z);
  return bbmnbbmx;
}

void draw_path_boundary(nav_msgs::Path pathin,cv::Scalar color){
	if(pathin.poses.size() == 0)
		return;
	std::vector<geometry_msgs::Point> bbmnmx = getinpath_boundingbox(pathin);
	geometry_msgs::Point bbmin_scan = bbmnmx[0];
	geometry_msgs::Point bbmax_scan = bbmnmx[1];
	cv::rectangle(img, pnt2cv(bbmin_scan),pnt2cv(bbmax_scan),color,1,8,0);
}
void draw_and_reset_img(){
	info_point.x = 100;
	info_point.y = 100;
	cv::Mat img_new2;
	cv::resize(img, img_new2, cv::Size(), 2.0, 2.0);
	cnt3++;
	img_blank.copyTo(img);
	cv::imwrite("/home/nuc/brain/fullpath/"+std::to_string(cnt3)+"navigator.png",img_new2);

}
cv::Scalar get_shifting_color(int count,int color_intensity){
  cv::Scalar color;
  if(count == 0)
    color[0] = color_intensity;
  else if(count == 1)
    color[1] = color_intensity;
  else
    color[2] = color_intensity;
  return color;
}

int get_float(float val){
	return int(round(val));
}
std::string write_xyz(geometry_msgs::Point pnt){
	return " x: " + std::to_string(get_float(pnt.x)) + " x: " + std::to_string(get_float(pnt.y)) + " z: " + std::to_string(get_float(pnt.z));
}
void draw_polys(std::string info,std::vector<geometry_msgs::PolygonStamped> polysin,int color_strength){
	if(polysin.size() == 0)
		return;
	int count_color = 0;
	for(int i = 0; i < polysin.size(); i++){
		if(polysin[i].polygon.points.size() > 2){
			geometry_msgs::Point p_cen = get_ave_pnt_poly(polysin[i]);
			std::string s1 = std::to_string(polysin[i].polygon.points.size());
			std::string s2 = write_xyz(p_cen);
			std::string s3 = std::to_string(i);

			std::string info_new = info + s1 + s2 + s3;
			ROS_INFO("3");
			count_color++;
			if(count_color == 3)
				count_color = 0;
			draw_poly(info,polysin[i],0,0,0,true,get_shifting_color(count_color,color_strength),true);
		}
	}
}
nav_msgs::Path get_streaking_building_path(nav_msgs::Path pathin){
	int num_is = 64;
	float tot_rads = M_PI * 2;
	float rads_pr_i = tot_rads / num_is;

	std::vector<std::vector<int>> ps_at_hdng;
	ps_at_hdng.resize(num_is);
	nav_msgs::Path pathout;
	pathin = sort_path(pathin,"building_rad");
	for(int i = 0; i < pathin.poses.size(); i++){
		float cand_hdng = get_hdng(pathin.poses[i].pose.position,building_centroid.point);
		int hdng_icand = int(round((cand_hdng + M_PI) / rads_pr_i));
		if(hdng_icand >= num_is)
			hdng_icand -= num_is;
		if(hdng_icand < 0)
			hdng_icand += num_is;

//		ROS_INFO("I: %i hdng_icand: %i gives %.2f degrees (was %.2f) ",i,hdng_icand,-M_PI+hdng_icand*rads_pr_i,cand_hdng);
		ps_at_hdng[hdng_icand].push_back(i);
	//	ROS_INFO("Path[%i]: %.2f -> %i",i,cand_hdng,hdng_icand);
	}
	//ROS_INFO("DONE");
	for(int i = 0; i < num_is; i++){
		nav_msgs::Path path_connected;
	//	ROS_INFO("I: %i",i);
		if(ps_at_hdng[i].size() > 0){
	//		ROS_INFO("ps_at_hdng[%i]: %i",i,ps_at_hdng[i].size());
			for(int k = 0; k < ps_at_hdng[i].size(); k++){
				path_connected.poses.push_back(pathin.poses[ps_at_hdng[i][k]]);
			}
	//		ROS_INFO("%i poses at hdng[%i] ",path_connected.poses.size(),i);
			geometry_msgs::PoseStamped ps = path_connected.poses[getinpath_closestindex2d(path_connected,building_centroid.point)];
			ps.header = hdr();
			pathout.poses.push_back(ps);
		}
	}
//	ROS_INFO("RETURNING: %i poses",pathout.poses.size());
	return pathout;
}
void drawimg(){
	for(int r = 0; r < img.rows; r++){
		for(int c = 0; c < img.cols; c++){
			img.at<cv::Vec3b>(r,c)[0] = img_height.at<cv::Vec3b>(r,c)[0]*10;
			img.at<cv::Vec3b>(r,c)[1] = img_height.at<cv::Vec3b>(r,c)[1]*10;
			img.at<cv::Vec3b>(r,c)[2] = img_height.at<cv::Vec3b>(r,c)[2]*10;
		}
	}
	ROS_INFO("polys_obstacles_received: %i",polys_obstacles_received.size());
	if(polys_obstacles_received.size() > 0){
		ROS_INFO("Drawing initial_polys: %i", polys_obstacles_received.size());
		draw_polys("initial_polys#:",polys_obstacles_received,50);
	}
	ROS_INFO("polys_heightimage_clustered: %i",polys_heightimage_clustered.size());
	if(polys_heightimage_clustered.size() > 0){
		ROS_INFO("Drawing structures: %i", polys_heightimage_clustered.size());
		draw_polys("structures#:",polys_heightimage_clustered,50);
	}
	draw_poly("poly_legal",poly_legal,1,2,0,false,cv::Scalar(0,200,25),true);
	draw_poly("poly_illegal",poly_illegal,2,1,0,false,cv::Scalar(27,22,25),true);
	draw_path("path_visited",path_visited,0,1,0,true,cv::Scalar(0,25,100),true);
	draw_path("path_building_active",path_building_active,0,1,1,true,cv::Scalar(100,43,20),false);
	draw_path("path_active_reduced",path_active_reduced,1,0,1,true,cv::Scalar(100,43,200),false);
//	draw_path("path_organize_1",path_organize_1,0,0,1,true,cv::Scalar(100,43,20),false);
//	draw_path("path_organize_2",path_organize_2,0,0,1,true,cv::Scalar(41,100,40),false);
	draw_path("targets",path_targets,0,0,1,false,cv::Scalar(200,100,45),true);

	draw_pose("target"+std::to_string(get_float(get_dst3d(target_pose.pose.position,pos))),target_pose.pose.position,2,2,5,false,tf::getYaw(target_pose.pose.orientation),cv::Scalar(200,200,100));
	draw_pose("pos",pos,2,2,5,false,pos_yaw,cv::Scalar(200,200,200));
	draw_pose("building",building_centroid.point,2,2,0,true,0,cv::Scalar(0,100,25));

	draw_poly("cleared",poly_cleared,0,0,0,false,cv::Scalar(0,200,25),true);
	draw_poly("poly_obstacles_raw",poly_obstacles_received,0,0,0,true,cv::Scalar(100,0,0),false);
	ROS_INFO("***************done drawing****************");
	draw_and_reset_img();
}

float get_zn(float z_target){
	return (floor(z_target / par_dz*2) + 1);
}
float get_rounded_z(float z_target){
	return par_dz * 2 * get_zn(z_target);
}
int get_elevation_at_xy(float x, float y, int radlen_xy){
  int zmx = 0;
  for(int c = x2c(x)-radlen_xy; c < x2c(x)+radlen_xy; c++){
    for(int r = y2r(y)-radlen_xy; r < y2r(y)+radlen_xy; r++){
      if(fmax(img_height.at<cv::Vec3b>(r,c)[1],img_height.at<cv::Vec3b>(r,c)[2]) > zmx)
				zmx = fmax(img_height.at<cv::Vec3b>(r,c)[1],img_height.at<cv::Vec3b>(r,c)[2]);
    }
  }
  return zmx;
}


int get_area_coverage(geometry_msgs::Point midpoint, int radlen_xy){
  int pnts = 0; int pnts_tot = 0;
	for(int c = x2c(midpoint.x)-radlen_xy; c < x2c(midpoint.x)+radlen_xy; c++){
    for(int r = y2r(midpoint.y)-radlen_xy; r < y2r(midpoint.y)+radlen_xy; r++){
      pnts_tot++;
      if(img_height.at<cv::Vec3b>(r,c)[1] > 0)
        pnts++;
    }
  }
  return 100 * pnts / (pnts_tot+1);
}

//img_side_delta.at<uchar>(r,c) = img_side_delta.at<cv::Vec3b>(r,c)[1];
geometry_msgs::PolygonStamped reduce_poly_to_minimal(geometry_msgs::PolygonStamped polyin){
	geometry_msgs::Point poly_centroid1 = get_poly_centroidarea(polyin);
	geometry_msgs::Point poly_average = get_ave_pnt_poly(polyin);
	geometry_msgs::Point32 poly_centroid;
	poly_centroid.x = poly_centroid1.x;
	poly_centroid.y = poly_centroid1.y;
	poly_centroid.z = poly_average.z;
	float poly_area = poly_centroid1.z;
	pnt_ref = poly_centroid1;
	ROS_INFO("ANALYZING POLYGONS: [p-size: %i] - poly_centroid: %.0f %.0f %.0f, area: %.0f",polyin.polygon.points.size(),poly_centroid.x,poly_centroid.y,poly_centroid.z,poly_area);
	ROS_INFO("ANALYZING POLYGONS: [p-size: %i] - poly_average:  %.0f %.0f %.0f, area: %.0f",polyin.polygon.points.size(),poly_average.x,poly_average.y,poly_average.z,poly_area);
		polyin = sort_poly_around_pnt(polyin,poly_centroid1);
	geometry_msgs::PolygonStamped polyout;
	geometry_msgs::Point32 p_current;
	int num_rads = 8;
	float rads_pr_i = 2*M_PI / num_rads;
	float current_a = -M_PI;
	float next_a = current_a + rads_pr_i;
	float dst_max = 0;
	for(int i = 0; i < polyin.polygon.points.size(); i++){
		float hdng = get_hdng32(polyin.polygon.points[i],poly_centroid);
		float dst  = get_dst2d32(polyin.polygon.points[i],poly_centroid);
		if(hdng > next_a){
			current_a = next_a;
			next_a = current_a + rads_pr_i;
			dst_max = 0;
			polyout.polygon.points.push_back(polyin.polygon.points[i]);
		}
		if(dst > dst_max){
			p_current = polyin.polygon.points[i];
			dst_max = dst;
		}
	}

	geometry_msgs::Point poly_centroid2 = get_poly_centroidarea(polyout);
	float poly_area2 = poly_centroid2.z;

	ROS_INFO("POLYGON reduce_poly_to_minimal %i -> %i",polyin.polygon.points.size(),polyout.polygon.points.size());
	ROS_INFO("POLYGON reduce_poly_to_minimal centroid: %.0f %.0f -> %.0f %.0f",poly_centroid1.x,poly_centroid1.y,poly_centroid2.x,poly_centroid2.y);
	ROS_INFO("POLYGON reduce_poly_to_minimal area    %.0f -> %.0f",poly_area,poly_area2);
	return polyout;
}
int floodfill(int r0, int c0, int fillcolor){
	int ffillMode = 1;  int connectivity = 8;  int newMaskVal = 255;
	int flags = connectivity + (newMaskVal << 8) +
			 (ffillMode == 1 ? cv::FLOODFILL_FIXED_RANGE : 0);
	cv::Rect ccomp;
	int area = cv::floodFill(img_paths,cv::Point(c0,r0),cv::Scalar(fillcolor), &ccomp, cv::Scalar(1),cv::Scalar(1), flags);
	return area;
}
int floodfill_building(geometry_msgs::Point32 p0, int fillcolor,int lo,int hi){
	int ffillMode = 1;  int connectivity = 8;  int newMaskVal = 255;
	int flags = connectivity + (newMaskVal << 8) +
			 (ffillMode == 1 ? cv::FLOODFILL_FIXED_RANGE : 0);
	cv::Rect ccomp;
//	cv::Rect rect_roi(seed_c-roi_rad,seed_r-roi_rad,roi_rad*2,roi_rad*2);
	return cv::floodFill(img_ff_bld,cv::Point(x2c(p0.x),y2r(p0.y)),cv::Scalar(fillcolor), &ccomp, cv::Scalar(lo),cv::Scalar(hi), flags);
}
int get_hi(float pz,float zn){
	return int(round(zn-pz-1));
}
int get_lo(float pz,float z0){
	return int(round(pz-z0+1));
}

void color_around_pathpixel(int r0, int c0,int radlen_xy,int color){
	img_paths.at<uchar>(r0,c0) = color;
	for(int c = c0-radlen_xy; c < c0+radlen_xy; c++){
    for(int r = r0-radlen_xy; r < r0+radlen_xy; r++){
			img_paths.at<uchar>(r,c) = color;
		}
	}
}
void color_around_pathpos(float x, float y,int radlen_xy,int color){
	img_paths.at<uchar>(y2r(y),x2c(x)) = color;
	for(int c = x2c(x)-radlen_xy; c < x2c(x)+radlen_xy; c++){
		for(int r = y2r(y)-radlen_xy; r < y2r(y)+radlen_xy; r++){
			img_paths.at<uchar>(r,c) = color;
		}
	}
}
void color_around_bldpixel(int r0, int c0,int radlen_xy,int color){
	img_ff_bld.at<uchar>(r0,c0) = color;
	for(int c = c0-radlen_xy; c < c0+radlen_xy; c++){
    for(int r = r0-radlen_xy; r < r0+radlen_xy; r++){
			img_ff_bld.at<uchar>(r,c) = color;
		}
	}
}
void color_around_bldpos(float x, float y,int radlen_xy,int color){
	img_ff_bld.at<uchar>(y2r(y),x2c(x)) = color;
	for(int c = x2c(x)-radlen_xy; c < x2c(x)+radlen_xy; c++){
		for(int r = y2r(y)-radlen_xy; r < y2r(y)+radlen_xy; r++){
			img_ff_bld.at<uchar>(r,c) = color;
		}
	}
}



void update_path_image(nav_msgs::Path pathin,int inflation_radlen_xy,int color){
	img_blank_mono.copyTo(img_paths);
	for(int i = 0; i < pathin.poses.size(); i++){
		color_around_pathpixel(y2r(pathin.poses[i].pose.position.y),x2c(pathin.poses[i].pose.position.x),inflation_radlen_xy,color);
	}
	cv::imwrite("/home/nuc/brain/raw/"+std::to_string(cnt3)+"path_raw.png",img_paths);
}
geometry_msgs::PolygonStamped update_building_image(geometry_msgs::Point midpoint,int radlen_xy,int color_building,int inflation_radlen_xy){
	//ROS_INFO("UPDATE_BUILDING_IMAGE: midpoint: x %.0f y %.0f z %.0f, radlen_xy: %i, color: %i,inflation_radlen_xy: %i",midpoint.x,midpoint.y,midpoint.z,radlen_xy,color_building,inflation_radlen_xy);
	geometry_msgs::PolygonStamped polyout;
	polyout.header = hdr();
	img_blank_mono.copyTo(img_ff_bld);
	float closest_dst = 100;
	geometry_msgs::Point32 closest_pnt;
	for(int c = x2c(midpoint.x)-radlen_xy; c < x2c(midpoint.x)+radlen_xy; c++){
		for(int r = y2r(midpoint.y)-radlen_xy; r < y2r(midpoint.y)+radlen_xy; r++){
			if(img_height.at<cv::Vec3b>(r,c)[1] > 5){
				geometry_msgs::Point32 pnt;
				pnt.x = c2x(c);
				pnt.y = r2y(r);
				pnt.z = img_height.at<cv::Vec3b>(r,c)[1];
				if(get_dst2d32d(building_centroid.point,pnt) < closest_dst){
					closest_pnt = pnt;
					closest_dst = get_dst2d32d(building_centroid.point,pnt);
				}
				polyout.polygon.points.push_back(pnt);
		//		ROS_INFO("COLOR:");
				color_around_bldpixel(r,c,inflation_radlen_xy,color_building);
			}
		}
	}
	if(polyout.polygon.points.size() > 0)
		polyout.polygon.points[0] = closest_pnt;
//	for(int i = 0; i < polyout.polygon.points.size(); i++){
//		color_around_bldpixel(y2r(polyout.polygon.points[i].y),x2c(polyout.polygon.points[i].x),0,255);
//	}
	cv::imwrite("/home/nuc/brain/raw/"+std::to_string(cnt3)+"building_raw.png",img_ff_bld);
	return polyout;
}

nav_msgs::Path get_connected_path(nav_msgs::Path pathin,geometry_msgs::Point pnt){
	ROS_INFO("get_active_cluster");
	int radlen_xy = 200;
	int fillcolor = 50;
	int color_paths = 100;
	int inflation_radlen_xy = 1;
	nav_msgs::Path pathout;
	pathout.header = hdr();
	update_path_image(pathin,inflation_radlen_xy,color_paths);
	geometry_msgs::Point p0 = pathin.poses[get_closest_i(pathin,target_pose.pose.position)].pose.position;
	int area = floodfill(y2r(p0.y),x2c(p0.x),fillcolor);
	for(int i = 0; i < pathin.poses.size(); i++){
		if(img_paths.at<uchar>(y2r(pathin.poses[i].pose.position.y),x2c(pathin.poses[i].pose.position.x)) == fillcolor)
			pathout.poses.push_back(pathin.poses[i]);
	}
	ROS_INFO("get_connected_path: %i of %i points ",pathin.poses.size(),pathout.poses.size());
	cv::imwrite("/home/nuc/brain/clusters/"+std::to_string(cnt3)+"img_paths.png",img_paths);
	return pathout;
}
int find_first_building_pixel(geometry_msgs::PolygonStamped polyin,int color_building){
	if(polyin.polygon.points.size() == 0)
		return -1;
	for(int i = 0; i < polyin.polygon.points.size(); i++){
		if(img_ff_bld.at<uchar>(y2r(polyin.polygon.points[i].y),x2c(polyin.polygon.points[i].x)) == color_building)
			return i;
	}
	return -1;
}
geometry_msgs::PolygonStamped get_connected_building(){
	ROS_INFO("get_connected_building");
	int radlen_xy = 200;
	int fillcolor = 200;
	int color_building = 25;
	int inflation_radlen_xy = 4;
	geometry_msgs::PolygonStamped poly_output,poly_remain;
	geometry_msgs::PolygonStamped poly_input = update_building_image(building_centroid.point,radlen_xy,color_building,inflation_radlen_xy);
	img_ff_bld.copyTo(img_ff_bld_backup);
	if(poly_input.polygon.points.size() == 0)
		return poly_output;
	int closest_i_poly = getinpath_closestindex2d_poly(poly_input,building_centroid.point);
	if(closest_i_poly == -1)
		return poly_output;
	int i0 = find_first_building_pixel(poly_input,color_building);
	if(i0 == -1)
		return poly_output;
	int area = floodfill_building(poly_input.polygon.points[0],fillcolor,1,1);
	ROS_INFO("AREA: %i",area);
	for(int i = 0; i < poly_input.polygon.points.size(); i++){
		if(img_ff_bld.at<uchar>(y2r(poly_input.polygon.points[i].y),x2c(poly_input.polygon.points[i].x)) == fillcolor){
			poly_output.polygon.points.push_back(poly_input.polygon.points[i]);
		}
	}
	ROS_INFO("FFILL-type: %i: pnts %i out",poly_input.polygon.points.size(),poly_output.polygon.points.size());
	poly_output.header = hdr();
	cv::imwrite("/home/nuc/brain/clusters/"+std::to_string(cnt3)+"img_ff_bld_out.png",img_ff_bld);
	return poly_output;
}
/////*************************PATH*************///////////


geometry_msgs::Point get_ave_pnt_ni(nav_msgs::Path pathin,int first_i,int last_i){
  geometry_msgs::Point pnt;
  int n = fmin(last_i,pathin.poses.size());
  for(int i = first_i; i < int(n); i++){
    pnt.x += pathin.poses[i].pose.position.x;
    pnt.y += pathin.poses[i].pose.position.y;
    pnt.z += pathin.poses[i].pose.position.z;
  }
	if(n - first_i > 0){
	  pnt.x /= (n-first_i);
	  pnt.y /= (n-first_i);
	  pnt.z /= (n-first_i);
	}
  return pnt;
}
float get_zmax_ni(nav_msgs::Path pathin,int last_i){
  float zmx = 0;
  for(int i = 0; i < pathin.poses.size(); i++){
    if(pathin.poses[i].pose.position.z > zmx){
      zmx = pathin.poses[i].pose.position.z;
    }
  }
  return zmx;
}

/////*************************PATH*************///////////



nav_msgs::Path get_targets_remaining(){
  nav_msgs::Path pathout;
  pathout.header = hdr();
  for(int i = path_targets_i; i < path_targets.poses.size(); i++){
    pathout.poses.push_back(path_targets.poses[i]);
  }
  return pathout;
}
void check_elev_and_publish(){
	float zmin_xy = fmax(get_elevation_at_xy(target_xyz.point.x,target_xyz.point.y,1),get_elevation_at_xy(cmd_pos.x,cmd_pos.y,1));
	if(zmin_xy > target_xyz.point.z - 2.0)
		target_xyz.point.z = zmin_xy + 2.0;
	pub_setp_xyz.publish(target_xyz);
}

void set_target_pose(geometry_msgs::PoseStamped pose){
	target_heading = get_hdng(pose.pose.position,target_pose.pose.position);
	target_pose_last = target_pose;
	target_pose = pose;
	last_pose_yaw = tf::getYaw(pose.pose.orientation);
	path_targets_sent.poses.push_back(target_pose);
	target_xyz.point   = target_pose.pose.position;
	target_xyz.header  = hdr();
	target_pose.header = hdr();
	if(state_building == "inspecting_building")
		path_building_targets.poses.push_back(target_pose);
	float dst = get_dst2d(target_pose.pose.position,pos);
//	if(dst < 15)
		check_elev_and_publish();
	//else
//			pub_target.publish(target_pose);
}
nav_msgs::Path process_path_candidates(nav_msgs::Path pathin){
	if(pathin.poses.size()==0)
		return pathin;

	nav_msgs::Path new_path     = get_new_path(path_visited,pathin,5.0,true);
	nav_msgs::Path cleared_path = constrain_path_bbpoly(new_path,poly_cleared_extended,true);
//	nav_msgs::Path path_hdng    = constrain_path_bbpoly(cleared_path,poly_heading,true);
	nav_msgs::Path path_hdng = sort_path(cleared_path,"dst_2d");
	ROS_INFO("Target path: pathin: %i ->clear-> %i ->hdng-> %i poses - new target: %i poses i: %i",pathin.poses.size(),cleared_path.poses.size(),path_targets.poses.size(),path_targets_i);
	return path_hdng;
}


void set_path_target_i(){
	ROS_INFO("Path_target_i0: %i",path_targets_i);
	path_targets_i = 0;
	if(path_targets.poses.size() == 0)
		return;
	float dst_target = get_dst3d(path_targets.poses[path_targets_i].pose.position,cmd_pos);
	while(dst_target < 5 && path_targets.poses.size() > (path_targets_i+1)){
		ROS_INFO("Dst target[%i/%i]: %.0f,",path_targets_i,path_targets.poses.size(),dst_target);
		path_targets_i++;
		dst_target = get_dst3d(path_targets.poses[path_targets_i].pose.position,cmd_pos);
	}
	ROS_INFO("Path_target_i: %i",path_targets_i);
}
void set_target_path(nav_msgs::Path pathin){
	if(pathin.poses.size() == 0)
		return;
//	path_targets = process_path_candidates(pathin);
	set_path_target_i();
	if(path_targets.poses.size() > path_targets_i)
		set_target_pose(path_targets.poses[path_targets_i]);
}
void next_target(){
//	set_target_path(path_targets);
}

nav_msgs::Path organize_path_around_building(nav_msgs::Path pathin){
	if(state_building == "inspecting_building"){
		geometry_msgs::PoseStamped ps_v2 = path_visited.poses[path_visited.poses.size()-1];
		geometry_msgs::PoseStamped ps_v1 = path_visited.poses[path_visited.poses.size()-2];
		float abs_hdng1 = get_hdng(ps_v1.pose.position,building_centroid.point);
		float abs_hdng2 = get_hdng(ps_v2.pose.position,building_centroid.point);
		pathin = sort_path(pathin,"building_rad");
		int num_is = 32;
		float tot_rads = M_PI * 2;
		float rads_pr_i = tot_rads / num_is;
		int hdngi1 = int(round((abs_hdng1 + M_PI) / rads_pr_i));
		int hdngi2 = int(round((abs_hdng2 + M_PI) / rads_pr_i));

		float yaw1 = tf::getYaw(ps_v1.pose.orientation);
		float yaw2 = tf::getYaw(ps_v2.pose.orientation);
		float hdng = get_hdng(ps_v2.pose.position,ps_v1.pose.position);
		float dyaw  = get_shortest(yaw2,yaw1);
		ROS_INFO("					 yaw1 -> yaw2: %.2f -> %.2f (%.2f)",yaw1,yaw2,dyaw);
		ROS_INFO("abs_hdng1 -> abs_hdng2:  %.2f -> %.2f (%.2f)",abs_hdng1,abs_hdng2,get_shortest(abs_hdng2,abs_hdng1));
		}
	}


geometry_msgs::PointStamped transformpoint(geometry_msgs::PointStamped pin,std::string frame_out){
  geometry_msgs::PointStamped pout;
  pin.header.stamp = ros::Time();
  try
  {
    pout = tfBuffer.transform(pin, frame_out);
  }
  catch (tf2::TransformException &ex)
  {
        ROS_WARN("Failure %s\n", ex.what());
  }
  return pout;
}
void get_poly_legal_illegal(){
	if(path_building_targets.poses.size() >= 2){
		float hdng = get_hdng(cmd_pos,pos);
		float yaw0 = tf::getYaw(path_building_targets.poses[0].pose.orientation);
		float yaw1 = tf::getYaw(path_building_targets.poses[path_building_targets.poses.size()-1].pose.orientation);
		geometry_msgs::Point pnt2 = path_building_targets.poses[0].pose.position;
		geometry_msgs::Point pnt1 = path_building_targets.poses[path_building_targets.poses.size()-1].pose.position;
		float hdng1 = get_hdng(pnt1,building_centroid.point);
		float hdng2 = get_hdng(pnt2,building_centroid.point);
		float dst1 = get_hdng(pnt1,building_centroid.point);
		float dst2 = get_hdng(pnt2,building_centroid.point);
		float dst   = get_dst2d(pnt2,pnt1);
		float drad  = get_shortest(hdng2,hdng1);
		float m_pr_rad = dst / drad;
		poly_illegal.polygon.points.resize(4);
		poly_legal.polygon.points.resize(4);
		float drad_target = 10/m_pr_rad;
		float next_a = constrainAngle(hdng2 + drad);
		ROS_INFO("HDNG: %.2f/%.0f -> %.2f/%.0f, dst: %.0f, drad: %.2f, m_pr_rad: %.2f, next: %.2f",hdng1,dst1,hdng2,dst2,dst,drad,m_pr_rad,next_a);
		poly_illegal.polygon.points[1].z = building_centroid.point.z;
		poly_illegal.polygon.points[2].z = building_centroid.point.z;
		poly_illegal.polygon.points[0].z = building_centroid.point.z;
		poly_illegal.polygon.points[0].x = building_centroid.point.x;
		poly_illegal.polygon.points[0].y = building_centroid.point.y;
		poly_illegal.polygon.points[1].x = 50 * cos(hdng1) + building_centroid.point.y;
		poly_illegal.polygon.points[1].y = 50 * sin(hdng1) + building_centroid.point.z;
		poly_illegal.polygon.points[2].x =  building_centroid.point.x;
		poly_illegal.polygon.points[2].x = 50 * cos(hdng2) + building_centroid.point.y;
		poly_illegal.polygon.points[2].y = 50 * sin(hdng2) + building_centroid.point.z;
		poly_illegal.polygon.points[3]   = poly_illegal.polygon.points[0];

		poly_legal.polygon.points[0]   = poly_illegal.polygon.points[0];
		poly_legal.polygon.points[1]   = poly_illegal.polygon.points[2];

		poly_legal.polygon.points[2].x = building_centroid.point.x + 50 * cos(next_a);
		poly_legal.polygon.points[2].y = building_centroid.point.y + 50 * sin(next_a);
		poly_legal.polygon.points[2].z = pnt1.z;
		poly_legal.polygon.points[3] = poly_illegal.polygon.points[0];
		poly_legal.header = poly_illegal.header = hdr();
		ROS_INFO("Poly: %i %i",poly_legal.polygon.points.size(),poly_illegal.polygon.points.size());
		for(int i = 0; i < poly_legal.polygon.points.size(); i++){
			ROS_INFO("poly_illegal[%i]: %.0f %.0f %.0f %.2f deg",i,poly_illegal.polygon.points[i].x,poly_illegal.polygon.points[i].y,poly_illegal.polygon.points[i].z,get_hdng32d(poly_illegal.polygon.points[i],building_centroid.point));
			ROS_INFO("poly_legal[%i]: %.0f %.0f %.0f %.2f deg",i,poly_legal.polygon.points[i].x,poly_legal.polygon.points[i].y,poly_legal.polygon.points[i].z,get_hdng32d(poly_legal.polygon.points[i],building_centroid.point));
			get_hdng32d(poly_legal.polygon.points[i],building_centroid.point);
		}
	}
}
geometry_msgs::PolygonStamped get_poly_heading(float hdng_delta,int num_points){
//	ROS_INFO("POLY HEADING");
	geometry_msgs::PolygonStamped polygon;
	polygon.header.frame_id = "map";
	polygon.header.stamp = ros::Time::now();
	polygon.polygon.points.resize(num_points+1);
	polygon.polygon.points[0].x = pos.x;
	polygon.polygon.points[0].y = pos.y;
	polygon.polygon.points[0].z = pos.z;
	float rads_pr_i = hdng_delta /num_points;
	geometry_msgs::PointStamped p,pout;
	p.header.frame_id = "base_stabilized";
	p.header.stamp = ros::Time();
	float hdng = get_hdng(target_pose.pose.position,target_pose_last.pose.position);
	float a0 = hdng - hdng_delta/2;
	for(int i = 1; i < num_points; i++){
		float a = -hdng_delta/2 + rads_pr_i * i;
		polygon.polygon.points[i].x = target_pose_last.pose.position.x + 50*cos(a0+a);
		polygon.polygon.points[i].y = target_pose_last.pose.position.y + 50*sin(a0+a);
		polygon.polygon.points[i].z = target_pose_last.pose.position.z;
	}
	polygon.polygon.points[num_points].x = target_pose_last.pose.position.x;
	polygon.polygon.points[num_points].y = target_pose_last.pose.position.y;
	polygon.polygon.points[num_points].z = target_pose_last.pose.position.z;
	return polygon;
}
nav_msgs::Path get_standardized_cluster_simple(nav_msgs::Path pathin){
  nav_msgs::Path pathout;
  pnt_ref       = pos;
  geometry_msgs::Point ave_pnt;
  int ave_pnt_i,indexes_in_segment;
  while(pathin.poses.size() > 0){
    pathin             = sort_path(pathin,"dst_3d_ref");
    indexes_in_segment = get_indexes_within_rad(pathin,5,pnt_ref);
    ave_pnt            = get_ave_pnt_ni(pathin,0,indexes_in_segment);
    float zmx          = get_zmax_ni(pathin,indexes_in_segment);
    ave_pnt_i          = get_closest_i(pathin,ave_pnt);
    pnt_ref            = pathin.poses[ave_pnt_i].pose.position;
    pathout.poses.push_back(pathin.poses[ave_pnt_i]);
    ROS_INFO("pathin %i -> pathout %i: indexes_in_segment: %i ave_pnt_i: %i",pathin.poses.size(),pathout.poses.size(),indexes_in_segment,ave_pnt_i);
    if(indexes_in_segment >= pathin.poses.size())
      break;
    else
      pathin = remove_nth_first(pathin,indexes_in_segment);
  }
//	ROS_INFO("done, %i",pathout.poses.size());
	pathout.header = hdr();
  return pathout;
}
void path_unknown_cb(const nav_msgs::Path::ConstPtr& msg){
	//ROS_INFO("path_unknown_cb");
	path_unknown = *msg;
}
void mb_result_cb(const std_msgs::Bool::ConstPtr& msg){
	//ROS_INFO("mb_result_cb");

//	if(!msg->data)
//		get_next_target();
}
void checktf1(){
  geometry_msgs::TransformStamped transformStamped;
  try{
    transformStamped = tfBuffer.lookupTransform("map","base_stabilized",
                             ros::Time(0));
  }
  catch (tf2::TransformException &ex) {
    ROS_WARN("%s",ex.what());
    ros::Duration(1.0).sleep();
  }
	pos.x = transformStamped.transform.translation.x;
	pos.y = transformStamped.transform.translation.y;
	pos.z = transformStamped.transform.translation.z;
	if(get_dst3d(last_pose.pose.position,pos) >= 1.00){
		last_pose.pose.position    = pos;
		last_pose.pose.orientation = transformStamped.transform.rotation;
		last_pose.header           = transformStamped.header;
		path_visited.poses.push_back(last_pose);
		path_visited.header.stamp  = ros::Time::now();
	}
}
void checktf2(){
  geometry_msgs::TransformStamped transformStamped;
  try{
    transformStamped = tfBuffer.lookupTransform("map","base_perfect_alt",
                             ros::Time(0));
  }
  catch (tf2::TransformException &ex) {
    ROS_WARN("%s",ex.what());
    ros::Duration(1.0).sleep();
  }
	target_distance = get_dst2d(pos,target_pose.pose.position);
	cmd_pos.x = transformStamped.transform.translation.x;
	cmd_pos.y = transformStamped.transform.translation.y;
	cmd_pos.z = transformStamped.transform.translation.z;
	cmd_pos_yaw = tf::getYaw(transformStamped.transform.rotation);
}

geometry_msgs::Point point_pointed_to(geometry_msgs::Point pnt,float yaw,float len){
	pnt.x += pnt.x + len * cos(yaw);
	pnt.y += pnt.y + len * sin(yaw);
	return pnt;
}

int get_completeness_by_radians(nav_msgs::Path pathin,geometry_msgs::Point origo){
ROS_INFO("get_completeness_by_radians");
	std::vector<bool> path2d_hdngs_represented;
	int num_rads = 72;
	float rads_pr_i = 2*M_PI / num_rads;
	path2d_hdngs_represented.resize(num_rads);
	for(int i = 0; i < pathin.poses.size(); i++){
		float rad = get_hdng(pathin.poses[i].pose.position,building_centroid.point);
		path2d_hdngs_represented[int(round((rad + M_PI) / rads_pr_i))] = true;
	}
	int represented = 0;
	for(int i = 0; i < num_rads; i++){
		if(path2d_hdngs_represented[i])
			represented++;
	}
	int percentage_completion = int(round(float(represented) / float(num_rads) * 100.0));
	ROS_INFO("Checking %i poses against origo %.0f %.0f gives %i percent completion ",pathin.poses.size(),origo.x,origo.y,percentage_completion);
	return percentage_completion;
}
bool check_if_floorpath_is_safe(nav_msgs::Path pathin, int radlen_xy){
	//ROS_INFO("check_if_floorpath_is_safe");

	std::vector<int> path2d_elevation;
	std::vector<int> path2d_elevation_relative;
	pathin = sort_path(pathin,"building_rad");
	for(int i = 0; i < pathin.poses.size(); i++){
		path2d_elevation[i] 				 = get_elevation_at_xy(pathin.poses[i].pose.position.x,pathin.poses[i].pose.position.y,radlen_xy);
		path2d_elevation_relative[i] = int(round(pathin.poses[i].pose.position.z)) - path2d_elevation[i];
	}
	std::vector<int> v_el 		 = vec_to_min_max_ave(path2d_elevation);
	std::vector<int> v_rel_el = vec_to_min_max_ave(path2d_elevation_relative);
	ROS_INFO("Elevation: %i -> %i, rel: %i -> %i ave: (%i -> %i)",v_el[0],v_el[1],v_rel_el[0],v_rel_el[1],v_el[2],v_rel_el[2]);
	if(v_rel_el[1] < par_dz*3)
		return false;
	else
		return true;
}
int get_orbit_data(nav_msgs::Path pathin){
	//ROS_INFO("get_orbit_data");

	std::vector<float> yaws;
	std::vector<float> hdngs;
	for(int i = 0; i < pathin.poses.size(); i++){
		hdngs.push_back(get_hdng(pathin.poses[i].pose.position,building_centroid.point));
		yaws.push_back(tf::getYaw(pathin.poses[i].pose.orientation));
	}
	float dhdng_sum = 0;
	float dyaws_sum = 0;
	for(int i = 1; i < pathin.poses.size(); i++){
		dhdng_sum += get_shortest(hdngs[i],hdngs[i-1]);
		dyaws_sum += get_shortest(yaws[i],yaws[i-1]);
	}
	float dhdng_ave = dhdng_sum / float(pathin.poses.size()-1);
	float dyaws_ave = dyaws_sum / float(pathin.poses.size()-1);
	int dhdng = int(round(((100.0 * dhdng_sum / (2*M_PI)))));
	int dyaws = int(round(((100.0 * dyaws_sum / (2*M_PI)))));
	ROS_INFO("ORBIT_DATA-HDNG(cen): %.2f -> %.2f (%.2f rads)",hdngs[0],hdngs[hdngs.size()-1],dhdng_sum,get_shortest(hdngs[0],hdngs[hdngs.size()-1]));
	ROS_INFO("ORBIT_DATA-YAW(tar): %.2f -> %.2f (%.2f rads)",yaws[0],yaws[yaws.size()-1],dyaws_sum,get_shortest(yaws[0],yaws[yaws.size()-1]));
	ROS_INFO("ORBIT_DATA-COMPLETION: hdng: %i, yaw: %i",dhdng,dyaws);
	return fmax(dhdng,dyaws);
}
int get_next_radius(int num_is){
	float tot_rads = M_PI * 2;
	float rads_pr_i = tot_rads / num_is;
	if(building_v0 < path_visited.poses.size()+2 && path_visited.poses.size() > 5){
		geometry_msgs::Point pnt_v0 = path_visited.poses[building_v0].pose.position;
		geometry_msgs::Point pnt_v1 = path_visited.poses[building_v0 + 2].pose.position;
		geometry_msgs::Point pnt_vn2 = path_visited.poses[path_visited.poses.size()-4].pose.position;
		geometry_msgs::Point pnt_vn1 = path_visited.poses[path_visited.poses.size()-1].pose.position;
		float hdng_vn2 = get_hdng(pnt_vn2,building_centroid.point);
		float hdng_vn1 = get_hdng(pnt_vn1,building_centroid.point);
		float hdng_0 = get_hdng(pnt_v0,building_centroid.point);
		float hdng_1 = get_hdng(pnt_v1,building_centroid.point);
		float hdng_p = get_hdng(pos,building_centroid.point);

		int hdng_i0 = int(round((hdng_0 + M_PI) / rads_pr_i));
		int hdng_i1 = int(round((hdng_1 + M_PI) / rads_pr_i));
		int hdng_ip = int(round((hdng_p + M_PI) / rads_pr_i));
		int hdng_ivn2 = int(round((hdng_vn2 + M_PI) / rads_pr_i));
		int hdng_ivn1 = int(round((hdng_vn1 + M_PI) / rads_pr_i));

		int delta_i = 2;
		int dh = hdng_ivn1 - hdng_ivn2;
		ROS_INFO("TARGET_INDEX: %i, hdng_ivn2: %i->%i hdng_ivn1headings: %.2f %.2f %.2f %.2f %.2f",dh,hdng_ivn2,hdng_ivn1,hdng_vn2,hdng_vn1,hdng_0,hdng_1,hdng_p);
		return dh * delta_i;
	}

	float hdng_p = get_hdng(pos,building_centroid.point);
	int hdng_ip = int(round((hdng_p + M_PI) / rads_pr_i));
	return hdng_ip + 2;
}
int get_completeness_by_radians2(nav_msgs::Path pathin){
	if(pathin.poses.size() < 10)
		return 0;
	nav_msgs::Path pathout;
	std::vector<float> building_active_ranges;
	std::vector<int> building_active_indexes;
	geometry_msgs::PoseStamped new_target;
	std::vector<float> visited_ranges;
	std::vector<int> visited_indexes;
	int num_is = 64;
	float tot_rads = M_PI * 2;
	float rads_pr_i = tot_rads / num_is;
	building_active_indexes.resize(num_is);
	building_active_ranges.resize(num_is);
	visited_indexes.resize(num_is);
	visited_ranges.resize(num_is);
//	pathin = get_active_cluster(pathin,pos);
	geometry_msgs::Point pathin_center = building_centroid.point;//et_ave_pnt(pathin);


	pnt_ref = pathin_center;
	pathin  = sort_path(pathin,"hdng_3d_ref");
	ROS_INFO("Sorted, %i poses",path_building_active.poses.size());
	for(int i = 0; i < path_building_active.poses.size(); i++){
		float hdng  = get_hdng(path_building_active.poses[i].pose.position,pnt_ref);
		float dst2d = get_dst2d(path_building_active.poses[i].pose.position,pnt_ref);
		int hdng_i  = int(round((hdng + M_PI) / rads_pr_i));
		if(building_active_ranges[hdng_i] == 0 || building_active_ranges[hdng_i] < dst2d){
			building_active_ranges[hdng_i]  = dst2d;
			building_active_indexes[hdng_i] = i;
		}
	}

	if(path_visited.poses.size() > building_v0 + 2){
		ROS_INFO("path_visited. %i building_vo : %i",path_visited.poses.size(),building_v0);
			for(int i = building_v0; i < path_visited.poses.size(); i++){
			float hdng  = get_hdng(path_visited.poses[i].pose.position,pnt_ref);
			float dst2d = get_dst2d(path_visited.poses[i].pose.position,pnt_ref);
			int hdng_i = int(round((hdng + M_PI) / rads_pr_i));
			if(visited_ranges[hdng_i] == 0 || visited_ranges[hdng_i] < dst2d){
				visited_ranges[hdng_i]  = dst2d;
				visited_indexes[hdng_i] = i;
			}
		}
	}
	nav_msgs::Path path_temp_building,path_temp_vstd;

	for(int i = 0; i < num_is; i++){
		float a = rads_pr_i * i - M_PI;
		if(building_active_indexes[i] > 0){
			int index = building_active_indexes[i];
			float hdng  = get_hdng(pathin.poses[index].pose.position,pnt_ref);
			ROS_INFO("BUILDING-temp: angle: %.2f %i(%i) hdng %.2f building[%i]: %i",a,i,index,hdng);
			path_temp_building.poses.push_back(path_building_active.poses[building_active_indexes[i]]);
		}
		if(visited_indexes[i] > 0){
			int index =visited_indexes[i];
			float hdng  = get_hdng(path_visited.poses[index].pose.position,pnt_ref);
			ROS_INFO("VISITED-temp:  angle: %.2f %i(%i) hdng %.2f building[%i]: %i",a,i,index,hdng);
			path_temp_vstd.poses.push_back(path_visited.poses[visited_indexes[i]]);
		}
	}

	ROS_INFO("path_temp_building. %i path_temp_vstd : %i",path_temp_building.poses.size(),path_temp_vstd.poses.size());

	int explored = int(round(100.0 * float(path_temp_building.poses.size()) / float(num_is)));
	int visited  = int(round(100.0 * float(path_temp_vstd.poses.size()) / float(num_is)));
	path_temp_building = sort_path(path_temp_building,"hdng_3d_ref");

	int next_i = get_next_radius(num_is);
	if(next_i < 0)
		next_i -= num_is;
	if(next_i > num_is)
		next_i += num_is;

	nav_msgs::Path path_building_active_new = get_new_path(path_visited,path_temp_building,1.0,true);
	draw_path("path_temp_building",path_temp_building,2,2,1,true,cv::Scalar(0,200,100),true);
	draw_path("path_temp_vstd",path_temp_vstd,2,2,4,false,cv::Scalar(200,100,45),true);
	draw_path("path_building_active_new",path_building_active_new,2,2,4,false,cv::Scalar(200,100,45),true);
	if(next_i < path_temp_building.poses.size() && next_i > 0){
		new_target = path_temp_building.poses[next_i];
		draw_pose("target"+std::to_string(get_float(get_dst3d(new_target.pose.position,pos))),new_target.pose.position,2,2,5,false,tf::getYaw(new_target.pose.orientation),cv::Scalar(200,200,100));
		draw_path("path_temp_building",path_temp_building,2,2,1,true,cv::Scalar(0,200,100),true);
		draw_path("path_temp_vstd",path_temp_vstd,2,2,4,false,cv::Scalar(200,100,45),true);
		draw_and_reset_img();
		set_target_pose(new_target);
	}
	else{
		set_target_path(path_building_active_new);
	}
	return visited;
}
void set_building_floor(nav_msgs::Path pathin){
	ROS_INFO("SET BUILDING FLOOR: %i",pathin.poses.size());
	//ROS_INFO("set_building_floor");
	path_building_targets.poses.resize(0);
	building_centroid.point = get_ave_pnt(pathin);
	building_centroid.header = hdr();
	set_target_path(get_standardized_cluster_simple(pathin));
}

void set_floorpath(){
	if(building_paths_available.size() == 0){
		//ROS_INFO("BUILDING COMPLETE");
		buildings_paths_complete.push_back(building_paths_complete);
		buildings_centroids.push_back(building_centroids);
		set_state_building("looking_for_building");
	}
	else{
		nav_msgs::Path path_temp = building_paths_available[0];
		building_paths_available.erase(building_paths_available.begin());
		set_building_floor(path_temp);
	}
}


void update_inspection(){
	ROS_INFO("update_inspection");
	int radlen_xy = 1;
	int complete_cutoff = 95;
	int building_maxrad = 100;

	//int percent 			= get_completeness_by_radians2(path_building_active);
	geometry_msgs::PolygonStamped building_poly_connected = get_connected_building();
	for(int i = 0; i < building_poly_connected.polygon.points.size(); i++){
		if(!is_point32_in_poly(poly_building_active,building_poly_connected.polygon.points[i],1)){
			poly_building_active.polygon.points.push_back(building_poly_connected.polygon.points[i]);
		}
	}
	if(building_poly_connected.polygon.points.size() < 3){
		return;
	}
	ROS_INFO("Building path active: %i",path_building_active.poses.size());
	if(path_building_active.poses.size() == 0)
		return;

	nav_msgs::Path path_connected	= get_connected_path(path_building_active,pos);
	nav_msgs::Path path_connected_notvstd = get_new_path(path_visited,path_connected,5.0,false);
	bool focus_on_buildings = false;
	if(focus_on_buildings){
		path_active_reduced 	 = get_streaking_building_path(path_connected);
		path_connected_notvstd = get_new_path(path_visited,path_active_reduced,5.0,false);
	}
	int ci = getinpath_closestindex2d(path_connected_notvstd,cmd_pos);
	set_target_pose(path_connected_notvstd.poses[ci]);
	ROS_INFO("Bulding_poly_connected: %i building_path_connected: %i,path_connected_notvstd %i",building_poly_connected.polygon.points.size(), path_connected.poses.size(),path_connected_notvstd.poses.size());
	int percent = 50;

	if(path_connected_notvstd.poses.size() > 0)
		set_target_path(path_connected_notvstd);

		ROS_INFO("Percent: %i",percent);
	if(percent > 70){
		ROS_INFO("ORBIT COMPLETE! - evaluating above and below");
		building_centroids.push_back(building_centroid);
		int percent_below = get_completeness_by_radians(path_building_below,building_centroid.point);
		int percent_above = get_completeness_by_radians(path_building_above,building_centroid.point);
		bool below_complete,above_complete;
		if(percent_below > complete_cutoff)
			below_complete = true;
		if(percent_above > complete_cutoff)
			above_complete = true;

		bool below_safe = check_if_floorpath_is_safe(path_building_below,radlen_xy);
		bool above_safe = check_if_floorpath_is_safe(path_building_above,radlen_xy);
		building_paths_complete.push_back(path_building_targets);
		if(above_complete && above_safe)
			building_paths_available.push_back(path_building_above);
		if(below_complete && below_safe)
			building_paths_available.push_back(path_building_below);
		set_floorpath();
	}
	ROS_INFO("update_inspection_Complete");
}

void get_target_looking(){
	geometry_msgs::Point pbest;
	geometry_msgs::PoseStamped ps;
	ps.pose.orientation.w = 1.0;
	for(int i =0; i < polys_obstacles_received.size(); i++){
		if(polys_obstacles_received[i].polygon.points.size() > 0){
			geometry_msgs::Point p_ave = get_ave_pnt_poly(polys_obstacles_received[i]);
			if(p_ave.z > pbest.z){
				ps.header = hdr();
				pbest = p_ave;
				ps.pose.position.x = (pbest.x + pos.x)/2.0;
				ps.pose.position.y = (pbest.y + pos.y)/2.0;
				ps.pose.position.z = (pbest.z + pos.z)/2.0;
				ps.pose.orientation = tf::createQuaternionMsgFromYaw(get_hdng(ps.pose.position,pos));
				set_target_pose(ps);//	set_target_pose(ps);
			}
		}
	}
	//	if(get_dst2d(ps.pose.position,pos) > 5.0 && ps.header.frame_id == "map"){
	//		set_target_pose(ps);
	//	}
	float score,best_score;
	best_score = 100;
	int best_score_i = -1;
	if((ros::Time::now()-closest_obstacle_2d.header.stamp).toSec() < 2.0){
		if(path_candidates.poses.size() > 0){
			for(int i = 0; i < path_candidates.poses.size(); i++){
				int ci = getinpath_closestindex2d(path_candidates,closest_obstacle_2d.point);
				float yaw_cand = tf::getYaw(path_candidates.poses[i].pose.orientation);
				float hdng_cand2obs = get_hdng(closest_obstacle_2d.point,path_candidates.poses[i].pose.position);
				float dst_cand2obs = get_dst3d(closest_obstacle_2d.point,path_candidates.poses[i].pose.position);
				float dhdng = get_shortest(hdng_cand2obs,yaw_cand);
				if(dhdng < 0)
					dhdng *= -1;
				if(dhdng < M_PI/3 && dst_cand2obs < best_score){
					best_score = dst_cand2obs;
					ROS_INFO("NEW BEST SCORE: %.2f - dst: %.0f yaw_cand: %.2f hdng_cand2obs: %.2f dhdng: %.2f ",best_score,dst_cand2obs,yaw_cand,hdng_cand2obs,dhdng);
					best_score_i = i;
				}
			}
		}
		if(best_score_i >= 0){
			set_state_building("approaching_building");
			building_centroid.point = closest_obstacle_2d.point;
			set_target_pose(path_candidates.poses[best_score_i]);//	set_target_pose(ps);
		}
	}
	/*
		building_centroid.point.x = closest_obstacle_2d.point.x;
		ps.pose.position.x = (building_centroid.point.x + pos.x)/2.0;
		building_centroid.point.y = closest_obstacle_2d.point.y;
		ps.pose.position.y = (building_centroid.point.y + pos.y)/2.0;
		building_centroid.point.z = closest_obstacle_2d.point.z;
		ps.pose.position.z = (building_centroid.point.z + pos.z)/2.0;
		float building_altlvl_active = building_centroid.point.z / int(round(par_dz) * 2);
		ROS_INFO("Building_z: %.0f / %.2f",building_centroid.point.z,building_altlvl_active);
		building_centroid.point.z = building_altlvl_active;
		set_state_building("approaching_building");
		set_target_pose(ps);//	set_target_pose(ps);
	}
*/
	else if((ros::Time::now()-path_visited.header.stamp).toSec() > 5.0){
		//ROS_INFO("setting path unknown");
		path_spiral_i++;
		set_target_pose(path_spiral.poses[path_spiral_i]);
		//get_target_from_path_unknown();
	}
}

void check_target(){
	//ROS_INFO("CHECK TARGET");
	target_distance = get_dst2d(pos,target_pose.pose.position);
	if((ros::Time::now()-path_visited.header.stamp).toSec() > 5.0){
		target_pose.pose.position = pos;
	}
	if(target_distance < 5.0){
	//	next_target();
	}
}
void update_approach(){
	//ROS_INFO("update_approach");
	if(target_distance < 4.0 && state_building == "approaching_building"){
		set_state_building("inspecting_building");
	//	next_target();
	}
}

void sideauto_path_cb(const nav_msgs::Path::ConstPtr& msg){
	//ROS_INFO("sideauto_path_cb");
	if(msg->poses.size() > 0){
		//ROS_INFO("PATH_SIDE_CB: %i poses ",msg->poses.size());
	//	if(state_building == "looking_for_building")
			//check_overlap(*msg);
	}
}

//*****side2d*****//

void side2d_path_large_cb(const nav_msgs::Path::ConstPtr& msg){
	//ROS_INFO("side2d_path_large_cb");
	if(state_building == "looking_for_building"){
		if(msg->poses.size() > 0){
			path_candidates = *msg;
			/*path_candidates.poses.push_back(path_candidates_new.poses[i]);
			nav_msgs::Path path_candidates_new = get_new_path(path_candidates,*msg,1.0,true);
			if(path_candidates_new.poses.size() > 0){
				for(int i = 0; i < path_candidates_new.poses.size(); i++){
					path_candidates.poses.push_back(path_candidates_new.poses[i]);
				}
			}*/
		}
	}
}

void side2d_path_building_active_cb(const nav_msgs::Path::ConstPtr& msg){
	if(state_building == "inspecting_building"){
		//ROS_INFO("side2d_path_building_active_cb");
		if(msg->poses.size() > 0){
			nav_msgs::Path path_building_active_new = get_new_path(path_building_active,*msg,1.0,true);
			if(path_building_active_new.poses.size() > 0){
				for(int i = 0; i < path_building_active_new.poses.size(); i++){
					path_building_active.poses.push_back(path_building_active_new.poses[i]);
				}
			}
		}
	}
	else{
		path_building_active.poses.resize(0);
	}
}

void side2d_path_building_above_cb(const nav_msgs::Path::ConstPtr& msg){
	if(state_building == "inspecting_building"){
	//ROS_INFO("side2d_path_building_above_cb");
		if(msg->poses.size() > 0){
			nav_msgs::Path path_building_above_new = get_new_path(path_building_above,*msg,1.0,true);
			if(path_building_above_new.poses.size() > 0){
				for(int i = 0; i < path_building_above_new.poses.size(); i++){
					path_building_above.poses.push_back(path_building_above_new.poses[i]);
				}
			}
		}
	}
	else{
		path_building_above.poses.resize(0);
	}
}
void side2d_path_building_below_cb(const nav_msgs::Path::ConstPtr& msg){
	if(state_building == "inspecting_building"){
	//ROS_INFO("side2d_path_building_below_cb");
		if(msg->poses.size() > 0){
			nav_msgs::Path path_building_below_new = get_new_path(path_building_below,*msg,1.0,true);
			if(path_building_below_new.poses.size() > 0){
				for(int i = 0; i < path_building_below_new.poses.size(); i++){
					path_building_below.poses.push_back(path_building_below_new.poses[i]);
				}
			}
		}
	}
	else{
		path_building_below.poses.resize(0);
	}
}
void side2d_obstacle_ready_cb(const geometry_msgs::PointStamped::ConstPtr& msg){
	//ROS_INFO("side2d_obstacle_ready_cb");
	closest_obstacle_2d = *msg;
	current_midpoint.point  = closest_obstacle_2d.point;
	current_midpoint.header = hdr();
	if(state_building == "looking_for_building")
		pub_pnt.publish(current_midpoint);
	else if(state_building == "inspecting_building"){
		current_midpoint.point.x = target_pose.pose.position.x + 5 * cos(tf::getYaw(target_pose.pose.orientation));
		current_midpoint.point.y = target_pose.pose.position.y + 5 * sin(tf::getYaw(target_pose.pose.orientation));
		current_midpoint.point.z = building_centroid.point.z;
		pub_small_pnt.publish(current_midpoint);
	}
}

void side2d_obstacles_cb(const geometry_msgs::PolygonStamped::ConstPtr& msg){
	for(int i = 0; i < msg->polygon.points.size(); i++){
		int r = y2r(msg->polygon.points[i].y);
		int c = x2c(msg->polygon.points[i].x);
		int z = fmax(msg->polygon.points[i].z,1);
		if(img_height.at<cv::Vec3b>(r,c)[0] > z)
			img_height.at<cv::Vec3b>(r,c)[0] = z;
		if(img_height.at<cv::Vec3b>(r,c)[1] < z)
			img_height.at<cv::Vec3b>(r,c)[1] = z;
	}
}
//*****side2d*****//


void downauto_path_cb(const nav_msgs::Path::ConstPtr& msg){
	//ROS_INFO("downauto_path_cb");
	nav_msgs::Path path_downauto;
	if(msg->poses.size() > 0){
		nav_msgs::Path path_down_new = get_new_path(path_downauto,*msg,1.0,false);
		if(path_down_new.poses.size() > 0){
			for(int i = 0; i < path_down_new.poses.size(); i++){
				path_downauto.poses.push_back(path_down_new.poses[i]);
			}
		}
	}
}
void sideauto_obstacles_cb(const geometry_msgs::PolygonStamped::ConstPtr& msg){
	//ROS_INFO("GOT: %i obstacles",msg->polygon.points.size());
	for(int i = 0; i < msg->polygon.points.size(); i++){
		int r = y2r(msg->polygon.points[i].y);
		int c = x2c(msg->polygon.points[i].x);
		int z = fmax(msg->polygon.points[i].z,1);
		if(img_height.at<cv::Vec3b>(r,c)[0] > z)
			img_height.at<cv::Vec3b>(r,c)[0] = z;
		if(img_height.at<cv::Vec3b>(r,c)[1] < z)
			img_height.at<cv::Vec3b>(r,c)[1] = z;
	}
}
void downauto_obstacles_cb(const geometry_msgs::PolygonStamped::ConstPtr& msg){
	//ROS_INFO("GOT: %i obstacles",msg->polygon.points.size());
	for(int i = 0; i < msg->polygon.points.size(); i++){
		int r = y2r(msg->polygon.points[i].y);
		int c = x2c(msg->polygon.points[i].x);
		int z = fmax(msg->polygon.points[i].z,1);
		if(img_height.at<cv::Vec3b>(r,c)[2] < z)
			img_height.at<cv::Vec3b>(r,c)[2] = z;
	}
}
void poly_cleared_cb(const geometry_msgs::PolygonStamped::ConstPtr& msg){
	poly_cleared_extended = project_poly(*msg,10);
	poly_cleared = *msg;
}
void poly_obstacles_cb(const geometry_msgs::PolygonStamped::ConstPtr& msg){
	//ROS_INFO("poly_obstacles_cb");
	float cluster_spacing_cutoff = 10;
	float cluster_visited_cutoff = 7;
	bool use_3d = true;
	for(int i = 0; i < msg->polygon.points.size(); i++){
		if(poly_obstacles_received.polygon.points.size() > 0){
			if(!is_point32_in_poly(poly_obstacles_received,msg->polygon.points[i],1.0))
				poly_obstacles_received.polygon.points.push_back(msg->polygon.points[i]);
		}
		else
			poly_obstacles_received.polygon.points.push_back(msg->polygon.points[i]);
	}
}
nav_msgs::Path create_path_spiral(float grid_sidelength){
	nav_msgs::Path pathout;
	pathout.header = hdr();
	geometry_msgs::PoseStamped pose;
	pose.pose.position.z    = 10;
	pose.pose.orientation.w = 1;
	pose.header = hdr();
	for(int i = 0; i < 10; i++){
		for(int k = 0; k < i; k++){
			pose.pose.position.x += pow(-1,i) * grid_sidelength;
			//ROS_INFO("Pnt[%i],x: %.0f y: %.0f",pathout.poses.size(),pose.pose.position.x,pose.pose.position.y);
			pathout.poses.push_back(pose);
		}
		for(int l = 0; l < i; l++){
			pose.pose.position.y += pow(-1,i) * grid_sidelength;
		//  ROS_INFO("Pnt[%i],x: %.0f y: %.0f",pathout.poses.size(),pose.pose.position.x,pose.pose.position.y);
			pathout.poses.push_back(pose);
		}
	}
	return pathout;
}
void scan_cb(const sensor_msgs::LaserScan::ConstPtr& scan){
	if((scan->header.stamp - last_scan).toSec() > 1.0){
		polys_scanin_obstacles.resize(0);
		polys_scanin_cleared.resize(0);
		geometry_msgs::PolygonStamped poly;
		geometry_msgs::PolygonStamped poly_clear;
		geometry_msgs::Point32 p0,p1,ps;
		last_scan = ros::Time::now();
		img.copyTo(img_copy);
		for(int i = 0; i < scan->ranges.size(); i++){
			float r = scan->ranges[i];
			float a =	scan->angle_increment * i + scan->angle_min;
			p0.x = r * cos(a);
			p0.y = r * sin(a);
			float clear_rad = poly_clear.polygon.points.size() * scan->angle_increment;
			float obs_rad = poly_clear.polygon.points.size() * scan->angle_increment;
			if(scan->ranges[i] < scan->range_max){
				poly.polygon.points.push_back(p0);
				if(clear_rad > 1.0 && poly_clear.polygon.points.size() >= 2){
					p0 = poly_clear.polygon.points[0];
			  	p1 = poly_clear.polygon.points[poly_clear.polygon.points.size()-1];
					polys_scanin_cleared.push_back(poly_clear);
					ROS_INFO("InitialScan[rads: %.2f]: Object[#%i of %i pnts] found between %.0f %.0f and %.0f %.0f",clear_rad,polys_scanin_cleared.size(),poly_clear.polygon.points.size(),p0.x,p0.y,p1.x,p1.y);
					cv::line (img_copy, pnt322cv(ps), pnt322cv(p0), cv::Scalar(0,0,200),1,cv::LINE_8,0);
					cv::line (img_copy, pnt322cv(ps), pnt322cv(p1), cv::Scalar(0,0,200),1,cv::LINE_8,0);
				}
				poly_clear.polygon.points.resize(0);
			}
			else{
				poly_clear.polygon.points.push_back(p0);
				if(clear_rad > 0.2 && poly.polygon.points.size() >= 2){
					p0 = poly.polygon.points[0];
					p1 = poly.polygon.points[poly_clear.polygon.points.size()-1];
					ROS_INFO("InitialScan[rads: %.2f]: Object[#%i of %i pnts] found between %.0f %.0f and %.0f %.0f",obs_rad,polys_scanin_obstacles.size(),poly.polygon.points.size(),p0.x,p0.y,p1.x,p1.y);
					cv::line (img_copy, pnt322cv(ps), pnt322cv(p0), cv::Scalar(0,200,0),1,cv::LINE_8,0);
					cv::line (img_copy, pnt322cv(ps), pnt322cv(p1), cv::Scalar(0,200,0),1,cv::LINE_8,0);
					polys_scanin_obstacles.push_back(poly);
					poly.polygon.points.resize(0);
				}
			}
		}
		cnt3++;
		cv::imwrite("/home/nuc/brain/cluster/"+std::to_string(cnt3)+"scanin.png",img_copy);
	}
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "tb_targertes_node");
	ros::NodeHandle nh;
	ros::NodeHandle private_nh("~");
	private_nh.param("update_area_sidelength", par_maprad, 15.0);
	private_nh.param("update_area_z_radius", par_dz, 1.5);
	private_nh.param("dst_target", par_dst_target, 3.0);
	private_nh.param("zn_start",par_scouting_zn_start, 3);
	private_nh.param("zn_end",par_scouting_zn_end, 8);
	tf2_ros::TransformListener tf2_listener(tfBuffer);
	for(int i = 0; i < 50; i++){
		z_lvls.push_back(par_dz*i*2);
	}
	building_polygon.polygon.points.resize(72);
	target_pose.pose.orientation.w = 1.0;
	target_pose.header = hdr();
	path_visited.poses.push_back(target_pose);
	ros::Subscriber s1 = nh.subscribe("/scan_stabilized",1,scan_cb);
	ros::Subscriber s2 = nh.subscribe("/tb_edto/downauto_path",1,downauto_path_cb);
	ros::Subscriber s3 = nh.subscribe("/tb_edto/sideauto_path",10,sideauto_path_cb);

	ros::Subscriber s4a = nh.subscribe("/tb_edto/downauto_obstacles",10,downauto_obstacles_cb);
	ros::Subscriber s4c = nh.subscribe("/tb_edto/sideauto_obstacles",10,sideauto_obstacles_cb);
	ros::Subscriber s9 = nh.subscribe("/tb_edto/side2d_obstacles",10,side2d_obstacles_cb);
	ros::Subscriber s4 = nh.subscribe("/tb_edto/poly_cleared",10,poly_cleared_cb);

	ros::Subscriber s5 = nh.subscribe("/tb_edto/side2d_path_large",10,side2d_path_large_cb);
	ros::Subscriber s6 = nh.subscribe("/tb_edto/side2d_path_building",10,side2d_path_building_active_cb);
	ros::Subscriber s8 = nh.subscribe("/tb_edto/side2d_path_building_above",10,side2d_path_building_above_cb);
	ros::Subscriber s7 = nh.subscribe("/tb_edto/side2d_path_building_below",10,side2d_path_building_below_cb);
	ros::Subscriber s11 = nh.subscribe("/tb_edto/side2d_obstacle_ready",10,side2d_obstacle_ready_cb);

	ros::Subscriber s10 = nh.subscribe("/tb_edto/poly_obstacles",10,poly_obstacles_cb);

	ros::Publisher pub_centroid 	= nh.advertise<geometry_msgs::PointStamped>("/tb_test/centroid",100);
	pub_pnt 			 								= nh.advertise<geometry_msgs::PointStamped>("/tb_edto/get_side2d",100);
	pub_small_pnt 			 					= nh.advertise<geometry_msgs::PointStamped>("/tb_edto/get_side2d_small",100);

	ros::Publisher pub_path_organize_1		= nh.advertise<nav_msgs::Path>("/tb_debug/path_organize_1",100);
	ros::Publisher pub_path_organize_2		= nh.advertise<nav_msgs::Path>("/tb_debug/path_organize_2",100);
	ros::Publisher pub_path_organize_raw	= nh.advertise<nav_msgs::Path>("/tb_debug/path_organize_raw",100);
	ros::Publisher pub_path_spiral			 	= nh.advertise<nav_msgs::Path>("/tb_debug/spiral",100);
	ros::Publisher pub_path_blding				= nh.advertise<nav_msgs::Path>("/tb_debug/bld",100);
	ros::Publisher pub_target_path  			= nh.advertise<nav_msgs::Path>("/tb_debug/path_targets",100);
	ros::Publisher pub_path_targets_sent	= nh.advertise<nav_msgs::Path>("/tb_debug/path_targets_sent",100);
	ros::Publisher building_polygon_pub   = nh.advertise<geometry_msgs::PolygonStamped>("/tb_debug/building_polygon",100);
	ros::Publisher pub_obs_poly      			 = nh.advertise<geometry_msgs::PolygonStamped>("/tb_debug/obspoly",100);
	ros::Publisher polyhdng_pub      			 = nh.advertise<geometry_msgs::PolygonStamped>("/tb_debug/polyhdng",100);
	ros::Publisher polybldng_pub      			 = nh.advertise<geometry_msgs::PolygonStamped>("/tb_debug/building",100);
	////
	//
	pub_setp_xyz       = nh.advertise<geometry_msgs::PointStamped>("/tb_cmd/set_xyz",100);
	pub_target 				= nh.advertise<geometry_msgs::PoseStamped>("/tb_cmd/mb_pose_target",100);
	ros::Publisher pub_poly_legal = nh.advertise<geometry_msgs::PolygonStamped>("/tb_test/poly_legal",10);
ros::Publisher pub_poly_illegal = nh.advertise<geometry_msgs::PolygonStamped>("/tb_test/poly_illegal",10);
	ros::Subscriber s01 = nh.subscribe("/tb_world/path_unknown",10,path_unknown_cb);
	ros::Subscriber s02 = nh.subscribe("/tb_cmd/mb_result",10,mb_result_cb);


	path_spiral = create_path_spiral(15);
	ros::Rate rate(5.0);
	cnt = 6;
	int ccnt = 0;
	ros::Time t_start = ros::Time::now();
	while(ros::ok()){
		checktf1();
		checktf2();
		check_target();
		if(state_building == "looking_for_building"){
			//ROS_INFO("UPDATE: LOOKING");
			if(target_distance < 5.0)
				get_target_looking();
			//ROS_INFO("UPDATE: LOOKING_DONE");
		}
		else if(state_building == "approaching_building"){
			update_approach();
		}
		else if(state_building == "inspecting_building"){
			get_poly_legal_illegal();
			if(poly_legal.polygon.points.size() > 0){
					pub_poly_legal.publish(poly_legal);
					pub_poly_illegal.publish(poly_illegal);
					get_completeness_by_radians2(path_building_targets);
			}

			//ROS_INFO("UPDATE: APPROACH_DONE");
			update_inspection();
			check_elev_and_publish();
		}
		poly_building_active.header = hdr();
		polybldng_pub.publish(poly_building_active);
		ROS_INFO("State: %s path targets: %i closest: %.0f %.0f %.0f ",state_building.c_str(),path_targets.poses.size(),closest_obstacle_2d.point.x,closest_obstacle_2d.point.y,closest_obstacle_2d.point.z);
		poly_obstacles_received.header = building_centroid.header = hdr();
		path_organize_raw.header.frame_id = "map";
		path_organize_1.header.frame_id = "map";
		path_organize_2.header.frame_id = "map";
		path_spiral.header.frame_id = "map";
		poly_obstacles_received.header.frame_id = "map";
		path_targets.header.frame_id = "map";
		path_targets_sent.header.frame_id = "map";
		path_building_active.header.frame_id = "map";
	//	if((ros::Time::now() - closest_obstacle_2d.header.stamp).toSec() < 2.0 && closest_obstacle_2d.header.frame_id == "map" && state_building == "inspecting_building"){
	//		geometry_msgs::PolygonStamped polybld = get_obstacle_building("comb",closest_obstacle_2d.point,0,15);
	//		geometry_msgs::Point p2 = get_ave_pnt_poly(polybld);
		//	if(get_dst2d(p2,building_centroid.point) < 30){
		//		ROS_INFO("BUILDING CENTROID MOVES");
		//		building_centroid.point = p2;
	//		}
		//	polybld.header = hdr();
	//		building_polygon_pub.publish(polybld);
	//	}
		if((ros::Time::now() - t_start).toSec() > 5.0){
			ROS_INFO("DRAW");
			drawimg();
			ROS_INFO("DRAW DONE");
			t_start = ros::Time::now();
		}
		//int percent_path_organize_raw	= get_completeness_by_radians2(path_building_active);
	//	int percent_path_organize_1	= get_completeness_by_radians2(path_targets);
		//int percent_path_organize_2	= get_completeness_by_radians2(path_organize_2);
		poly_heading = get_poly_heading(M_PI,16);
		polyhdng_pub.publish(poly_heading);

		pub_path_blding.publish(path_building_active);
	//	int bld_active = get_completeness_by_radians2(path_building_active);
	//	ROS_INFO("bld_active: %i",bld_active);
		pub_path_spiral.publish(path_spiral);
		pub_centroid.publish(building_centroid);
		pub_path_organize_raw.publish(path_organize_raw);
		pub_path_organize_1.publish(path_organize_1);
		pub_path_organize_2.publish(path_organize_2);
		pub_obs_poly.publish(poly_obstacles_received);

		pub_target_path.publish(path_targets);
		pub_path_targets_sent.publish(path_targets_sent);
		rate.sleep();
		ros::spinOnce();
	}
	return 0;
}
