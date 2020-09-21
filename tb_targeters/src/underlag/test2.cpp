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
cv::Mat img_ff_bld_copy(1000,1000,CV_8U,cv::Scalar(0)); //create image, set encoding and size, init pixels to default val
cv::Mat img_blank_mono(1000,1000,CV_8U,cv::Scalar(0)); //create image, set encoding and size, init pixels to default val
cv::Mat img_mono(1000,1000,CV_8U,cv::Scalar(0)); //create image, set encoding and size, init pixels to default val
cv::Mat img_paths(1000,1000,CV_8U,cv::Scalar(0)); //create image, set encoding and size, init pixels to default val
cv::Mat img(1000,1000,CV_8UC3,cv::Scalar(0,0,0)); //create image, set encoding and size, init pixels to default val
cv::Mat img_perm(1000,1000,CV_8UC3,cv::Scalar(0,0,0)); //create image, set encoding and size, init pixels to default val
cv::Mat img_copy(1000,1000,CV_8UC3,cv::Scalar(0,0,0)); //create image, set encoding and size, init pixels to default val
cv::Mat imgdeb(1000,1000,CV_8UC3,cv::Scalar(0,0,0)); //create image, set encoding and size, init pixels to default val
cv::Mat img_height(1000,1000,CV_8UC3,cv::Scalar(0,0,0)); //create image, set encoding and size, init pixels to default val
geometry_msgs::PointStamped target_xyz;

std::vector<cv::Mat> mapimgs_at_zlvlz;
geometry_msgs::Point pos,pnt_ref,last_pos,cmd_pos;
std::vector<float> z_lvls;
nav_msgs::Path path_visited;
std::string state_building = "looking_for_building";
std::string state_building_scan = "initial_vertical";
ros::Time state_building_change,state_building_scan_change;
geometry_msgs::PointStamped target_point,current_midpoint,building_centroid;
int par_scouting_zn_start,par_scouting_zn_end;
double par_maprad,par_dz;
int cnt = 0;
int zn = 3;
int cnt2 = 0;
int cnt3 = 0;
nav_msgs::Path path_side,last_path,gridpath;
std::vector<nav_msgs::Path> paths_vec;
geometry_msgs::PolygonStamped building_obstacle_points,building_polygon,building_polygon_min,poly_cleared,poly_obstacles_received;
int last_i = 0;
ros::Time last_rosinfo;
float pos_yaw = 0;
float last_pose_yaw = 0;
float last_yaw;
ros::Time last_scan;
geometry_msgs::PoseStamped last_pose,target_pose,current_midpose;
std::vector<geometry_msgs::PolygonStamped> buildings_polygons;
std::vector<geometry_msgs::PolygonStamped> polys_heightimage_clustered;
std::vector<geometry_msgs::PolygonStamped> polys_obstacles_received;
std::vector<geometry_msgs::PolygonStamped> polys_scanin_obstacles;
std::vector<geometry_msgs::PolygonStamped> polys_scanin_cleared;

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
nav_msgs::Path path_unknown,path_building_above,path_building_below;
nav_msgs::Path path_side_norm,path_sideauto_small,path_sideauto_big;
nav_msgs::Path path_organize_1,path_organize_2,path_organize_raw;
nav_msgs::Path path_building_targets,path_spiral,path_targets,path_targets_sent;
std::vector<int> targets_complete;
int path_targets_i = 0;
geometry_msgs::Point info_point;
float info_interval = 20;
int building_v0 = 0;
float rad2deg = 180.0/M_PI;
int path_spiral_i = 0;
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
void set_substate_building(std::string newstate){
  if(newstate != substate_building){
    float dt = (ros::Time::now() - substate_building_change).toSec();
    ROS_INFO("substate_building: %s -> %s (%.3f seconds in state)",substate_building.c_str(),newstate.c_str(),dt);
    substate_building_change  = ros::Time::now();
		bool end_state = false;
		if(susubstate_building == "inspecting_building")
			end_state = true;
		substate_building = newstate;
		if(newstate == "inspecting_building"){
			path_building_targets.poses.resize(0);
			path_building_targets.poses.push_back(target_pose);
		}
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
  if(path_visited.poses.size() == 0)
    return true;
  for(int i = 0; i < poly_to_check.polygon.points.size(); i++){
     if(get_dst3d32(poly_to_check.polygon.points[i],pin) < lim)
        return false;
  }
  return true;
}
bool is_point32_in_path(nav_msgs::Path path_to_check,geometry_msgs::Point32 pin,float lim){
  float res,dst;
  if(path_visited.poses.size() == 0)
    return true;
  for(int i = 0; i < path_to_check.poses.size(); i++){
     if(get_dst2d32d(path_to_check.poses[i].pose.position,pin) < lim)
        return false;
  }
  return true;
}
nav_msgs::Path get_new_path(nav_msgs::Path path_to_check,nav_msgs::Path pathin,float cutoff_dst,bool use_3d){
	nav_msgs::Path pathout;
	pathout.header = hdr();
	for(int i = 0; i < path_to_check.poses.size(); i++){
		if(!is_point_in_path(path_to_check,pathin.poses[i].pose.position,cutoff_dst,use_3d)){
			pathout.poses.push_back(pathin.poses[i]);
		}
	}
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
geometry_msgs::PolygonStamped sort_poly_around_dst(geometry_msgs::PolygonStamped polyin){
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
	geometry_msgs::Point32 centroid;
	centroid = get_poly_centroid(polyin);
	for(int i = 0; i < polyin.polygon.points.size(); i++){
		float rad = get_hdng32(polyin.polygon.points[i],centroid);
		polyin.polygon.points[i] = offset32(polyin.polygon.points[i],rad,offset);
	}
	return polyin;
}
geometry_msgs::PolygonStamped path2poly(nav_msgs::Path pathin){
  geometry_msgs::PolygonStamped polyout;
  polyout.header = hdr();
  polyout.polygon.points.resize(pathin.poses.size());
  for(int i = 0; i < pathin.poses.size(); i++){
    polyout.polygon.points[i].x = round(path_visited.poses[i].pose.position.x);
    polyout.polygon.points[i].y = round(path_visited.poses[i].pose.position.y);
    polyout.polygon.points[i].z = round(path_visited.poses[i].pose.position.z);
  }
  return polyout;
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

bool in_poly(geometry_msgs::PolygonStamped polyin, geometry_msgs::Point point)
{
  int cross = 0;
  for(int i = 0,j = polyin.polygon.points.size()-1; i < polyin.polygon.points.size(); j=i++){
//  for (int i = 0, j = vertices_.size() - 1; i < vertices_.size(); j = i++) {
    if ( ((polyin.polygon.points[i].y > point.y) != (polyin.polygon.points[j].y > point.y))
           && (point.x < (polyin.polygon.points[j].x - polyin.polygon.points[i].x) * (point.y - polyin.polygon.points[i].y) /
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
    if(in_poly(poly_bb,pathin.poses[i].pose.position))
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
void draw_path(nav_msgs::Path pathin,cv::Scalar color, int size){
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
void draw_clusters_at_height(std::vector<nav_msgs::Path> pathsin){
	for(int i = 0; i < pathsin.size(); i++){
		if(pathsin[i].poses.size() > 0){
			ROS_INFO("Draw cluster %i - %i poses",i,pathsin[i].poses.size());
			float rel_score = 255*(pathsin[i].poses[0].pose.position.z) / 40.0;
			draw_path(pathsin[i],cv::Scalar(0,int(rel_score),0),1);
			putText(img,"c#:"+std::to_string(int(pathsin[i].poses[0].pose.position.z)), pnt2cv(pathsin[i].poses[0].pose.position),
					cv::FONT_HERSHEY_COMPLEX_SMALL, 0.8, cv::Scalar(0,int(rel_score),0), 1, CV_AA);
		}
	}
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
	for(int i = 0; i < pathin.poses.size(); i++){
		draw_pnt(pathin.poses[i].pose.position,rectangle_size,circle_size,yaw_size,pixel,tf::getYaw(pathin.poses[i].pose.orientation),color);
		if(draw_line && i > 0)
			cv::line (img, pnt2cv(pathin.poses[i-1].pose.position), pnt2cv(pathin.poses[i].pose.position),color,1,cv::LINE_8,0);
	}
	draw_info(info,rectangle_size,circle_size,yaw_size,pixel,0.0,color);
}
void draw_poly(std::string info, geometry_msgs::PolygonStamped poly, int rectangle_size, int circle_size, int yaw_size, bool pixel, cv::Scalar color, bool draw_lines){
	for(int i = 0; i < poly.polygon.points.size(); i++){
		geometry_msgs::Point p;
		p.x = poly.polygon.points[i].x;
		p.y = poly.polygon.points[i].y;
		draw_pnt(p,rectangle_size,circle_size,0,pixel,0,color);
		if(draw_lines){
			if(i > 0)
				cv::line (img, pnt322cv(poly.polygon.points[i-1]), pnt322cv(poly.polygon.points[i]),color,1,cv::LINE_8,0);
			else
				cv::line (img, pnt322cv(poly.polygon.points[i-1]), pnt322cv(poly.polygon.points[i]),color,1,cv::LINE_8,0);
		}
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
	for(int i = 0; i < poylsin.size(); i++){
		if(poylsin[i].polygon.points.size() > 2){
			geometry_msgs::Point p_cen = get_ave_pnt_poly(poylsin[i]);
			std::string info = info + std::to_string(i) + ": " + std::to_string(poylsin[i].polygon.points.size()) + write_xyz(p_cen);
			count_color++;
			if(count_color == 3)
				count_color = 0;
			if(poylsin[i].polygon.points.size() > 3)
				draw_poly(info,poylsin[i],0,0,0,true,get_shifting_color(count_color,color_strength),true);
			else
				draw_poly(info,poylsin[i],0,0,0,true,get_shifting_color(count_color,color_strength),true);
		}
	}
}
void drawimg(){
	for(int r = 0; r < img.rows; r++){
		for(int c = 0; c < img.cols; c++){
			img.at<cv::Vec3b>(r,c)[0] = img_height.at<cv::Vec3b>(r,c)[0]*10;
			img.at<cv::Vec3b>(r,c)[1] = img_height.at<cv::Vec3b>(r,c)[1]*10;
			img.at<cv::Vec3b>(r,c)[2] = img_height.at<cv::Vec3b>(r,c)[2]*10;
		}
	}
	polys_obstacles_received = reorganize_polys(find_clusters(poly_obstacles,5));
	draw_polys("initial_polys#:"polys_obstacles_received,50);
	draw_polys("structures#:",polys_structures,50);
	draw_path("path_visited",path_visited,0,1,0,true,cv::Scalar(0,25,100),true);
	draw_path("path_organize_1",path_organize_1,0,0,1,true,cv::Scalar(100,43,20),false);
	draw_path("path_organize_2",path_organize_2,0,0,1,true,cv::Scalar(41,100,40),false);
	draw_path("targets",path_targets,2,2,4,false,cv::Scalar(200,100,45),true);

	draw_pose("target"+std::to_string(get_float(get_dst3d(target_pose.pose.position,pos))),target_pose.pose.position,2,2,5,false,tf::getYaw(target_pose.pose.orientation),cv::Scalar(200,200,100));
	draw_pose("pos",pos,2,2,5,false,pos_yaw,cv::Scalar(200,200,200));
	draw_pose("building",building_centroid.point,2,2,0,true,0,cv::Scalar(0,100,25));

	draw_poly("cleared",poly_cleared,0,0,0,false,cv::Scalar(0,200,25),true);
	draw_poly("poly_obstacles_raw",poly_obstacles_received,0,0,0,true,cv::Scalar(100,0,0),false);
	if(building_polygon_min.polygon.points.size() > 0)
		draw_poly("building_active",building_polygon_min,0,0,0,true,cv::Scalar(100,200,100),true);
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

geometry_msgs::PolygonStamped update_inputimg(std::string type,geometry_msgs::Point midpoint, int radlen_xy, int z0, int zn){
	geometry_msgs::PolygonStamped polyout;
	polyout.header = hdr();
	int pnts = 0; int pnts_tot = 0;	int obs_z = 0;
	for(int c = x2c(midpoint.x)-radlen_xy; c < x2c(midpoint.x)+radlen_xy; c++){
    for(int r = y2r(midpoint.y)-radlen_xy; r < y2r(midpoint.y)+radlen_xy; r++){
			if(type == "side")
				img_ff_bld.at<uchar>(r,c) 		= img_height.at<cv::Vec3b>(r,c)[1];
			else if(type == "down")
				obs_z = img_height.at<cv::Vec3b>(r,c)[2];
			else
				obs_z = fmax(img_height.at<cv::Vec3b>(r,c)[1],img_height.at<cv::Vec3b>(r,c)[2]);
			if(obs_z > 0){
				pnts++;
				if(obs_z > z0 && obs_z < zn){
					geometry_msgs::Point32 pnt;
					pnt.x = c2x(c);				pnt.y = r2y(r);				pnt.z = obs_z;				pnts++;
					polyout.polygon.points.push_back(pnt);
					img_ff_bld.at<uchar>(r,c) = obs_z;
				}
			}
			pnts_tot++;
		}
	}
	int coverage = 100 * pnts / (pnts_tot+1);
	ROS_INFO("FFILL-out([%i] of %i pnts) - [params: xy_radius: %i z0: %i zn: %i], coverage: %i percent",polyout.polygon.points.size(),pnts,pnts_tot,radlen_xy,z0,zn,coverage);
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
	return cv::floodFill(img_ff_bld,cv::Point(y2r(p0.y),x2c(p0.x)),cv::Scalar(fillcolor), &ccomp, cv::Scalar(lo),cv::Scalar(hi), flags);
}
int get_hi(float pz,float zn){
	return int(round(zn-pz-1));
}
int get_lo(float pz,float z0){
	return int(round(pz-z0+1));
}
std::vector<geometry_msgs::PolygonStamped> get_obstacles(std::string type,geometry_msgs::Point midpoint, int radlen_xy, float z0, float z1){
	std::vector<geometry_msgs::PolygonStamped> polysout;
	geometry_msgs::PolygonStamped poly_input = update_inputimg(type,midpoint,radlen_xy,int(round(z0)),int(round(z1)));
	cnt++;
	cv::imwrite("/home/nuc/brain/clusters/"+std::to_string(cnt)+"get_paths_begs.png",img_ff_bld);
	geometry_msgs::PolygonStamped poly_cluster,poly_input_leftover;
	poly_cluster.header = hdr();
	int size_last = 5;
	while(poly_input.polygon.points.size() > 5 && size_last > 0){
		int fillcolor = poly_input.polygon.points[0].z * 10;
		int area = floodfill_actual(poly_input.polygon.points[0],fillcolor,get_hi(poly_input.polygon.points[0].z,zn),fmax(1,get_lo(poly_input.polygon.points[0].z)),z0));
		for(int i = 0; i < poly_input.polygon.points.size(); i++){
			if(img_ff_bld.at<uchar>(y2r(poly_input.polygon.points[i].y),x2c(poly_input.polygon.points[i].x)) == fillcolor)
				poly_cluster.polygon.points.push_back(poly_input.polygon.points[i]);
			else
				poly_input_leftover.polygon.points.push_back(poly_input.polygon.points[i]);
		}
		size_last = poly_cluster.polygon.points.size();
		poly_input = poly_input_leftover;
		polysout.push_back(poly_cluster);
		ROS_INFO("FFILL-while_loop: C#%i: pnts: %i area: %i pnts_left: %i",poly_input.polygon.points.size(),polysout.size(),poly_cluster.polygon.points.size(),area,poly_input_leftover.polygon.points.size());
		poly_cluster.polygon.points.resize(0);
		poly_input_leftover.polygon.points.resize(0);
	}
	cv::imwrite("/home/nuc/brain/clusters/"+std::to_string(cnt)+"get_paths_clusters.png",img_ff_bld);
	return polysout;
}

void get_obstacles_ffill(std::string type){
	ros::Time t0 = ros::Time::now();
	geometry_msgs::Point pnt = building_centroid.point;
	if(poly_obstacles_received.polygon.points.size() < 5){
		return;
	}
	if(state_building == "looking_for_building"){
		poly_obstacles_received = sort_poly_around_dst(poly_obstacles_received);
		pnt.x = poly_obstacles_received.polygon.points[0].x;
		pnt.y = poly_obstacles_received.polygon.points[0].y;
		pnt.z = poly_obstacles_received.polygon.points[0].z;
	}
	int maprad_used = 50;
	polys_heightimage_clustered = get_obstacles(type,pnt,maprad_used,5,50);
	int biggest = 0;
	int biggest_i = 0;
	int pnts_count = 0;
	for(int i = 0; i < polys_heightimage_clustered.size();i++){
		ROS_INFO("B#i: %i (%i)",i,polys_heightimage_clustered[i].polygon.points.size(),pnts_count);
		pnts_count += polys_heightimage_clustered[i].polygon.points.size();
		if(polys_heightimage_clustered[i].polygon.points.size() > biggest){
			biggest = polys_heightimage_clustered[i].polygon.points.size();
			biggest_i = i;
		}
	}
	ROS_INFO("Total:",);
	float dt = (ros::Time::now()-t0).toSec();
	ROS_INFO("GetObstacles[%s - %.4f sec]:  %i points in %i clusters around %.0f %.0f %.0f (radius: %i)",type.c_str(),dt,pnts_count,polys_heightimage_clustered.size()
	,pnt.x,pnt.y,pnt.z,maprad_used);

//	if(biggest > 0 && polys_heightimage_clustered[biggest].polygon.points.size() > 5){
//		ROS_INFO("GOTIT");
	//	building_centroid.point = get_ave_pnt_poly(polys_heightimage_clustered[biggest]);
	//	ROS_INFO("GOTIT2");
//	}
}
void colorize_pathimg(nav_msgs::Path pathin){
	img_blank_mono.copyTo(img_paths);
	for(int i = 0; i < pathin.poses.size(); i++){
		img_paths.at<uchar>(y2r(pathin.poses[i].pose.position.y),x2c(pathin.poses[i].pose.position.x)) = pathin.poses[i].pose.position.z;
	}
}

/////*************************PATH*************///////////

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
	//	//ROS_INFO("in vec: %i",k);
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
		if(!is_point_in_path){
			float dst = get_dst2d(pathin.poses[i].pose.position,pnt);
 	    if(dst < lowest_dist){
 	      lowest_dist_i = i;
 	      lowest_dist = dst;
	 		}
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


void set_target_pose(geometry_msgs::PoseStamped pose){
	pose.z = get_rounded_z(pose.pose.position.z);
	target_pose = pose;
	last_pose_yaw = tf::getYaw(pose.pose.orientation);
	path_targets_sent.poses.push_back(target_pose);
	target_xyz.point  = target_pose.pose.position;
	target_xyz.header = hdr();
	if(state_building == "inspecting_building")
		path_building_targets.poses.push_back(target_pose);
	target_pose.header = hdr();
	pub_setp_xyz.publish(target_xyz);
//	else
	//	pub_target.publish(target_pose);
}
void set_target_path(nav_msgs::Path pathin){
	if(pathin.poses.size() > 0){
		path_targets       = pathin;
		path_targets_i     = 0;
		set_target_pose(path_targets.poses[path_targets_i]);
	}
}
int get_next_target(std::vector<int> vec_blacklist,nav_msgs::Path pathin,geometry_msgs::Point current_pos,float current_yaw){
  int best_i = -1;
  float lowest_dist = 1000;
  for(int i = 0; i < pathin.poses.size(); i++){
    if(!in_vec(vec_blacklist,i)){
      float dyaw = get_shortest(get_hdng(pathin.poses[i].pose.position,pos),pos_yaw);
      if(dyaw < 0)
        dyaw *= -1;
      if(dyaw < M_PI/2){
        float dst = get_dst3d(pos,pathin.poses[i].pose.position);
        if(dst < lowest_dist){
          lowest_dist = dst;
          best_i = i;
        }
      }
    }
  }
  return best_i;
}
void get_continous_path(nav_msgs::Path pathin){
	float closest_to_centroid = 100000;
  nav_msgs::Path pathout = get_targets_remaining();
	geometry_msgs::Point last_target = target_pose.pose.position;
	if(pathout.poses.size() > 0)
     last_target = pathout.poses[pathout.poses.size()-1].pose.position;
  for(int i = 0; i < pathin.poses.size(); i++){
    float closest_dst = get_dst2d(last_target,pathin.poses[i].pose.position);
    if(closest_dst > 0 && closest_dst < 6){
      float dhdng = get_shortest(get_hdng(pathin.poses[i].pose.position,last_target),get_hdng(last_target,pos));
      if(dhdng < M_PI/3 && dhdng > -M_PI/3){
				last_target = pathin.poses[i].pose.position;
        pathout.poses.push_back(pathin.poses[i]);
	    }
	  }
	}
	if(pathin.poses.size() > path_targets.poses.size())
		set_target_path(pathout);
}
void get_target_from_path_unknown(){
	geometry_msgs::Point p0;
	p0 = pos;
	p0.y += 20;
	int ci = getinpath_closestindex2d_blacklist(path_unknown,p0);
	if(ci >= 0 && path_unknown.poses.size() > ci){
		float dst = get_dst2d(path_unknown.poses[ci].pose.position,pos);
		blacklist.push_back(ci);
		ROS_INFO("New target[%.0f,%.0f] closest index unknown path: %i / %i, blacklist: %i points, dst_to_point: %.0f",path_unknown.poses[ci].pose.position.x,path_unknown.poses[ci].pose.position.y,ci,path_unknown.poses.size(),blacklist.size(),dst);
		set_target_pose(path_unknown.poses[ci]);
	}
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
    //ROS_INFO("pathin %i -> pathout %i: indexes_in_segment: %i ave_pnt_i: %i",pathin.poses.size(),pathout.poses.size(),indexes_in_segment,ave_pnt_i);
    if(indexes_in_segment >= pathin.poses.size())
      break;
    else
      pathin = remove_nth_first(pathin,indexes_in_segment);
  }
	ROS_INFO("done, %i",pathout.poses.size());
	pathout.header = hdr();
  return pathout;
}
nav_msgs::Path create_ordered_path(nav_msgs::Path pathin){
  std::vector<int> vec_blacklist;
  geometry_msgs::Point current_pos;
  nav_msgs::Path pathout;
  if(pathin.poses.size() < 2){
    return pathout;
  }
  float current_yaw = pos_yaw;
  current_pos = pos;
  int best_i  = get_next_target(vec_blacklist,pathin,current_pos,current_yaw);
  while(best_i >= 0){
    current_yaw = get_hdng(pathin.poses[best_i].pose.position,current_pos);
    current_pos = pathin.poses[best_i].pose.position;
    vec_blacklist.push_back(best_i);
    best_i = get_next_target(vec_blacklist,pathin,current_pos,current_yaw);
  }
  for(int i = 0; i < vec_blacklist.size(); i++){
    pathout.poses.push_back(pathin.poses[vec_blacklist[i]]);
  }
  ROS_INFO("poses_in: %i, poses out: %i",pathin.poses.size(),pathout.poses.size());
	pathout.header = hdr();
  return pathout;
}
void path_unknown_cb(const nav_msgs::Path::ConstPtr& msg){
	path_unknown = *msg;
}
void mb_result_cb(const std_msgs::Bool::ConstPtr& msg){
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

std::vector<geometry_msgs::PolygonStamped> find_clusters(geometry_msgs::PolygonStamped polyin, float cluster_spacing_cutoff){
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

int overlap_percent_in_path(nav_msgs::Path pathin){
	if(pathin.poses.size() == 0)
		return 0;
	pathin = sort_path(pathin,"hdng");
	geometry_msgs::Point pnt_current = pathin.poses[0].pose.position;
	std::vector<int> indexes_at_point;
	std::vector<std::vector<int>> indexes_at_points;
	nav_msgs::Path path_base;
	path_base.header = hdr();
	for(int k = 0; k < pathin.poses.size(); k++){
		geometry_msgs::Point pnt = pathin.poses[k].pose.position;
		if(!((abs(pnt.x) == abs(pnt_current.x)) && (abs(pnt.y) == abs(pnt_current.y)))){
			indexes_at_points.push_back(indexes_at_point);
			indexes_at_point.resize(0);
			path_base.poses.push_back(pathin.poses[k]);
		}
		indexes_at_point.push_back(k);
	}
	indexes_at_points.push_back(indexes_at_point);
	float pb = float(path_base.poses.size());
	float pi = float(pathin.poses.size());
	float overlap = pb/pi;
	float percent = overlap * 100.0;
	int   intperc = int(round((percent)));
	ROS_INFO("Path in: %i base_path: %i overlap: %i percent",pathin.poses.size(),path_base.poses.size(),intperc);
	return intperc;
}

std::vector<nav_msgs::Path> get_clusters(nav_msgs::Path pathin){
	ros::Time t0 = ros::Time::now();
	std::vector<nav_msgs::Path> pathsout;
	nav_msgs::Path path_remaining,path_cluster;
	path_remaining = pathin;
	colorize_pathimg(pathin);
	if(pathin.poses.size() > 0 && pathin.poses[0].pose.position.z > zlvl_max)
		zlvl_max = pathin.poses[0].pose.position.z;
	int size_change = 5;
	int size_last = path_remaining.poses.size();
	while(path_remaining.poses.size() > 5 && size_last > 0){
		int fillcolor = 150;
		int area = floodfill(y2r(path_remaining.poses[0].pose.position.y),x2c(path_remaining.poses[0].pose.position.x),fillcolor);
		nav_msgs::Path path_remaining_new;
		for(int i = 0; i < path_remaining.poses.size(); i++){
			if(img_paths.at<uchar>(y2r(path_remaining.poses[i].pose.position.y),x2c(path_remaining.poses[i].pose.position.x)) == fillcolor)
				path_cluster.poses.push_back(pathin.poses[i]);
			else
				path_remaining_new.poses.push_back(pathin.poses[i]);
		}
		size_last = path_cluster.poses.size();
		path_remaining = path_remaining_new;
		path_cluster.header.frame_id = "map";
		pathsout.push_back(path_cluster);
		path_cluster.poses.resize(0);
	}
	cv::imwrite("/home/nuc/brain/clusters/"+std::to_string(cnt)+"get_paths_clusters.png",img_paths);
	return pathsout;
}
geometry_msgs::Point point_pointed_to(geometry_msgs::Point pnt,float yaw,float len){
	pnt.x += pnt.x + len * cos(yaw);
	pnt.y += pnt.y + len * sin(yaw);
	return pnt;
}
void set_target_scanpoint(geometry_msgs::Point pnt){
	current_midpoint.point = pnt;
	current_midpoint.header = hdr();
	if(state_building == "looking_for_building")
		pub_pnt.publish(current_midpoint);
	else
		pub_small_pnt.publish(current_midpoint);
}

int get_completion_percent(){
	int cc = 0;
	for(int i = 0; i < building_polygon.polygon.points.size(); i++){
		if(building_polygon.polygon.points[i].z + building_polygon.polygon.points[i].x != 0)
			cc++;
	}
	ROS_INFO("BLD: Completion: %i percentage: %i",cc,percentage_completion);
	return percentage_completion;
}

int get_completeness_by_radians(nav_msgs::Path pathin,geometry_msgs::Point origo){
	std::vector<bool> path2d_hdngs_represented;
	int num_rads = 16;
	float rads_pr_i = 2*M_PI / num_rads;
	path2d_hdngs_represented.resize(num_rads);
	for(int i = 0; i < pathin.poses.size(); i++){
		float rad = get_hdng(pathin.poses[i].pose.position,building_centroid.point);
		path2d_hdngs_represented[int(round((rad + M_PI) / rads_pr_i)] = true;
	}
	int represented = 0;
	for(int i = 0; i < num_rads; i++){
		if(path2d_hdngs_represented[i])
			represented++;
	}
	int percentage_completion = int(round(float(represented) / float(num_rads) * 100.0));
	ROS_INFO("Checking %i poses against origo %.0f %.0 gives %i percent completion ",pathin.poses.size(),origo.x,origo.y,percentage_completion);
	return percentage_completion;
}
bool check_if_floorpath_is_complete(nav_msgs::Path pathin,geometry_msgs::Point origo,int min_percent){
	int percent = get_completeness_by_radians(pathin,origo);
	if(percent > min_percent)
		return true;
	else
		return false;
}
bool check_if_floorpath_is_safe(nav_msgs::Path pathin, int radlen_xy){
	std::vector<int> path2d_elevation;
	std::vector<int> path2d_elevation_relative;
	pathin = sort_path(pathin,"building_rad");
	for(int i = 0; i < pathin.poses.size(); i++){
		path2d_elevation[i] 				 = get_elevation_at_xy(pathin.poses[i].pose.positio.x,pathin.poses[i].pose.positio.y,radlen_xy);
		path2d_elevation_relative[i] = int(roumd(pathin.poses[i].pose.position.z)) - path2d_elevation[i];
	}
	v_el 		= vec_to_min_max_ave(path2d_elevation);
	v_rel_el = vec_to_min_max_ave(path2d_elevation_relative);
	ROS_INFO("Elevation: %i -> %i, rel: %i -> %i ave: (%i -> %i)",v_el[0],v_el[1],v_rel_el[0],v_rel_el[1],v_el[2],v_rel_el[2])
	if(v_rel_el[1] < par_dz*3)
		return false;
	else
		return true;
}
nav_msgs::Path check_floorpath(nav_msgs::Path pathin){
	nav_msgs::Path path_building_above = create_ordered_path(get_standardized_cluster_simple(path_building_above));
	nav_msgs::Path path_building_below = create_ordered_path(get_standardized_cluster_simple(path_building_below));
	if(get_completeness_by_radians)
	if(path_building_above.poses.size() > 2 * path_building_below.poses.size()){
		ROS_INFO()
	}
}
geometry_msgs::PolygonStamped get_orbit_data(nav_msgs::Path pathin){
	std::vector<float> yaws,hdngs;
	for(int i = 0; i < pathin.poses.size(); i++){
		hdngs.push_back(get_hdng(pathin.poses[i].pose.position,building_centroid.point));
		yaws.push_back(tf::getYaw(pathin.poses[i].pose.position));
	}
	float dhdng_sum = 0;
	float dyaws_sum = 0;
	for(int i = 1; i < pathin.poses.size(); i++){
		dhdng_sum += get_shortest(hdngs[i],hdngs[i-1]);
		dyaws_sum += get_shortest(yaws[i],yaws[i-1]);
	}
	float dhdng_ave = dhdng_sum / float(pathin.poses.size()-1);
	float dyaws_ave = dyaws_sum / float(pathin.poses.size()-1);
	int dhdng_sum = int(round(((100.0 * dhdng_sum / (2*M_PI)))));
	int dyaws_sum = int(round(((100.0 * dyaws_sum / (2*M_PI)))));
	ROS_INFO("ORBIT_DATA-HDNG(cen): %.2f -> %.2f (%.2f rads)",hdngs[0],hdngs[hdngs.size()-1],dhdng_sum,get_shortest(hdngs[0],hdngs[hdngs.size()-1]));
	ROS_INFO("ORBIT_DATA-YAW(tar): %.2f -> %.2f (%.2f rads)",yaws[0],yaws[yaws.size()-1],dyaws_sum,get_shortest(yaws[0],yaws[yaws.size()-1]));
	ROS_INFO("ORBIT_DATA-COMPLETION: hdng: %i, yaw: %i",dhdng_sum,dyaws_sum);
	return fmax(dhdng_sum,dyaws_sum);
}

bool is_orbit_complete(nav_msgs::Path pathin){
	bool is_complete_dst,is_complete_yaw,is_complete_hdngs;
	float cutoff_hdng = M_PI/4;
	float cutoff_yaw = M_PI/4;
	float cutoff_dst = 10.0;
	int complete_count = 0;
	float dst_startpnt   = get_dst2d(pathin.poses[pathin.poses.size()-1].pose.position,pathin.poses[0].pose.position);
	float hdngs_startpnt = get_dst2d(pathin.poses[pathin.poses.size()-1].pose.position,pathin.poses[0].pose.position);
	float yaw_startpnt   = get_shortest(tf::getYaw(pathin.poses[pathin.poses.size()-1].pose.orientation),tf::getYaw(pathin.poses[0].pose.position));
	if(hdngs_startpnt < cutoff_hdng && hdngs_startpnt > cutoff_hdng){
		is_complete_hdngs = true;
		complete_count++;
	}
	if(yaw_startpnt < cutoff_yaw && yaw_startpnt > cutoff_yaw){
		is_complete_yaw = true;
		complete_count++;
	}
	if(dst_startpnt < cutoff_dst){
		is_complete_dst = true;
		complete_count++;
	}

	ROS_INFO("ORBIT_CHECK: dst: %.0f hdng: %.2f yaw: %.2f",dst_startpnt,hdngs_startpnt,yaw_startpnt);
	ROS_INFO("ORBIT_CHECK: dst: %i 	 hdng: %i 	yaw: %i",is_complete_dst,is_complete_hdngs,is_complete_yaw);
	ROS_INFO("ORBIT_CHECK: dst: %i 	 hdng: %i 	yaw: %i",is_complete_dst,is_complete_hdngs,is_complete_yaw);
	if(is_complete_dst && is_complete_count && is_complete_yaw)
		return true;
	else
		return false;
}
std::vector<geometry_msgs::Point> building_centroids;
std::vector<geometry_msgs::Point> building_centroids;
std::vector<nav_msgs::Path> building_paths_complete;
std::vector<nav_msgs::Path> building_paths_available;
std::vector<std::vector<nav_msgs::Path>> buildings_paths_complete;
std::vector<std::vector<geometry_msgs::PointStamped>> buildings_centroids;

void set_building_floor(nav_msgs::Path pathin){
	building_path.poses.resize(0);
	building_centroid.point = get_ave_pnt(pathin);
	building_centroid.header = hdr();
	set_target_path(pathin);
}

void set_floorpath(){
	if(building_paths_available.size() == 0){
		ROS_INFO("BUILDING COMPLETE");
		buildings_paths_complete.push_back(building_paths_complete);
		buildings_centroids.push_back(building_centroids);
		set_state_building("looking_for_building");
	}
	else{
		nav_msgs::Path path_temp = building_paths_available[0];
		building_paths_available.erase(building_paths_available.begin()+i);
		set_building_floor(path_temp);
	}
}

void update_inspection(){
	int complete_cutoff = 95;
  bool is_orbit_com = check_if_floorpath_is_complete(path_building_targets,building_centroid.point,complete_cutoff);
	building_polygon 			 	= get_orbit_data(path_building_targets);
	building_centroid.point = get_ave_pnt_poly(building_polygon);
	if(is_orbit_complete(path_building_targets) || is_orbit_com){
		ROS_INFO("ORBIT COMPLETE! - evaluating above and below");
		building_centroids.push_back(builiding_centroid);

		bool below_complete = check_if_floorpath_is_complete(path_building_below);
		bool below_safe     = check_if_floorpath_is_safe(path_building_below,radlen_xy);

		bool above_complete = check_if_floorpath_is_complete(path_building_above);
		bool above_safe     = check_if_floorpath_is_safe(path_building_above,radlen_xy);
		building_paths_complete.push_back(path_building_targets);
		if(above_complete && above_safe)
			building_paths_available.push_back(path_building_above);
		if(below_complete && below_safe)
			building_paths_available.push_back(path_building_below);
		set_floorpath();
	}
}

void get_target_looking(){
	geometry_msgs::Point pbest;
	geometry_msgs::PoseStamped ps;
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
			}
		}
	}
	if(get_dst2d(ps.pose.position,pos) > 5.0 && ps.header.frame_id == "map"){
		set_target_pose(ps);
	}
	else if((ros::Time::now()-path_visited.header.stamp).toSec() > 5.0){
		ROS_INFO("setting path unknown");
		path_spiral_i++;
		set_target_pose(path_spiral.poses[path_spiral_i]);
		//get_target_from_path_unknown();
	}
}

void update_scan(){
	if((get_dst3d(cmd_pos,last_pos) > 5.0) || (abs(get_shortest(cmd_pos_yaw,last_yaw)) > 1.0) || ((ros::Time::now()-last_sec).toSec() > 1.5)){
		last_pos = cmd_pos;
		last_yaw = cmd_pos_yaw;
		last_sec = ros::Time::now();
		set_target_scanpoint(target_pose.pose.position);
	}
}
void next_target(){
	if(path_targets.poses.size() > path_targets_i+1){
		ROS_INFO("Path_targets: %i, (of %i)",path_targets_i,path_targets.poses.size());
		path_targets_i++;
		set_target_pose(path_targets.poses[path_targets_i]);
	}
	else
		set_target_scanpoint(target_pose.pose.position);
}

void check_target(){
	target_distance = get_dst2d(pos,target_pose.pose.position);
	if((ros::Time::now()-path_visited.header.stamp).toSec() > 5.0){
		target_pose.pose.position = pos;
	}
	if(target_distance < 8.0){
		next_target();
	}
}
void update_approach(){
	if(target_distance < 4.0 && state_building == "approaching_building"){
		set_state_building("inspecting_building");
		next_target();
	}
}
void init_building(nav_msgs::Path pathin){
	if(pathin.poses.size() == 0)
		return;
	ROS_INFO("Initialising building scan");
	float building_direction = get_path_yaw(pathin);
	int ci = getinpath_closestindex2d(pathin,pos);
	if(pathin.poses.size() > ci && ci >= 0){
		building_centroid.point.x = pathin.poses[ci].pose.position.x + 10 * cos(building_direction);
		building_centroid.point.y = pathin.poses[ci].pose.position.y + 10 * sin(building_direction);
		building_centroid.point.z = get_rounded_z(pathin.poses[ci].pose.position.z);
		building_altlvl_active = pathin.poses[ci].pose.position.z / int(round(par_dz) * 2)
		set_state_building("approaching_building");
		ROS_INFO("path_targets[%i]: %.2f (building_dir: %.2f)",ci,tf::getYaw(pathin.poses[ci].pose.orientation));
		set_target_pose(pathin.poses[ci]);
	}
}
void check_overlap(nav_msgs::Path pathin){
	std::vector<nav_msgs::Path> paths = get_clusters(pathin);
	int best_overlap = 0;
	int best_overlap_i = 0;
	for(int i = 0; i < paths.size(); i++){
		if(paths[i].poses.size() > paths[best_overlap_i].poses.size()){
			best_overlap_i = i;
		}
	}
	if(best_overlap_i < paths[best_overlap_i].poses.size() &&  paths[best_overlap_i].poses.size() > 10)
		init_building(paths[best_overlap_i]);
}
void check_path_proc(nav_msgs::Path pathin){
	path_organize_2 = create_ordered_path(get_standardized_cluster_simple(path_organize_1));
	if(state_building == "inspecting_building"){
		if(path_organize_1.poses.size() > 0){
			if(path_targets.poses.size() > path_targets_i){
				ROS_INFO("get_continous_path");
				get_continous_path(path_organize_1);
			}
			else{
				ROS_INFO("set_target_path");
				set_target_path(path_organize_1);
			}
		}
	}
}
void path_side_cb(const nav_msgs::Path::ConstPtr& msg){
	if(msg->poses.size() > 0){
		ROS_INFO("PATH_SIDE_CB: %i poses ",msg->poses.size());
		if(state_building == "looking_for_building")
			check_overlap(*msg);
	}
}
void path2d_side_large_cb(const nav_msgs::Path::ConstPtr& msg){
	check_overlap(*msg);
}
void path2d_side_small_cb(const nav_msgs::Path::ConstPtr& msg){
	check_path_proc(*msg);
}
void path2d_side_above_cb(const nav_msgs::Path::ConstPtr& msg){
	if(msg->poses.size() > 0){
		nav_msgs::Path path_building_above_new = get_new_path(path_building_above,*msg,1.0);
		if(path_building_above_new.poses.size() > 0){
			for(int i = 0; i < path_building_above_new.poses.size(); i++){
				path_building_above.poses.push_back(path_building_above_new.poses[i]);
			}
		}
	}
}
void path2d_side_below_cb(const nav_msgs::Path::ConstPtr& msg){
	if(msg->poses.size() > 0){
		nav_msgs::Path path_building_below_new = get_new_path(path_building_below,*msg,1.0);
		if(path_building_below_new.poses.size() > 0){
			for(int i = 0; i < path_building_below_new.poses.size(); i++){
				path_building_below.poses.push_back(path_building_below_new.poses[i]);
			}
		}
	}
}

void path2d_side_down_cb(const nav_msgs::Path::ConstPtr& msg){
	if(msg->poses.size() > 0){
		nav_msgs::Path path_down_new = get_new_path(path_down,*msg,1.0);
		count_target_paths++;
		if(path_down_new.poses.size() > 0){
			for(int i = 0; i < path_down_new.poses.size(); i++){
				path_down.poses.push_back(path_down_new.poses[i]);
			}
		}
	}
}

void poly_side_obstacles_cb(const geometry_msgs::PolygonStamped::ConstPtr& msg){
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
void poly_down_obstacles_cb(const geometry_msgs::PolygonStamped::ConstPtr& msg){
	for(int i = 0; i < msg->polygon.points.size(); i++){
		int r = y2r(msg->polygon.points[i].y);
		int c = x2c(msg->polygon.points[i].x);
		int z = fmax(msg->polygon.points[i].z,1);
		if(img_height.at<cv::Vec3b>(r,c)[2] < z)
			img_height.at<cv::Vec3b>(r,c)[2] = z;
	}
}
void poly_auto_side_obstacles_cb(const geometry_msgs::PolygonStamped::ConstPtr& msg){
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
void poly_cleared_cb(const geometry_msgs::PolygonStamped::ConstPtr& msg){
	poly_cleared = *msg;
}
void poly_auto_side_cb(const geometry_msgs::PolygonStamped::ConstPtr& msg){
	poly_obstacles_received = *msg;
	/*float cluster_spacing_cutoff = 10;
	float cluster_visited_cutoff = 7;
	bool use_3d = true;
//	poly_obstacles_received = remove_path_overlap_from_poly(path_visited,*msg,cluster_visited_cutoff,use_3d);
	for(int i = 0; i < msg->polygon.points.size(); i++){
		if(poly_obstacles_received.polygon.points.size() > 0){
			if(!is_point32_in_poly(poly_obstacles_received,msg->polygon.points[i],1.0))
				poly_obstacles_received.polygon.points.push_back(msg->polygon.points[i]);
		}
		else
			poly_obstacles_received.polygon.points.push_back(msg->polygon.points[i]);
	}*/
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
					objects_polys.push_back(poly);
					poly.polygon.points.resize(0);
				}
			}
		}
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
	//ros::Subscriber s1 = nh.subscribe("/tb_world/path_visited",1,path_visited_cb);
	ros::Subscriber s0 = nh.subscribe("/tb_edto/path_side",1,path_side_cb);
	ros::Subscriber s2 = nh.subscribe("/tb_edto/path2d_side",1,get_side_path2d_cb);
	ros::Subscriber s3 = nh.subscribe("/tb_edto/path2d_small",10,path_side2d_cb);
	ros::Subscriber s4 = nh.subscribe("/tb_edto/poly_cleared",10,poly_cleared_cb);

	//	ros::Subscriber s5 = nh.subscribe("/tb_edto/path2d_side_large",10,path2d_side_large_cb);
	ros::Subscriber s5 = nh.subscribe("/tb_edto/path2d_side_small",10,path2d_side_small_cb);
	ros::Subscriber s1d = nh.subscribe("/tb_edto/path2d_side_up",10,path2d_side_up_cb);
	ros::Subscriber s1s = nh.subscribe("/tb_edto/path2d_side_down",10,path2d_side_down_cb);
	ros::Subscriber s1f = nh.subscribe("/tb_edto/poly_auto_side",10,poly_auto_side_cb);

	ros::Publisher pub_centroid 	= nh.advertise<geometry_msgs::PointStamped>("/tb_test/centroid",100);
	pub_pnt 			 								= nh.advertise<geometry_msgs::PointStamped>("/tb_edto/get_side2d",100);
	pub_small_pnt 			 					= nh.advertise<geometry_msgs::PointStamped>("/tb_edto/get_side2d_small",100);
	ros::Publisher pub_path_organize_1		= nh.advertise<nav_msgs::Path>("/tb_test/path_organize_1",100);
	ros::Publisher pub_path_organize_2		= nh.advertise<nav_msgs::Path>("/tb_test/path_organize_2",100);
	ros::Publisher pub_path_organize_raw	= nh.advertise<nav_msgs::Path>("/tb_test/path_organize_raw",100);
	ros::Publisher pub_path_spiral	= nh.advertise<nav_msgs::Path>("/tb_test/spiral",100);

	ros::Publisher building_polygon_pub       = nh.advertise<geometry_msgs::PolygonStamped>("/tb_bld/building_polygon",100);
	ros::Publisher pub_obs_poly       = nh.advertise<geometry_msgs::PolygonStamped>("/tb_test/obspoly",100);
	pub_setp_xyz       = nh.advertise<geometry_msgs::PointStamped>("/tb_cmd/set_xyz",100);
	////
	//

	ros::Subscriber s01 = nh.subscribe("/tb_world/path_unknown",10,path_unknown_cb);
	ros::Subscriber s02 = nh.subscribe("/tb_cmd/mb_result",10,mb_result_cb);

	pub_target 				= nh.advertise<geometry_msgs::PoseStamped>("/tb_cmd/mb_pose_target",100);
	pub_target_path  	= nh.advertise<nav_msgs::Path>("/tb_test/path_targets",100);
	path_spiral = create_path_spiral(15);
	ros::Rate rate(5.0);
	cnt = 6;
	int ccnt = 0;
	ros::Time t_start = ros::Time::now();
	while(ros::ok()){
		checktf1();
		checktf2();
		check_target();

		poly_obstacles_received.header = building_centroid.header = hdr();
		pub_centroid.publish(building_centroid);
		building_polygon_pub.publish(building_polygon);
		pub_path_organize_raw.publish(path_organize_raw);
		pub_path_organize_1.publish(path_organize_1);
		pub_path_organize_2.publish(path_organize_2);
		pub_path_spiral.publish(path_spiral);
		pub_obs_poly.publish(poly_obstacles_received);
		pub_target_path.publish(path_targets);
		rate.sleep();
		ros::spinOnce();
		//if(ros::Time::now() - t_start).toSec() > 5.0){
			if(ccnt == 1)
				get_obstacles_ffill("side");
			if(ccnt == 3)
				get_obstacles_ffill("comb");
			if(ccnt == 4)
				get_obstacles_ffill("down");
	//	}
			ROS_INFO("DONE WITH OBST, ccnt: %i",ccnt);
	//		update_scan();
			if(ccnt >= 5){
			ccnt = 0;
			if(state_building == "looking_for_building"){
				ROS_INFO("UPDATE: LOOKING");
				if(target_distance < 5.0)
					get_target_looking();
				ROS_INFO("UPDATE: LOOKING_DONE");
			}
			if(state_building == "approaching_building"){
				update_approach();
				ROS_INFO("UPDATE: APPROACH_DONE");
			}
			if(state_building == "inspecting_building"){
				update_inspection();
					pub_setp_xyz.publish(target_xyz);
			}
			ROS_INFO("state_building: %s",state_building.c_str());
			drawimg();
		}
		else
			ccnt++;
	}
	return 0;
}
