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
ros::Publisher pub_target,pub_target_point,pub_pnt,pub_small_pnt,pub_elevate,pub_target_path,pub_setp_xy;

cv::Mat img_blank_mono(1000,1000,CV_8U,cv::Scalar(0)); //create image, set encoding and size, init pixels to default val
cv::Mat img_mono(1000,1000,CV_8U,cv::Scalar(0)); //create image, set encoding and size, init pixels to default val
cv::Mat img_paths(1000,1000,CV_8U,cv::Scalar(0)); //create image, set encoding and size, init pixels to default val
cv::Mat img(1000,1000,CV_8UC3,cv::Scalar(0,0,0)); //create image, set encoding and size, init pixels to default val
cv::Mat img_perm(1000,1000,CV_8UC3,cv::Scalar(0,0,0)); //create image, set encoding and size, init pixels to default val
cv::Mat img_blank(1000,1000,CV_8UC3,cv::Scalar(0,0,0)); //create image, set encoding and size, init pixels to default val
cv::Mat imgdeb(1000,1000,CV_8UC3,cv::Scalar(0,0,0)); //create image, set encoding and size, init pixels to default val
cv::Mat img_height(1000,1000,CV_8UC3,cv::Scalar(0,0,0)); //create image, set encoding and size, init pixels to default val

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
geometry_msgs::PolygonStamped building_polygon,poly_cleared,poly_obstacles;
std::vector<float>building_ranges_at_rad;
int last_i = 0;
ros::Time last_rosinfo;
float pos_yaw = 0;
float last_pose_yaw = 0;
float last_yaw;
geometry_msgs::PoseStamped last_pose,target_pose,current_midpose;
std::vector<geometry_msgs::PolygonStamped> buildings_polygons;
std::vector<geometry_msgs::PolygonStamped> obstacles_polygons;
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
nav_msgs::Path path_unknown;
nav_msgs::Path path_organize_1,path_organize_2,path_organize_raw;
nav_msgs::Path path_targets,path_targets_sent;
std::vector<int> targets_complete;
int path_targets_i = 0;
geometry_msgs::Point info_point;
float info_interval = 20;
int building_v0 = 0;
float rad2deg = 180.0/M_PI;

geometry_msgs::Point get_ave_pnt_poly(geometry_msgs::PolygonStamped polyin){
	geometry_msgs::Point pnt;
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
    state_building  = newstate;
  }
}
float get_dst2d(geometry_msgs::Point p2, geometry_msgs::Point p1){
  return(sqrt(pow(p1.x-p2.x,2)+pow(p1.y-p2.y,2)));
}
float get_dst2d32(geometry_msgs::Point32 p2, geometry_msgs::Point32 p1){
  return(sqrt(pow(p1.x-p2.x,2)+pow(p1.y-p2.y,2)));
}
float get_dst3d32(geometry_msgs::Point32 p2, geometry_msgs::Point32 p1){
  return(sqrt(pow(p1.x-p2.x,2)+pow(p1.y-p2.y,2)));
}
float get_dst2d3(geometry_msgs::Point32 p2, geometry_msgs::Point p1){
  return(sqrt(pow(p1.x-p2.x,2)+pow(p1.y-p2.y,2)));
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
bool is_point_in_path(nav_msgs::Path path_to_check,geometry_msgs::Point pin,float lim){
  float res,dst;
  if(path_to_check.poses.size() == 0)
    return false;
  for(int i = 0; i < path_to_check.poses.size(); i++){
     if(get_dst3d(path_to_check.poses[i].pose.position,pin) < lim)
        return true;
  }
  return false;
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
nav_msgs::Path remove_path(nav_msgs::Path path_to_check,nav_msgs::Path pathin,float cutoff_dst){
	nav_msgs::Path pathout;
	pathout.header = hdr();
	for(int i = 0; i < path_to_check.poses.size(); i++){
		if(!is_point_in_path(path_to_check,pathin.poses[i].pose.position,cutoff_dst)){
			pathout.poses.push_back(pathin.poses[i]);
		}
	}
	return pathout;
}

geometry_msgs::PolygonStamped remove_poly_path(nav_msgs::Path path_to_check,geometry_msgs::PolygonStamped polyin, float visited_dst_cutoff){
	geometry_msgs::PolygonStamped polyout;
	polyout.header = hdr();
	for(int i = 0; i < polyin.polygon.points.size(); i++){
		if(is_point32_in_path(path_to_check,polyin.polygon.points[i],visited_dst_cutoff)){
			polyout.polygon.points.push_back(polyin.polygon.points[i]);
		}
	}
	return polyout;
}
nav_msgs::Path remove_path_other(nav_msgs::Path path_to_check,nav_msgs::Path pathin,float cutoff_dst){
	return remove_path(path_to_check,pathin,cutoff_dst);
}
nav_msgs::Path remove_path_visited(nav_msgs::Path pathin,float cutoff_dst){
	return remove_path(path_visited,pathin,cutoff_dst);
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

std::string create_info(std::vector<std::string> types, std::vector<float> vals){
	std::string string_out;
	for(int i = 0; i < types.size(); i++){
		string_out += types[i];
		string_out += std::to_string(int(round(vals[i])));
	}
	return string_out;
}
int get_float(float val){
	return int(round(val));
}
std::string write_xyz(geometry_msgs::Point pnt){
	return " x: " + std::to_string(get_float(pnt.x)) + " x: " + std::to_string(get_float(pnt.y)) + " z: " + std::to_string(get_float(pnt.z));
}
void drawimg(){
	for(int r = 0; r < img.rows; r++){
		for(int c = 0; c < img.cols; c++){
			img.at<cv::Vec3b>(r,c)[0] = img_height.at<cv::Vec3b>(r,c)[0]*10;
			img.at<cv::Vec3b>(r,c)[1] = img_height.at<cv::Vec3b>(r,c)[1]*10;
			img.at<cv::Vec3b>(r,c)[2] = img_height.at<cv::Vec3b>(r,c)[2]*10;
		}
	}
	draw_path("path_visited",path_visited,0,1,0,true,cv::Scalar(0,25,100),true);
	draw_path("path_organize_1",path_organize_1,0,0,1,true,cv::Scalar(100,0,20),false);
	draw_path("path_organize_2",path_organize_2,0,0,1,true,cv::Scalar(0,100,40),false);
	draw_path("targets",path_targets,2,2,4,false,cv::Scalar(0,25,200),true);

	draw_pose("target",target_pose.pose.position,2,2,5,false,tf::getYaw(target_pose.pose.orientation),cv::Scalar(0,0,200));
	draw_pose("pos",pos,2,2,5,false,pos_yaw,cv::Scalar(200,200,200));
	draw_pose("building",building_centroid.point,2,2,0,true,0,cv::Scalar(0,100,25));

	draw_poly("cleared",poly_cleared,0,0,0,false,cv::Scalar(0,200,25),true);
	int count_color = 0;
	std::vector<std::string> types;
	std::vector<std::string> vals;
	for(int i = 0; i < obstacles_polygons.size(); i++){
		geometry_msgs::Point p_cen = get_ave_pnt_poly(obstacles_polygons[i]);
		std::string info = "poly_obs#" + std::to_string(i) + ": " + std::to_string(obstacles_polygons[i].polygon.points.size()) + write_xyz(p_cen);
		count_color++;
		if(count_color == 3)
			count_color = 0;
		cv::Scalar color = get_shifting_color(count_color,100);
		if(obstacles_polygons[i].polygon.points.size() > 3)
			draw_poly(info,obstacles_polygons[i],0,0,0,true,color,true);
		else
			draw_poly(info,obstacles_polygons[i],0,0,0,true,color,true);
	}
	draw_poly("poly_obstacles_raw",poly_obstacles,0,0,0,true,cv::Scalar(100,0,0),false);
	draw_and_reset_img();
}


//void drawimg(){
//	draw_path(path_visited,get_color(100,50,50),1);
//	draw_and_boxin_path(path_organize_1,get_color(200,0,20));
//	draw_and_boxin_path(path_organize_2,get_color(0,20,200));
//	draw_target_and_pos();
//}

int floodfill(int r0, int c0, int fillcolor){
	int ffillMode = 1;  int connectivity = 8;  int newMaskVal = 255;
	int flags = connectivity + (newMaskVal << 8) +
			 (ffillMode == 1 ? cv::FLOODFILL_FIXED_RANGE : 0);
	cv::Rect ccomp;
	cnt2++;
	int area = cv::floodFill(img_paths,cv::Point(c0,r0),cv::Scalar(fillcolor), &ccomp, cv::Scalar(1),cv::Scalar(1), flags);
	return area;
}

void colorize_pathimg(nav_msgs::Path pathin){
	img_blank_mono.copyTo(img_paths);
	for(int i = 0; i < pathin.poses.size(); i++){
		img_paths.at<uchar>(y2r(pathin.poses[i].pose.position.y),x2c(pathin.poses[i].pose.position.x)) = pathin.poses[i].pose.position.z;
	}
}

void colorize_zlvlimg(nav_msgs::Path pathin){
	int iz;
	for(int i = 0; i < pathin.poses.size(); i++){
		float yaw_pose = tf::getYaw(pathin.poses[i].pose.orientation);
		int ro = y2r(int(pathin.poses[i].pose.position.y + par_dst_target * sin(yaw_pose)));
		int co = x2c(int(pathin.poses[i].pose.position.x + par_dst_target * cos(yaw_pose)));
		int r = y2r(pathin.poses[i].pose.position.y);
		int c = x2c(pathin.poses[i].pose.position.x);
		iz = 1+int(round(pathin.poses[i].pose.position.z / par_dz));
		mapimgs_at_zlvlz[iz].at<cv::Vec3b>(r,c)[0] 	 = pathin.poses[i].pose.position.z*5;
		mapimgs_at_zlvlz[iz].at<cv::Vec3b>(ro,co)[2] = pathin.poses[i].pose.position.z*5;
	}
	cv::imwrite("/home/nuc/brain/scanproc/test.png",mapimgs_at_zlvlz[iz]);
}
std::vector<nav_msgs::Path> get_clusters(nav_msgs::Path pathin){
	ros::Time t0 = ros::Time::now();
	std::vector<nav_msgs::Path> pathsout;
	nav_msgs::Path path_remaining,path_cluster;
	path_remaining = pathin;
	colorize_pathimg(pathin);
	colorize_zlvlimg(pathin);
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
	cv::imwrite("/home/nuc/brain/cluster/"+std::to_string(cnt)+"get_paths_clusters.png",img_paths);
	return pathsout;
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
int get_completion_percent_orig(){
	int cc = 0;
	for(int i = 0; i < building_polygon.polygon.points.size(); i++){
		if(building_polygon.polygon.points[i].z + building_polygon.polygon.points[i].x != 0)
			cc++;
	}
	int percentage_completion = int(round(float(cc) / float(building_polygon.polygon.points.size()) * 100.0));
	ROS_INFO("BLD: Completion: %i percentage: %i",cc,percentage_completion);
	return percentage_completion;
}
int get_completion_percent_old(){
	int cc = 0;
	for(int i = building_v0; i < path_visited.poses.size(); i++){
		if(building_polygon.polygon.points[i].z + building_polygon.polygon.points[i].x != 0)
			cc++;
	}
	int percentage_completion = int(round(float(cc) / float(building_polygon.polygon.points.size()) * 100.0));
	ROS_INFO("BLD: Completion: %i percentage: %i",cc,percentage_completion);
	return percentage_completion;
}
void set_target_scanpoint(geometry_msgs::Point pnt){
	current_midpose.pose.position = pnt;
	current_midpose.pose.orientation.w = 1.0;
	current_midpoint.point = pnt;
	current_midpoint.header = hdr();
	if(state_building == "looking_for_building")
		pub_pnt.publish(current_midpoint);
	else
		pub_small_pnt.publish(current_midpoint);
}

void set_target_pose(geometry_msgs::PoseStamped pose){
	target_pose = pose;
	last_pose_yaw = tf::getYaw(pose.pose.orientation);
	target_point.point  = pose.pose.position;
	target_pose.header = target_point.header = hdr();
	path_targets_sent.poses.push_back(target_pose);
	geometry_msgs::PointStamped target_xy;
	target_xy.point = target_pose.pose.position;
	target_xy.header = hdr();
	if(state_building == "inspecting_building")
		pub_setp_xy.publish(target_xy);
	else
		pub_target.publish(target_pose);
}

nav_msgs::Path remove_reverse(nav_msgs::Path pathin,float max_delta_hdng){
	nav_msgs::Path pathout;
	pathout.header = hdr();
	for(int i = 0; i < pathin.poses.size(); i++){
		float hdng = get_hdng(pathin.poses[i].pose.position,pos);
		float dhdng = get_shortest(hdng,pos_yaw);
		if(dhdng < max_delta_hdng && dhdng > -max_delta_hdng)
			pathout.poses.push_back(pathin.poses[i]);
	}
	return pathout;
}
nav_msgs::Path remove_reverse2(nav_msgs::Path pathin,float max_delta_hdng){
	nav_msgs::Path pathout;
	pathout.header = hdr();
	for(int i = 0; i < pathin.poses.size(); i++){
		float dhdng = get_shortest(tf::getYaw(pathin.poses[i].pose.orientation),last_pose_yaw);
		if(dhdng < max_delta_hdng && dhdng > -max_delta_hdng)
			pathout.poses.push_back(pathin.poses[i]);
	}
	return pathout;
}

void request_next_hori(nav_msgs::Path pathin){
	nav_msgs::Path pathout;
	pathout.header = hdr();
	colorize_pathimg(pathin);
	floodfill(y2r(target_pose.pose.position.y),x2c(target_pose.pose.position.x),200);
	for(int i = 0; i < pathin.poses.size(); i++){
		if(img_paths.at<uchar>(y2r(pathin.poses[i].pose.position.y),x2c(pathin.poses[i].pose.position.x)) == 200)
			pathout.poses.push_back(pathin.poses[i]);
	}
	ROS_INFO("request_next_hori pathout: %i",pathout.poses.size());
	pathout = remove_path_visited(pathout,5);
	pathout = remove_reverse(pathout,M_PI/2);
	pathout = remove_reverse2(pathout,M_PI/2);
	int best_i = 0;
	float best_dst = 0;
	float target_dst = 10.0;
	for(int i = 0; i < pathout.poses.size(); i++){
		float dst = get_dst2d(pathout.poses[i].pose.position,target_pose.pose.position);
		float ddst = abs(dst-target_dst);
		if(ddst < best_dst){
			best_dst = ddst;
			best_i = i;
		}
	}
	ROS_INFO("request_next_hori NextPos: %.0f %.0f %.0f Request next hori: %i pathin %i pathout  best_i %i",pos.x,pos.y,pos.z,pathin.poses.size(),pathout.poses.size(), pathout);
	if(best_i < pathout.poses.size() && best_i >= 0){
	//	set_target_scanpoint(pathout.poses[best_i].pose.position);
	}
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
void init_building(nav_msgs::Path pathin){
	if(pathin.poses.size() == 0)
		return;
	ROS_INFO("Initialising building scan");
	float building_direction = get_path_yaw(pathin);
	int ci = getinpath_closestindex2d(pathin,pos);
	if(pathin.poses.size() > ci && ci >= 0){
		building_centroid.point.x = pathin.poses[ci].pose.position.x + 10 * cos(building_direction);
		building_centroid.point.y = pathin.poses[ci].pose.position.y + 10 * sin(building_direction);
		building_centroid.point.z = pathin.poses[ci].pose.position.z;
		building_ranges_at_rad.resize(0);
		building_ranges_at_rad.resize(building_polygon.polygon.points.size());
		set_state_building("approaching_building");
		ROS_INFO("path_targets[%i]: %.2f (building_dir: %.2f)",ci,tf::getYaw(pathin.poses[ci].pose.orientation));
		set_target_pose(pathin.poses[ci]);
	}
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

void path_visited_cb(const nav_msgs::Path::ConstPtr& msg){

}
void update_scanning_building(nav_msgs::Path pathin){

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

nav_msgs::Path get_targets_remaining(){
  nav_msgs::Path pathout;
  pathout.header = hdr();
  for(int i = path_targets_i; i < path_targets.poses.size(); i++){
    pathout.poses.push_back(path_targets.poses[i]);
  }
  return pathout;
}
void set_target_path(nav_msgs::Path pathin){
	if(pathin.poses.size() > 0){
		path_targets       = pathin;
		path_targets_i     = 0;
		set_target_pose(path_targets.poses[path_targets_i]);
	}
}
void get_continous_path(nav_msgs::Path pathin){
  nav_msgs::Path path_targets_remaining = get_targets_remaining();
  if(path_targets_remaining.poses.size() > 0){
    int last_target_i = path_targets_remaining.poses.size()-1;
    geometry_msgs::Point last_target = path_targets_remaining.poses[last_target_i].pose.position;
    for(int i = 0; i < pathin.poses.size(); i++){
      int closest_i = get_closest_i(path_targets_remaining,pathin.poses[i].pose.position);
      if(closest_i == last_target_i){
        float closest_dst = get_dst3d(last_target,pathin.poses[i].pose.position);
        float dst_pos = get_dst3d(last_target,pos);
        if(dst_pos < 15){
          if(closest_dst > 3 && closest_dst < 10){
            float hdng_pos    = get_hdng(pathin.poses[i].pose.position,pos);
            float hdng_target = get_hdng(pathin.poses[i].pose.position,last_target);
            float dhdng = get_shortest(hdng_target,hdng_pos);
            if(dhdng < 0)
              dhdng *= -1;
            if(dhdng < M_PI/3){
              last_target = pathin.poses[i].pose.position;
              path_targets_remaining.poses.push_back(pathin.poses[i]);
              path_targets.poses.push_back(pathin.poses[i]);
              last_target_i = path_targets_remaining.poses.size()-1;
              ROS_INFO("Adding pose");
            }
          }
        }
      }
    }
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
nav_msgs::Path remove_nth_first(nav_msgs::Path pathin,int i0){
  nav_msgs::Path pathout;
  pathout.header = hdr();
  if(i0 < 0)
    return pathout;
	ROS_INFO("pathin: %i, i0: %i",pathin.poses.size(),i0);
  for(int i = i0; i < pathin.poses.size(); i++){
    pathout.poses.push_back(pathin.poses[i]);
  }
	ROS_INFO("pathout: %i, i0: %i",pathout.poses.size(),i0);

  return pathout;
}
int get_indexes_within_rad(nav_msgs::Path pathin,float max_dst,geometry_msgs::Point centroid){
  for(int i = 0; i < pathin.poses.size(); i++){
    if(get_dst3d(pathin.poses[i].pose.position,centroid) > max_dst)
      return i;
  }
  return pathin.poses.size();
}
float get_tot_len(nav_msgs::Path pathin){
  float len = 0;
  for(int i = 1; i < pathin.poses.size(); i++){
    len += get_dst3d(pathin.poses[i].pose.position,pathin.poses[i-1].pose.position);
  }
  return len;
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
geometry_msgs::PoseStamped get_ave_pose(nav_msgs::Path pathin){
  geometry_msgs::PoseStamped ps;
  float sum_x  = 0; float sum_y = 0; float sum_z  = 0; float sum_yaw = 0;
  float zmn = 100; float zmx = -100;
  int down_count = 0;
  for(int i = 0; i < pathin.poses.size(); i++){
    if(pathin.poses[i].pose.orientation.y == 0.7071)
      down_count++;
    else
      sum_yaw += tf::getYaw(pathin.poses[i].pose.orientation);
    sum_x += pathin.poses[i].pose.position.x;
    sum_y += pathin.poses[i].pose.position.y;
    sum_z += pathin.poses[i].pose.position.z;
    if(zmn > pathin.poses[i].pose.position.z)
      zmn = pathin.poses[i].pose.position.z;
    if(zmx > pathin.poses[i].pose.position.z)
      zmx = pathin.poses[i].pose.position.z;
  }
  int side_count = pathin.poses.size() - down_count;
  if(side_count > down_count)
    ps.pose.orientation = tf::createQuaternionMsgFromYaw(sum_yaw / side_count);
  else
    ps.pose.orientation.y = ps.pose.orientation.w = 0.7071;
  ps.pose.position.x = sum_x / pathin.poses.size();
  ps.pose.position.y = sum_y / pathin.poses.size();
  ps.pose.position.z = sum_z / pathin.poses.size();
  if(down_count > side_count){
    ps.pose.position.z = zmx + 4;
  }
  return ps;
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
//    if(pathin.poses[ave_pnt_i].pose.orientation.y = 0.7071)
  //    pathin.poses[ave_pnt_i].pose.position.z = zmx + 3;
    ave_pnt_i          = get_closest_i(pathin,ave_pnt);
    pnt_ref            = pathin.poses[ave_pnt_i].pose.position;
    pathout.poses.push_back(pathin.poses[ave_pnt_i]);
    ROS_INFO("pathin %i -> pathout %i: indexes_in_segment: %i ave_pnt_i: %i",pathin.poses.size(),pathout.poses.size(),indexes_in_segment,ave_pnt_i);
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
  ////ROS_INFO("poses_in: %i, poses out: %i",pathin.poses.size(),pathout.poses.size());
	pathout.header = hdr();
  return pathout;
}
void check_overlap(nav_msgs::Path pathin){
	std::vector<nav_msgs::Path> paths = get_clusters(pathin);
	int best_overlap = 0;
	int best_overlap_i = 0;
	for(int i = 0; i < paths.size(); i++){

		int overlap = overlap_percent_in_path(pathin);
		if(paths[i].poses.size() > paths[best_overlap_i].poses.size()){
			//  && (overlap > best_overlap && paths[i].poses.size() > paths[best_overlap_i].poses.size()){
			ROS_INFO("check_overlap Overlap[#%i]: %i percent (%i poses)",i,overlap,paths[i].poses.size());
			best_overlap_i = i;
		//	best_overlap   = overlap;
		}
	}
	if(best_overlap_i < paths[best_overlap_i].poses.size() &&  paths[best_overlap_i].poses.size() > 10)
		init_building(paths[best_overlap_i]);
}
void evaluate_potential_targetpath(nav_msgs::Path pathin){
//	nav_msgs::Path org_2_notvstd = remove_path_visited(path_organize_1,5);
	nav_msgs::Path org_2_notargt = remove_path_other(path_targets,pathin,5);
	nav_msgs::Path potential_targets = constrain_path_bbpoly(org_2_notargt,poly_cleared,true);
	if(state_building == "inspecting_building"){
		ROS_INFO("CHECK p1: %i -> %i ->clear-> %i ",pathin.poses.size(),org_2_notargt.poses.size(),potential_targets.poses.size());
		if(potential_targets.poses.size() == 0){
			potential_targets = org_2_notargt;
			ROS_INFO("returned path to pre-polygon-constrain(0->%i)",potential_targets.poses.size());
		}
		if(potential_targets.poses.size() > 0){
			if(path_targets.poses.size() > path_targets_i)
				get_continous_path(potential_targets);
			else
				set_target_path(potential_targets);
		}
	}
}
void update_building_hori(nav_msgs::Path pathin){
	building_polygon.header = hdr();
	ROS_INFO("update_building_hori");
	float hdng_cutoff = M_PI/3;
	float rads_pr_i = 2*M_PI / 72;
	for(int i = building_v0; i < path_visited.poses.size(); i++){
		float rel_hdng  = get_hdng(pathin.poses[i].pose.position,building_centroid.point);
		float pose_hdng = tf::getYaw(pathin.poses[i].pose.orientation);
		float d_hdng = get_shortest(pose_hdng,rel_hdng);
		float pose_dist = get_dst2d(pathin.poses[i].pose.position,building_centroid.point);
		int 	ii  	    = int(round((rel_hdng + M_PI) / rads_pr_i));
		if(d_hdng < hdng_cutoff && d_hdng > -hdng_cutoff){
			if((building_ranges_at_rad[ii] == 0) || (pose_dist < building_ranges_at_rad[i])){
				 building_ranges_at_rad[ii] = pose_dist;
				 building_polygon.polygon.points[ii].x = pathin.poses[i].pose.position.x;
				 building_polygon.polygon.points[ii].y = pathin.poses[i].pose.position.y;
				 building_polygon.polygon.points[ii].z = pathin.poses[i].pose.position.z;
		 	}
		}
	}
	int percent = get_completion_percent_old();
	//if(percent > 90){
	//	set_state_building("looking_for_building");
	//	buildings_polygons.push_back(building_polygon);
	//}
}


void check_path_proc(nav_msgs::Path pathin){
	if(state_building == "inspecting_building")
		update_building_hori(pathin);
	path_organize_raw = pathin;
	path_organize_1 = remove_path_visited(get_standardized_cluster_simple(pathin),5.0);
//	path_organize_2 = create_ordered_path(get_standardized_cluster_simple(path_organize_1));
	evaluate_potential_targetpath(path_organize_1);

	//path_organize_2 = create_ordered_path(pathin);
//	ROS_INFO("path1: %i path2: %i",path_organize_1.poses.size(),path_organize_2.poses.size());
}

void path_side_cb(const nav_msgs::Path::ConstPtr& msg){
	if(msg->poses.size() > 0){
		ROS_INFO("PATH_SIDE_CB: %i poses ",msg->poses.size());
		if(state_building == "looking_for_building")
			check_overlap(*msg);
	}
}
void path_side2d_cb(const nav_msgs::Path::ConstPtr& msg){
	if(msg->poses.size() > 0){
		ROS_INFO("PATH_SIDE_2d_SMALL_CB: %i poses ",msg->poses.size());
		check_path_proc(*msg);
	}
}
void get_side_path2d_cb(const nav_msgs::Path::ConstPtr& msg){
	if(msg->poses.size() > 0){
		ROS_INFO("PATH_SIDE_2d_BIG_CB: %i poses ",msg->poses.size());
		check_path_proc(*msg);
	}
}

void path_unknown_cb(const nav_msgs::Path::ConstPtr& msg){
	path_unknown = remove_path_visited(*msg,8);
}
void mb_result_cb(const std_msgs::Bool::ConstPtr& msg){
//	if(!msg->data)
//		get_next_target();
}
void next_target_end(){
	nav_msgs::Path path_targets_notvstd = remove_path_visited(path_targets,7);
	if(path_targets_notvstd.poses.size() > 0)
		set_target_pose(path_targets.poses[path_targets.poses.size()-1]);
	else
		set_target_scanpoint(target_pose.pose.position);
}
void next_target(){
	if(path_targets.poses.size() > path_targets_i+1){
		path_targets_i++;
		set_target_pose(path_targets.poses[path_targets_i]);
	}
	else
		set_target_scanpoint(target_pose.pose.position);
}
void check_target(){
	target_distance = get_dst2d(target_pose.pose.position,cmd_pos);
	if(target_distance < 10){
		bool hovering = false;
		if((ros::Time::now()-path_visited.header.stamp).toSec() > 5.0){
			hovering = true;
			ROS_INFO("CHECKING TARGET: %.0f %.0f %.0f -> %.0f %.0f %.0f [dist: %.0f]",pos.x,pos.y,pos.z,target_pose.pose.position.x,target_pose.pose.position.y,target_pose.pose.position.z,target_distance);
			if(target_distance > 5.0)
				set_target_pose(target_pose);
		}
		if(state_building == "approaching_building"){
			if(target_distance < 7.0){
				set_state_building("inspecting_building");
				building_v0 = path_visited.poses.size();
				next_target();
			}
		}
		else if(state_building == "inspecting_building"){
			next_target();
		}
		else if(state_building == "looking_for_building"){
			if(poly_obstacles.polygon.points.size() > 0){
				geometry_msgs::PoseStamped ps;
				ps.pose.position 		= get_ave_pnt_poly(poly_obstacles);
				ps.pose.orientation = tf::createQuaternionMsgFromYaw(get_hdng(ps.pose.position,pos));
				ps.header = hdr();
				ps.pose.position.x = (ps.pose.position.x + pos.x)/2.0;
				ps.pose.position.y = (ps.pose.position.y + pos.y)/2.0;
				ps.pose.position.z = (ps.pose.position.z + pos.z)/2.0;
				float dst = get_dst2d(ps.pose.position,pos);
				if(dst > 5.0)
					set_target_pose(ps);
				else
					get_target_from_path_unknown();
			}
			else
				get_target_from_path_unknown();
		}
	}
}
/*
if((ros::Time::now()-path_visited.header.stamp).toSec() > 5.0){
	if(state_building == "inspecting_building"){
		ROS_INFO("IDLING IN STATE inspecting_building");
		//set_target_pose(path_side.poses[path_side.poses.size()-1]);
	}
	else if(state_building == "approaching_building"){
		ROS_INFO("IDLING IN STATE approaching_building");
		//set_target_pose(path_side.poses[path_side.poses.size()-1]);
	}
	else{
		ROS_INFO("IDLING get_target_from_path_unknown");
		get_target_from_path_unknown();
	}
}*/

//else if(target_distance < 5.0)
//	get_target_from_path_unknown();
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
	if((get_dst3d(cmd_pos,last_pos) > 5.0) || (abs(get_shortest(cmd_pos_yaw,last_yaw)) > 1.0) || ((ros::Time::now()-last_sec).toSec() > 5.0)){
		last_pos = cmd_pos;
		last_yaw = cmd_pos_yaw;
		last_sec = ros::Time::now();
		set_target_scanpoint(target_pose.pose.position);
	}
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

void poly_cleared_cb(const geometry_msgs::PolygonStamped::ConstPtr& msg){
	poly_cleared = *msg;
}
void poly_obstacles_cb(const geometry_msgs::PolygonStamped::ConstPtr& msg){
	float cluster_spacing_cutoff = 10;
	float cluster_visited_cutoff = 7;
	poly_obstacles = remove_poly_path(path_visited,*msg,cluster_visited_cutoff);
	if(poly_obstacles.polygon.points.size() > 0){
		obstacles_polygons = reorganize_polys(find_clusters(poly_obstacles,cluster_spacing_cutoff));
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
float get_completion_percent(int v0,int vn,geometry_msgs::Point cen){
	int radians_segmentsize = 72;
	std::vector<bool> visited;
	float a_pr_i = rad2deg * (2*M_PI / radians_segmentsize);
	visited.resize(radians_segmentsize);
	int count = 0;
	for(int i = v0; i < vn; i++){
		int deg = int(round((rad2deg * (M_PI + get_hdng(path_visited.poses[i].pose.position,cen)) ) ));
		visited[deg/a_pr_i] = true;
	}
	for(int i = 0; i < radians_segmentsize; i++){
		if(visited[i])
			count++;
	}
	int percentage_completion = count / radians_segmentsize * 100;
	ROS_INFO("BLD: Completion: %i radians visited of %i segmentsize, percentage: %i",count,radians_segmentsize,percentage_completion);
	return percentage_completion;
}

void update_centroid(){
	int completion 					= get_completion_percent(building_v0,path_visited.poses.size(),building_centroid.point);
	building_centroid.point = get_ave_pnt_ni(path_visited,building_v0,path_visited.poses.size());
	ROS_INFO("BLD: Centroid Calculated %.2f %.2f %.2, completion: %i",building_centroid.point.x,building_centroid.point.y,building_centroid.point.z,completion);
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
	for(int i = 0; i < 20; i++){
		z_lvls.push_back(par_dz*i);
		mapimgs_at_zlvlz.push_back(img);
	}
	building_polygon.polygon.points.resize(72);
	building_ranges_at_rad.resize(72);
	target_pose.pose.orientation.w = 1.0;
	target_pose.header = hdr();
	path_visited.poses.push_back(target_pose);
	//ros::Subscriber s1 = nh.subscribe("/tb_world/path_visited",1,path_visited_cb);
	ros::Subscriber s0 = nh.subscribe("/tb_edto/path_side",1,path_side_cb);
	ros::Subscriber s2 = nh.subscribe("/tb_edto/path2d_side",1,get_side_path2d_cb);
	ros::Subscriber s3 = nh.subscribe("/tb_edto/path2d_small",10,path_side2d_cb);
	ros::Subscriber s4 = nh.subscribe("/tb_edto/poly_cleared",10,poly_cleared_cb);
	//	ros::Subscriber s5 = nh.subscribe("/tb_edto/poly_obstacles",10,poly_obstacles_cb);
	ros::Subscriber s5 = nh.subscribe("/tb_edto/poly_obstacles",10,poly_obstacles_cb);
	ros::Subscriber s1d = nh.subscribe("/tb_edto/poly_side",10,poly_side_obstacles_cb);
	ros::Subscriber s1s = nh.subscribe("/tb_edto/poly_down",10,poly_down_obstacles_cb);
	ros::Subscriber s1f = nh.subscribe("/tb_edto/poly_auto_side",10,poly_auto_side_obstacles_cb);

	ros::Publisher pub_centroid 	= nh.advertise<geometry_msgs::PointStamped>("/tb_test/centroid",100);
	pub_pnt 			 								= nh.advertise<geometry_msgs::PointStamped>("/tb_edto/get_side2d",100);
	pub_small_pnt 			 					= nh.advertise<geometry_msgs::PointStamped>("/tb_edto/get_side2d_small",100);
	pub_target_point		 					= nh.advertise<geometry_msgs::PointStamped>("/tb_edto/test_target",100);
	ros::Publisher pub_path_organize_1		= nh.advertise<nav_msgs::Path>("/tb_test/path_organize_1",100);
	ros::Publisher pub_path_organize_2		= nh.advertise<nav_msgs::Path>("/tb_test/path_organize_2",100);
	ros::Publisher pub_path_organize_raw	= nh.advertise<nav_msgs::Path>("/tb_test/path_organize_raw",100);
	pub_elevate	= nh.advertise<nav_msgs::Path>("/tb_abmap/get_elevation_pat",100);

	ros::Publisher building_polygon_pub       = nh.advertise<geometry_msgs::PolygonStamped>("/tb_bld/building_polygon",100);
	pub_setp_xy       = nh.advertise<geometry_msgs::PointStamped>("/tb_cmd/set_xy",100);
	////
	//

	ros::Subscriber s01 = nh.subscribe("/tb_world/path_unknown",10,path_unknown_cb);
	ros::Subscriber s02 = nh.subscribe("/tb_cmd/mb_result",10,mb_result_cb);

	pub_target 				= nh.advertise<geometry_msgs::PoseStamped>("/tb_cmd/mb_pose_target",100);
//	pub_target_path  	= nh.advertise<nav_msgs::Path>("/tb_cmd/path_targets",100);
	ros::Rate rate(5.0);
	cnt = 6;
	int ccnt = 0;
	while(ros::ok()){
		checktf1();
		checktf2();
		check_target();
		if(state_building == "inspecting_building"){
			update_centroid();
		}
		building_centroid.header = hdr();
		pub_centroid.publish(building_centroid);
		building_polygon_pub.publish(building_polygon);
		pub_path_organize_raw.publish(path_organize_raw);
		pub_path_organize_1.publish(path_organize_1);
		pub_path_organize_2.publish(path_organize_2);
		//if(path_targets.poses.size() > path_targets_i)
		//	pub_target_path.publish(path_targets);
		rate.sleep();
		ros::spinOnce();
		if(ccnt >= 5){
			ccnt = 0;
			ROS_INFO("building_state: %s",state_building.c_str());
			drawimg();
		}
		else
			ccnt++;
	}
	return 0;
}
