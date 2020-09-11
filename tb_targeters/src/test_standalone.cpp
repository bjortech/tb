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
ros::Publisher pub_pnt,pub_small_pnt;

cv::Mat img_blank_mono(1000,1000,CV_8U,cv::Scalar(0)); //create image, set encoding and size, init pixels to default val
cv::Mat img_mono(1000,1000,CV_8U,cv::Scalar(0)); //create image, set encoding and size, init pixels to default val
cv::Mat img_paths(1000,1000,CV_8U,cv::Scalar(0)); //create image, set encoding and size, init pixels to default val
cv::Mat img(1000,1000,CV_8UC3,cv::Scalar(0,0,0)); //create image, set encoding and size, init pixels to default val
cv::Mat img_perm(1000,1000,CV_8UC3,cv::Scalar(0,0,0)); //create image, set encoding and size, init pixels to default val
cv::Mat img_blank(1000,1000,CV_8UC3,cv::Scalar(0,0,0)); //create image, set encoding and size, init pixels to default val
std::vector<cv::Mat> mapimgs_at_zlvlz;
geometry_msgs::Point last_pos;
std::vector<float> z_lvls;
nav_msgs::Path path_visited;
std::string state_building = "looking_for_building";
std::string state_building_scan = "initial_vertical";
ros::Time state_building_change,state_building_scan_change;
geometry_msgs::PointStamped current_midpoint,building_centroid;
int par_scouting_zn_start,par_scouting_zn_end;
double par_maprad,par_dz;
int cnt = 0;
int zn = 3;
int cnt2 = 0;
int cnt3 = 0;
nav_msgs::Path last_path,gridpath;
std::vector<nav_msgs::Path> paths_vec;
nav_msgs::Path path_best;
geometry_msgs::PolygonStamped building_polygon;
std::vector<float>building_ranges_at_rad;
int last_i = 0;
ros::Time last_rosinfo;
float last_pos_yaw = 0;
float last_pose_yaw = 0;
geometry_msgs::PoseStamped last_pose;
std::vector<geometry_msgs::PolygonStamped> buildings_polygons;
double par_dst_target;
int zlvl_max;
void set_state_building_scan(std::string newstate){
  if(newstate != state_building_scan){
    float dt = (ros::Time::now() - state_building_scan_change).toSec();
    ROS_INFO("state_building_scan: state_building_scan: %s -> %s (%.3f seconds in state)",state_building_scan.c_str(),newstate.c_str(),dt);
    state_building_scan_change  = ros::Time::now();
    state_building_scan  = newstate;
  }
}
void set_state_building(std::string newstate){
  if(newstate != state_building){
    float dt = (ros::Time::now() - state_building_change).toSec();
    ROS_INFO("state_building: state_building: %s -> %s (%.3f seconds in state)",state_building.c_str(),newstate.c_str(),dt);
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
std::vector<float> path2bbvec(nav_msgs::Path pathin){
	std::vector<float> bbvec;
	bbvec.resize(6);
	bbvec[0] = bbvec[1] = bbvec[2] = 1000;
	bbvec[3] = bbvec[4] = bbvec[5] = -1000;
	for(int i = 0; i < pathin.poses.size(); i++){
		if(pathin.poses[i].pose.position.x < bbvec[0])
			bbvec[0] = pathin.poses[i].pose.position.x;
		if(pathin.poses[i].pose.position.y < bbvec[1])
			bbvec[1] = pathin.poses[i].pose.position.y;
		if(pathin.poses[i].pose.position.z < bbvec[2])
			bbvec[2] = pathin.poses[i].pose.position.z;
		if(pathin.poses[i].pose.position.x > bbvec[3])
			bbvec[3] = pathin.poses[i].pose.position.x;
		if(pathin.poses[i].pose.position.y > bbvec[4])
			bbvec[4] = pathin.poses[i].pose.position.y;
		if(pathin.poses[i].pose.position.z > bbvec[5])
			bbvec[5] = pathin.poses[i].pose.position.z;
	}
	return bbvec;
}
std::vector<std::vector<float>> get_bbvecs_paths(std::vector<nav_msgs::Path> pathsin){
	std::vector<std::vector<float>> bbvecs_pathsin;
	bbvecs_pathsin.resize(pathsin.size());
	for(int i = 0; i < pathsin.size(); i++){
		bbvecs_pathsin.push_back(path2bbvec(pathsin[i]));
	}
	return bbvecs_pathsin;
}
/*
std::vector<std::vector<float>> paths_bbvec1 = get_bbvecs_paths(pathsin1);
std::vector<std::vector<float>> paths_bbvec2 = get_bbvecs_paths(pathsin2);
for(int i = 0; i < paths_bbvec1.size(); i++){
	for(int k = 0; k < paths_bbvec2.size(); k++){
		if()
	}
}
*/
void draw_and_boxin_path(nav_msgs::Path pathin){
	if(pathin.poses.size() == 0)
		return;
	std::vector<geometry_msgs::Point> bbmnmx = getinpath_boundingbox(pathin);
	geometry_msgs::Point bbmin_scan = bbmnmx[0];
	geometry_msgs::Point bbmax_scan = bbmnmx[1];
	float rel_score = 255*(pathin.poses[0].pose.position.z) / 40.0;
	cv::Scalar color = cv::Scalar(rel_score,rel_score,0);
	draw_path(pathin,color,0);
	cv::rectangle(img, pnt2cv(bbmin_scan),pnt2cv(bbmax_scan),color,1,8,0);
	putText(img,"sp:"+std::to_string(int(pathin.poses[0].pose.position.z)), pnt2cv(pathin.poses[0].pose.position),
			cv::FONT_HERSHEY_COMPLEX_SMALL, 0.8, color, 1, CV_AA);
}

void drawimg(){
//  draw_path(path_visited,get_color(100,50,50),1);
  cv::Scalar color_pos = get_color(200,200,200);
  //draw_line(current_midpoint.point,constrainAngle(current_heading + M_PI/3),50,color_pos);
  //draw_line(current_midpoint.point,constrainAngle(current_heading - M_PI/3),50,color_pos);
	ROS_INFO("drawimg: paths_vec: %i",paths_vec.size());
	draw_clusters_at_height(paths_vec);
	paths_vec.resize(0);
  cv::Mat img_new2;
  putText(img,"mid:"+std::to_string(int(current_midpoint.point.z)), pnt2cv(current_midpoint.point),
      cv::FONT_HERSHEY_COMPLEX_SMALL, 0.8, color_pos, 1, CV_AA);

  cv::resize(img, img_new2, cv::Size(), 2.0, 2.0);
  cnt3++;
	cv::imwrite("/home/nuc/brain/fullpath/"+std::to_string(cnt3)+"navigator.png",img_new2);
	cv::imwrite("/home/nuc/brain/fullpath/"+std::to_string(cnt3)+"img_perm.png",img_perm);
	ROS_INFO("Drawimg complete");
	img_blank.copyTo(img);
}

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
		ROS_INFO("int iz: %i r: %i c: %i, ro: %i co: %i, yaw: %.2f z: %.0f",iz,r,c,ro,co,yaw_pose,pathin.poses[i].pose.position.z);
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
		ROS_INFO("Path rema: %i %i",path_remaining.poses.size(),size_last);
		path_remaining = path_remaining_new;
		path_cluster.header.frame_id = "map";
		pathsout.push_back(path_cluster);
		path_cluster.poses.resize(0);
	}
	float dt = (ros::Time::now()-t0).toSec();
	cv::imwrite("/home/nuc/brain/cluster/"+std::to_string(cnt)+"get_paths_clusters.png",img_paths);
	ROS_INFO("PATHPROCESS: %i poses segmented into %i clusters in %.4f sec",pathin.poses.size(),pathsout.size(),dt);
	return pathsout;
}
void draw_pathclusters(int color_channel){
	int c1 = x2c(current_midpoint.point.x-par_maprad*2);
	int c0 = x2c(current_midpoint.point.x+par_maprad*2);
	int r1 = y2r(current_midpoint.point.y-par_maprad*2);
	int r0 = y2r(current_midpoint.point.y+par_maprad*2);
	int pnts = 0; int pnts_tot = 0;
	for(int c = fmin(c1,c0); c < fmax(c1,c0); c++){
		for(int r = fmin(r1,r0); r < fmax(r1,r0); r++){
			if(img_paths.at<uchar>(r,c) > 0)
				img.at<cv::Vec3b>(r,c)[color_channel] = img_paths.at<uchar>(r,c);
		}
	}
}
int get_overlap(nav_msgs::Path pathin1,nav_msgs::Path pathin2){
	nav_msgs::Path path_overlap;
	path_overlap.header = hdr();
	for(int i = 0; i < pathin1.poses.size(); i++){
		for(int k = 0; k < pathin2.poses.size(); k++){
			if(get_dst2d(pathin1.poses[i].pose.position,pathin2.poses[k].pose.position) < 2.0)
				path_overlap.poses.push_back(pathin1.poses[i]);
		}
	}
	int percent = int(round((100.0 * float(path_overlap.poses.size()) / float(pathin1.poses.size()))));
	ROS_INFO("Path overlap[%i]: %i p1: %i p2: %i ",percent,path_overlap.poses.size(),pathin1.poses.size(),pathin2.poses.size());
	return percent;
}
int get_overlap_orig(){
	img_blank.copyTo(img);
	int c1 = x2c(current_midpoint.point.x-par_maprad*2);
	int c0 = x2c(current_midpoint.point.x+par_maprad*2);
	int r1 = y2r(current_midpoint.point.y-par_maprad*2);
	int r0 = y2r(current_midpoint.point.y+par_maprad*2);
	int pnts = 0; int pnts_tot = 0; int pnts_overlap = 0;
	for(int c = fmin(c1,c0); c < fmax(c1,c0); c++){
		for(int r = fmin(r1,r0); r < fmax(r1,r0); r++){
			pnts_tot++;
			if(img.at<cv::Vec3b>(r,c)[0] > 0 && img.at<cv::Vec3b>(r,c)[1] > 0)
				pnts_overlap++;
		//	else
		//		img.at<cv::Vec3b>(r,c)[2] = 200;
		}
	}
	int overlap = 100 * pnts / (pnts_tot+1);
	ROS_INFO("OVERLAP: %i, pnts: %i pnts_tot: %i",overlap,pnts,pnts_tot);
	cv::imwrite("/home/nuc/brain/cluster/"+std::to_string(cnt)+"get_overlap.png",img);
	return overlap;
}

void get_next_search_grid(){
	cnt++;
	if(cnt < gridpath.poses.size()){
		zn = par_scouting_zn_start;
		current_midpoint.point.x = gridpath.poses[cnt + gridpath.poses.size()/2].pose.position.x;
		current_midpoint.point.y = gridpath.poses[cnt + gridpath.poses.size()/2].pose.position.y;
		current_midpoint.point.z = z_lvls[zn];
	}
	ROS_INFO("get next search grid %i->%i X: %.0f Y: %.0f",cnt-1,cnt,current_midpoint.point.x,current_midpoint.point.y);
}

void elevate_midpoint(){
	zn++;
	current_midpoint.point.z = z_lvls[zn];
	ROS_INFO("elevate midpoint: zn %i->%i Z: %.0f -> %.0f",zn-1,zn,z_lvls[zn-1],z_lvls[zn]);
	pub_pnt.publish(current_midpoint);
}
int check_overlap(nav_msgs::Path pathin1,nav_msgs::Path pathin2){
	ROS_INFO("check_overlap");
std::vector<nav_msgs::Path> paths1 = get_clusters(pathin1);
	for(int i = 0; i < paths1.size(); i++){
		paths_vec.push_back(paths1[i]);
	}
//	draw_pathclusters(0);
	std::vector<nav_msgs::Path> paths2 = get_clusters(pathin2);
	int best_overlap = 0;
	int best_overlap_i = 0;
	int best_overlap_k = 0;
	for(int i = 0; i < paths1.size(); i++){
		for(int k = 0; k < paths2.size(); k++){
			int overlap = get_overlap(paths1[i],paths2[k]);
			if(overlap > best_overlap){
				best_overlap_i = i;
				best_overlap_k = k;
				best_overlap   = overlap;
				path_best			 = paths2[k];
				ROS_INFO("Path overlap[%i]: p1: %i p2: %i ",overlap,paths1[i].poses.size(),paths2[k].poses.size());
			}
		}
	}
	if(paths1[best_overlap_i].poses.size() > 0 && paths2[best_overlap_k].poses.size() > 0){
		float rel_score = 255*(paths1[best_overlap_i].poses[0].pose.position.z) / 40.0;
		float rel_score2 = 255*(paths2[best_overlap_k].poses[0].pose.position.z) / 40.0;
		draw_path(paths1[best_overlap_i],cv::Scalar(int(rel_score),0,0),1);
		draw_path(paths2[best_overlap_k],cv::Scalar(0,int(rel_score2),0),1);
	}
	//draw_clusters_at_height(paths2);
	//draw_pathclusters(1);
	return best_overlap;
}

int getinpath_closestindex2d(nav_msgs::Path pathin,geometry_msgs::Point pnt){
	ROS_INFO("getinpath_closestindex2d");
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
int get_completion_percent(){
	int cc = 0;
	for(int i = 0; i < building_polygon.polygon.points.size(); i++){
		if(building_polygon.polygon.points[i].z + building_polygon.polygon.points[i].x != 0)
			cc++;
	}
	int percentage_completion = int(round(float(cc) / float(building_polygon.polygon.points.size()) * 100.0));
	ROS_INFO("BLD: Completion: %i percentage: %i",cc,percentage_completion);
	return percentage_completion;
}
geometry_msgs::PointStamped get_centroid(){
	geometry_msgs::PointStamped p;
	p.header = hdr();
	int visited_count = 0;
	for(int i = 0; i < building_polygon.polygon.points.size(); i++){
		if(building_polygon.polygon.points[i].x != 0){
			p.point.x += building_polygon.polygon.points[i].x;
			p.point.y += building_polygon.polygon.points[i].y;
			p.point.z += building_polygon.polygon.points[i].z;
			visited_count++;
		}
	}
	p.point.x /= visited_count;
	p.point.y /= visited_count;
	p.point.z /= visited_count;
	ROS_INFO("BLD: Centroid Calculated: %i points -> %.2f %.2f %.2f",visited_count,p.point.x,p.point.y,p.point.z);
	return p;
}
void update_building_hori(nav_msgs::Path pathin){
	ROS_INFO("update_building_hori");
	float rads_pr_i = 2*M_PI / 72;
	for(int k = 0; k < pathin.poses.size(); k++){
		float rel_hdng  = get_hdng(pathin.poses[k].pose.position,building_centroid.point);
		float pose_hdng = tf::getYaw(pathin.poses[k].pose.orientation);
		float pose_dist = get_dst2d(pathin.poses[k].pose.position,building_centroid.point);
		int 	i   	    = int(round((rel_hdng + M_PI) / rads_pr_i));
		if(building_ranges_at_rad[i] == 0){
			building_ranges_at_rad[i] = pose_dist;
			building_polygon.polygon.points[i].x = pathin.poses[k].pose.position.x;
			building_polygon.polygon.points[i].y = pathin.poses[k].pose.position.y;
			building_polygon.polygon.points[i].z = pathin.poses[k].pose.position.z;
		}
	}
	int percent = get_completion_percent();
	if(percent > 90){
		set_state_building("looking_for_building");
		buildings_polygons.push_back(building_polygon);
	}
//	if(percent > 30)
	//	building_centroid = get_centroid();
}
bool is_point_visited(geometry_msgs::Point pin,float lim){

  float res,dst;
  if(path_visited.poses.size() == 0)
    return false;
  for(int i = 0; i < path_visited.poses.size(); i++){
     if(get_dst2d(path_visited.poses[i].pose.position,pin) < lim)
        return true;
  }
  return false;
}
nav_msgs::Path remove_visited(nav_msgs::Path pathin,float cutoff_dst){
	ROS_INFO("remove_visited");

	nav_msgs::Path pathout;
	pathout.header = hdr();
	for(int i = 0; i < pathin.poses.size(); i++){
		if(!is_point_visited(pathin.poses[i].pose.position,cutoff_dst)){
			pathout.poses.push_back(pathin.poses[i]);
		}
	}
	return pathout;
}
double constrainAngle(double x){
  if(x > M_PI)
    return (x - M_PI*2);
  else if(x < -M_PI)
    return (x + M_PI*2);
  else
    return x;
}
void set_last_pos(geometry_msgs::PoseStamped pose){
	last_pose = pose;
	geometry_msgs::Point pnt = pose.pose.position;
	last_pose_yaw = tf::getYaw(pose.pose.orientation);
	last_pos_yaw 	= get_hdng(pnt,last_pos);
	cv::line (img_perm, pnt2cv(last_pos), pnt2cv(pnt),cv::Scalar(0,0,200),1,cv::LINE_8,0);
	last_pos = pnt;
	current_midpoint.point = pnt;
	geometry_msgs::PoseStamped ps;
	ps.pose.position = pnt;
	ps.header = hdr();
	ps.pose.orientation.w = 1.0;
	float dx = pnt.x-last_pos.x;
	float dy = pnt.y-last_pos.y;
	float dz = pnt.z-last_pos.z;
	float dst = get_dst3d(pnt,last_pos);
	float dx_norm = dx / dst;
	float dy_norm = dy / dst;
	float dz_norm = dz / dst;

	 cv::Scalar color_pos = get_color(200,200,200);
   geometry_msgs::Point pyaw0,pyaw1;
   pyaw0.x = last_pos.x - 50 * cos(last_pos_yaw - M_PI/3);
   pyaw0.y = last_pos.y - 50 * sin(last_pos_yaw - M_PI/3);
	 pyaw1.x = last_pos.x + 50 * cos(last_pos_yaw + M_PI/3);
	 pyaw1.y = last_pos.y + 50 * sin(last_pos_yaw + M_PI/3);
	 cv::line (img, pnt2cv(last_pos), pnt2cv(pyaw0),color_pos,1,cv::LINE_8,0);
	 cv::line (img, pnt2cv(last_pos), pnt2cv(pyaw1),color_pos,1,cv::LINE_8,0);
	ROS_INFO("drawimg: paths_vec: %i",paths_vec.size());
	draw_clusters_at_height(paths_vec);
	paths_vec.resize(0);
  cv::Mat img_new2;
  putText(img,"mid:"+std::to_string(int(current_midpoint.point.z)), pnt2cv(current_midpoint.point),
      cv::FONT_HERSHEY_COMPLEX_SMALL, 0.8, color_pos, 1, CV_AA);
  cnt3++;
	cv::imwrite("/home/nuc/brain/fullpath/"+std::to_string(cnt3)+"img_perm.png",img_perm);
	ROS_INFO("Drawimg complete");
for(int i = 0; i < dst; i++){
	ps.pose.position.x = last_pos.x + dx_norm * i;
	ps.pose.position.y = last_pos.y + dy_norm * i;
	ps.pose.position.z = last_pos.z + dz_norm * i;
	path_visited.poses.push_back(ps);
	ROS_INFO("PNT: %.0f %.0f %.0f dst: %i",ps.pose.position.x,ps.pose.position.y,ps.pose.position.z,i);
}
ROS_INFO("DEST: %.0f %.0f %.0f",ps.pose.position.x,ps.pose.position.y,ps.pose.position.z);
ROS_INFO("DEST: %.0f %.0f %.0f ",last_pos.x,last_pos.y,last_pos.z);
path_visited.poses.push_back(ps);
current_midpoint.point = last_pos;
pub_small_pnt.publish(current_midpoint);
}
void elevate_midpoint_point(float dz){
	ROS_INFO("elevate_midpoint_point");
	geometry_msgs::Point pnt = last_pos;
	last_pose.pose.position.z += dz;
	set_last_pos(last_pose);
}
nav_msgs::Path remove_reverse(nav_msgs::Path pathin,float max_delta_hdng){
	nav_msgs::Path pathout;
	pathout.header = hdr();
	for(int i = 0; i < pathin.poses.size(); i++){
		float hdng = get_hdng(pathin.poses[i].pose.position,last_pos);
		float dhdng = get_shortest(hdng,last_pos_yaw);
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
	ROS_INFO("request_next_hori");

	nav_msgs::Path pathout;
	pathout.header = hdr();
	colorize_pathimg(pathin);
	floodfill(y2r(last_pos.y),x2c(last_pos.x),200);
	for(int i = 0; i < pathin.poses.size(); i++){
		if(img_paths.at<uchar>(y2r(pathin.poses[i].pose.position.y),x2c(pathin.poses[i].pose.position.x)) == 200)
			pathout.poses.push_back(pathin.poses[i]);
	}
	ROS_INFO("pathout: %i",pathout.poses.size());
	pathout = remove_visited(pathout,5);
	pathout = remove_reverse(pathout,M_PI/2);
	pathout = remove_reverse2(pathout,M_PI/2);
	path_best = pathout;
	int best_i = 0;
	float best_dst = 0;
	float target_dst = 7.5;
	for(int i = 0; i < pathout.poses.size(); i++){
		float dst = get_dst2d(pathout.poses[i].pose.position,last_pos);
		float ddst = abs(dst-target_dst);
		if(ddst < best_dst){
			best_dst = ddst;
			best_i = i;
		}
	}
	ROS_INFO("NextPos: %.0f %.0f %.0f Request next hori: %i pathin %i pathout  best_i %i",last_pos.x,last_pos.y,last_pos.z,pathin.poses.size(),pathout.poses.size(), pathout);
	if(best_i < pathout.poses.size() && best_i >= 0)
		set_last_pos(pathout.poses[best_i]);
	else
		elevate_midpoint_point(3.0);
}

void init_building(){
	ROS_INFO("Initialising building scan");
	int ci = getinpath_closestindex2d(path_best,last_pos);
	float building_direction = tf::getYaw(path_best.poses[ci].pose.orientation);
	building_centroid.point.x = path_best.poses[ci].pose.position.x + 10 * cos(building_direction);
	building_centroid.point.y = path_best.poses[ci].pose.position.y + 10 * sin(building_direction);
	building_centroid.point.z = path_best.poses[ci].pose.position.z;
	building_ranges_at_rad.resize(0);
	building_ranges_at_rad.resize(building_polygon.polygon.points.size());
	update_building_hori(path_best);
	set_last_pos(path_best.poses[ci]);
}
void next_scoutpoint(){
	ROS_INFO("next_scoutpoint");
	if(state_building == "looking_for_building"){
		ROS_INFO("next scoutpoint: zn: %i, z: %.0f",zn,z_lvls[zn]);
		if(zn < par_scouting_zn_end){
			elevate_midpoint();
		}
		else{
			get_next_search_grid();
			drawimg();
		}
		ROS_INFO("next scoutpoint: zn: %i, z: %.0f",zn,z_lvls[zn]);
		current_midpoint.header = hdr();
		pub_pnt.publish(current_midpoint);
	}
}
void path_visited_cb(const nav_msgs::Path::ConstPtr& msg){
	path_visited = *msg;
}


void update_scanning_building(nav_msgs::Path pathin){
	ROS_INFO("update_scanning_building");
		if(state_building_scan == "initial_vertical"){
			if(pathin.poses.size() == 0){
				set_state_building_scan("initial_horisontal");
				elevate_midpoint_point(-3.0);
			}
		else if(check_overlap(last_path,pathin) > 40)
				elevate_midpoint_point(3.0);
		else{
			set_state_building_scan("initial_horisontal");
			elevate_midpoint_point(-3.0);
		}
	}
	else if(state_building_scan == "initial_horisontal"){
		update_building_hori(pathin);
		request_next_hori(pathin);
	}
	else{
		ROS_INFO("empty");
	}

}
void update_looking_for_building(nav_msgs::Path pathin){
	ROS_INFO("update_looking_for_building: new: %i / last: %i",pathin.poses.size(),last_path.poses.size());
	if(last_path.poses.size() > 0 && pathin.poses.size() > 0){
		int overlap = check_overlap(last_path,pathin);
		last_path = pathin;
		if(overlap > 60){
			set_state_building("scanning_building");
			set_state_building_scan("initial_vertical");
			init_building();
		}
	}
	last_path = pathin;
	next_scoutpoint();
}

void update_permanent(nav_msgs::Path pathin){
	ROS_INFO("update_permanent");
	for(int i = 0; i < pathin.poses.size(); i++){
		int r = y2r(pathin.poses[i].pose.position.y);
		int c = x2c(pathin.poses[i].pose.position.x);
		int z = int(pathin.poses[i].pose.position.z);
		if(img_perm.at<cv::Vec3b>(r,c)[0] > z || img_perm.at<cv::Vec3b>(r,c)[0] == 0)
			img_perm.at<cv::Vec3b>(r,c)[0] = z;
		if(img_perm.at<cv::Vec3b>(r,c)[1] < z)
			img_perm.at<cv::Vec3b>(r,c)[1] = z;
		img_perm.at<cv::Vec3b>(r,c)[2] = img_perm.at<cv::Vec3b>(r,c)[2] + 1;
	}
}
bool in_poly(geometry_msgs::PolygonStamped polyin, float x, float y){
  int cross = 0;
	geometry_msgs::Point point;
	point.x = x;
	point.y = y;
  for(int i = 0,j = polyin.polygon.points.size()-1; i < polyin.polygon.points.size(); j=i++){//  for (int i = 0, j = vertices_.size() - 1; i < vertices_.size(); j = i++) {
    if ( ((polyin.polygon.points[i].y > point.y) != (polyin.polygon.points[j].y > point.y))
           && (point.x < (polyin.polygon.points[j].x - polyin.polygon.points[i].x) * (point.y - polyin.polygon.points[i].y) /
            (polyin.polygon.points[j].y - polyin.polygon.points[i].y) + polyin.polygon.points[i].x) )
    {
      cross++;
    }
  }
  return bool(cross % 2);
}
nav_msgs::Path constrain_path_notin_bbpoly(nav_msgs::Path pathin,geometry_msgs::PolygonStamped poly_bb){
  nav_msgs::Path pathout;
  pathout.header = hdr();
  if(pathin.poses.size() == 0 || poly_bb.polygon.points.size() == 0)
    return pathin;
  for(int i = 0; i < pathin.poses.size(); i++){
    if(!in_poly(poly_bb,pathin.poses[i].pose.position.x,pathin.poses[i].pose.position.y)){
      pathout.poses.push_back(pathin.poses[i]);
    }
  }
  return pathout;
}
void get_side_path2d_cb(const nav_msgs::Path::ConstPtr& msg){
	ROS_INFO("get_side_path2d_cb");
	update_permanent(*msg);
	nav_msgs::Path pathin = *msg;
	if(buildings_polygons.size() > 0)
		pathin = constrain_path_notin_bbpoly(*msg,buildings_polygons[0]);

	if(state_building == "looking_for_building")
		update_looking_for_building(pathin);
	else if(state_building == "scanning_building")
		update_scanning_building(pathin);
	else{
		ROS_INFO("UNKNOWN STATE (%s)",state_building.c_str());
	}
}
nav_msgs::Path create_gridpath(float area_sidelength,float radlen_xy){
	ROS_INFO("Create gridpath: %.0f %.0f",area_sidelength,radlen_xy);
  float centroid_sides = 2*radlen_xy;
  int num_grids = area_sidelength / centroid_sides;
  nav_msgs::Path pathout;
  pathout.header = hdr();
  int i = 0;
  for (int y = 0; y < num_grids; y++)
  {
    for (int x = 0; x < num_grids; x++)
    {
		//	for(int zn = 0; zn < 10; zn++){
				geometry_msgs::PoseStamped ps;
				ps.pose.position.x = float(area_sidelength*-0.5 + x * centroid_sides+centroid_sides*0.5);
				ps.pose.position.y = float(area_sidelength*-0.5 + y * centroid_sides+centroid_sides*0.5);
		//		ps.pose.position.z = zn * 3;
				ps.pose.orientation.w = 1.0;
				ps.header = hdr();
				pathout.poses.push_back(ps);
		//	}
    }
  }
  return pathout;
}
void draw_imgs(){
	for(int i = 0; i < mapimgs_at_zlvlz.size(); i++){
		ROS_INFO("writing %i",i);
		cv::imwrite("/home/nuc/brain/scanproc/"+std::to_string(cnt3)+"_lvl_"+std::to_string(i)+"_.png",mapimgs_at_zlvlz[i]);
	}
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "tb_edto_side_auto_node");
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

	gridpath = create_gridpath(200,10);
	ros::Subscriber s1 = nh.subscribe("/tb_world/path_visited",1,path_visited_cb);
	ros::Subscriber s2  = nh.subscribe("/tb_edto/path2d_side",1,get_side_path2d_cb);
	ros::Publisher pub_centroid = nh.advertise<geometry_msgs::PointStamped>("/tb_test/centroid",100);
	pub_pnt 			 							= nh.advertise<geometry_msgs::PointStamped>("/tb_edto/get_side2d",100);
	pub_small_pnt 			 				= nh.advertise<geometry_msgs::PointStamped>("/tb_edto/get_side2d_small",100);
	ros::Publisher pub_path 		= nh.advertise<nav_msgs::Path>("/tb_edto/path",100);
	ros::Publisher pub_path_vis 		= nh.advertise<nav_msgs::Path>("/tb_edto/path_visited",100);
	ros::Publisher pub_path_best		= nh.advertise<nav_msgs::Path>("/tb_edto/path_best",100);

	ros::Publisher building_polygon_pub       = nh.advertise<geometry_msgs::PolygonStamped>("/tb_bld/building_polygon",100);

	ros::Rate rate(1.0);
	cnt = 6;
	int ccnt = 0;
	while(ros::ok()){
		ccnt++;
		if(ccnt == 10){
			ccnt = 0;
			draw_imgs();
		}
		ROS_INFO("Cnt %i",cnt);
		building_centroid.header = hdr();
		gridpath.header = hdr();
		path_best.header = hdr();
		building_polygon.header = hdr();
		pub_centroid.publish(building_centroid);
		pub_path.publish(gridpath);
		building_polygon_pub.publish(building_polygon);
		pub_path_best.publish(path_best);
		pub_path_vis.publish(path_visited);
		rate.sleep();
		ros::spinOnce();
		drawimg();

	}
	return 0;
}
