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
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <image_transport/image_transport.h>
#include <dynamicEDT3D/dynamicEDTOctomap.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/OccupancyGrid.h>
#include <map_msgs/OccupancyGridUpdate.h>
#include <chrono>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/UInt8.h>
#include <visualization_msgs/MarkerArray.h>
#include <eigen3/Eigen/Core>
#include <geometry_msgs/Vector3Stamped.h>
#include <cv_bridge/cv_bridge.h>


using namespace octomap;
using namespace std;

tf2_ros::Buffer tfBuffer;
double par_zjump,par_maprad,par_vlpmintilt,par_vlptiltinterval,par_vlpmaxtilt,par_takeoffaltitude;
int vlpstate,zlvl,buildingstate,mainstate,missionstate;
geometry_msgs::PolygonStamped poly_vlp,poly_heading,poly_heading_rel,poly_safe;
nav_msgs::Odometry odom;
geometry_msgs::Point pos;
geometry_msgs::PointStamped target_obstacle;
geometry_msgs::PoseStamped target,target_last;
float pos_yaw,target_yaw;
nav_msgs::Path path_targets,path_candidates,path_visited,path_floor,path_test,path_candidates_sides;
sensor_msgs::LaserScan scan_cleared,scan_roofs,scan_dangers,scan_buildings;
std::vector<int> z_lvls;
std::vector<geometry_msgs::PoseStamped> targetcmds_sent;
float z,xy;
geometry_msgs::PointStamped xy_obs,z_obs;
geometry_msgs::Vector3Stamped armstate;
geometry_msgs::Vector3 vlp_rpy;

bool got_path_down,got_path_sides,requested_paths,evaluated_paths,tilting,tiltlimit_reached,looking_for_paths,got_candidates,got_candidates_sides;

std_msgs::String current_activity;
ros::Publisher pub_get_next_path,pub_cmd,pub_tiltvlp,pub_tilt,pub_pan,pub_armstate,pub_altlvl,pub_altcmd,pub_activity,pub_targetalt;
std_msgs::Float64 arm2_tilt_msg,arm2_pan_msg,arm1_tilt_msg,target_alt_msg;
std_msgs::UInt8 altlvl_msg;
float target_dxy,target_dz,min_time;
int current_changefactor = 1;
ros::Time last_time,vlpstate_change,tilt_start,tilt_stop;
int path_targets_i = 0;
ros::Time last_update;

geometry_msgs::Point pnt_midpoint;
std_msgs::Header hdr(){
  std_msgs::Header header;
  header.frame_id = "map";
  header.stamp    = ros::Time::now();
  return header;
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

bool in_poly(geometry_msgs::PolygonStamped polyin, float x, float y){
	ros::Time t0 = ros::Time::now();
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
//	float dt = (ros::Time::now() - t0).toSec();
//	ROS_INFO("sr: %.5f",dt);
  return bool(cross % 2);
}
nav_msgs::Path constrain_path_bbpoly(nav_msgs::Path pathin,geometry_msgs::PolygonStamped poly_bb){
  nav_msgs::Path pathout;
  pathout.header = hdr();
  if(pathin.poses.size() == 0 || poly_bb.polygon.points.size() == 0)
    return pathin;
  for(int i = 0; i < pathin.poses.size(); i++){
    if(in_poly(poly_bb,pathin.poses[i].pose.position.x,pathin.poses[i].pose.position.y)){
      pathout.poses.push_back(pathin.poses[i]);
    }
  }
  return pathout;
}

int get_zn(float z){
	for(int i = 0; i < z_lvls.size()-1; i++){
		if(z_lvls[i] <= z && z_lvls[i+1] >= z){
			return i;
		}
	}
	return 0;
}
float get_dst2d(geometry_msgs::Point p1, geometry_msgs::Point p2){
  return(sqrt(pow(p1.x-p2.x,2)+pow(p1.y-p2.y,2)));
}
float get_dst3d(geometry_msgs::Point p1, geometry_msgs::Point p2){
  return(sqrt(pow(p1.x-p2.x,2)+pow(p1.y-p2.y,2)+pow(p1.z-p2.z,2)));
}
float get_slope(geometry_msgs::Point p1, geometry_msgs::Point p2){
  return (p2.z - p1.z) / get_dst2d(p1,p2);
}
float get_inclination(geometry_msgs::Point p1, geometry_msgs::Point p2){
  return atan2(p2.z - p1.z,get_dst2d(p1,p2));
}
float get_inclination_offset(geometry_msgs::Point pnt,float inclination_target){
  return (pnt.z - inclination_target*get_dst2d(pos,pnt) + pos.z);
}

bool sort_dst_pair(const std::tuple<int,float>& a,
               const std::tuple<int,float>& b)
{
    return (std::get<1>(a) < std::get<1>(b));
}
nav_msgs::Path sort_path_by_inclinationoffsetabs(nav_msgs::Path pathin,float inclination_target){
  nav_msgs::Path pathout;
  if(pathin.poses.size() <= 1)
    return pathin;
  pathout.header = pathin.header;
  std::vector<std::tuple<int,float>>i_dst;
  for(int i = 0; i < pathin.poses.size(); i++){
    i_dst.push_back(std::make_tuple(i,abs(get_inclination_offset(pathin.poses[i].pose.position,inclination_target))));
  }
  sort(i_dst.begin(),i_dst.end(),sort_dst_pair);
  for(int i = 0; i < i_dst.size(); i++){
    pathout.poses.push_back(pathin.poses[std::get<0>(i_dst[i])]);
  }
	return pathout;
}
nav_msgs::Path sort_path_by_inclinationoffset(nav_msgs::Path pathin,float inclination_target){
  nav_msgs::Path pathout;
  if(pathin.poses.size() <= 1)
    return pathin;
  pathout.header = pathin.header;
  std::vector<std::tuple<int,float>>i_dst;
  for(int i = 0; i < pathin.poses.size(); i++){
    float inclination = get_inclination(pathin.poses[i].pose.position,pos);
    i_dst.push_back(std::make_tuple(i,inclination_target - inclination));
  }
  sort(i_dst.begin(),i_dst.end(),sort_dst_pair);
  for(int i = 0; i < i_dst.size(); i++){
    pathout.poses.push_back(pathin.poses[std::get<0>(i_dst[i])]);
  }
	return pathout;
}
nav_msgs::Path sort_path_by_dst(nav_msgs::Path pathin,bool use_3d){
  nav_msgs::Path pathout;
  if(pathin.poses.size() <= 1)
    return pathin;
  pathout.header = pathin.header;
  std::vector<std::tuple<int,float>>i_dst;
  for(int i = 0; i < pathin.poses.size(); i++){
    if(use_3d)
      i_dst.push_back(std::make_tuple(i,get_dst3d(pos,pathin.poses[i].pose.position)));
    else
      i_dst.push_back(std::make_tuple(i,get_dst2d(pos,pathin.poses[i].pose.position)));
  }
  sort(i_dst.begin(),i_dst.end(),sort_dst_pair);
  for(int i = 0; i < i_dst.size(); i++){
    pathout.poses.push_back(pathin.poses[std::get<0>(i_dst[i])]);
  }
	return pathout;
}
nav_msgs::Path sort_path_by_zabs(nav_msgs::Path pathin){
  nav_msgs::Path pathout;
  if(pathin.poses.size() <= 1)
    return pathin;
  pathout.header = pathin.header;
  std::vector<std::tuple<int,float>>i_dst;
  for(int i = 0; i < pathin.poses.size(); i++){
      i_dst.push_back(std::make_tuple(i,abs(pathin.poses[i].pose.position.z - pos.z)));
  }
  sort(i_dst.begin(),i_dst.end(),sort_dst_pair);
  for(int i = 0; i < i_dst.size(); i++){
    pathout.poses.push_back(pathin.poses[std::get<0>(i_dst[i])]);
  }
	return pathout;
}
nav_msgs::Path sort_path_by_hdng(nav_msgs::Path pathin){
  float rad2deg = 180.0/M_PI;
  nav_msgs::Path pathout;
  if(pathin.poses.size() <= 1)
    return pathin;
  pathout.header = pathin.header;
  std::vector<std::tuple<int,float>>i_dst;
  for(int i = 0; i < pathin.poses.size(); i++){
    	float hdng_shortest = abs(get_shortest(get_hdng(pathin.poses[i].pose.position,pos),pos_yaw) * rad2deg);
      i_dst.push_back(std::make_tuple(i,hdng_shortest));
  }
  sort(i_dst.begin(),i_dst.end(),sort_dst_pair);
  for(int i = 0; i < i_dst.size(); i++){
    pathout.poses.push_back(pathin.poses[std::get<0>(i_dst[i])]);
  }
	return pathout;
}
float get_dt(std::string n){
	float dt = (ros::Time::now() - last_time).toSec();
	int millisec = dt * 1000;
	last_time = ros::Time::now();
	if(n != ""){
		ROS_INFO("BRAIN: %s took %.5f microseconds %i millisec",n.c_str(),dt,millisec);
	}
	return dt;
}
int set_vlpstate(int newstate){
  float dt = (ros::Time::now() - vlpstate_change).toSec();
	float vlp_tilt_increment,abs_tilt_increment;
	float orig_tilt = arm1_tilt_msg.data;
	float new_tilt = orig_tilt;
	int min_vlptstate = par_vlpmintilt/par_vlptiltinterval;
	int max_vlptstate = par_vlpmaxtilt/par_vlptiltinterval;
//	if(dt > min_time){
		if(newstate == 1)
			vlpstate += 1;
		if(newstate == -1)
			vlpstate -= 1;

		if(newstate * par_vlptiltinterval > par_vlpmaxtilt)
			vlpstate = 0;
		else if(newstate * par_vlptiltinterval < par_vlpmintilt)
			vlpstate = 0;
		else if(newstate > 2 || newstate < -2)
			vlpstate = newstate;
		else if(newstate == 2)
			vlpstate = max_vlptstate;
		else if(newstate == -2)
			vlpstate = min_vlptstate;

		new_tilt = vlpstate * par_vlptiltinterval;
		vlp_tilt_increment = new_tilt - arm1_tilt_msg.data;
		if(vlp_tilt_increment < 0)
			abs_tilt_increment = -1 * vlp_tilt_increment;
		min_time = 2.0 + abs_tilt_increment * 0.5;//wait_pr_tilting_radian;
    ROS_INFO("MAIN: VLPSTATE[]: %i -> %i (%.3f seconds in state, %.3 seconds to next)",orig_tilt,new_tilt,dt,min_time);
    vlpstate_change = ros::Time::now();
		arm1_tilt_msg.data = new_tilt;
//  }
	return vlpstate;
}
geometry_msgs::Point get_pose_pair(geometry_msgs::PoseStamped posein){
	geometry_msgs::Point point_out;
	float yaw   			= tf::getYaw(posein.pose.orientation);
	point_out.x = posein.pose.position.x + 5 * cos(yaw);
	point_out.y = posein.pose.position.y + 5 * sin(yaw);
	point_out.z = posein.pose.position.z;
	return point_out;
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
int get_closest_pose_not_in_vec(nav_msgs::Path pathin,std::vector<int> vec,geometry_msgs::Point cur_tar, geometry_msgs::Point cur_obs, float cur_yaw, float cur_hdng){
	float best_tot = 1000;
	int best_tot_i;
	for(int i = 0; i < pathin.poses.size(); i++){
		if(!in_vec(vec,i)){
			geometry_msgs::Point cand_tar  = pathin.poses[i].pose.position;
			geometry_msgs::Point cand_obs  = get_pose_pair(pathin.poses[i]);
			float 			    		 cand_yaw  = tf::getYaw(pathin.poses[i].pose.orientation);
			float 							 cand_hdng = get_hdng(cand_tar,cur_tar);
			float dst_hdng = get_shortest(cand_hdng,cur_hdng);
			float dst_yaw  = get_shortest(cand_yaw,cur_yaw);
			float dst_pos  = get_dst3d(cur_tar,cand_tar);
			float dst_obs  = get_dst3d(cur_obs,cand_obs);
			float dst_tot  = dst_pos + dst_obs;
			if(dst_tot < best_tot && abs(dst_hdng) < 3){
				best_tot_i = i;
				best_tot = dst_tot;
			}
		}
	}
	return best_tot_i;
}
nav_msgs::Path get_path(nav_msgs::Path pathin){
	nav_msgs::Path pathout;
	pathout.header = hdr();
	std::vector<int> vec_i;
	geometry_msgs::PoseStamped current_pose;
	geometry_msgs::Point current_tar = target.pose.position;
	geometry_msgs::Point current_obs = get_pose_pair(target);
	float current_yaw  = tf::getYaw(target.pose.orientation);
	float current_hdng = get_hdng(current_tar,target_last.pose.position);
	while(vec_i.size() < pathin.poses.size()){
		int new_i 	 = get_closest_pose_not_in_vec(pathin,vec_i,current_tar,current_obs,current_yaw,current_hdng);
		if(new_i == 1000)
			break;
		current_hdng = get_hdng(pathin.poses[new_i].pose.position,current_tar);
		current_tar  = pathin.poses[new_i].pose.position;
		current_obs  = get_pose_pair(pathin.poses[new_i]);
		current_yaw  = tf::getYaw(pathin.poses[new_i].pose.orientation);
		vec_i.push_back(new_i);
	}
	float dst,dst_sum,dst_ave;
	geometry_msgs::Point p0,p1;
	dst_sum = 0;
	for(int i = 0; i < vec_i.size(); i++){
		pathout.poses.push_back(pathin.poses[vec_i[i]]);
	}
	return pathout;
}
nav_msgs::Path cutoff_at_zabs(nav_msgs::Path pathin,int cutoff_zabs){
	nav_msgs::Path pathout;
  pathout.header = hdr();
	for(int i = 0; i < pathin.poses.size(); i++){
		if(abs(pathin.poses[i].pose.position.z - pos.z) < cutoff_zabs)
			pathout.poses.push_back(pathin.poses[i]);
		else
			return pathout;
	}
	return pathout;
}

void lowrateodom_cb(const nav_msgs::Odometry::ConstPtr& msg){
  odom = *msg;
}
void mainstate_cb(const std_msgs::UInt8::ConstPtr& msg){
  mainstate = msg->data;
}
void missionstate_cb(const std_msgs::UInt8::ConstPtr& msg){
 	missionstate = msg->data;
}


void new_target(geometry_msgs::PoseStamped new_target_pose){
	target_last.pose.position    = pos;
	target_last.pose.orientation = tf::createQuaternionMsgFromYaw(pos_yaw);
	target_last.header.stamp 		 = ros::Time::now();
	target 									     = new_target_pose;
	target_yaw  					      = tf::getYaw(target.pose.orientation);
	target_obstacle.point      	= get_pose_pair(target);
	ROS_INFO("New target at: %.0f %.0f %.0f yaw: %.2f, ->obstacle-> %.0f %.0f %.0f",target.pose.position.x,target.pose.position.y,target.pose.position.z,target_yaw,target_obstacle.point.x,target_obstacle.point.y,target_obstacle.point.z);
	geometry_msgs::Point target_pos;
	target_pos = target.pose.position;
	pub_cmd.publish(target_pos);
}

int evaluate_path(nav_msgs::Path pathin){
	std::vector<geometry_msgs::Point>pairpoints;
	std::vector<float>distance_pathpoint;
	std::vector<float>distance_pairpoint;

	for(int i = 0; i < pathin.poses.size(); i++){
		geometry_msgs::Point pairpoint;
		pairpoint = get_pose_pair(pathin.poses[i]);
		pairpoints.push_back(pairpoint);
		distance_pathpoint.push_back(get_dst2d(target.pose.position,pathin.poses[i].pose.position));
		distance_pairpoint.push_back(get_dst2d(pairpoint,target_obstacle.point));
	}
	int best_i = 0;
	float best_score = 10000;
	for(int i = 0; i < pairpoints.size(); i++){
		float dst_pair     = distance_pairpoint[i];
		float dst_path 		 = distance_pathpoint[i];
		float rel_distance = fmax(dst_pair,dst_path) / fmin(dst_path,dst_pair);
		float score = 1.00 - rel_distance;
		ROS_INFO("ROS_INFO: dst_pair / path = rel_dist %.0f / %.0f = %.2f dst_path  score: %.2f",dst_pair,dst_path,rel_distance,score);
		if(score > best_score){
			best_i = i;
			best_score = score;
		}
	}
	return best_i;
}
void increment_vlp(){
	int expected_vlpstate = vlpstate + current_changefactor;
	int new_vlpstate = set_vlpstate(current_changefactor);
	if(new_vlpstate == 0)
		current_changefactor *= -1;
}

void get_next_target(){
	if(path_candidates.poses.size() > 0)
		new_target(path_candidates.poses[evaluate_path(path_candidates)]);
}


void checktfvlp(){
  geometry_msgs::TransformStamped transformStamped;
  try{
    transformStamped = tfBuffer.lookupTransform("map","velodyne_aligned",
                             ros::Time(0));
  }
  catch (tf2::TransformException &ex) {
    ROS_WARN("%s",ex.what());
    ros::Duration(1.0).sleep();
  }
	tf2::Matrix3x3 q(tf2::Quaternion(transformStamped.transform.rotation.x, transformStamped.transform.rotation.y, transformStamped.transform.rotation.z, transformStamped.transform.rotation.w));
	q.getRPY(vlp_rpy.x,vlp_rpy.y,vlp_rpy.z);
}
void checktf(){
  geometry_msgs::TransformStamped transformStamped;
  try{
    transformStamped = tfBuffer.lookupTransform("map","base_stabilized",
                             ros::Time(0));
  }
  catch (tf2::TransformException &ex) {
    ROS_WARN("%s",ex.what());
    ros::Duration(1.0).sleep();
  }
	pos.x   = transformStamped.transform.translation.x;
	pos.y   = transformStamped.transform.translation.y;
	pos.z   = transformStamped.transform.translation.z;
	pos_yaw = tf::getYaw(transformStamped.transform.rotation);
}
float get_remaining_dst(){
	return get_dst2d(pos,target.pose.position);
}
float get_target_alt(){

	float slope 	   					= get_slope(target.pose.position,target_last.pose.position);
	float dst_tot	   					= get_dst2d(target.pose.position,target_last.pose.position);
	float dst_passed 					= get_remaining_dst();
	float relative_dst_passed = dst_tot / dst_passed;
	float target_z   					= target.pose.position.z;
  //target_last.pose.position.z + slope * dst_passed;
  if(std::isnan(target_z) || std::isinf(target_z))
    return target.pose.position.z;
  return target_z;
}
float get_tilt_vlp(){
  float s = get_slope(target.pose.position,target_last.pose.position);
  float i = get_inclination(target.pose.position,target_last.pose.position);
  float s0 = get_inclination(target.pose.position,pos);
  ROS_INFO("Slope: %.2f incl: %.2f used: %.2f",s,i,s0);
  if((get_dst2d(target.pose.position,pos) < 2) || (abs(target.pose.position.z - pos.z) < 1))
    return 0;
	else
   return get_inclination(target.pose.position,pos);
}


float get_pan_slamdunk(){
	float delta_yaw = target_yaw - pos_yaw;
	ROS_INFO("target_yaw %.2f -> pos_yaw %.2f -> Yaw_delta: %.2f",target_yaw,pos_yaw,delta_yaw);
	return delta_yaw;
}

float getinpath_closestdst(nav_msgs::Path pathin,geometry_msgs::PoseStamped pose_to_check,bool use_3d){
  float lowest_dist = 1000;  float dst;
  if(pathin.poses.size() == 0){
    return lowest_dist;
  }
  for(int i = 0; i < pathin.poses.size(); i++){
    if(use_3d)
      dst = get_dst3d(pathin.poses[i].pose.position,pose_to_check.pose.position);
    else
      dst = get_dst2d(pathin.poses[i].pose.position,pose_to_check.pose.position);
    if(dst < lowest_dist)
      lowest_dist   = dst;
  }
  return lowest_dist;
}
nav_msgs::Path constrain_path_vstd(nav_msgs::Path pathin,nav_msgs::Path pathin_vstd,float cutoff_dst,bool use_3d){
  nav_msgs::Path pathout;
  pathout.header = hdr();
  if(fmin(pathin.poses.size(),pathin_vstd.poses.size()) == 0){
    return pathin;
  }
  for(int i = 0; i < pathin.poses.size(); i++){
    if(getinpath_closestdst(pathin_vstd,pathin.poses[i],use_3d) > cutoff_dst)
      pathout.poses.push_back(pathin.poses[i]);
  }
  return pathout;
}
void pntmid_cb(const geometry_msgs::PointStamped::ConstPtr& msg){
	pnt_midpoint = msg->point;
}
void polyvlp_cb(const geometry_msgs::PolygonStamped::ConstPtr& msg){
	poly_vlp = *msg;
}
int closest_in_path(nav_msgs::Path pathin,geometry_msgs::Point pnt){
  float res,dst;
  res = 1000;
  int res_i;
  if(pathin.poses.size() == 0)
    return -1;
  for(int i = 1; i < pathin.poses.size(); i++){
    dst = get_dst3d(pathin.poses[i].pose.position,pnt);
    if(dst < res){
      res = dst;
      res_i = i;
    }
  }
  return res_i;
}
bool pose_is_side(geometry_msgs::PoseStamped pose_in){
  if(pose_in.pose.orientation.y = 0.7071)
    return false;
  else
    return true;
}
nav_msgs::Path get_path_side(nav_msgs::Path pathin,bool get_side){
  nav_msgs::Path pathout;
  pathout.header = hdr();
  for(int i = 0; i < pathin.poses.size(); i++){
    if(get_side && pose_is_side(pathin.poses[i]))
      pathout.poses.push_back(pathin.poses[i]);
    if(!get_side && !pose_is_side(pathin.poses[i]))
      pathout.poses.push_back(pathin.poses[i]);
  }
  return pathout;
}

geometry_msgs::PoseStamped check_closest_pose(bool get_side){
  nav_msgs::Path path_cand_side,path_cand_top;
  geometry_msgs::PoseStamped pose_closest_side,pose_closest_top;

  path_cand_side = get_path_side(path_candidates,true);
  path_cand_top  = get_path_side(path_candidates,false);

  if(get_side && path_cand_side.poses.size() > 0){
    pose_closest_side     = path_cand_side.poses[closest_in_path(path_cand_side,pnt_midpoint)];
    float dst_pnt_side    = get_dst3d(pose_closest_side.pose.position,pos);
    ROS_INFO("Closest side point: (dst: %.0f) x: %.0f y: %.0f z: %.0f",dst_pnt_side,pose_closest_side.pose.position.x,pose_closest_side.pose.position.y,pose_closest_side.pose.position.z);
    return pose_closest_side;
  }
  if(!get_side && path_cand_top.poses.size() > 0){
    pose_closest_top = path_cand_top.poses[closest_in_path(path_cand_top,pnt_midpoint)];
    float dst_pnt_top      = get_dst3d(pose_closest_top.pose.position,pos);
    ROS_INFO("Closest top point: (dst: %.0f) x: %.0f y: %.0f z: %.0f",dst_pnt_top,pose_closest_top.pose.position.x,pose_closest_top.pose.position.y,pose_closest_top.pose.position.z);
    return pose_closest_top;
  }
  return pose_closest_side;
}
void evaluate_options(){

}/*
nav_msgs::Path get_largest_segment(nav_msgs::Path pathin){
  nav_msgs::Path pathout;
  pathout.header = hdr();
  float val_min = get_shortest(get_hdng(pathin.poses[0].pose.position,pos),pos_yaw);
  float val_max = get_shortest(get_hdng(pathin.poses[pathin.poses.size()-1].pose.position,pos),pos_yaw);

  int percent_length = int(round(pathin.poses.size() * percentage / 100));
  for(int i = segment_size; i < pathin.poses.size(); i++){
    float v0 = get_shortest(get_hdng(pathin.poses[i].pose.position,pos),pos_yaw);
    float v1 = get_shortest(get_hdng(pathin.poses[i-segment_size].pose.position,pos),pos_yaw);

      pathin.poses[i-k]
    pathout.poses.push_back(pathin.poses[i]);
  }
  ROS_INFO("Percent: %i, pose_in: %i, percent_len: %i, path_out: %i",percentage,pathin.poses.size(),percent_length,pathout.poses.size());
  return pathout;
}*/
nav_msgs::Path cutoff_percentage(nav_msgs::Path pathin,int percentage){
  nav_msgs::Path pathout;
  pathout.header = hdr();
  int percent_length = int(round(pathin.poses.size() * percentage / 100));
  for(int i = 0; i < fmax(percent_length,fmin(pathin.poses.size(),3)); i++){
    pathout.poses.push_back(pathin.poses[i]);
  }
  ROS_INFO("Percent: %i, pose_in: %i, percent_len: %i, path_out: %i",percentage,pathin.poses.size(),percent_length,pathout.poses.size());
  return pathout;
}
nav_msgs::Path work_path(nav_msgs::Path pathin){
  pathin = sort_path_by_zabs(pathin);
  pathin = cutoff_percentage(pathin,30);
  pathin = sort_path_by_hdng(pathin);
  pathin = cutoff_percentage(pathin,30);
  pathin = sort_path_by_dst(pathin,true);
  return pathin;
}

void pathtest_cb(const nav_msgs::Path::ConstPtr& msg){
    path_test = *msg;
}
void pathvstd_cb(const nav_msgs::Path::ConstPtr& msg){
	path_visited = constrain_path_bbpoly(*msg,poly_vlp);
}

void pathfloor_cb(const nav_msgs::Path::ConstPtr& msg){
  path_floor = *msg;
}
void set_target_path(nav_msgs::Path pathin){
  path_targets      = pathin;
  looking_for_paths = false;
  path_targets_i    = 0;
  arm1_tilt_msg.data = 0;
  new_target(path_targets.poses[path_targets_i]);
}

void set_tilt(float radians){
  float new_tilt = radians;
  if(new_tilt < par_vlpmintilt){
    tiltlimit_reached  = true;
    new_tilt = par_vlpmintilt;
  }
  if(new_tilt > par_vlpmaxtilt){
    tiltlimit_reached  = true;
    new_tilt = par_vlpmaxtilt;
  }
  if(arm1_tilt_msg.data - new_tilt > 0.1)
    tilting = true;
  ROS_INFO("BRAIN: ARM - tilt from %i -> %i",arm1_tilt_msg.data,new_tilt);
  arm1_tilt_msg.data = new_tilt;
}

void increment_tilt(float radians){
  float new_tilt = arm1_tilt_msg.data + radians;
  set_tilt(new_tilt);
}
void set_tilt_degrees(float degrees){
  float deg2rad = M_PI/180.0;
  set_tilt(deg2rad*degrees);
}
void increment_tilt_degrees(float degrees){
  float deg2rad = M_PI/180.0;
  increment_tilt(deg2rad*degrees);
}
void request_paths(){
  got_candidates       = false;
  got_candidates_sides = false;
  std_msgs::UInt8 get_path;
  requested_paths = true;
  ROS_INFO("BRAIN: Requesting paths");
  pub_get_next_path.publish(get_path);
}


void no_paths_found(){
  if(tiltlimit_reached){
    ROS_INFO("BRAIN: Tiltlimit reached");
    geometry_msgs::PoseStamped ps;
    nav_msgs::Path path;
    ps.pose.position = pnt_midpoint;
    ps.pose.orientation.w = 1;
    path.poses.push_back(ps);
    set_tilt(0);
    set_target_path(path);
  }
  else{
    ROS_INFO("BRAIN: Testing another tilt");
    increment_tilt_degrees(10);
  }
}

void check_is_tilting(){
  float delta_vlp = get_shortest(arm1_tilt_msg.data,vlp_rpy.y);
  if(delta_vlp < 0.15 && delta_vlp > -0.15){
    float dt_tilting = (ros::Time::now() - tilt_stop).toSec();
    if(dt_tilting > 1.0){
      ROS_INFO("BRAIN: Tilt complete");
      tilting = false;
      if(looking_for_paths && !requested_paths){
        ROS_INFO("BRAIN: tilt complete -requesting paths");
        request_paths();
      }
    }
  }
  else
    tilt_stop = ros::Time::now();
}

void evaluate_paths(){
  requested_paths      = false;
  got_candidates       = false;
  got_candidates_sides = false;
  if(path_candidates_sides.poses.size() > 2){
    ROS_INFO("BRAIN: path sides %i targets",path_targets);
    set_target_path(path_candidates_sides);
  }
  else if(path_candidates.poses.size() > 0){
    ROS_INFO("BRAIN: path floor %i targets",path_targets);
    set_target_path(path_candidates);
  }
  else{
    no_paths_found();
  }
}
void pathcand_cb(const nav_msgs::Path::ConstPtr& msg){
  path_candidates = *msg;
  got_candidates = true;
  ROS_INFO("PathCandidates %i",path_candidates.poses.size());
  if(requested_paths && got_candidates_sides){
    evaluate_paths();
  }
}
void pathcandsides_cb(const nav_msgs::Path::ConstPtr& msg){
  path_candidates_sides = *msg;
  got_candidates_sides = true;
  ROS_INFO("PathCandidates %i",path_candidates_sides.poses.size());
  if(requested_paths && got_candidates){
    evaluate_paths();
  }
}
void check_path_progress(){
  if(path_targets_i < path_targets.poses.size()){
    float dst_to_target = get_dst3d(path_targets.poses[path_targets_i].pose.position,pos);
    if(dst_to_target < 5){
      ROS_INFO("BRAIN: Dst to target[%i of %i]: %.0f",path_targets_i,path_targets.poses.size(),dst_to_target);
      if(path_targets_i+1 < path_targets.poses.size()){
        path_targets_i++;
        new_target(path_targets.poses[path_targets_i]);
      }
      else
        request_paths();
    }
  }
  else
    request_paths();
}
void test_tilt_cb(const std_msgs::Float64::ConstPtr& msg){
  last_update = ros::Time::now();
  set_tilt_degrees(msg->data);
}
int main(int argc, char** argv)
{
  ros::init(argc, argv, "tb_brain_node");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");
  private_nh.param("par_zjump",   par_zjump, 3.0);//*2.0);
	private_nh.param("map_sidelength",par_maprad, 300.0);
	private_nh.param("par_vlpmaxtilt",par_vlpmaxtilt, M_PI/5);
	private_nh.param("par_vlpmintilt",par_vlpmintilt, -M_PI/5);
	private_nh.param("par_vlptiltinterval",par_vlptiltinterval, M_PI/10);
	private_nh.param("takeoff_altlvl",par_takeoffaltitude, 5.0);

	//private_nh.param("par_vlptbasetime",par_vlptbasetime, 1.5);
	tf2_ros::TransformListener tf2_listener(tfBuffer);
	for(int i = 0; i < 40; i++){
		z_lvls.push_back(int(round(-par_zjump*3 + par_zjump*i)));
	}
	target.pose.position.z 	  = par_takeoffaltitude;
	target.pose.orientation.w = 1;
  target_last = target;
  pub_get_next_path	 	= nh.advertise<std_msgs::UInt8>("/tb_path/get", 100);

  pub_altlvl			 	 	= nh.advertise<std_msgs::UInt8>("/tb_cmd/set_altlvl", 100);
	pub_altcmd			 	 	= nh.advertise<std_msgs::Float64>("/tb_cmd/alt_target", 100);

	pub_tiltvlp					= nh.advertise<std_msgs::Float64>("/tb_cmd/arm1_tilt", 10);
	pub_tilt  					= nh.advertise<std_msgs::Float64>("/tb_cmd/arm2_tilt", 10);
	pub_pan							= nh.advertise<std_msgs::Float64>("/tb_cmd/arm2_pan", 10);

	pub_activity		 	 	= nh.advertise<std_msgs::String>("/tb_cmd/set_activity", 100);
	pub_armstate			 	= nh.advertise<geometry_msgs::Vector3>("/tb_cmd/arm_motion", 100);
  pub_cmd      				= nh.advertise<geometry_msgs::Point>("/cmd_pos",10);
  ros::Publisher pub_target_pose = nh.advertise<geometry_msgs::PoseStamped>("/cmd_pose",10);
  ros::Publisher pub_path = nh.advertise<nav_msgs::Path>("/tb_path/pathout",10);

	ros::Subscriber s5  = nh.subscribe("/tb_nav/lowrate_odom",10,lowrateodom_cb);
	ros::Subscriber s2  = nh.subscribe("/tb_path/visited",10,pathvstd_cb);
//  ros::Subscriber s1  = nh.subscribe("/tb_path/candidates",10,pathcand_cb);
//  ros::Subscriber s8  = nh.subscribe("/tb_path/candidates_sides",10,pathcandsides_cb);
  ros::Subscriber s1  = nh.subscribe("/tb_path/out_down",10,pathcand_cb);
  ros::Subscriber s8  = nh.subscribe("/tb_path/out_side",10,pathcandsides_cb);
  ros::Subscriber s7  = nh.subscribe("/tb_path/floor",10,pathfloor_cb);
  ros::Subscriber s12  = nh.subscribe("/tb_path_tester",10,pathtest_cb);

  ros::Subscriber s6  = nh.subscribe("/tb_path/poly_vlp",10,polyvlp_cb);
  ros::Subscriber s9  = nh.subscribe("/tb_path/midpoint",10,pntmid_cb);
	ros::Subscriber s4  = nh.subscribe("/tb_fsm/mission_state",10,missionstate_cb);
  ros::Subscriber s3  = nh.subscribe("/tb_fsm/main_state",100,&mainstate_cb);
  ros::Subscriber s11  = nh.subscribe("/tb_test/tilt",100,&test_tilt_cb);
  ros::Rate rate(2.0);
  ros::Time start = ros::Time::now();
  last_update = ros::Time::now();


  while(ros::ok()){

		checktf();
		checktfvlp();

    if(mainstate == 1 && ((ros::Time::now() - last_update).toSec() > 5)) {
      if(tilting)
        check_is_tilting();
      if(!looking_for_paths)
        check_path_progress();

    }

    //if(mainstate == 0)
  //    last_update = ros::Time::now();
  //  else{

  //  }
//		if(get_remaining_dst() < 8 && ) ){

    //  nav_msgs::Path path_out;
    //  path_out = constrain_path_vstd(path_test,path_visited,7,false);
/*  path_candidates = constrain_path_vstd(path_candidates,path_visited,5,true);
  path_floor      = constrain_path_vstd(path_floor,path_visited,7,false);

      if(path_candidates.poses.size() > 0)
        path_out = get_path(cutoff_at_zabs(sort_path_by_zabs(path_candidates),1));
      if(path_out.poses.size() == 0)
        path_out = get_path(cutoff_at_zabs(sort_path_by_zabs(path_candidates),3));
      if(path_out.poses.size() == 0 && path_floor.poses.size() > 0)
        path_out = work_path(path_floor);
*/


  //  if(get_remaining_dst() > 3){
  //    arm1_tilt_msg.data = get_tilt_vlp();
  //    arm2_pan_msg.data  = get_pan_slamdunk();
  //  }
  	target_alt_msg.data = get_target_alt();
    pub_target_pose.publish(target);
    pub_path.publish(path_targets);
    pub_tiltvlp.publish(arm1_tilt_msg);
    pub_altcmd.publish(target_alt_msg);

		//arm2_tilt_msg.data = get_tilt_slamdunk();
	//
	//	pub_tiltvlp.publish(arm1_tilt_msg);
	//	pub_tilt.publish(arm2_tilt_msg);
	//	pub_pan.publish(arm2_pan_msg);
	//	assess_situation();
    rate.sleep();
    ros::spinOnce();
  }
  return 0;
}
