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
#include <nav_msgs/Odometry.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib_msgs/GoalID.h>
#include <nav_msgs/GetPlan.h>

tf2_ros::Buffer tfBuffer;
ros::Publisher pub_target;
geometry_msgs::Point p,pos,cmd_pos;
geometry_msgs::Vector3 rpy,vlp_rpy;
geometry_msgs::PoseStamped target_active;
nav_msgs::Path path_visited,path_unknown;
double par_maxrad_pathcen,par_visited_cutoff;
float cmd_yaw,pos_yaw;
std::vector<std::vector<float>>paths_side_bboxes;
std::vector<nav_msgs::Path> paths_side;
std::vector<geometry_msgs::Point> paths_side_centers;
std::vector<int> targetindexes_sent;
std::vector<int> blacklist;
std::vector<int> blacklist_paths_side;

void vstd_cb(const nav_msgs::Path::ConstPtr& msg){
  path_visited = *msg;
}
std_msgs::Header hdr(){
  std_msgs::Header header;
  header.frame_id = "map";
  header.stamp    = ros::Time::now();
  return header;
}

double constrainAngle(double x){
  if(x > M_PI)
    return (x - M_PI*2);
  else if(x < -M_PI)
    return (x + M_PI*2);
  else
    return x;
}

double saturate(double val, double max){
    if (abs(val) >= max)
      val = (val>0) ? max : -1 * max;
    else
      val = val;
    if((std::isnan(val)) || (std::isinf(val))){
      return 0;
    }
    else{
      return val;
    }
}
float get_hdng(geometry_msgs::Point p1,geometry_msgs::Point p0){
  float dx = p1.x - p0.x;
  float dy = p1.y - p0.y;
  return atan2(dy,dx);
}
float get_dst2d(geometry_msgs::Point p1, geometry_msgs::Point p2){
  return(sqrt(pow(p1.x-p2.x,2)+pow(p1.y-p2.y,2)));
}

float get_dst3d(geometry_msgs::Point p1, geometry_msgs::Point p2){
  return(sqrt(pow(p1.x-p2.x,2)+pow(p1.y-p2.y,2)+pow(p1.z-p2.z,2)));
}
float get_inclination(geometry_msgs::Point p2, geometry_msgs::Point p1){
  return atan2(p2.z - p1.z,get_dst2d(p1,p2));
}
float get_shortest(float target_heading,float actual_hdng){
  float a = target_heading - actual_hdng;
  if(a > M_PI)a -= M_PI*2;
  else if(a < -M_PI)a += M_PI*2;
  return a;
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
void update_pos_vlp(){
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
	tf2::Matrix3x3 q(tf2::Quaternion(transformStamped.transform.rotation.x, transformStamped.transform.rotation.y, transformStamped.transform.rotation.z, transformStamped.transform.rotation.w));
	geometry_msgs::Vector3 v;
	q.getRPY(v.x,v.y,v.z);
	pos_yaw = v.z;
}
void update_pos_cmd(){
	geometry_msgs::TransformStamped transformStamped;
	try{
		transformStamped = tfBuffer.lookupTransform("map","base_perfect_alt",
														 ros::Time(0));
	}
	catch (tf2::TransformException &ex) {
		ROS_WARN("%s",ex.what());
		ros::Duration(1.0).sleep();
	}
	cmd_pos.x = transformStamped.transform.translation.x;
	cmd_pos.y = transformStamped.transform.translation.y;
	cmd_pos.z = transformStamped.transform.translation.z;
	tf2::Matrix3x3 q(tf2::Quaternion(transformStamped.transform.rotation.x, transformStamped.transform.rotation.y, transformStamped.transform.rotation.z, transformStamped.transform.rotation.w));
	geometry_msgs::Vector3 v;
	q.getRPY(v.x,v.y,v.z);
	cmd_yaw = v.z;
}

bool is_point_not_visited(geometry_msgs::Point pin,float lim){
  float res,dst;
  if(path_visited.poses.size() == 0)
    return true;
  for(int i = 0; i < path_visited.poses.size(); i++){
     if(get_dst3d(path_visited.poses[i].pose.position,pin) < lim)
        return false;
  }
  return true;
}
nav_msgs::Path not_visited(nav_msgs::Path pathin){
	nav_msgs::Path pathout;
	pathout.header = hdr();
	for(int i = 0; i < pathin.poses.size(); i++){
		if(is_point_not_visited(pathin.poses[i].pose.position,par_visited_cutoff))
			pathout.poses.push_back(pathin.poses[i]);
	}
	return pathout;
}
void path_visited_cb(const nav_msgs::Path::ConstPtr& msg){
	path_visited = *msg;
}
std::vector<float> getinpath_boundingbox(nav_msgs::Path pathin){
  std::vector<float> bbvec;
  bbvec.resize(6);
  bbvec[3] = bbvec[4] = bbvec[5] = -1000;
  bbvec[0] = bbvec[1] = bbvec[2] = 1000;
  for(int i = 0; i < pathin.poses.size(); i++){
    if(pathin.poses[i].pose.position.x > bbvec[3])bbvec[3] = pathin.poses[i].pose.position.x;
    if(pathin.poses[i].pose.position.y > bbvec[4])bbvec[4] = pathin.poses[i].pose.position.y;
    if(pathin.poses[i].pose.position.z > bbvec[5])bbvec[5] = pathin.poses[i].pose.position.z;
    if(pathin.poses[i].pose.position.x < bbvec[0])bbvec[0] = pathin.poses[i].pose.position.x;
    if(pathin.poses[i].pose.position.y < bbvec[1])bbvec[1] = pathin.poses[i].pose.position.y;
    if(pathin.poses[i].pose.position.z < bbvec[2])bbvec[2] = pathin.poses[i].pose.position.z;
  }
  return bbvec;
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
void path_side_cb(const nav_msgs::Path::ConstPtr& msg){
	if(msg->poses.size() > 10){
		ros::Time t0 = ros::Time::now();
		ROS_INFO("Path side cb");
		paths_side.push_back(not_visited(*msg));
		ROS_INFO("path: %i path_in: %i",paths_side[paths_side.size()-1].poses.size(),msg->poses.size());
		paths_side_bboxes.push_back(getinpath_boundingbox(paths_side[paths_side.size()-1]));
		ROS_INFO("bbox: xy: %.0f %.0f %.0f -> %.0f %.0f %.0f",paths_side_bboxes[paths_side_bboxes.size()-1][0],paths_side_bboxes[paths_side_bboxes.size()-1][1],paths_side_bboxes[paths_side_bboxes.size()-1][2],paths_side_bboxes[paths_side_bboxes.size()-1][3],paths_side_bboxes[paths_side_bboxes.size()-1][4],paths_side_bboxes[paths_side_bboxes.size()-1][5]);
		paths_side_centers.push_back(get_ave_pnt(paths_side[paths_side.size()-1]));
		ROS_INFO("pcen: %.0f %.0f %.0f",paths_side_centers[paths_side_centers.size()-1].x,paths_side_centers[paths_side_centers.size()-1].y,paths_side_centers[paths_side_centers.size()-1].z);
		float dt = (ros::Time::now()-t0).toSec();
		ROS_INFO("dt: %.4f",dt);
	}
}
void path_unknown_cb(const nav_msgs::Path::ConstPtr& msg){
	path_unknown = not_visited(*msg);
}
int getinpath_closestindex2d(nav_msgs::Path pathin,geometry_msgs::Point pnt){
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
	int ci = getinpath_closestindex2d(path_unknown,pos);
	if(ci >= 0 && path_unknown.poses.size() > ci){
		float dst = get_dst2d(path_unknown.poses[ci].pose.position,pos);
		blacklist.push_back(ci);
		target_active.pose.position = path_unknown.poses[ci].pose.position;
		target_active.pose.orientation.w = 1.0;
		ROS_INFO("New target[%.0f,%.0f] closest index unknown path: %i / %i, blacklist: %i points, dst_to_point: %.0f",target_active.pose.position.x,target_active.pose.position.y,ci,path_unknown.poses.size(),blacklist.size(),dst);
		target_active.header = hdr();
		pub_target.publish(target_active);
	}
	else{
		ROS_INFO("FAILED TO FIND PATH UNKNOWN TARGET");
	}
}
bool get_target_from_sidepaths(){
	float closest = 1000;
	int closest_i = -1;
	ROS_INFO("Getting target from %i sidepaths",paths_side_centers.size());
	for(int i = 0; i < paths_side.size(); i++){
		if(!in_vec(blacklist_paths_side,i)){
			float dst = get_dst2d(pos,paths_side_centers[i]);
			ROS_INFO("paths_side[%i] center: %.0f %.0f, dst: %.0f",i,paths_side_centers[i].x,paths_side_centers[i].y,dst);
			if(dst < closest){
				closest = dst;
				closest_i = i;
			}
		}
	}
	if(closest_i < paths_side.size() && closest_i >= 0){
		ROS_INFO("closest_i: %i paths_side: %i",closest_i,paths_side.size());
		target_active.pose.position = paths_side_centers[closest_i];
		blacklist_paths_side[closest_i];
		target_active.pose.orientation.w = 1.0;
		target_active.header = hdr();
		ROS_INFO("Target active updated. %.0f %.0f",target_active.pose.position.x,target_active.pose.position.y);
		return true;
	}
	else{
		ROS_INFO("target not updated with sides");
		return false;
	}
}
void get_next_target(){
	if(!get_target_from_sidepaths){
		get_target_from_path_unknown();
	}
	else{
		get_target_from_path_unknown();
		ROS_INFO("Publishing target active: %.0f %.0f",target_active.pose.position.x,target_active.pose.position.y);
		pub_target.publish(target_active);
	}
}
void update_pos(){
	geometry_msgs::TransformStamped transformStamped;
	try{
		transformStamped = tfBuffer.lookupTransform("map","base_perfect_alt",
														 ros::Time(0));
	}
	catch (tf2::TransformException &ex) {
		ROS_WARN("%s",ex.what());
		ros::Duration(1.0).sleep();
	}
	pos.x = transformStamped.transform.translation.x;
	pos.y = transformStamped.transform.translation.y;
	pos.z = transformStamped.transform.translation.z;
	pos_yaw = tf::getYaw(transformStamped.transform.rotation);
	float target_dst = get_dst2d(pos,target_active.pose.position);
	if(target_dst < 6 || (ros::Time::now()-target_active.header.stamp).toSec() > 10.0 && path_unknown.poses.size() > 0){
		get_next_target();
	}
}
void update_paths_sides(){
	float cmd_yaw,pos_yaw;
	std::vector<std::vector<float>>paths_side_bboxes_temp;
	std::vector<nav_msgs::Path> paths_side_temp;
	std::vector<geometry_msgs::Point> paths_side_centers_temp;
	for(int i = paths_side.size(); i > 0; i++){
		if(get_dst2d(paths_side_centers[i],pos) < par_maxrad_pathcen){
			paths_side_bboxes_temp.push_back(paths_side_bboxes[i]);
			paths_side_temp.push_back(paths_side[i]);
			paths_side_centers_temp.push_back(paths_side_centers[i]);
		}
	}
	paths_side = paths_side_temp;
	paths_side_bboxes = paths_side_bboxes_temp;
	paths_side_centers = paths_side_centers_temp;
}
void mb_result_cb(const std_msgs::Bool::ConstPtr& msg){
	if(!msg->data)
		get_next_target();
}
int main(int argc, char** argv)
{
  ros::init(argc, argv, "tb_targeter_node");
  ros::NodeHandle nh;
	ros::NodeHandle private_nh("~");
	private_nh.param("obstacle_distance_cutoff", par_maxrad_pathcen, 40.0);
	private_nh.param("visited_dst_cutoff", par_visited_cutoff, 8.0);

	tf2_ros::TransformListener tf2_listener(tfBuffer);
	ros::Subscriber s01 = nh.subscribe("/tb_world/path_unknown",10,path_unknown_cb);
	ros::Subscriber s02 = nh.subscribe("/tb_world/path_visited",10,path_visited_cb);
	ros::Subscriber s03 = nh.subscribe("/tb_edto/path_side",10,path_side_cb);
	ros::Subscriber s04 = nh.subscribe("/tb_cmd/mb_result",10,mb_result_cb);

	pub_target 				= nh.advertise<geometry_msgs::PoseStamped>("/tb_cmd/mb_pose_target",100);
//
	ros::Rate rate(2.0);

	ros::Time time_last = ros::Time::now();
	while(ros::ok()){
    rate.sleep();
    ros::spinOnce();
		//pdate_paths_sides();
		update_pos();
		update_pos_cmd();
  }
  return 0;
}
