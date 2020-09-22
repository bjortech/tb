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
ros::Publisher pub_target_state,pub_path_to_interpolate,pub_target_setpoint_xy,pub_target_pathplan,pub_target_mb,pub_path_closest_dst,pub_point_closest_dst,pub_roi_poly,pub_roi2d_poly;
ros::Publisher pub_path_to_zsmooth,pub_path_cmd,pub_elevate_poly,pub_elevate_point,pub_elevate_path,pub_costmapelevate_point,pub_costmapelevate_path,pub_costmapupdate_edto,pub_costmapupdate_img,pub_target_setpoint_yaw,pub_target_setpoint_z;
geometry_msgs::Point p,pos,cmd_pos;
geometry_msgs::PointStamped target_point,target_point_received,setpoint_xy;
geometry_msgs::PoseStamped target_active;
geometry_msgs::Vector3Stamped cmdmb;
std_msgs::Float64 setpoint_z,setpoint_yaw;
bool use_mb;
float pos_yaw;
tf2_ros::Buffer tfBuffer;
std::vector<int> targetindexes_sent;
std::vector<int> blacklist;
int costmap_elevation_meters;
ros::Time process_start;
geometry_msgs::PolygonStamped poly_roi2d,poly_roi;
nav_msgs::Path path_starsquare,path_cmd;
std::string active_process;
std::vector<float> zmin_vec;
cv::Mat img(1000,1000,CV_8UC3,cv::Scalar(0, 0, 0)); //create image, set encoding and size, init pixels to default val
cv::Mat img_blank(1000,1000,CV_8UC3,cv::Scalar(0, 0, 0)); //create image, set encoding and size, init pixels to default val
float zmin_ave;
int que_size = 10;
float pos_z_min = 5.0;
geometry_msgs::PointStamped pos_front;
geometry_msgs::Vector3 vlp_rpy;
int path_cmd_i = 0;
int count_target_paths = 0;
nav_msgs::Odometry odom_global;
float par_res = 1.0;
nav_msgs::Path path_visited;
ros::Time state_internal_change,targetstate_change,state_cmd_change;
std::string state_internal = "idle";
std::string state_cmd 	 = "idle";
std::string state_target = "idle";
std_msgs::String targetstate_msg;
geometry_msgs::PointStamped forward_point;
int mainstate = 0;
int state_costmap_elevation = 5;
bool par_update_costmap_edto;
float cmd_yaw;
double par_zclearing;
void set_state_cmd(std::string newstate){
  if(newstate != state_cmd){
    float dt = (ros::Time::now() - state_cmd_change).toSec();
    ROS_INFO("TARGETER: cmd: %s -> %s (%.3f seconds in state)",state_cmd,newstate,dt);
    state_cmd_change = ros::Time::now();
    state_cmd = newstate;
  }
}
void set_internalstate(std::string newstate){
  if(newstate != state_internal){
    float dt = (ros::Time::now() - state_internal_change).toSec();
    ROS_INFO("TARGETER: internal: %s -> %s (%.3f seconds in state)",state_internal,newstate,dt);
    state_internal_change = ros::Time::now();
    state_internal = newstate;
  }
}
void set_targetstate(std::string newstate){
  if(newstate != targetstate_msg.data){
    float dt = (ros::Time::now() - targetstate_change).toSec();
    ROS_INFO("MAINSTATE: TARGET: %s -> %s (%.3f seconds in state)",targetstate_msg.data,newstate,dt);
    targetstate_change  	= ros::Time::now();
    targetstate_msg.data  = newstate;
		if(newstate == "target_active"){

		}
		state_target = newstate;
		pub_target_state.publish(targetstate_msg);
  }
}
void vstd_cb(const nav_msgs::Path::ConstPtr& msg){
  path_visited = *msg;
}
std_msgs::Header hdr(){
  std_msgs::Header header;
  header.frame_id = "map";
  header.stamp    = ros::Time::now();
  return header;
}

float y2r(float y){
  return (img.rows / 2 - y / par_res);
}
float x2c(float x){
  return (x / par_res + img.cols/2);
}
int r2y(float r){
  return int((img.rows / 2 - r) * par_res);
}
int c2x(float c){
  return int((c - img.cols / 2) * par_res);
}
cv::Point pnt2cv(geometry_msgs::Point pin){
	int c = x2c(pin.x);
	int r = y2r(pin.y);
	return cv::Point(c,r);
}

void draw_path(nav_msgs::Path pathin,cv::Scalar color, int size){
	for(int i = 0; i < pathin.poses.size(); i++){
		geometry_msgs::Point pnt = pathin.poses[i].pose.position;
    if(size == 0){
      img.at<cv::Vec3b>( y2r(pnt.y),x2c(pnt.x) )[0] = color[0];
      img.at<cv::Vec3b>( y2r(pnt.y),x2c(pnt.x) )[1] = color[1];
      img.at<cv::Vec3b>( y2r(pnt.y),x2c(pnt.x) )[2] = color[2];
    }
    else
      cv::circle(img,pnt2cv(pnt),size,color,1);
	}
}

void draw_line(geometry_msgs::Point p0,float a,float r,cv::Scalar color){
  geometry_msgs::Point pyaw;
  pyaw.x = p0.x + r * cos(a);
  pyaw.y = p0.y + r * sin(a);
  cv::line (img, pnt2cv(pos), pnt2cv(pyaw),color,1,cv::LINE_8,0);
}

double constrainAngle(double x){
  if(x > M_PI)
    return (x - M_PI*2);
  else if(x < -M_PI)
    return (x + M_PI*2);
  else
    return x;
}
void drawimg(){
  img_blank.copyTo(img);

  draw_line(pos,constrainAngle(pos_yaw + M_PI/3),50,cv::Scalar(200,200,200));
  draw_line(pos,constrainAngle(pos_yaw - M_PI/3),50,cv::Scalar(200,200,200));
  cv::circle(img,pnt2cv(pos),4,cv::Scalar(200,200,200),1);
  cv::Mat img_new2;
  cv::resize(img, img_new2, cv::Size(), 2.0, 2.0);
  count_target_paths++;
  cv::imwrite("/home/nuc/brain/fullpath/"+std::to_string(count_target_paths)+"navigator.png",img_new2);
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
geometry_msgs::PolygonStamped create_poly2d(geometry_msgs::Point pin, float rad){
	geometry_msgs::PolygonStamped poly;
	poly.header = hdr();
	poly.polygon.points.resize(5);
	poly.polygon.points[0].x = round(pin.x + rad);
	poly.polygon.points[0].y = round(pin.y + rad);
	poly.polygon.points[1].x = round(pin.x - rad);
	poly.polygon.points[1].y = round(pin.y + rad);
	poly.polygon.points[2].x = round(pin.x - rad);
	poly.polygon.points[2].y = round(pin.y - rad);
	poly.polygon.points[3].x = round(pin.x + rad);
	poly.polygon.points[3].y = round(pin.y - rad);
	poly.polygon.points[4]   = poly.polygon.points[0];
	return poly;
}
void create_poly_volume(geometry_msgs::Point pin, float sides,float z0,float z1){
	poly_roi2d = create_poly2d(pin,sides/2);
	poly_roi = poly_roi2d;
	for(int i = 0; i < poly_roi.polygon.points.size(); i++){
		poly_roi.polygon.points[i].z = z0;
	}
	poly_roi.polygon.points.push_back(poly_roi.polygon.points[0]);
	poly_roi.polygon.points.push_back(poly_roi.polygon.points[1]);
	poly_roi.polygon.points.push_back(poly_roi.polygon.points[2]);
	poly_roi.polygon.points.push_back(poly_roi.polygon.points[3]);
	poly_roi.polygon.points.push_back(poly_roi.polygon.points[4]);

	for(int i = 5; i < poly_roi.polygon.points.size(); i++){
		poly_roi.polygon.points[i].z = z1;
	}
	for(int i = 0; i < poly_roi.polygon.points.size(); i++){
		if(poly_roi.polygon.points[i].z > z1)
			poly_roi.polygon.points[i].z = z1;
		if(poly_roi.polygon.points[i].z < z0)
			poly_roi.polygon.points[i].z = z0;
	}
//  ROS_INFO("x %.2f y %.2f x %.2f y %.2f x %.2f y %.2f x %.2f y %.2f x %.2f y %.2f",poly.polygon.points[0].x,poly.polygon.points[0].y,poly.polygon.points[1].x,poly.polygon.points[1].y,poly.polygon.points[2].x,poly.polygon.points[2].y,poly.polygon.points[3].x,poly.polygon.points[3].y,poly.polygon.points[4].x,poly.polygon.points[4].y);
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
void mainstate_cb(const std_msgs::UInt8::ConstPtr& msg){
	mainstate = msg->data;
}

void check_path_cmd(){
	if(path_cmd.poses.size() > path_cmd_i && state_target == "path_cmd"){
		set_targetstate("target_active");
	}
	else if(state_target == "path_cmd" && state_internal == "path_cmd_active")
		set_targetstate("idle");
}

void path_cmd_cb(const nav_msgs::Path::ConstPtr& msg){
	path_cmd = *msg;
	check_path_cmd();
}
void pathcmd_i_cb(const std_msgs::UInt8::ConstPtr& msg){
	path_cmd_i = msg->data;
	check_path_cmd();
}
void path_plan_cb(const nav_msgs::Path::ConstPtr& msg){
	set_internalstate("interpolating_path");
	pub_path_to_interpolate.publish(*msg);
}
void path_interpolated_cb(const nav_msgs::Path::ConstPtr& msg){
	set_internalstate("elevating_path");
	pub_elevate_path.publish(*msg);
}
nav_msgs::Path get_path_with_safety_margin(nav_msgs::Path pathin,float margin){
	for(int i = 0; i < pathin.poses.size(); i++){
		pathin.poses[i].pose.position.z += margin;
	}
	return pathin;
}
void elevated_path_cb(const nav_msgs::Path::ConstPtr& msg){
	set_internalstate("zsmoothing_path");
	pub_path_to_zsmooth.publish(get_path_with_safety_margin(*msg,par_zclearing));
}
void path_zsmoothed_cb(const nav_msgs::Path::ConstPtr& msg){
	set_internalstate("path_cmd_sent");
	pub_path_cmd.publish(*msg);
}

std::vector<float> getinpath_boundingbox(nav_msgs::Path pathin){
  std::vector<float> bbvec;
  bbvec.resize(6);
  bbvec[3] = bbvec[4] = bbvec[5] = -100;
  bbvec[0] = bbvec[1] = bbvec[2] = 100;
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
std::vector<float> inflate_bbvec(std::vector<float> bbvec,float inflation_xy,float inflation_z){
  bbvec[0] = bbvec[0] - inflation_xy;
  bbvec[1] = bbvec[1] - inflation_xy;
	bbvec[2] = bbvec[2] - inflation_z;
	bbvec[3] = bbvec[3] + inflation_xy;
  bbvec[4] = bbvec[4] + inflation_xy;
  bbvec[5] = bbvec[5] + inflation_z;
  return bbvec;
}
nav_msgs::Path get_start_end_path(geometry_msgs::Point p0, geometry_msgs::Point p1){
	nav_msgs::Path pathout;
	pathout.poses.resize(2);
	pathout.poses[0].pose.position = p0;
	pathout.poses[1].pose.position = p1;
	pathout.poses[0].pose.orientation.w = 1.0;
	pathout.poses[1].pose.orientation.w = 1.0;
	pathout.header = pathout.poses[0].header = pathout.poses[1].header = hdr();
	return pathout;
}
void send_elevation_request(geometry_msgs::Point p0, geometry_msgs::Point p1){
	set_internalstate("updating_costmap_elevation");
	pub_costmapelevate_path.publish(get_start_end_path(p0,p1));
}
nav_msgs::Path set_path_z(nav_msgs::Path pathin,float fixed_z){
	for(int i = 0; i < pathin.poses.size(); i++){
		pathin.poses[i].pose.position.z = fixed_z;
	}
	return pathin;
}
void update_costmap(){
	set_internalstate("updating_costmap");
	float inflate_xy = 20.0;
	float inflate_z  = 1.5;
	target_active.pose.position.z  = state_costmap_elevation;
	target_active.pose.orientation = tf::createQuaternionMsgFromYaw(get_hdng(target_active.pose.position,pos));
	nav_msgs::Path path_endpoints = get_start_end_path(pos,target_active.pose.position);
	path_endpoints 								= set_path_z(path_endpoints,float(target_active.pose.position.z));
	std::vector<float> bbvec 			= inflate_bbvec(getinpath_boundingbox(path_endpoints),inflate_xy,inflate_z);
	p.x = (bbvec[0] + bbvec[3])/2;
	p.y = (bbvec[1] + bbvec[4])/2;
	p.z = target_active.pose.position.z;
	float dx = bbvec[3] - bbvec[0];
	float dy = bbvec[4] - bbvec[1];
	float sides = fmax(dx,dy);
	target_point.point  = target_active.pose.position;
	target_point.header = hdr();
	create_poly_volume(p,sides,bbvec[2],bbvec[5]);
	if(par_update_costmap_edto)
		pub_costmapupdate_edto.publish(poly_roi);
	else
		pub_costmapupdate_img.publish(poly_roi);
}
void costmap_elevation_cb(const std_msgs::UInt8::ConstPtr& msg){
	state_costmap_elevation = msg->data + int(round(par_zclearing+10));
	update_costmap();
	//if(state_internal == "updating_costmap_elevation"){

	//	ROS_ERROR("TARGETER: COSTMAP ELEAVTION (%i) while in states: [target: %s / internal: %s)]",msg->data,state_cmd.c_str(),state_internal.c_str());
//	}
//	else{
	//	ROS_ERROR("TARGETER NODE DID NOT EXPECT COSTMAP UPDATE(%i) while in states: [target: %s / internal: %s)]",msg->data,state_cmd.c_str(),state_internal.c_str());
//	}
}

void update_target_offset(){
	float vz,vx,vy,vxy,vxyz,vaz;
	float target_dst3d,target_dst2d,target_dz,target_dyaw;
	float dx = target_active.pose.position.x - pos.x;
	float dy = target_active.pose.position.y - pos.y;
	float dz = target_active.pose.position.z - pos.z;
	float target_active_yaw = tf::getYaw(target_active.pose.orientation);
	vz   = odom_global.twist.twist.linear.z;
	vxy  = sqrt(pow(odom_global.twist.twist.linear.x,2)+pow(odom_global.twist.twist.linear.y,2));
	vxyz = sqrt(pow(odom_global.twist.twist.linear.x,2)+pow(odom_global.twist.twist.linear.y,2)+pow(odom_global.twist.twist.linear.z,2));
	vaz  = odom_global.twist.twist.angular.z;
	vx = odom_global.twist.twist.linear.x;
	vy = odom_global.twist.twist.linear.y;
	float yaw_future = constrainAngle(pos_yaw + vaz * 1.5);

	target_dst3d = get_dst3d(target_active.pose.position,pos);
	target_dst2d = get_dst2d(target_active.pose.position,pos);
	target_dz 	 = target_active.pose.position.z-pos.z;
	target_dyaw  = get_shortest(target_active_yaw,pos_yaw);
	float time_vz = target_dst3d/vxyz;
	float time_vxy = target_dst2d/vxy;
	float time_vxyz = target_dyaw/vaz;
	float time_vaz = target_dz/vz;
	ROS_INFO("*******************************TARGET**************************************");
	ROS_INFO("											 [  xx   yy   zz   rad]");
	ROS_INFO("TARGET-vel[xyz][yaw]:  [%.0f %.0f %.0f][%.2f]",vx,vy,vz,vaz);
	ROS_INFO("TARGET-pos[xyz][yaw]:  [%.0f %.0f %.0f][%.2f]",pos.x,pos.y,pos.z,pos_yaw);
//	ROS_INFO("TARGET-ps1[xyz]: 			 [%.0f %.0f %.0f][%.2f]",fp.x,fp.y,fp.z,yaw_future);
	ROS_INFO("TARGET-tar[xyz][yaw]:  [%.0f %.0f %.0f][%.2f]",target_active.pose.position.x,target_active.pose.position.y,target_active.pose.position.z,target_active_yaw);
	ROS_INFO("TARGET-dst[xyz][yaw]:  [%.0f %.0f %.0f][%.2f]",dx,dy,dz,target_dyaw);
	ROS_INFO("TARGET-dst[3d,2d,dz][yaw]:  [%.0f %.0f %.0f][%.2f]",target_dst3d,target_dst2d,target_dz,target_dyaw);
	ROS_INFO("TARGET-sec[xyz][dt]:   [%.2f %.2f %.2f][%.2f]",time_vz,time_vxy,time_vxyz,time_vaz);
	ROS_INFO("*******************************TARGET**************************************");
	ROS_INFO("*******************************STATES**************************************");
	ROS_INFO("											[state (sec)]");

	float dt_internalstate = (ros::Time::now()-state_internal_change).toSec();
	float dt_targetsstate  = (ros::Time::now()-targetstate_change).toSec();
	float dt_state_cmd 		 = (ros::Time::now()-state_cmd_change).toSec();
	float dt_cmdmb 				 = (ros::Time::now()-cmdmb.header.stamp).toSec();
	ROS_INFO("TARGET-internalstate: [%s (%.3f sec)]",state_internal.c_str(),dt_internalstate);
	ROS_INFO("TARGET-targetsstate:  [%s (%.3f sec)]",state_target.c_str(),dt_targetsstate);
	ROS_INFO("TARGET-cmd_state:     [%s (%.3f sec)]",state_cmd.c_str(),dt_state_cmd);
	ROS_INFO("TARGET-cmd[xyz][yaw]: [%.2f %.2f 	0.0][%.2f][(%.3f sec)]",cmdmb.vector.x,cmdmb.vector.y,cmdmb.vector.z,dt_cmdmb);
	if(target_dst2d < 4.0)
		set_targetstate("idle");
}

void odomglobal_cb(const nav_msgs::Odometry::ConstPtr& msg){
  if((ros::Time::now() - forward_point.header.stamp).toSec() > 0.1){
    odom_global = *msg;
  }
}
void twist_cb(const geometry_msgs::Twist::ConstPtr& msg)
{
  cmdmb.header.stamp = ros::Time::now();
	cmdmb.vector.x 		 = msg->linear.x;
	cmdmb.vector.y 		 = msg->linear.y;
	cmdmb.vector.z 		 = msg->linear.z;
	if(state_cmd == "cmd_mb" && state_internal == "target_sent")
		set_targetstate("target_active");
}

float get_zmax_que(){
	float zmx = 0;
	float zmin_zum = 0;
	for(int i = 0; i < zmin_vec.size(); i++){
		zmin_zum += zmin_vec[i];
		if(zmx < zmin_vec[i])
			zmx = zmin_vec[i];
	}
	zmin_ave = zmin_zum / zmin_vec.size();
	return zmx;
}
void elevated_point_cb(const geometry_msgs::PointStamped::ConstPtr& msg){
	float zmin = msg->point.z;
	zmin_vec.push_back(msg->point.z);
	if(zmin_vec.size() > que_size)
		zmin_vec.erase(zmin_vec.begin());
	pos_z_min = get_zmax_que();
	ROS_INFO("pos_z_min: %.0f",pos_z_min);
}
void pathplan_failed_cb(const std_msgs::Empty::ConstPtr& msg){
	set_targetstate("idle");
}
void send_setpoint_yaw(float yaw){
	set_targetstate("setpoint");
	pub_target_setpoint_yaw.publish(setpoint_yaw);
}
void send_setpoint_z(float z){
	set_targetstate("setpoint");
	pub_target_setpoint_z.publish(setpoint_z);
}
void send_setpoint_xy(geometry_msgs::Point p){
	set_targetstate("setpoint");
	pub_target_setpoint_xy.publish(setpoint_xy);
}
void target_reception(geometry_msgs::PoseStamped cmd_pose){
	float cmd_pnt_yaw = tf::getYaw(cmd_pose.pose.orientation);
	float dxy  = get_dst2d(cmd_pose.pose.position,pos);
	float dyaw = get_shortest(cmd_pnt_yaw,pos_yaw);
	float dz   = cmd_pose.pose.position.z - pos.z;
	if(dxy < 2.0 && abs(dyaw) < 1.0)
		send_setpoint_z(dz);
	else if(abs(dz) < 1.0 && abs(dyaw) < 1.0 && dxy < 10.0)
		send_setpoint_xy(cmd_pose.pose.position);
	else if(dxy < 2.0 && abs(dz) < 1.0)
		send_setpoint_yaw(cmd_pnt_yaw);
	else{
		target_point_received.point = cmd_pose.pose.position;
		set_targetstate("target_received");
		send_elevation_request(pos,target_point_received.point);
	}
}
void targetpoint_cb(const geometry_msgs::PointStamped::ConstPtr& msg){
	if(get_dst2d(msg->point,pos) > 5.0){
		target_active.pose.position = msg->point;
		target_active.header = hdr();
		target_active.pose.orientation = tf::createQuaternionMsgFromYaw(get_hdng(target_active.pose.position,pos));
		target_reception(target_active);
	}
}
void targetpose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
	if(get_dst2d(msg->pose.position,pos) > 5.0){
		target_active = *msg;
		target_active.header = hdr();
		target_reception(target_active);
	}
}
void map_updates_cb(const map_msgs::OccupancyGridUpdate::ConstPtr& msg){
	if(state_cmd == "cmd_mb"){
		set_internalstate("target_sent");
		pub_target_mb.publish(target_active);
	}
	else if(state_cmd == "cmd_path"){
		set_internalstate("target_sent");
		pub_target_pathplan.publish(target_active);
	}
}
int main(int argc, char** argv)
{
  ros::init(argc, argv, "tb_autonomy_target_node");
  ros::NodeHandle nh;
	ros::NodeHandle private_nh("~");
	tf2_ros::TransformListener tf2_listener(tfBuffer);

	private_nh.param("use_edto_for_costmapupdate", par_update_costmap_edto, true);
	private_nh.param("safety_margin_z", par_zclearing, 5.0);

	ros::Subscriber s0 			= nh. subscribe("/tb_fsm/main_state",100,&mainstate_cb);
	ros::Subscriber sf 			= nh. subscribe("/cmd_vel",100,&twist_cb);

	pub_path_to_interpolate  = nh.advertise<nav_msgs::Path>("/tb_path/path_to_interpolate",100);
	pub_path_to_zsmooth      = nh.advertise<nav_msgs::Path>("/tb_path/path_to_zsmooth",100);
	pub_path_cmd      			 = nh.advertise<nav_msgs::Path>("/tb_cmd/path",100);

	ros::Subscriber pe = nh.subscribe("/tb_mb/path_plan",1,path_plan_cb);
	ros::Subscriber p0 = nh.subscribe("/tb_path/path_zsmoothed",1,path_zsmoothed_cb);
	ros::Subscriber p1 = nh.subscribe("/tb_path/path_interpolated",1,path_interpolated_cb);
	ros::Subscriber sa = nh.subscribe("/tb_cmd/path_active_index", 100,&pathcmd_i_cb);
	ros::Subscriber ds = nh.subscribe("/tb_cmd/path_active",  100,&path_cmd_cb);

	ros::Subscriber a0   = nh.subscribe("/tb_abmap/elevated_point",10,elevated_point_cb);
//	ros::Subscriber a1   = nh.subscribe("/tb_abmap/elevated_poly",10,elevated_poly_cb);
	ros::Subscriber a2   = nh.subscribe("/tb_abmap/elevated_path",10,elevated_path_cb);

	pub_roi_poly	  = nh.advertise<geometry_msgs::PolygonStamped>("/tb_autonomy/roi_poly",10);
	pub_roi2d_poly	= nh.advertise<geometry_msgs::PolygonStamped>("/tb_autonomy/roi_poly2d",10);
	ros::Publisher pub_target			= nh.advertise<geometry_msgs::PoseStamped>("/tb_autonomy/active_target",10);

	pub_elevate_poly	= nh.advertise<geometry_msgs::PolygonStamped>("/tb_abmap/get_elevation_poly",10);
	pub_elevate_point	= nh.advertise<geometry_msgs::PointStamped>("/tb_abmap/get_elevation_point",10);
	pub_elevate_path  = nh.advertise<nav_msgs::Path>("/tb_abmap/get_elevation_path",100);

	pub_costmapelevate_point = nh.advertise<geometry_msgs::PointStamped>("/tb_costmap/get_elevation_to_point",10);
	pub_costmapelevate_path	 = nh.advertise<nav_msgs::Path>("/tb_costmap/get_elevation_in_path",10);
	ros::Subscriber c0 		   = nh. subscribe("/tb_costmap/elevation",100,&costmap_elevation_cb);

	pub_costmapupdate_edto   = nh.advertise<geometry_msgs::PolygonStamped>("/tb_costmap/update_EDTO",10);
	pub_costmapupdate_img    = nh.advertise<geometry_msgs::PolygonStamped>("/tb_costmap/update_heightimg",10);
	ros::Subscriber u1 			= nh. subscribe("/map_updates",100,&map_updates_cb);
	ros::Subscriber t1   = nh.subscribe("/tb_autonomy/set_target_point",10,targetpoint_cb);
	ros::Subscriber t2   = nh.subscribe("/tb_autonomy/set_target_pose",10,targetpose_cb);

	pub_target_setpoint_yaw	= nh.advertise<std_msgs::Float64>("/tb_cmd/set_yaw",10);
	pub_target_setpoint_z   = nh.advertise<std_msgs::Float64>("/tb_cmd/set_z",10);
	pub_target_setpoint_xy  = nh.advertise<geometry_msgs::PointStamped>("/tb_cmd/set_xy",10);

	pub_target_pathplan 		= nh.advertise<geometry_msgs::PoseStamped>("/tb_plan/target_point",10);
	pub_target_mb 					= nh.advertise<geometry_msgs::PoseStamped>("/tb_cmd/mb_pose_target",10);
	ros::Subscriber pf  		= nh.subscribe("/tb_path/planning_failed",10,pathplan_failed_cb);
	pub_target_state  			= nh.advertise<std_msgs::String>("/tb_fsm/set_target_state",100);
	set_state_cmd("cmd_mb");
//	pub_path_closest_dst 	  = nh.advertise<std_msgs::Float64>("/tb_obstacles/path_closest_dst",10);
//	pub_point_closest_dst   = nh.advertise<std_msgs::Float64>("/tb_obstacles/point_closest_dst",10);
//	ros::Subscriber o2  = nh.subscribe("/tb_obstacles/get_path_closest_obstacle",1,get_path_closest_obstacle_cb);
//	ros::Subscriber o1  = nh.subscribe("/tb_obstacles/get_point_closest_obstacle",1,get_point_closest_obstacle_cb);

	ros::Rate rate(2.0);
  ros::Time last_radialupdate = ros::Time::now();
	int count = 0;

	ros::Time time_last = ros::Time::now();
	while(ros::ok()){
		geometry_msgs::PointStamped pos_front;
		pos_front.header = hdr();
		pos_front.point = pos;
		pub_elevate_point.publish(pos_front);
    rate.sleep();
    ros::spinOnce();
		update_pos_vlp();
		update_pos_cmd();
		update_target_offset();
	//	if(state_cmd == "cmd_mb")
	//		update_costmap();
		target_active.header.frame_id = "map";
		poly_roi.header.frame_id = "map";
		poly_roi2d.header.frame_id = "map";
		pub_target.publish(target_active);
		pub_roi_poly.publish(poly_roi);
		pub_roi2d_poly.publish(poly_roi2d);
  }
  return 0;
}
