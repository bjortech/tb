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

ros::Publisher pub_set_z,pub_set_yaw,pub_set_xy,pub_get_poly,pub_get_side,pub_get_sphere,pub_get_down,pub_set_mbpose,pub_get_elevation,pub_costmapupdate_edto,pub_costmapupdate_img;
geometry_msgs::Point p,pos;
std::string state_target = "idle";
geometry_msgs::PointStamped setpoint_xy,new_target_pnt;
geometry_msgs::PoseStamped setpose;
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
ros::Time internalstate_change,targetstate_change,cmdstate_change;
std::string state_internal = "idle";
std::string state_cmd 	 = "idle";
std::string state_target = "idle";
geometry_msgs::PointStamped forward_point;


void set_cmdstate(std::string newstate){
  if(newstate != cmdstate){
    float dt = (ros::Time::now() - cmdstate_change).toSec();
    ROS_INFO("TARGETER: internal: %s -> %s (%.3f seconds in state)",cmdstate,newstate,dt);
    cmdstate_change = ros::Time::now();
    cmdstate = newstate;
  }
}
void set_internalstate(std::string newstate){
  if(newstate != internalstate_msg){
    float dt = (ros::Time::now() - internalstate_change).toSec();
    ROS_INFO("TARGETER: internal: %s -> %s (%.3f seconds in state)",internalstate_msg.data,newstate,dt);
    internalstate_change = ros::Time::now();
    internalstate = newstate;
  }
}
void set_targetstate(std::string newstate){
  if(newstate != targetstate_msg.data){
    float dt = (ros::Time::now() - targetstate_change).toSec();
    ROS_INFO("MAINSTATE: TARGET: %s -> %s (%.3f seconds in state)",targetstate_msg.data,newstate,dt);
    targetstate_change  	= ros::Time::now();
    targetstate_msg.data  = newstate;
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
void mainstate_cb(const std_msgs::UInt8::ConstPtr& msg){

}


void check_path_cmd(){
	if(path_cmd.poses.size() > path_cmd_i){
		set_targetstate("path_cmd_active");
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
void elevated_path_cb(const nav_msgs::Path::ConstPtr& msg){
	if(state_target == "path_cmd" && state_internal == "elevating_path"){
		set_internalstate("interpolating_path");
		pub_path_to_interpolate.publish(*msg);
	}
}
void path_interpolated_cb(const nav_msgs::Path::ConstPtr& msg){
	if(state_target == "path_cmd" && state_internal == "interpolating_path"){
		set_internalstate("zsmoothing_path");
		pub_path_to_zsmooth.publish(*msg);
	}
}
void path_zsmoothed_cb(const nav_msgs::Path::ConstPtr& msg){
	if(state_target == "path_cmd" && state_internal == "zsmoothing_path"){
		set_internalstate("path_cmd_sent");
		pub_path_cmd.publsih(*msg);
	}
}
void map_updates_cb(const map_msgs::OccupancyGridUpdate::ConstPtr& msg){
	if(state_internal == "updating_costmap"){
		if(state_cmd == "cmd_mb"){
			set_internalstate("target_sent");
			pub_target_mb.publish(target_point);
		}
		else if(state_cmd == "cmd_path"){
			set_internalstate("target_sent");
			pub_target_pathplan.publish(target_point);
		}
		else{
			ROS_ERROR("TARGETER NODE IN UNKNOWN STATE [%s]",state_cmd.c_str());
		}
	}
	//if(new_target_pnt.point.z > 0)
//		send_setpoint(new_target_pnt.point);
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
  bbvec[3] = bbvec[3] + inflation_z;
  bbvec[1] = bbvec[1] - inflation_xy;
  bbvec[4] = bbvec[4] + inflation_xy;
  bbvec[2] = bbvec[2] - inflation_xy;
  bbvec[5] = bbvec[5] + inflation_z;
  return bbvec;
}
void update_costmap(int costmap_elevation){
	set_internalstate("updating_costmap");
	inflate_xy = 10.0;
	inflate_z = 1.5;
	std::vector<float> bbvec = inflate_bbvec(getinpath_boundingbox(set_path_z(get_start_end_path(pos,target_point_received.point),float(costmap_elevation)),inflate_xy,inflate_z);
	bbvec[3] = pathin.poses[i].pose.position.x
	bbvec[4] = pathin.poses[i].pose.position.y
	bbvec[5] = pathin.poses[i].pose.position.z
	p.x = (bbvec[0] + bbvec[3])/2;
	p.y = (bbvec[1] + bbvec[4])/2;
	p.z = (bbvec[2] + bbvec[5])/2;
	flaot dx = bbvec[3] - bbvec[0]);
	float dy = bbvec[4] - bbvec[1]);
	float sides = fmax(dx,dy);
	create_poly_volume(p,sides,bbvec[2],bbvec[5]);
	if(par_update_costmap_edto)
		pub_costmapupdate_edto.publish(poly_roi);
	else
		pub_costmapupdate_img.publish(poly_roi);
}
void costmap_elevation_cb(const std_msgs::UInt8::ConstPtr& msg){
	last_costmap_elevation =
	ROS_ERROR("TARGETER: COSTMAP ELEAVTION (%i) while in states: [target: %s / internal: %s)]",msg->data,state_cmd.c_str(),state_internal.c_str());
	if(state_internal == "checking_costmap_elevation"){
		update_costmap(msg->data);
	}
	else{
		ROS_ERROR("TARGETER NODE DID NOT EXPECT UPDATE COSTMAP");
	}
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

	target_hdng  = get_hdng(target_active.pose.position,pos);
	target_dst3d = get_dst3d(target_active.pose.position,pos);
	target_dst2d = get_dst2d(target_active.pose.position,pos);
	target_dz 	 = target_active.pose.position.z-pos;
	target_dyaw  = get_shortest(target_active_yaw,pos_yaw);
	time_vz = target_dst3d/vxyz;
	time_vxy = target_dst2d/vxy;
	time_vxyz = target_dyaw/vaz;
	time_vaz = target_dz/vz;
	float vz,vxy,vxyz,vaz;
	float time_vz,time_vxy,time_vxyz,time_vaz;
	ROS_INFO("*******************************TARGET**************************************");
	ROS_INFO("											 [  xx   yy   zz   rad]");
	ROS_INFO("TARGET-vel[xyz][yaw]:  [%.0f %.0f %.0f][%.2f]",vx,vy,vz,vaz);
	ROS_INFO("TARGET-pos[xyz][yaw]:  [%.0f %.0f %.0f][%.2f]",pos.x,pos.y,pos.z,pos_yaw);
	ROS_INFO("TARGET-ps1[xyz]: 			 [%.0f %.0f %.0f][%.2f]",fp.x,fp.y,fp.z,yaw_future);
	ROS_INFO("TARGET-tar[xyz][yaw]:  [%.0f %.0f %.0f][%.2f]",target_active.pose.position.x,target_active.pose.position.y,target_active.pose.position.z,target_active_yaw);
	ROS_INFO("TARGET-dst[xyz][yaw]:  [%.0f %.0f %.0f][%.2f]",dx,dy,dz,target_active_yaw);
	ROS_INFO("TARGET-sec[xyz][dt]:   [%.2f %.2f %.2f][%.2f]",time_vz,time_vxy,time_vxyz,time_vaz);
	ROS_INFO("*******************************TARGET**************************************");
	ROS_INFO("*******************************STATES**************************************");
	ROS_INFO("											[state (sec)]");

	float dt_internalstate = (ros::Time::now()-internalstate_change).toSec();
	float dt_targetsstate  = (ros::Time::now()-targetsstate_change).toSec();
	float dt_cmdstate 		 = (ros::Time::now()-cmdstate_change).toSec();
	float dt_cmdmb 				 = (ros::Time::now()-cmdmb.header.stamp).toSec();
	ROS_INFO("TARGET-internalstate: [%s (%.3f sec)]",internalstate,dt_internalstate);
	ROS_INFO("TARGET-targetsstate:  [%s (%.3f sec)]",targetsstate,dt_targetsstate);
	ROS_INFO("TARGET-cmd_state:     [%s (%.3f sec)]",state_cmd,dt_cmdstate);
	ROS_INFO("TARGET-cmd[xyz][yaw]: [%.2f %.2f 	0.0][%.2f][(%.3f sec)]",cmdmb.vector.x,cmdmb.vector.y,cmdmb.vector.z,dt_cmdmb);
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
nav_msgs::Path set_path_z(nav_msgs::Path pathin,float fixed_z){
	for(int i = 0; i < pathin.poses.size(); i++){
		pathin.poses[i].pose.position.z = fixed_z;
	}
	return pathin;
}

nav_msgs::Path get_start_end_path(geometry_msgs::Point p0, geometry_msgs::Point p1){
	nav_msgs::Path pathout;
	path_to_elevate.poses.resize(2);
	pathout.poses[0].pose.position = p0;
	pathout.poses[1].pose.position = p1;
	pathout.poses[0].pose.orientation.w = 1.0;
	pathout.poses[1].pose.orientation.w = 1.0;
	pathout.header = pathout.poses[0].header = pathout.poses[1].header = hdr();
	return pathout;
}
void send_elevation_request(geometry_msgs::Point p0, geometry_msgs::Point p1){
	pub_get_elevation_path.publish(get_start_end_path(p0,p1));
}
void send_collision_check(geometry_msgs::Point p0, geometry_msgs::Point p1){
	pub_get_collision_path.publish(get_start_end_path(p0,p1));
}

void update_state(){
	if(state_target == "target_received" && state_internal == "idle"){
		if((state_cmd == "cmd_path") || state_cmd == "cmd_mb")){
			send_elevation_request(pos,target_point_received.point);
		}
		else if(state_cmd == "cmd_point"){
			send_collision_check(target_point_received.point);
		}
	}
	else if(state_target == "target_active" && state_cmd == "cmd_path")
		check_path_cmd();
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
void point_elevated_cb(const geometry_msgs::PointStamped::ConstPtr& msg){
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

void send_setpoint(geometry_msgs::Point pnt){
	float new_yaw = get_hdng(pnt,pos);
	setpose.header 					= hdr();
	setpoint_xy.header = hdr();
	setpose.pose.position 	= pnt;
	setpose.pose.position.z = 0;
	setpose.pose.orientation = tf::createQuaternionMsgFromYaw(new_yaw);
	setpoint_xy.point.x = pnt.x;
	setpoint_xy.point.y = pnt.y;
	setpoint_z.data 		= pnt.z;
	setpoint_yaw.data 	= new_yaw;
	new_target_pnt.point.z = 0;
	if(use_mb)
		pub_set_mbpose.publish(setpose);
}
void update_target_offset(geometry_msgs::PoseStamped targetpose_active){

}

void set_new_target(geometry_msgs::Point pos_xyz,float setpoint_yaw){
	set_targetstate("target_received");
}
void target_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
	target_received.point = msg->point;
	target_received_yaw   = tf::getYaw(msg->pose.orientation);
	set_targetstate("target_received");
}



	if(new_target_pnt.point.z == 0){
		new_target_pnt.point.z = msg->data;
		pub_costmapupdate_img.publish(poly_roi2d);
	}
}
void twist_cb(cons
ros::Subscriber s1  = nh.subscribe("/tb_abmap/elevated_path",10,elevated_path_cb);
ros::Subscriber s2  = nh.subscribe("/tb_path/path_interpolated",10,path_interpolated_cb);
ros::Subscriber s3  = nh.subscribe("/tb_path/path_zsmoothed",10,path_zsmoothed_cb);
ros::Subscriber s4  = nh.subscribe("/tb_path/planning_failed",10,pathplan_failed_cb);
ros::Subscriber s5  = nh.subscribe("/tb_obstacles/get_point_closest_obstacle",1,get_point_closest_obstacle_cb);

ros::Subscriber s4  = nh.subscribe("/tb_path/planning_failed",10,pathplan_failed_cb);
ros::Subscriber s5  = nh.subscribe("/tb_obstacles/get_point_closest_obstacle",1,get_point_closest_obstacle_cb);

pub_path_to_zsmooth     	 = nh.advertise<nav_msgs::Path>("/tb_obstacles/get_path_closest_dst",100);
pub_path_to_zsmooth      	 = nh.advertise<nav_msgs::Path>("/tb_obstacles/get_path_closest_obstacle",100);

pub_point_closest_obstacle = nh.advertise<geometry_msgs::PointStamped>("/tb_obstacles/get_path_closest_dst",10);
pub_point_closest_obstacle = nh.advertise<geometry_msgs::PointStamped>("/tb_obstacles/get_point_closest_dst",10);

ros::Subscriber s4  = nh.subscribe("/tb_obstacles/get_path_closest_obstacle",1,get_path_closest_obstacle_cb);
ros::Subscriber s5  = nh.subscribe("/tb_obstacles/get_point_closest_obstacle",1,get_point_closest_obstacle_cb);


pub_path_closest_dst 	  = nh.advertise<std_msgs::Float64>("/tb_obstacles/path_closest_dst",10);
pub_point_closest_dst   = nh.advertise<std_msgs::Float64>("/tb_obstacles/point_closest_dst",10);

pub_point_closest_obstacle = nh.advertise<geometry_msgs::PointStamped>("/tb_obstacles/point_closest_obstacle",10);
pub_path_closest_obstacle  = nh.advertise<geometry_msgs::PointStamped>("/tb_obstacles/path_closest_obstacle",10);

int main(int argc, char** argv)
{
  ros::init(argc, argv, "tb_autonomy_node");
  ros::NodeHandle nh;
	ros::NodeHandle private_nh("~");
	tf2_ros::TransformListener tf2_listener(tfBuffer);

	ros::Subscriber s0 			= nh. subscribe("/tb_fsm/main_state",100,&mainstate_cb);
	ros::Subscriber s1 			= nh. subscribe("/tb_mbmap/costmap_elevation",100,&costmap_elevation_cb);
	ros::Subscriber s2 			= nh. subscribe("/map_updates",100,&map_updates_cb);

	ros::Subscriber s5 			= nh.subscribe("/tb_cmd/path_active_index", 100,&pathcmd_i_cb);
	ros::Subscriber s6 			= nh.subscribe("/tb_cmd/path_active",  100,&path_cmd_cb);
	ros::Subscriber s4 = nh.subscribe("/tb_world/path_visited",10,vstd_cb);

	pub_target_setpoint_yaw = nh.advertise<std_msgs::Float64>("/tb_cmd/set_z",10);
	pub_target_setpoint 		= nh.advertise<std_msgs::Float64>("/tb_cmd/set_yaw",10);
	pub_target_setpoint_z   = nh.advertise<geometry_msgs::PointStamped>("/tb_cmd/set_xy",10);
	pub_target_pathplan 		= nh.advertise<geometry_msgs::PointStamped>("/tb_plan/target_point",10);

	pub_target_mb 					= nh.advertise<geometry_msgs::PoseStamped>("/tb_md/pose_target",10);

	pub_path_to_elevate      = nh.advertise<nav_msgs::Path>("/tb_abmap/get_elevation_path",100);
	pub_path_to_interpolate  = nh.advertise<nav_msgs::Path>("/tb_path/path_to_interpolate",100);
	pub_path_to_zsmooth      = nh.advertise<nav_msgs::Path>("/tb_path/path_to_zsmooth",100);

	ros::Publisher pub_roi_poly	  = nh.advertise<geometry_msgs::PolygonStamped>("/tb_roi/poly",10);
	ros::Publisher pub_roi2d_poly	= nh.advertise<geometry_msgs::PolygonStamped>("/tb_roi/poly2d",10);

	pub_get_poly	  = nh.advertise<geometry_msgs::PolygonStamped>("/tb_edto/get_poly",10);
	pub_get_side	  = nh.advertise<geometry_msgs::PolygonStamped>("/tb_edto/get_side",10);
	pub_get_sphere	= nh.advertise<geometry_msgs::PolygonStamped>("/tb_edto/get_sphere",10);
	pub_get_down		= nh.advertise<geometry_msgs::PolygonStamped>("/tb_edto/get_down",10);

	pub_get_elevation_path  = nh.advertise<nav_msgs::Path>("/tb_mbmap/get_elevation_in_path",10);
	pub_get_elevation	 			= nh.advertise<geometry_msgs::PointStamped>("/tb_mbmap/get_elevation_to_point",10);
	pub_costmapupdate_edto  = nh.advertise<geometry_msgs::PolygonStamped>("/tb_mbmap/costmap_update_EDTO",10);
	pub_costmapupdate_img   = nh.advertise<geometry_msgs::PolygonStamped>("/tb_mbmap/costmap_update_heightimg",10);

	ros::Publisher pub_point_to_elevate	= nh.advertise<geometry_msgs::PointStamped>("/tb_abmap/get_elevation_point",10);

	ros::Subscriber s1   = nh.subscribe("/tb_autonomy/target_point",10,point_elevated_cb);
	ros::Subscriber s2   = nh.subscribe("/tb_fsm/target_point",10,point_elevated_cb);
	ros::Subscriber s3   = nh.subscribe("/tb_abmap/elevated_point",10,point_elevated_cb);

	ros::Rate rate(2.0);
  ros::Time last_radialupdate = ros::Time::now();
	int count = 0;

	ros::Time time_last = ros::Time::now();
	while(ros::ok()){
		pub_point_to_elevate.publish(pos_front);
    rate.sleep();
    ros::spinOnce();

		update_pos_vlp();
		geometry_msgs::Point p0;

		if(count == 1)
			pub_get_poly.publish(poly_roi);
		else if(count == 2)
			pub_get_side.publish(poly_roi);
		else if(count == 3)
			pub_get_down.publish(poly_roi);
		else if(count == 4)
			pub_get_sphere.publish(poly_roi);
		else
			count = 0;
		count++;
		if(!use_mb){
			if(pos_z_min < 3)
				pos_z_min = 10.0;
			setpoint_z.data = pos_z_min + 5;
			pub_set_z.publish(setpoint_z);
			pub_set_yaw.publish(setpoint_yaw);
			pub_set_xy.publish(setpoint_xy);
		}
		drawimg();
		pub_roi_poly.publish(poly_roi);
		pub_roi2d_poly.publish(poly_roi2d);
  }
  return 0;
}
