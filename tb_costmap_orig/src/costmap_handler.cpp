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
ros::Publisher pub_costmapupdated,pub_pathplan_from_to,pub_path_cmd,pub_target_mb,pub_elevate_point,pub_target_state,pub_target_setpoint_yaw,pub_target_setpoint_z,pub_target_setpoint_xy;
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
void mainstate_cb(const std_msgs::UInt8::ConstPtr& msg){
	mainstate = msg->data;
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
		target_point.point = cmd_pose.pose.position;
		target_point.header = hdr();
		pub_costmapupdated.publish(target_point);
	}
}
void send_target_active(){
	set_internalstate("target_sent");
	pub_target_mb.publish(target_active);
}
void costmapupdated_cb(const nav_msgs::Path::ConstPtr& msg){
	if(msg->header.frame_id == "failed")
		set_internalstate("idle");
	else{
		if(state_cmd == "cmd_mb"){
			send_target_active();
		}
		else if(state_cmd == "cmd_path"){
			nav_msgs::Path from_to;
			from_to.poses.resize(2);
			from_to.header = from_to.poses[0].header = from_to.poses[1].header = hdr();
			from_to.poses[0].pose.position = pos;
			from_to.poses[0].pose.orientation.w = 1.0;
			from_to.poses[1] = target_active;
			set_internalstate("target_sent");
			pub_pathplan_from_to.publish(from_to);
		}
	}
}
void pathplanned_cb(const nav_msgs::Path::ConstPtr& msg){
	if(msg->header.frame_id == "failed"){
	//	set_internalstate("idle");
		send_target_active();
	}
	else{
		pub_path_cmd.publish(*msg);
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


int main(int argc, char** argv)
{
  ros::init(argc, argv, "tb_autonomy_handler_node");
  ros::NodeHandle nh;
	ros::NodeHandle private_nh("~");
	tf2_ros::TransformListener tf2_listener(tfBuffer);
	pub_costmapupdated   = nh.advertise<geometry_msgs::PointStamped>("/tb_autonomy/update_costmap_to_point",10);
	ros::Subscriber t1   = nh.subscribe("/tb_autonomy/costmap_updated",10,costmapupdated_cb);
	pub_pathplan_from_to = nh.advertise<nav_msgs::Path>("/tb_autonomy/get_pathplan_from_to",10);
	ros::Subscriber pf 	 = nh.subscribe("/tb_autonomy/path_planned",10,pathplanned_cb);
	pub_path_cmd      	 = nh.advertise<nav_msgs::Path>("/tb_cmd/path",100);
	pub_target_mb 			 = nh.advertise<geometry_msgs::PoseStamped>("/tb_cmd/mb_pose_target",10);

	ros::Subscriber s0 			= nh. subscribe("/tb_fsm/main_state",100,&mainstate_cb);
	ros::Subscriber sf 			= nh. subscribe("/cmd_vel",100,&twist_cb);

//	ros::Subscriber sa = nh.subscribe("/tb_cmd/path_active_index", 100,&pathcmd_i_cb);
	//ros::Subscriber ds = nh.subscribe("/tb_cmd/path_active",  100,&path_cmd_cb);

//	ros::Subscriber a0   = nh.subscribe("/tb_abmap/elevated_point",10,elevated_point_cb);


//	pub_elevate_point	= nh.advertise<geometry_msgs::PointStamped>("/tb_abmap/get_elevation_point",10);

	ros::Subscriber t21   = nh.subscribe("/tb_autonomy/set_target_point",10,targetpoint_cb);
	ros::Subscriber t3   = nh.subscribe("/tb_autonomy/set_target_pose",10,targetpose_cb);

	pub_target_state  			= nh.advertise<std_msgs::String>("/tb_fsm/set_target_state",100);

	pub_target_setpoint_yaw		= nh.advertise<std_msgs::Float64>("/tb_cmd/set_yaw",10);
	pub_target_setpoint_z   	= nh.advertise<std_msgs::Float64>("/tb_cmd/set_z",10);
	pub_target_setpoint_xy  	= nh.advertise<geometry_msgs::PointStamped>("/tb_cmd/set_xy",10);
	ros::Publisher pub_target	= nh.advertise<geometry_msgs::PoseStamped>("/tb_autonomy/active_target",10);

	set_state_cmd("cmd_path");
//	pub_path_closest_dst 	  = nh.advertise<std_msgs::Float64>("/tb_obstacles/path_closest_dst",10);
//	pub_point_closest_dst   = nh.advertise<std_msgs::Float64>("/tb_obstacles/point_closest_dst",10);
//	ros::Subscriber o2  = nh.subscribe("/tb_obstacles/get_path_closest_obstacle",1,get_path_closest_obstacle_cb);
//	ros::Subscriber o1  = nh.subscribe("/tb_obstacles/get_point_closest_obstacle",1,get_point_closest_obstacle_cb);

	ros::Rate rate(2.0);

	ros::Time time_last = ros::Time::now();
	while(ros::ok()){
    rate.sleep();
    ros::spinOnce();
		update_pos_vlp();
		update_pos_cmd();
		update_target_offset();
		target_active.header.frame_id = "map";
		pub_target.publish(target_active);
  }
  return 0;
}
