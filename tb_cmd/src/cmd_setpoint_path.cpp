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
geometry_msgs::TransformStamped tff,tfh,tfrp,tfm,tfo,tfoa,tfa,tfa2,tfp,tfpos,tfb;
geometry_msgs::Vector3 vlp_rpy;
nav_msgs::Odometry odom,odom_global;
ros::Time time_last_v;
std_msgs::Float64 pz;
double yaw_odom;
tf2_ros::Buffer tfBuffer;
bool active_path;
nav_msgs::Path path_cmd,path_in;
int path_cmd_i = 0;
ros::Time path_start;
float poses_pr_sec   = 1.0;
float meters_pr_pose = 0.0;
float meters_pr_sec  = 8.0;
float poses_pr_meter = 0.0;
float current_dst = 0;
double par_zclearing;
geometry_msgs::Point setpoint,refpoint,pos;
std_msgs::UInt8 pathindex;

double get_shortest(double target_yaw,double actual_yaw){
  double a = target_yaw - actual_yaw;
  if(a > M_PI)a -= M_PI*2;
  else if(a < -M_PI)a += M_PI*2;
  return a;
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
	float h = atan2(dy,dx);
	if(std::isnan(h) || abs(h) > 7)
		h = 0.0;
  return h;
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
void update_pos(){
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
	if(path_cmd.poses.size() > path_cmd_i+10){
		int closest = getinpath_closestindex2d(path_cmd,pos);
		ROS_INFO("Closest: %i ",closest);
		float target_dst = 5.0;
		int i = 0;
		float dst = 0;
		while(dst < target_dst && (closest+i)<path_cmd.poses.size()-1){
			i++;
		//	ROS_INFO("closest+i: %i+%i / %i dst: %.0f",closest,i,path_cmd.poses.size(),dst);
			dst = get_dst3d(path_cmd.poses[closest+i].pose.position,pos);
		}
		if(closest+i >= path_cmd.poses.size()-1)
			path_cmd_i = path_cmd.poses.size()-1;
		else
			path_cmd_i = closest+i;
		//ROS_INFO("Path cmd i: %i",path_cmd_i);
		setpoint = path_cmd.poses[path_cmd_i].pose.position;
		pathindex.data = path_cmd_i;
	}
	else if(path_cmd.poses.size() > path_cmd_i){
		path_cmd_i = path_cmd.poses.size();
		setpoint = path_cmd.poses[path_cmd_i].pose.position;
		pathindex.data = path_cmd_i;
	}
}
void path_cb(const nav_msgs::Path::ConstPtr& msg){
	int ci     = getinpath_closestindex2d(*msg,pos);
	path_cmd_i = 0;
	path_cmd.poses.resize(0);
	for(int i = ci; i < msg->poses.size(); i++){
		path_cmd.poses.push_back(msg->poses[i]);
	}
	update_pos();
}
int main(int argc, char **argv){
  ros::init(argc, argv, "tb_cmd_setpoint_path_node");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");
  ros::Rate rate(50);

  tf2_ros::TransformListener tf2_listener(tfBuffer);

  tf2_ros::TransformBroadcaster tf_b;

  tfo.header.frame_id           = "odom";
  tfo.child_frame_id            = "base_perfect";
  tfo.transform.rotation.w      = 1;

  tfoa.header.frame_id          = "base_perfect";
  tfoa.child_frame_id           = "base_perfect_alt";
  tfoa.transform.rotation.w     = 1;

  ros::Subscriber s1           = nh.subscribe("/tb_cmd/path", 100,&path_cb);
  ros::Publisher pub_pathindex = nh.advertise<std_msgs::UInt8>("/tb_cmd/path_active_index",10);
	ros::Publisher pub_path      = nh.advertise<nav_msgs::Path>("/tb_cmd/path_active",10);
	ros::Publisher pub_pnt       = nh.advertise<geometry_msgs::PointStamped>("/tb_cmd/path_active_point",10);

  tfoa.transform.translation.z = 10;
	bool use_distance = false;
	ros::Time last_change_i;
	geometry_msgs::PointStamped setpoint_msg;

  while(ros::ok()){
		update_pos();
		if(pathindex.data < path_cmd.poses.size()){
			tfo.transform.translation.x  = setpoint.x;
			tfo.transform.translation.y  = setpoint.y;
			tfoa.transform.translation.z = setpoint.z;
			tfo.transform.rotation       = tf::createQuaternionMsgFromYaw(get_hdng(setpoint,pos));
			setpoint_msg.header.stamp = tfo.header.stamp = tfoa.header.stamp = ros::Time::now();
			tf_b.sendTransform(tfoa);
			tf_b.sendTransform(tfo);
			path_cmd.header.frame_id = "map";
			setpoint_msg.header  = path_cmd.header;
			setpoint_msg.point   = setpoint;
			setpoint_msg.point.z = tfoa.transform.translation.z;
			pub_pnt.publish(setpoint_msg);
		}
		else{
			path_cmd.poses.resize(0);
			pathindex.data = 0;
			pub_path.publish(path_cmd);
			pub_pathindex.publish(pathindex);
		}
    rate.sleep();
    ros::spinOnce();
  }
  return 0;
}
