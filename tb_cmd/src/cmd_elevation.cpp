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
ros::Publisher pub_elevate_point,pub_elevate_path,pub_path_closest_dst,pub_point_closest_dst;
geometry_msgs::Point pos;
geometry_msgs::PoseStamped target_active;
float pos_yaw;
tf2_ros::Buffer tfBuffer;
nav_msgs::Path path_cmd;
std::vector<float> zmin_vec;
float zmin_ave;
int que_size = 10;
float pos_z_min = 5.0;
geometry_msgs::PointStamped pos_front;
geometry_msgs::Vector3 vlp_rpy;
int path_cmd_i = 0;
nav_msgs::Path path_visited,path_unknown,path_active_in,path_active_elevated;
std_msgs::Float64 zmin_msg;
double par_zclearing;
bool everyother;
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
	//ROS_INFO("pos_z_min: %.0f",pos_z_min);
}

float get_zmx_path(nav_msgs::Path pathin){
	float zmn = 0;
	for(int i = 0; i < fmin(pathin.poses.size(),15); i++){
		if(pathin.poses[i].pose.position.z > zmn)
			zmn =pathin.poses[i].pose.position.z;
	}
	return zmn;
}
void elevated_path_cb(const nav_msgs::Path::ConstPtr& msg){
	path_active_elevated = *msg;
}
void global_plan_cb(const nav_msgs::Path::ConstPtr& msg){
	pub_elevate_path.publish(*msg);
}

void update_pos(){
	geometry_msgs::TransformStamped transformStamped;

	try{
		if(everyother){
			everyother = false;
			transformStamped = tfBuffer.lookupTransform("map","base_perfect",
															 ros::Time(0));
		}
		else{
			everyother = true;
			transformStamped = tfBuffer.lookupTransform("map","base_stabilized",
															 ros::Time(0));
		}

	}
	catch (tf2::TransformException &ex) {
		ROS_WARN("%s",ex.what());
		ros::Duration(1.0).sleep();
	}
	geometry_msgs::PointStamped cmd_elev_point;
	cmd_elev_point.header.frame_id = "map";
	cmd_elev_point.header.stamp = ros::Time::now();
	cmd_elev_point.point.x = transformStamped.transform.translation.x;
	cmd_elev_point.point.y = transformStamped.transform.translation.y;
	cmd_elev_point.point.z = transformStamped.transform.translation.z;
	pub_elevate_point.publish(cmd_elev_point);
}
int main(int argc, char** argv)
{
  ros::init(argc, argv, "tb_cmd_elevation_node");
  ros::NodeHandle nh;
	ros::NodeHandle private_nh("~");
	private_nh.param("safety_margin_z", par_zclearing, 5.0);

	tf2_ros::TransformListener tf2_listener(tfBuffer);
	pub_elevate_point	 = nh.advertise<geometry_msgs::PointStamped>("/tb_abmap/get_elevation_point",10);
	pub_elevate_path   = nh.advertise<nav_msgs::Path>("/tb_abmap/get_elevation_path",100);

	ros::Subscriber a2  = nh.subscribe("/tb_abmap/elevated_path",10,elevated_path_cb);
	ros::Subscriber a0  = nh.subscribe("/tb_abmap/elevated_point", 10,elevated_point_cb);
	ros::Subscriber sm1 = nh.subscribe("/move_base/NavfnROS/plan",100,&global_plan_cb);
	ros::Publisher pub_target_setpoint_z = nh.advertise<std_msgs::Float64>("/tb_cmd/set_z",10);
	ros::Rate rate(2.0);

	ros::Time time_last = ros::Time::now();
	while(ros::ok()){
		rate.sleep();
    ros::spinOnce();
		update_pos();
		zmin_msg.data = fmax(pos_z_min,get_zmx_path(path_active_elevated)) + par_zclearing;
		pub_target_setpoint_z.publish(zmin_msg);
  }
  return 0;
}
