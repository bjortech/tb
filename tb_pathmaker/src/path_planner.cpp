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
ros::Publisher pub_target_pathplan,pub_path_to_interpolate,pub_elevate_path,pub_path_to_zsmooth,pub_path_cmd;
nav_msgs::Path path_start_end;
ros::Time state_internal_change;
std::string state_internal = "idle";
void set_internalstate(std::string newstate){
  if(newstate != state_internal){
    float dt = (ros::Time::now() - state_internal_change).toSec();
    ROS_INFO("TARGETER: internal: %s -> %s (%.3f seconds in state)",state_internal,newstate,dt);
    state_internal_change = ros::Time::now();
    state_internal = newstate;
  }
}
void path_start_end_cb(const nav_msgs::Path::ConstPtr& msg){
	set_internalstate("received_costmap_ready");
	path_start_end = *msg;
	pub_target_pathplan.publish(path_start_end);
}
void path_plan_cb(const nav_msgs::Path::ConstPtr& msg){
	set_internalstate("interpolating_path");
	pub_path_to_interpolate.publish(*msg);
}
void path_interpolated_cb(const nav_msgs::Path::ConstPtr& msg){
	set_internalstate("elevating_path");
	pub_elevate_path.publish(*msg);
}
void elevated_path_cb(const nav_msgs::Path::ConstPtr& msg){
	set_internalstate("zsmoothing_path");
	pub_path_to_zsmooth.publish(*msg);
}
void path_zsmoothed_cb(const nav_msgs::Path::ConstPtr& msg){
	set_internalstate("path_cmd_sent");
	pub_path_cmd.publish(*msg);
}
void pathplan_failed_cb(const std_msgs::Empty::ConstPtr& msg){
	set_internalstate("idle");
	path_start_end.header.frame_id = "planning_failed";
	pub_path_cmd.publish(path_start_end);
}
int main(int argc, char** argv)
{
  ros::init(argc, argv, "tb_autonomy_pathplanner_node");
  ros::NodeHandle nh;
	ros::NodeHandle private_nh("~");
	ros::Subscriber pd = nh.subscribe("/tb_autonomy/get_pathplan_from_to",1,path_start_end_cb);
	ros::Subscriber pe = nh.subscribe("/tb_path/plan_from_to",1,path_plan_cb);
	ros::Subscriber p0 = nh.subscribe("/tb_path/path_zsmoothed",1,path_zsmoothed_cb);
	ros::Subscriber p1 = nh.subscribe("/tb_path/path_interpolated",1,path_interpolated_cb);
	ros::Subscriber a2 = nh.subscribe("/tb_abmap/elevated_path",10,elevated_path_cb);
	ros::Subscriber pf = nh.subscribe("/tb_path/plan_from_to_failed",10,pathplan_failed_cb);

	pub_target_pathplan 		 = nh.advertise<nav_msgs::Path>("/tb_path/get_plan_from_to",10);
	pub_elevate_path  			 = nh.advertise<nav_msgs::Path>("/tb_abmap/get_elevation_path",100);
	pub_path_to_interpolate  = nh.advertise<nav_msgs::Path>("/tb_path/path_to_interpolate",100);
	pub_path_to_zsmooth      = nh.advertise<nav_msgs::Path>("/tb_path/path_to_zsmooth",100);
	pub_path_cmd      			 = nh.advertise<nav_msgs::Path>("/tb_autonomy/path_planned",100);
	ros::spin();
	return 0;
}
