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

geometry_msgs::PointStamped point_elevated;
geometry_msgs::PolygonStamped poly_elevated;
nav_msgs::Path path_elevated;
ros::Publisher pub_poly_to_elevate,pub_point_to_elevate,pub_path_to_elevate;

void point_elevated_cb(const geometry_msgs::PointStamped::ConstPtr& msg){
	point_elevated = *msg;
}
void poly_elevated_cb(const geometry_msgs::PolygonStamped::ConstPtr& msg){
	poly_elevated = *msg;
}
void path_elevated_cb(const nav_msgs::Path::ConstPtr& msg){
	path_elevated = *msg;
}
int main(int argc, char** argv)
{
  ros::init(argc, argv, "tb_abmap_elevator_client_node");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");
	private_nh.param("resolution", par_res, 1.0);
	private_nh.param("inflation_radius", par_inflation, 3.0);

  ros::Subscriber s01 = nh.subscribe("/tb_fsm/main_state",10,mainstate_cb);
	pub_point_to_elevate = nh.advertise<geometry_msgs::PointStamped>("/tb_abmap/get_elevation_point",10);
	pub_poly_to_elevate	 = nh.advertise<geometry_msgs::PolygonStamped>("/tb_abmap/get_elevation_poly",10);
	pub_path_to_elevate  = nh.advertise<nav_msgs::Path>("/tb_abmap/get_elevation_path",10);
	ros::Subscriber s2   = nh.subscribe("/tb_abmap/elevated_poly",10,poly_elevated_cb);
	ros::Subscriber s3   = nh.subscribe("/tb_abmap/elevated_point",10,point_elevated_cb);
	ros::Subscriber s4   = nh.subscribe("/tb_abmap/elevated_path",10,path_elevated_cb);
	draw_height_atstart(5,1);
  ros::spin();


  return 0;
}
//
