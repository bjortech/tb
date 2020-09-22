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
#include <tb_msgsrv/Paths.h>
#include <tb_msgsrv/Polygons.h>

ros::Publisher pub_path_to_cluster,pub_paths_to_bbpoly;

tb_msgsrv::Polygons paths_bbpolys;
tb_msgsrv::Paths paths_clusters;

void path_clusters_cb(const tb_msgsrv::Paths::ConstPtr& msg){
	paths_clusters = *msg;
	pub_paths_to_bbpoly.publish(paths_clusters);
}
void path_bbpolys_cb(const tb_msgsrv::Polygons::ConstPtr& msg){
	paths_bbpolys = *msg;
}
void path_to_segment_cb(const nav_msgs::Path::ConstPtr& msg){
	pub_path_to_cluster.publish(*msg);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "tb_process_client_down_node");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");
	ros::Subscriber os2 = nh.subscribe("/tb_process/path_down_clusters",1,path_clusters_cb);
	ros::Subscriber os1 = nh.subscribe("/tb_process/paths_down_bbpolys",1,path_bbpolys_cb);

	pub_path_to_cluster = nh.advertise<nav_msgs::Path>("/tb_process/get_path_down_clusters",10);
	pub_paths_to_bbpoly = nh.advertise<tb_msgsrv::Paths>("/tb_process/get_paths_down_bbpolys",10);
	ros::spin();
	return 0;
}
//
