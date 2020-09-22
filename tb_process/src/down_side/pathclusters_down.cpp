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

cv::Mat img_paths(1000,1000,CV_8U,cv::Scalar(0)); //create image, set encoding and size, init pixels to default val
cv::Mat imb_blank_mono(1000,1000,CV_8U,cv::Scalar(0)); //create image, set encoding and size, init pixels to default val
ros::Publisher pub_paths;
float y2r(float y){
  return (img_paths.rows / 2 - y);
}
float x2c(float x){
  return (x + img_paths.cols/2);
}
int r2y(float r){
  return int((img_paths.rows / 2 - r));
}
int c2x(float c){
  return int((c - img_paths.cols / 2));
}
int floodfill(int r0, int c0){
	int ffillMode = 1;  int connectivity = 8;  int newMaskVal = 255;
	int flags = connectivity + (newMaskVal << 8) +
			 (ffillMode == 1 ? cv::FLOODFILL_FIXED_RANGE : 0);
	cv::Rect ccomp;
	int area = cv::floodFill(img_paths,cv::Point(c0,r0),cv::Scalar(150), &ccomp, cv::Scalar(1),cv::Scalar(1), flags);
	return area;
}
void colorize_pathimg(nav_msgs::Path pathin){
	imb_blank_mono.copyTo(img_paths);
	for(int i = 0; i < pathin.poses.size(); i++){
		img_paths.at<uchar>(y2r(pathin.poses[i].pose.position.y),x2c(pathin.poses[i].pose.position.x)) = 100;
	}
}
tb_msgsrv::Paths get_clusters(nav_msgs::Path pathin){
	ros::Time t0 = ros::Time::now();
	tb_msgsrv::Paths pathsout;
	nav_msgs::Path path_remaining,path_cluster;
	path_remaining = pathin;
	colorize_pathimg(pathin);
	while(path_remaining.poses.size() > 5){
		int area = floodfill(y2r(path_remaining.poses[0].pose.position.y),x2c(path_remaining.poses[0].pose.position.x));
		nav_msgs::Path path_remaining_new;
		for(int i = 0; i < path_remaining.poses.size(); i++){
			if(img_paths.at<uchar>(y2r(path_remaining.poses[i].pose.position.y),x2c(path_remaining.poses[i].pose.position.x)) == 150)
				path_cluster.poses.push_back(pathin.poses[i]);
			else
				path_remaining_new.poses.push_back(pathin.poses[i]);
		}
		path_remaining = path_remaining_new;
		path_cluster.header.frame_id = "map";
		pathsout.paths.push_back(path_cluster);
		path_cluster.poses.resize(0);
	}
	float dt = (ros::Time::now()-t0).toSec();
	ROS_INFO("PATHPROCESS: %i poses segmented into %i clusters in %.4 sec",pathin.poses.size(),pathsout.paths.size(),dt);
	return pathsout;
}
void path_to_segment_cb(const nav_msgs::Path::ConstPtr& msg){

	pub_paths.publish(get_clusters(*msg));
}
int main(int argc, char** argv)
{
  ros::init(argc, argv, "tb_process_clusters_down_node");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");
	ros::Subscriber os = nh.subscribe("/tb_process/get_path_down_clusters",1,path_to_segment_cb);
	pub_paths = nh.advertise<tb_msgsrv::Paths>("/tb_process/path_down_clusters",10);
	ros::spin();
	return 0;
}
//
