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
#include "path_smoothing_ros/cubic_spline_interpolator.h"

ros::Publisher pub_path;

////////////////////////////////////////////****************/////////////////////////////////////////////////////
////////////////////////////////////////////DEBUG-IMGDRAWING/////////////////////////////////////////////////////
////////////////////////////////////////////****************/////////////////////////////////////////////////////
bool par_debug_img;
int count_target_paths = 0;
cv::Mat img(1000,1000,CV_8UC3,cv::Scalar(0, 0, 0)); //create image, set encoding and size, init pixels to default val
cv::Mat img_blank(1000,1000,CV_8UC3,cv::Scalar(0, 0, 0)); //create image, set encoding and size, init pixels to default val
float par_res = 1.0;
float y2r(float y){
  return (img.rows / 2 - y / par_res);
}
float x2c(float x){
  return (x / par_res + img.cols/2);
}
cv::Point pnt2cv(geometry_msgs::Point pin){
	int c = x2c(pin.x);
	int r = y2r(pin.y);
	return cv::Point(c,r);
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
void draw_rectangle(geometry_msgs::Point midpoint,float size, cv::Scalar color){
  geometry_msgs::Point p0,p1;
  p0.x = midpoint.x-size; p1.x = midpoint.x+size*2;
  p0.y = midpoint.y-size; p1.y = midpoint.y+size*2;
  cv::rectangle(img, pnt2cv(p0),pnt2cv(p1),color,1,8,0);
}
float get_path_dst_hdng(nav_msgs::Path pathin,bool is_needing_heading){
	float dst_sum  = 0;
	float hdng_sum = 0;
	for(int i = 1; i < pathin.poses.size(); i++){
		dst_sum  += get_dst2d(pathin.poses[i-1].pose.position,pathin.poses[i].pose.position);
		float hdng = get_hdng(pathin.poses[i-1].pose.position,pathin.poses[i].pose.position);
		if(hdng < 0)
			hdng *= -1;
		hdng_sum += hdng;
	}
	if(is_needing_heading)
		return hdng_sum;
	else
		return dst_sum;
}
////////////////////////////////////////////****************/////////////////////////////////////////////////////
////////////////////////////////////////////DEBUG-IMGDRAWING/////////////////////////////////////////////////////
////////////////////////////////////////////****************/////////////////////////////////////////////////////

nav_msgs::Path check_path(nav_msgs::Path pathin,int maprad_max){
  int startsize = pathin.poses.size();
  for(int i = pathin.poses.size()-1; i > 0; i--){
    geometry_msgs::Point pnt = pathin.poses[i].pose.position;
    if(std::isnan(pnt.x) || std::isnan(pnt.y) || std::isnan(pnt.z) || (abs(pnt.x) > maprad_max) || (abs(pnt.y) > maprad_max)){
      ROS_INFO("PATHCREATOR: WARNING i: %i x: %.0f y: %.0f",i,pnt.x,pnt.y);
      pathin.poses.erase(pathin.poses.begin()+i);
    }
  }
  int change = startsize - pathin.poses.size();
//  ROS_INFO("PATHCREATOR: Path checked - %i/%i poses removed",change,startsize);
  return pathin;
}
nav_msgs::Path interpolate_path(nav_msgs::Path pathin){
  nav_msgs::Path smoothedPath;
  // create a cubic spline interpolator
  path_smoothing::CubicSplineInterpolator csi("lala");
    // pointsPerUnit, skipPoints, useEndConditions, useMiddleConditions);
  csi.setPointsPerUnit(1.0);//(double ppu) {pointsPerUnit_ = ppu;}
  csi.setSkipPoints(0);//(unsigned int sp) {skipPoints_ = sp;}
  csi.setUseEndConditions(false); //(bool uec) {useEndConditions_ = uec;}
  csi.setUseMiddleConditions(false);//(bool umc) {useMiddleConditions_ = umc;}
  csi.interpolatePath(pathin, smoothedPath);
	smoothedPath.header.frame_id = "map";
	smoothedPath.header.stamp = ros::Time::now();
  ROS_INFO("PATHCREATOR: SMoothed path: %i -> %i poses",pathin.poses.size(),smoothedPath.poses.size());
  return smoothedPath;
}

void path_to_check_cb(const nav_msgs::Path::ConstPtr& msg){
	if(msg->poses.size() > 10){
		nav_msgs::Path smooth_path = interpolate_path(*msg);
		pub_path.publish(check_path(*msg,500));
	}
}
//CHECK PATH - FIXING NANS

int main(int argc, char** argv)
{
  ros::init(argc, argv, "tb_pathinterpolator_node");
  ros::NodeHandle nh;
	ros::NodeHandle private_nh("~");
	ros::Subscriber s2  = nh.subscribe("/tb_path/path_to_interpolate",1,path_to_check_cb);
  pub_path 					  = nh.advertise<nav_msgs::Path>("/tb_path/path_interpolated",10);
	ros::spin();

  return 0;
}
