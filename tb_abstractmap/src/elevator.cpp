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

cv::Mat img(1000,1000,CV_8UC3,cv::Scalar(0, 0, 0)); //create image, set encoding and size, init pixels to default val
cv::Mat img_blank(1000,1000,CV_8UC3,cv::Scalar(0, 0, 0)); //create image, set encoding and size, init pixels to default val
cv::Mat img_height(1000,1000,CV_8U,cv::Scalar(0)); //create image, set encoding and size, init pixels to default val
///********FRONTIER*************////////
int z_highest = 0;
int mainstate = 0;
double par_res;
ros::Publisher pub_elevated_poly,pub_elevated_point,pub_elevated_path;
///********FRONTIER*************////////
double par_inflation;
std_msgs::Header hdr(){
  std_msgs::Header header;
  header.frame_id = "map";
  header.stamp    = ros::Time::now();
  return header;
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
float y2r(float y){
  return (img_height.rows / 2 - y / par_res);
}
float x2c(float x){
  return (x / par_res + img_height.cols/2);
}
int r2y(float r){
  return int((img_height.rows / 2 - r) * par_res);
}
int c2x(float c){
  return int((c - img_height.cols / 2) * par_res);
}
cv::Point pnt2cv(geometry_msgs::Point pin){
	int c = x2c(pin.x);
	int r = y2r(pin.y);
	return cv::Point(c,r);
}
int get_z(float x, float y, int radlen_xy){
  int zmx = 0;
  for(int c = x2c(x)-radlen_xy; c < x2c(x)+radlen_xy; c++){
    for(int r = y2r(y)-radlen_xy; r < y2r(y)+radlen_xy; r++){
      if(img_height.at<uchar>(r,c) > zmx)
        zmx = img_height.at<uchar>(r,c);
    }
  }
  return zmx;
}
geometry_msgs::PointStamped elevate_point(geometry_msgs::PointStamped point,float inflation){
	geometry_msgs::PointStamped elevated_point;
 	elevated_point.point.z = get_z(point.point.x,point.point.y,inflation);
	elevated_point.point.x = point.point.x;
	elevated_point.point.y = point.point.y;
	elevated_point.header  = hdr();
	return elevated_point;
}
geometry_msgs::PolygonStamped elevate_poly(geometry_msgs::PolygonStamped polyin,float inflation){
	for(int i = 0; i < polyin.polygon.points.size(); i++){
		polyin.polygon.points[i].z = get_z(polyin.polygon.points[i].x,polyin.polygon.points[i].y,inflation);
	}
	return polyin;
}

nav_msgs::Path elevate_path(nav_msgs::Path pathin,float inflation){
	for(int i = 0; i < pathin.poses.size(); i++){
		pathin.poses[i].pose.position.z = get_z(pathin.poses[i].pose.position.x,pathin.poses[i].pose.position.y,inflation);
	}
	return pathin;
}

void elevation_point_cb(const geometry_msgs::PointStamped::ConstPtr& msg){
	pub_elevated_point.publish(elevate_point(*msg,par_inflation));
}
void elevation_poly_cb(const geometry_msgs::PolygonStamped::ConstPtr& msg){
	pub_elevated_poly.publish(elevate_poly(*msg,par_inflation));
}
void path_to_elevate_cb(const nav_msgs::Path::ConstPtr& msg){
	pub_elevated_path.publish(elevate_path(*msg,par_inflation));
}
void pc2_cb(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
	if(msg->data.size() > 10 && msg->header.frame_id == "map"){
		sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x");
		sensor_msgs::PointCloud2ConstIterator<float> iter_y(*msg, "y");
		sensor_msgs::PointCloud2ConstIterator<float> iter_z(*msg, "z");
		for(int i = 0; iter_x != iter_x.end(); ++i, ++iter_x, ++iter_y, ++iter_z)
		{
			if(!std::isnan(*iter_x) && !std::isnan(*iter_y) && !std::isnan(*iter_z))
	    {
	      int r = y2r(*iter_y);
	      int c = x2c(*iter_x);
	      int z = fmax(*iter_z,1);
				if(z > img_height.at<uchar>(r,c))
					img_height.at<uchar>(r,c) = z;
	    }
		}
	}
}
///********FRONTIER*************////////
///********FRONTIER*************////////
void draw_height_atstart(int radlen_xy,int zval){
  for(int x = -radlen_xy; x < radlen_xy; x++){
    for(int y = -radlen_xy; y < radlen_xy; y++){
      img_height.at<uchar>(y2r(y),x2c(x)) = zval;
    }
  }
}
void mainstate_cb(const std_msgs::UInt8::ConstPtr& msg){
  mainstate = msg->data;
}
int main(int argc, char** argv)
{
  ros::init(argc, argv, "tb_abmap_elevator_node");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");
	private_nh.param("resolution", par_res, 1.0);
	private_nh.param("inflation_radius", par_inflation, 3.0);

  ros::Subscriber s01 = nh.subscribe("/tb_fsm/main_state",10,mainstate_cb);
	ros::Subscriber s1   = nh.subscribe("/assembled_cloud2",10,pc2_cb);
	ros::Subscriber s2   = nh.subscribe("/tb_abmap/get_elevation_point",10,elevation_point_cb);
	ros::Subscriber s3   = nh.subscribe("/tb_abmap/get_elevation_poly",10,elevation_poly_cb);
	ros::Subscriber s4   = nh.subscribe("/tb_abmap/get_elevation_path",10,path_to_elevate_cb);
 	pub_elevated_poly	 = nh.advertise<geometry_msgs::PolygonStamped>("/tb_abmap/elevated_poly",10);
	pub_elevated_point = nh.advertise<geometry_msgs::PointStamped>("/tb_abmap/elevated_point",10);
	pub_elevated_path  = nh.advertise<nav_msgs::Path>("/tb_abmap/elevated_path",10);
	draw_height_atstart(5,10);
  ros::spin();
  return 0;
}
//
