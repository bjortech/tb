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
#include <tb_msgsrv/PathsStamped.h>
#include <tb_msgsrv/PolygonsStamped.h>
cv::Mat img(1000,1000,CV_8UC3,cv::Scalar(0, 0, 0)); //create image, set encoding and size, init pixels to default val
cv::Mat img_blank(1000,1000,CV_8UC3,cv::Scalar(0, 0, 0)); //create image, set encoding and size, init pixels to default val
cv::Mat img_height(1000,1000,CV_8U,cv::Scalar(0)); //create image, set encoding and size, init pixels to default val
cv::Mat img_filled(1000,1000,CV_8U,cv::Scalar(0)); //create image, set encoding and size, init pixels to default val
cv::Mat img_paths(1000,1000,CV_8U,cv::Scalar(0)); //create image, set encoding and size, init pixels to default val
cv::Mat imb_blank_mono(1000,1000,CV_8U,cv::Scalar(0)); //create image, set encoding and size, init pixels to default val
tb_msgsrv::PathsStamped paths_clusters;
tb_msgsrv::PolygonsStamped paths_bbpolys;
///////////********CTRL***********//////////
ros::Time process_start;
std::string active_process;
///********FRONTIER*************////////
int count_target_paths= 0;
std::vector<int> blacklist;
geometry_msgs::PolygonStamped poly_side,poly_roi2d,poly_roi;

ros::Publisher pub_get_side,pub_path_to_cluster,pub_get_down,pub_get_poly;
nav_msgs::Path path_getting_processed,path_side,path_starsquare,path_side_full;
ros::Time request_time;
ros::Time path_sent_time;
int side_size;
float dt_side;
int count_colorshift = 0;
int last_i = 0;
float extra_length = 5.0;
bool path_sent,path_clusters_received,path_bbpolys_received;
int down_size;
float dt_down;
nav_msgs::Path path_master;
nav_msgs::Path path_down;
geometry_msgs::PolygonStamped poly_master,poly_cleared;
///********FRONTIER*************////////

std_msgs::Header hdr(){
  std_msgs::Header header;
  header.frame_id = "map";
  header.stamp    = ros::Time::now();
  return header;
}
bool sort_dst_pair(const std::tuple<int,float>& a,
               const std::tuple<int,float>& b)
{
    return (std::get<1>(a) < std::get<1>(b));
}
void start_process(std::string name){
  float dt = (ros::Time::now() - process_start).toSec();
  if(active_process != "")
    ROS_INFO("Process: %s took %.4f sec",active_process.c_str(),dt);
  active_process = name;
  process_start  = ros::Time::now();
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
  return (img_height.rows / 2 - y);
}
float x2c(float x){
  return (x + img_height.cols/2);
}
int r2y(float r){
  return int((img_height.rows / 2 - r));
}
int c2x(float c){
  return int((c - img_height.cols / 2));
}
cv::Point pnt2cv(geometry_msgs::Point pin){
	int c = x2c(pin.x);
	int r = y2r(pin.y);
	return cv::Point(c,r);
}
cv::Point pnt322cv(geometry_msgs::Point32 pin){
	int c = x2c(pin.x);
	int r = y2r(pin.y);
	return cv::Point(c,r);
}
void draw_poly(geometry_msgs::PolygonStamped polyin,cv::Scalar color){
  for(int i = 1; i < polyin.polygon.points.size(); i++){
		cv::line (img, pnt322cv(polyin.polygon.points[i-1]), pnt322cv(polyin.polygon.points[i]),color,1,cv::LINE_8,0);
  }
	cv::line (img, pnt322cv(polyin.polygon.points[polyin.polygon.points.size()-1]), pnt322cv(polyin.polygon.points[0]),color,1,cv::LINE_8,0);
}
void draw_circle(float x,float y, float size,cv::Scalar color){
	geometry_msgs::Point p1;
	p1.x = x; p1.y = y;
	cv::circle(img,pnt2cv(p1),size,color,1);
}
void draw_rectangle(float x,float y,float size,cv::Scalar color){
	geometry_msgs::Point p1,p2;
	p1.x = x-size;
	p1.y = y-size;
	p2.x = x+size*2;
	p2.y = y+size*2;
	cv::rectangle(img, pnt2cv(p1),pnt2cv(p2),color,1,8,0);
}
void draw_path(nav_msgs::Path pathin,cv::Scalar color){
	for(int i = 0; i < pathin.poses.size(); i++){
		geometry_msgs::Point pnt = pathin.poses[i].pose.position;
		float yaw = tf::getYaw(pathin.poses[i].pose.orientation);
    img.at<cv::Vec3b>( y2r(pnt.y),x2c(pnt.x) )[0] = color[0];
    img.at<cv::Vec3b>( y2r(pnt.y),x2c(pnt.x) )[1] = color[1];
    img.at<cv::Vec3b>( y2r(pnt.y),x2c(pnt.x) )[2] = color[2];
		geometry_msgs::Point pyaw;
		pyaw.x = pnt.x + 3 * cos(yaw);
		pyaw.y = pnt.y + 2 * sin(yaw);
		cv::line (img,  pnt2cv(pnt), pnt2cv(pyaw),color,1,cv::LINE_8,0);
	}
}
geometry_msgs::PolygonStamped get_master_poly(tb_msgsrv::PolygonsStamped polysin){
	geometry_msgs::PolygonStamped polysout;
	polysout.header = hdr();
	for(int i = 0; i < polysin.polygons.size(); i++){
		for(int k = 0; k < polysin.polygons[i].polygon.points.size(); k++){
			polysout.polygon.points.push_back(polysin.polygons[i].polygon.points[k]);
		}
	}
	return polysout;
}
nav_msgs::Path get_master_path(tb_msgsrv::PathsStamped pathsin){
	nav_msgs::Path pathout;
	pathout.header = hdr();
	for(int i = 0; i < pathsin.paths.size(); i++){
		for(int k = 0; k < pathsin.paths[i].poses.size(); k++){
			pathout.poses.push_back(pathsin.paths[i].poses[k]);
		}
	}
	return pathout;
}
void path_clusters_cb(const tb_msgsrv::PathsStamped::ConstPtr& msg){
	paths_clusters = *msg;
	float dt = (ros::Time::now() - path_sent_time).toSec();
	path_clusters_received = true;
	path_master = get_master_path(*msg);
	ROS_INFO("CLUSTERS(num: %i total: %i) received after %.3f seconds",paths_clusters.paths.size(),path_master.poses.size(),dt);
}
void path_bbpolys_cb(const tb_msgsrv::PolygonsStamped::ConstPtr& msg){
	paths_bbpolys = *msg;
	float dt = (ros::Time::now() - path_sent_time).toSec();
	path_bbpolys_received = true;
	poly_master = get_master_poly(*msg);
	ROS_INFO("BBPOLYS(num: %i total: %i) received after %.3f seconds",paths_bbpolys.polygons.size(),poly_master.polygon.points.size(),dt);
}
void poly_cb(const geometry_msgs::PolygonStamped::ConstPtr& msg){
	poly_cleared = *msg;
}
bool dst_point_in_path_lim(nav_msgs::Path pathin,geometry_msgs::Point pin,float lim){
  float res,dst;
  if(pathin.poses.size() == 0)
    return true;
  for(int i = 0; i < pathin.poses.size(); i++){
     if(get_dst3d(pathin.poses[i].pose.position,pin) < lim)
        return false;
  }
  return true;
}
nav_msgs::Path get_new_path(nav_msgs::Path path_base,nav_msgs::Path pathin,float cutoff){
	nav_msgs::Path pathout;
	pathout.header = hdr();
	for(int i = 0; i < pathin.poses.size(); i++){
		if(dst_point_in_path_lim(path_side,pathin.poses[i].pose.position,cutoff))
			pathout.poses.push_back(pathin.poses[i]);
	}
	return pathout;
}
void path_down_cb(const nav_msgs::Path::ConstPtr& msg){
	nav_msgs::Path path_down_new = get_new_path(path_down,*msg,1.0);
	down_size = msg->poses.size();
	dt_down = (ros::Time::now() - request_time).toSec();
	ROS_INFO("dt_down: %.3f down_size: %i ",dt_down,down_size);
	if(path_down_new.poses.size() > 0){
		for(int i = 0; i < path_down_new.poses.size(); i++){
			path_down.poses.push_back(path_down_new.poses[i]);
		}
	}
	ROS_INFO("down IN: %i, new: %i, final: %i",msg->poses.size(),path_down_new.poses.size(),path_down.poses.size());
}
void path_side_cb(const nav_msgs::Path::ConstPtr& msg){
	nav_msgs::Path path_side_new = get_new_path(path_side,*msg,1.0);
	side_size = msg->poses.size();
	dt_side = (ros::Time::now() - request_time).toSec();
	ROS_INFO("dt_side: %.3f side_size: %i ",dt_side,side_size);
	if(path_side_new.poses.size() > 0){
		for(int i = 0; i < path_side_new.poses.size(); i++){
			path_side.poses.push_back(path_side_new.poses[i]);
		}
	}
	ROS_INFO("SIDE IN: %i, new: %i, final: %i",msg->poses.size(),path_side_new.poses.size(),path_side.poses.size());
}
cv::Scalar get_shifting_color(){
  cv::Scalar color;
  color[count_colorshift] = 255;
	count_colorshift++;
	if(count_colorshift >= 3)
		count_colorshift = 0;
  return color;
}
int main(int argc, char** argv)
{
  ros::init(argc, argv, "tb_debug_clusterviz_node");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");

	pub_path_to_cluster = nh.advertise<nav_msgs::Path>("/tb_process/get_path_clusters",10);

	ros::Subscriber os2 = nh.subscribe("/tb_process/paths_clusters",1,path_clusters_cb);
	ros::Subscriber os1 = nh.subscribe("/tb_process/paths_bbpolys",1,path_bbpolys_cb);
	ros::Subscriber o1s = nh.subscribe("/tb_edto/side",1,path_side_cb);
	ros::Subscriber o2s = nh.subscribe("/tb_edto/down",1,path_down_cb);
	ros::Subscriber o3s = nh.subscribe("/tb_edto/poly_cleared",1,poly_cb);

	ros::Publisher pub_path_master = nh.advertise<nav_msgs::Path>("/tb_path_master",10);
	ros::Publisher pub_poly_master = nh.advertise<geometry_msgs::PolygonStamped>("/tb_poly_master",10);
	ros::Publisher pub_path_master_segments = nh.advertise<nav_msgs::Path>("/tb_pathsegments",10);
	ros::Publisher pub_poly_master_segments = nh.advertise<geometry_msgs::PolygonStamped>("/tb_polysegments",10);

  ros::Rate rate(1.0);
  ros::Time last_radialupdate = ros::Time::now();
	bool first = true;
	bool use_random = false;
	int count_segment = 0;
  while(ros::ok()){
    rate.sleep();
    ros::spinOnce();
		if(path_clusters_received && path_bbpolys_received){
			path_sent = false;
			ROS_INFO("PRocess received!",paths_clusters.paths.size(),paths_bbpolys.polygons.size());
			start_process("draw");
			int tot = 0;
			path_clusters_received = false;
			path_bbpolys_received = false;
			for(int i = 0; i < paths_clusters.paths.size(); i++){
				cv::Scalar color = get_shifting_color();
				tot += paths_clusters.paths[i].poses.size();
				int s1 = paths_clusters.paths[i].poses.size();
				int s2 = paths_bbpolys.polygons[i].polygon.points.size();
				ROS_INFO("s1: %i s2 %i",s1,s2);
				draw_path(paths_clusters.paths[i],color);
				draw_poly(paths_bbpolys.polygons[i],color);
			//	poly_master.
			}
			count_target_paths++;
			cv::imwrite("/home/nuc/brain/clusters/process"+std::to_string(count_target_paths)+".png",img);
			start_process("");
		}
		count_segment++;

		if(count_segment < paths_clusters.paths.size()){
			pub_path_master_segments.publish(paths_clusters.paths[count_segment]);
		}
		else{
			count_segment = 0;
		}
		if(count_segment < paths_bbpolys.polygons.size()){
			pub_poly_master_segments.publish(paths_bbpolys.polygons[count_segment]);
			ROS_INFO("count_segment, %i",count_segment);
		}



		pub_path_master.publish(path_master);
		pub_poly_master.publish(poly_master);
	}
	return 0;
}
//
