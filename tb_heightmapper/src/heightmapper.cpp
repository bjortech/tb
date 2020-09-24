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

cv::Mat img_height(1000,1000,CV_8UC3,cv::Scalar(0, 0, 0)); //create image, set encoding and size, init pixels to default val
///********FRONTIER*************////////
double par_res,par_imwrite_interval;
ros::Publisher pub_heightcloud_registered,pub_heightcloud_ordered;
sensor_msgs::PointCloud heightcloud;
geometry_msgs::Point pos;
bool everyother;
int count_image_name= 0;
tf2_ros::Buffer tfBuffer;
int radlen_used = 50;
ros::Time process_start;
int par_maprad,par_maprad_pos;
std::string active_process;

void start_process(std::string name){
  float dt = (ros::Time::now() - process_start).toSec();
  if(active_process != "")
    ROS_INFO("PATHCREATOR: Process: %s took %.4f sec",active_process.c_str(),dt);
  active_process = name;
  process_start  = ros::Time::now();
}

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
sensor_msgs::PointCloud get_points_ordered_aroundpnt(std::string type,geometry_msgs::Point midpoint,int radlen_xy){
	start_process("get_points_ordered_aroundpnt");
	sensor_msgs::PointCloud heightcloud;
	heightcloud.header = hdr();
	int sidelength_xy = (radlen_xy * 2);
	ROS_INFO("Sidelength: %i = %i ",radlen_xy,sidelength_xy);
	heightcloud.points.resize(sidelength_xy*sidelength_xy + 2);
	int i = 0;
	for(int y = midpoint.y - radlen_xy; y < midpoint.y + radlen_xy; y++){
		for(int x = midpoint.x - radlen_xy; x < midpoint.x + radlen_xy; x++){
			int r = y2r(y);
			int c = x2c(x);
			i++;
			heightcloud.points[i].x = x;
			heightcloud.points[i].y = y;
			ROS_INFO("I: %i / %i",i,heightcloud.points.size());
			if(r > 0 && c > 0 && r < img_height.rows && c < img_height.cols){
				//int c = x2c(x);	int r = y2r(y);		if(r > 0 && c > 0 && r < img_height.rows && c < img_height.cols)
				if(type == "min")
					heightcloud.points[i].z = img_height.at<cv::Vec3b>(y2r(y),x2c(x))[0];
	    	else if(type == "minmax" && img_height.at<cv::Vec3b>(r,c)[0] < img_height.at<cv::Vec3b>(r,c)[2])
					heightcloud.points[i].z = img_height.at<cv::Vec3b>(y2r(y),x2c(x))[2];
				else
					heightcloud.points[i].z = img_height.at<cv::Vec3b>(y2r(y),x2c(x))[2];
	  	}
		}
	}
	start_process("");
	ROS_INFO("MaxPoints[type: %s]: %i pnts around %.0f %.0f radius_side: %i",type.c_str(),heightcloud.points.size(),midpoint.x,midpoint.y,radlen_xy);
	return heightcloud;
}
sensor_msgs::PointCloud get_points_registered_aroundpoint(std::string type,geometry_msgs::Point midpoint,int radlen_xy){
	start_process("get_points_registered_aroundpoint");
	sensor_msgs::PointCloud heightcloud;
	geometry_msgs::Point32 p;
	for(int y = midpoint.y - radlen_xy; y < midpoint.y + radlen_xy; y++){
  	for(int x = midpoint.x - radlen_xy; x < midpoint.x + radlen_xy; x++){
			int r = y2r(y);
			int c = x2c(x);
			if(r > 0 && c > 0 && r < img_height.rows && c < img_height.cols){
				if(img_height.at<cv::Vec3b>(r,c)[2] > 0){
					p.x = x;
					p.y = y;
					if(type == "min"){
						p.z = img_height.at<cv::Vec3b>(r,c)[0];
						heightcloud.points.push_back(p);
					}
					else if(type == "minmax" && img_height.at<cv::Vec3b>(r,c)[0] < img_height.at<cv::Vec3b>(r,c)[2]){
						p.z = img_height.at<cv::Vec3b>(r,c)[0];
						heightcloud.points.push_back(p);
						p.z = img_height.at<cv::Vec3b>(r,c)[2];
						heightcloud.points.push_back(p);
					}
					else{
						p.z = img_height.at<cv::Vec3b>(r,c)[2];
						heightcloud.points.push_back(p);
					}
				}
	    }
		}
  }
	start_process("");
	ROS_INFO("MaxPoints[type: %s]: %i pnts around %.0f %.0f radius_side: %i",type.c_str(),heightcloud.points.size(),midpoint.x,midpoint.y,radlen_xy);
	heightcloud.header = hdr();
	return heightcloud;
}

void pc2_cb(const sensor_msgs::PointCloud2::ConstPtr& msg){
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
				int z_min = img_height.at<cv::Vec3b>(r,c)[0];
				int z_num = img_height.at<cv::Vec3b>(r,c)[1];
				int z_max = img_height.at<cv::Vec3b>(r,c)[2];

				if(z_min == 0 || z < z_min)
					img_height.at<cv::Vec3b>(r,c)[0] = z;

				if(z > z_max)
					img_height.at<cv::Vec3b>(r,c)[2] = z;

				img_height.at<cv::Vec3b>(r,c)[1] = z_num + 1;
	    }
		}
	}
}
void get_points_ordered_aroundpnt_cb(const geometry_msgs::PointStamped::ConstPtr& msg){
		pub_heightcloud_ordered.publish(get_points_ordered_aroundpnt(msg->header.frame_id,msg->point,radlen_used));
}
void get_points_registered_aroundpnt_cb(const geometry_msgs::PointStamped::ConstPtr& msg){
		pub_heightcloud_registered.publish(get_points_registered_aroundpoint(msg->header.frame_id,msg->point,radlen_used));
}
void set_sidelength_cb(const std_msgs::UInt8::ConstPtr& msg){
	ROS_INFO("New radlen: %i -> %i",radlen_used,msg->data);
	radlen_used = msg->data;
}
void update_pos(){
	geometry_msgs::TransformStamped transformStamped;
	try{
		transformStamped = tfBuffer.lookupTransform("map","base_perfect_alt",
														 ros::Time(0));
	}
	catch (tf2::TransformException &ex) {
		ROS_WARN("%s",ex.what());
		ros::Duration(1.0).sleep();
	}
	pos.x = transformStamped.transform.translation.x;
	pos.y = transformStamped.transform.translation.y;
	pos.z = transformStamped.transform.translation.z;
}
int main(int argc, char** argv)
{
  ros::init(argc, argv, "tb_heightmapper_node");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");
	private_nh.param("resolution", par_res, 1.0);
	private_nh.param("area_heightrequest_radius", par_maprad, 25);
	private_nh.param("area_autopos_radius", par_maprad_pos, 5);
	private_nh.param("write_heightimage_interval", par_imwrite_interval, 2.0);
	tf2_ros::TransformListener tf2_listener(tfBuffer);
	heightcloud.points.resize(1000*1000+2);
	radlen_used = par_maprad;
	ros::Subscriber s1   								= nh.subscribe("/assembled_cloud2",10,pc2_cb);
	ros::Subscriber s2   								= nh.subscribe("/tb_heightmapper/get_points_ordered_aroundpnt",10,get_points_ordered_aroundpnt_cb);
	ros::Subscriber s3   								= nh.subscribe("/tb_heightmapper/get_points_registered_aroundpnt",10,get_points_registered_aroundpnt_cb);
	ros::Subscriber s4   								= nh.subscribe("/tb_heightmapper/set_sidelength",10,set_sidelength_cb);
	pub_heightcloud_ordered							= nh.advertise<sensor_msgs::PointCloud>("/tb_heightmapper/heightcloud_ordered",10);
	pub_heightcloud_registered					= nh.advertise<sensor_msgs::PointCloud>("/tb_heightmapper/heightcloud_registered",10);
	ros::Publisher pub_heightcloud_area = nh.advertise<sensor_msgs::PointCloud>("/tb_heightmapper/heightcloud_around_pos_ordered",10);
	ros::Rate rate(5.0);
	int naming_count = 0;
	ros::Time last_imrwite = ros::Time::now();
  while(ros::ok()){
		update_pos();
		if((ros::Time::now() - last_imrwite).toSec() > par_imwrite_interval){
			last_imrwite = ros::Time::now();
			naming_count++;
			cv::imwrite("/home/nuc/brain/"+std::to_string(naming_count)+"heightimage.png",img_height);
		}
		pub_heightcloud_area.publish(get_points_ordered_aroundpnt("",pos,par_maprad_pos));
		rate.sleep();
		ros::spinOnce();
	}
  return 0;
}
//
