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
ros::Publisher pub_costmapelevate_path,pub_costmapupdate_edto,pub_costmapupdate_img,pub_costmapupdated;
geometry_msgs::Point pos,start_pnt,end_pnt;
std_msgs::String targetstate_msg;
bool par_update_costmap_edto;
double par_zclearing;
nav_msgs::Path start_end_path;
tf2_ros::Buffer tfBuffer;

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
bool in_vec(std::vector<int> vec,int k){
	if(vec.size() == 0)
		return false;
	//	//ROS_INFO("in vec: %i",k);
	for(int i = 0; i < vec.size(); i++){
		if(vec[i] == k)
			return true;
	}
	return false;
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
}
nav_msgs::Path get_start_end_path(geometry_msgs::Point p0, geometry_msgs::Point p1){
	nav_msgs::Path pathout;
	pathout.poses.resize(2);
	pathout.poses[0].pose.position = p0;
	pathout.poses[1].pose.position = p1;
	pathout.poses[0].pose.orientation.w = 1.0;
	pathout.poses[1].pose.orientation.w = 1.0;
	pathout.header = pathout.poses[0].header = pathout.poses[1].header = hdr();
	return pathout;
}
geometry_msgs::PolygonStamped create_poly2d(float inflate_xy){
	geometry_msgs::PolygonStamped poly;
	poly.header = hdr();
	poly.polygon.points.resize(5);
	float xmax = fmax(start_pnt.x,end_pnt.x)+inflate_xy;
	float xmin = fmin(start_pnt.x,end_pnt.x)-inflate_xy;
	float ymax = fmax(start_pnt.y,end_pnt.y)+inflate_xy;
	float ymin = fmin(start_pnt.y,end_pnt.y)-inflate_xy;
	poly.polygon.points[0].x = round(xmax);
	poly.polygon.points[0].y = round(ymax);
	poly.polygon.points[1].x = round(xmin);
	poly.polygon.points[1].y = round(ymax);
	poly.polygon.points[2].x = round(xmin);
	poly.polygon.points[2].y = round(ymin);
	poly.polygon.points[3].x = round(xmax);
	poly.polygon.points[3].y = round(ymin);
	poly.polygon.points[4] = poly.polygon.points[0];
	return poly;
}
geometry_msgs::PolygonStamped create_poly_volume(float inflate_xy,float z0,float z1){
	geometry_msgs::PolygonStamped poly;
	poly   = create_poly2d(inflate_xy);
	for(int i = 0; i < poly.polygon.points.size(); i++){
		poly.polygon.points[i].z = z0;
	}
	poly.polygon.points.push_back(poly.polygon.points[0]);
	poly.polygon.points.push_back(poly.polygon.points[1]);
	poly.polygon.points.push_back(poly.polygon.points[2]);
	poly.polygon.points.push_back(poly.polygon.points[3]);
	poly.polygon.points.push_back(poly.polygon.points[4]);

	for(int i = 5; i < poly.polygon.points.size(); i++){
		poly.polygon.points[i].z = z1;
	}
	return poly;
}

void update_costmap(int z){
	float inflate_xy = 20.0;
	float inflate_z  = 1.5;
	float z0 = float(z) - inflate_z;
	float z1 = float(z) + inflate_z;
	if(par_update_costmap_edto)
		pub_costmapupdate_edto.publish(create_poly_volume(inflate_xy,z0,z1));
	else
		pub_costmapupdate_img.publish(create_poly_volume(inflate_xy,z0,z1));
}

void costmap_elevation_cb(const std_msgs::UInt8::ConstPtr& msg){
	start_pnt.z = msg->data + par_zclearing;
	end_pnt.z = msg->data + par_zclearing;
	update_costmap(msg->data + par_zclearing);
}
void send_elevation_request(geometry_msgs::Point p0, geometry_msgs::Point p1){
	pub_costmapelevate_path.publish(get_start_end_path(start_pnt,end_pnt));
}
void targetpoint_cb(const geometry_msgs::PointStamped::ConstPtr& msg){
	if(get_dst2d(msg->point,pos) > 5.0){
		update_pos();
		start_pnt = pos;
		end_pnt 	= msg->point;
		send_elevation_request(pos,msg->point);
	}
}
void map_updates_cb(const map_msgs::OccupancyGridUpdate::ConstPtr& msg){
	pub_costmapupdated.publish(get_start_end_path(start_pnt,end_pnt));
}
int main(int argc, char** argv)
{
  ros::init(argc, argv, "tb_autonomy_costmap_node");
  ros::NodeHandle nh;
	ros::NodeHandle private_nh("~");
	tf2_ros::TransformListener tf2_listener(tfBuffer);

	private_nh.param("use_edto_for_costmapupdate", par_update_costmap_edto, true);
	private_nh.param("safety_margin_z", par_zclearing, 5.0);

	pub_costmapelevate_path	 = nh.advertise<nav_msgs::Path>("/tb_costmap/get_elevation_in_path",10);
	ros::Subscriber c0 		   = nh. subscribe("/tb_costmap/elevation",100,&costmap_elevation_cb);
	ros::Subscriber u1 			 = nh. subscribe("/map_updates",100,&map_updates_cb);

	pub_costmapupdate_edto   = nh.advertise<geometry_msgs::PolygonStamped>("/tb_costmap/update_EDTO",10);
	pub_costmapupdate_img    = nh.advertise<geometry_msgs::PolygonStamped>("/tb_costmap/update_heightimg",10);
	ros::Subscriber t1   		 = nh.subscribe("/tb_autonomy/update_costmap_to_point",10,targetpoint_cb);
	pub_costmapupdated    	 = nh.advertise<nav_msgs::Path>("/tb_autonomy/costmap_updated",10);

	ros::spin();
  return 0;
}
