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
#include <std_msgs/Empty.h>
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


tf2_ros::Buffer tfBuffer;
using namespace octomap;
using namespace std;
nav_msgs::Odometry odom_global;
shared_ptr<DynamicEDTOctomap> edf_ptr;
AbstractOcTree* abs_octree;
shared_ptr<OcTree> octree;
bool got_map = false;
geometry_msgs::Point pos,forward_point,cmd_pos;
int oct_xmin,oct_ymin,oct_zmin,oct_xmax,oct_ymax,oct_zmax,oct_range_x,oct_range_y,oct_range_z;
double par_dz,par_collision_radius;
std::vector<float> zmin_vec;
float zmin_ave;
int que_size = 10;
float pos_z_min = 5.0;
std::vector <geometry_msgs::PointStamped> obs_vec;
ros::Publisher pub_zmin;
float saturate(float val, float max){
  if((std::isnan(val)) || (std::isinf(val)))
    return 0;
  else if(val < 0 && val*-1 > max)
    return -max;
  else if(val > max)
    return max;
  else
    return val;
}
float get_dst3d(geometry_msgs::Point p1, geometry_msgs::Point p2){
  return(sqrt(pow(p1.x-p2.x,2)+pow(p1.y-p2.y,2)+pow(p1.z-p2.z,2)));
}
float get_dst2d(geometry_msgs::Point p1, geometry_msgs::Point p2){
  return(sqrt(pow(p1.x-p2.x,2)+pow(p1.y-p2.y,2)));
}
float get_inclination(geometry_msgs::Point p2, geometry_msgs::Point p1){
  return atan2(p2.z - p1.z,get_dst2d(p1,p2));
}
float get_hdng(geometry_msgs::Point p1,geometry_msgs::Point p0){
  float dx = p1.x - p0.x;
  float dy = p1.y - p0.y;
  return atan2(dy,dx);
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
void heightpos_cb(const sensor_msgs::PointCloud::ConstPtr& msg){
	float zmx = 0;
	for(int i = 0; i < msg->points.size(); i++){
		if(msg->points[i].z > zmx)
		 zmx = msg->points[i].z;
	}
	zmin_vec.push_back(zmx);
	if(zmin_vec.size() > que_size)
		zmin_vec.erase(zmin_vec.begin());
	pos_z_min = get_zmax_que();
	std_msgs::Float64 zmin_msg;
	zmin_msg.data = pos_z_min;
	pub_zmin.publish(zmin_msg);
	//ROS_INFO("pos_z_min: %.0f",pos_z_min);
}
bool is_in_vector(geometry_msgs::PointStamped pnt){
	for(int  i = 0; i < obs_vec.size(); i++){
		if(get_dst3d(obs_vec[i].point,pnt.point) < 1.0)
			return true;
	}
	return false;
}
void closest_obstacle_cb(const geometry_msgs::PointStamped::ConstPtr& msg){
	if(!is_in_vector(*msg))
		obs_vec.push_back(*msg);
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
  cmd_pos.x = transformStamped.transform.translation.x;
  cmd_pos.y = transformStamped.transform.translation.y;
  cmd_pos.z = transformStamped.transform.translation.z;

}
void odomglobal_cb(const nav_msgs::Odometry::ConstPtr& msg){
  if((ros::Time::now() - odom_global.header.stamp).toSec() > 0.1){
    odom_global = *msg;
    if(!std::isnan(odom_global.twist.twist.linear.x) && !std::isnan(odom_global.twist.twist.linear.y) && !std::isnan(odom_global.twist.twist.linear.z)){
      forward_point.x = odom_global.twist.twist.linear.x * 2.0;//  pos.x + 15 * cos(vlp_rpy.z);
      forward_point.y = odom_global.twist.twist.linear.y * 2.0;// pos.y + 15 * sin(vlp_rpy.z);
      forward_point.z = odom_global.twist.twist.linear.z * 2.0;// pos.y + 15 * sin(vlp_rpy.z);
    }
  }
}
/*
void check_collision(){
	for(int i = 0; i < obs_vec.size(); i++){
		float obs_dst3d1  = get_dst3d(pos,obs_vec[i].point);
	  float obs_dst3d2  = get_dst3d(cmd_pos,obs_vec[i].point);
	  float obs_dst3d3  = get_dst3d(forward_point,obs_vec[i].point);
		float dst_min = fmin(fmin(obs_dst3d1,obs_dst3d2),fmin(obs_dst3d1,obs_dst3d3));
	  if(dst_min < 3){
	  	colliding = true;
	  	collision_object.point = obs;
	  	collision_object.header.stamp = ros::Time::now();
	  }
	}
}
*/
int main(int argc, char** argv)
{
  ros::init(argc, argv, "tb_autoevade_obstaclefinder_node");
	ros::NodeHandle nh;
	ros::NodeHandle private_nh("~");
	tf2_ros::TransformListener tf2_listener(tfBuffer);

	ros::Subscriber s1 = nh.subscribe("/tb_heightmapper/heightcloud_around_pos_ordered",1,heightpos_cb);
	ros::Subscriber s2 = nh.subscribe("/tb_autoevade/closest_obstacle",1,closest_obstacle_cb);
	ros::Subscriber s3 = nh.subscribe("/tb_autoevade/odom_global",1,odomglobal_cb);
	pub_zmin  = nh.advertise<std_msgs::Float64>("/tb_autoevade/z_min",10);

	ros::Rate rate(1.0);
	while(ros::ok()){
		//update_pos();
		//cmd_x = -(collision_object.point.x - pos.x);
		//cmd_y = -(collision_object.point.y - pos.y);
		//cmd_z = fmax(collision_object.point.z+2.0,cmd_pos.z);
		rate.sleep();
		ros::spinOnce();
	}
	return 0;
}
