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

ros::Publisher pub_set_z,pub_set_yaw,pub_set_xy,pub_get_poly,pub_get_side,pub_get_sphere,pub_get_down,pub_new_target,pub_costmapupdate_edto,pub_costmapupdate_img;
geometry_msgs::Point p,pos;
std::string state_target = "idle";
geometry_msgs::PointStamped setpoint_xy,new_target_pnt;
geometry_msgs::PoseStamped setpose;
std_msgs::Float64 setpoint_z,setpoint_yaw;
bool use_mb;
float pos_yaw;
tf2_ros::Buffer tfBuffer;
std::vector<int> targetindexes_sent;
std::vector<int> blacklist;
int costmap_elevation_meters;
ros::Time process_start;
geometry_msgs::PolygonStamped poly_roi2d,poly_roi;
nav_msgs::Path path_starsquare,path_cmd;
std::string active_process;
std::vector<float> zmin_vec;
cv::Mat img(1000,1000,CV_8UC3,cv::Scalar(0, 0, 0)); //create image, set encoding and size, init pixels to default val
cv::Mat img_blank(1000,1000,CV_8UC3,cv::Scalar(0, 0, 0)); //create image, set encoding and size, init pixels to default val
float zmin_ave;
int que_size = 10;
float pos_z_min = 5.0;
geometry_msgs::PointStamped pos_front;
geometry_msgs::Vector3 vlp_rpy;
int path_cmd_i = 0;
int count_target_paths = 0;
nav_msgs::Odometry odom_global;
float par_res = 1.0;
nav_msgs::Path path_visited;
nav_msgs::Path path_unknown;
int poly_size,side_size,sphere_size,down_size;
float dt_poly,dt_side,dt_sphere,dt_down;
ros::Time request_time;
bool ready = true;
void vstd_cb(const nav_msgs::Path::ConstPtr& msg){
  path_visited = *msg;
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
float get_dst2d(geometry_msgs::Point p1, geometry_msgs::Point p2){
  return(sqrt(pow(p1.x-p2.x,2)+pow(p1.y-p2.y,2)));
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
int get_random_i(int nummax){
  srand (time(NULL));
  return rand() % nummax;
}
geometry_msgs::Point get_random_pnt(nav_msgs::Path pathin){
	std::vector<int> possible_i;
	for(int i = 0; i < pathin.poses.size(); i++){
		if(!in_vec(blacklist,i))
			possible_i.push_back(i);
	}
	int iout = get_random_i(possible_i.size());
	int iact = possible_i[iout];
	blacklist.push_back(iact);
	return pathin.poses[iact].pose.position;
}
int getinpath_closestindex2d(nav_msgs::Path pathin,geometry_msgs::Point pnt){
  int lowest_dist_i = 0;
  float lowest_dist = 1000;
  for(int i = 0; i < pathin.poses.size(); i++){
    float dst = get_dst2d(pathin.poses[i].pose.position,pnt);
    if(dst < lowest_dist){
      lowest_dist_i = i;
      lowest_dist = dst;
    }
	}
	return lowest_dist_i;
}
geometry_msgs::Point get_next_pnt(nav_msgs::Path pathin){
	int ci = getinpath_closestindex2d(pathin,pos);
	return pathin.poses[ci].pose.position;
}
geometry_msgs::PolygonStamped create_poly2d(geometry_msgs::Point pin, float rad){
	geometry_msgs::PolygonStamped poly;
	poly.header = hdr();
	poly.polygon.points.resize(5);
	poly.polygon.points[0].x = round(pin.x + rad);
	poly.polygon.points[0].y = round(pin.y + rad);
	poly.polygon.points[1].x = round(pin.x - rad);
	poly.polygon.points[1].y = round(pin.y + rad);
	poly.polygon.points[2].x = round(pin.x - rad);
	poly.polygon.points[2].y = round(pin.y - rad);
	poly.polygon.points[3].x = round(pin.x + rad);
	poly.polygon.points[3].y = round(pin.y - rad);
	poly.polygon.points[4]   = poly.polygon.points[0];
	return poly;
}
void create_poly_volume(geometry_msgs::Point pin, float sides,float z0,float z1){
	poly_roi2d = create_poly2d(pin,sides/2);
	poly_roi = poly_roi2d;
	for(int i = 0; i < poly_roi.polygon.points.size(); i++){
		poly_roi.polygon.points[i].z = z0;
	}
	poly_roi.polygon.points.push_back(poly_roi.polygon.points[0]);
	poly_roi.polygon.points.push_back(poly_roi.polygon.points[1]);
	poly_roi.polygon.points.push_back(poly_roi.polygon.points[2]);
	poly_roi.polygon.points.push_back(poly_roi.polygon.points[3]);
	poly_roi.polygon.points.push_back(poly_roi.polygon.points[4]);

	for(int i = 5; i < poly_roi.polygon.points.size(); i++){
		poly_roi.polygon.points[i].z = z1;
	}
	for(int i = 0; i < poly_roi.polygon.points.size(); i++){
		if(poly_roi.polygon.points[i].z > z1)
			poly_roi.polygon.points[i].z = z1;
		if(poly_roi.polygon.points[i].z < z0)
			poly_roi.polygon.points[i].z = z0;
	}
//  ROS_INFO("x %.2f y %.2f x %.2f y %.2f x %.2f y %.2f x %.2f y %.2f x %.2f y %.2f",poly.polygon.points[0].x,poly.polygon.points[0].y,poly.polygon.points[1].x,poly.polygon.points[1].y,poly.polygon.points[2].x,poly.polygon.points[2].y,poly.polygon.points[3].x,poly.polygon.points[3].y,poly.polygon.points[4].x,poly.polygon.points[4].y);
}

void clickedpoint_cb(geometry_msgs::Point p){

}

nav_msgs::Path create_gridpath(geometry_msgs::Point midpoint,float area_sidelength,float radlen_xy){
  float centroid_sides = 2*radlen_xy;
  int num_grids = area_sidelength / centroid_sides;
  nav_msgs::Path pathout;
  pathout.header = hdr();
  ////ROS_INFO("Area side: %.2f radlen: %.2f num_grids: %i centroid_sides: %.2f",area_sidelength,radlen_xy,num_grids,centroid_sides);
  int i = 0;
  for (int y = 0; y < num_grids; y++)
  {
    for (int x = 0; x < num_grids; x++)
    {
     geometry_msgs::PoseStamped ps;
     ps.pose.position.x = midpoint.x + float(area_sidelength*-0.5 + x * centroid_sides+centroid_sides*0.5);
     ps.pose.position.y = midpoint.y + float(area_sidelength*-0.5 + y * centroid_sides+centroid_sides*0.5);
     ps.pose.orientation.w = 1.0;
     ps.header = hdr();
     pathout.poses.push_back(ps);
    }
  }
  return pathout;
}




void poly_cleared_cb(const geometry_msgs::PolygonStamped::ConstPtr& msg){
	poly_size = msg->polygon.points.size();
	dt_poly = (ros::Time::now() - request_time).toSec();
	ROS_INFO("dt_poly: %.3f poly_size: %i ",dt_poly,poly_size);
}
void poly_obstacles_cb(const geometry_msgs::PolygonStamped::ConstPtr& msg){
	poly_size = msg->polygon.points.size();
	dt_poly = (ros::Time::now() - request_time).toSec();
	ROS_INFO("dt_poly: %.3f poly_size: %i ",dt_poly,poly_size);
}
void poly_unknown_cb(const geometry_msgs::PolygonStamped::ConstPtr& msg){
	poly_size = msg->polygon.points.size();
	dt_poly = (ros::Time::now() - request_time).toSec();
	ROS_INFO("dt_poly: %.3f poly_size: %i ",dt_poly,poly_size);
}
void get_side_cb(const nav_msgs::Path::ConstPtr& msg){
	ready = true;
	side_size = msg->poses.size();
	dt_side = (ros::Time::now() - request_time).toSec();
	ROS_INFO("dt_side: %.3f side_size: %i ",dt_side,side_size);
}
void get_sphere_cb(const nav_msgs::Path::ConstPtr& msg){
	ready = true;
	sphere_size = msg->poses.size();
	dt_sphere = (ros::Time::now() - request_time).toSec();
	ROS_INFO("dt_sphere: %.3f sphere_size: %i ",dt_sphere,sphere_size);
}
void get_down_cb(const nav_msgs::Path::ConstPtr& msg){
	ready = true;
	down_size = msg->poses.size();
	dt_down = (ros::Time::now() - request_time).toSec();
	ROS_INFO("dt_down: %.3f down_size: %i ",dt_down,down_size);
}
int main(int argc, char** argv)
{
  ros::init(argc, argv, "tb_abmap_requester_node");
  ros::NodeHandle nh;
	ros::NodeHandle private_nh("~");
	tf2_ros::TransformListener tf2_listener(tfBuffer);

	ros::Publisher pub_roi_poly	  = nh.advertise<geometry_msgs::PolygonStamped>("/tb_roi/poly",10);
	ros::Publisher pub_roi2d_poly	= nh.advertise<geometry_msgs::PolygonStamped>("/tb_roi/poly2d",10);
	ros::Publisher pub_tb_fsm	= nh.advertise<std_msgs::UInt8>("/tb_fsm/main_state",10);

	pub_get_poly	  = nh.advertise<geometry_msgs::PolygonStamped>("/tb_edto/get_poly",10);
	pub_get_side	  = nh.advertise<geometry_msgs::PolygonStamped>("/tb_edto/get_side",10);
	pub_get_sphere	= nh.advertise<geometry_msgs::PolygonStamped>("/tb_edto/get_sphere",10);
	pub_get_down		= nh.advertise<geometry_msgs::PolygonStamped>("/tb_edto/get_down",10);
	ros::Subscriber a1  	= nh.subscribe("/tb_edto/poly_cleared",1,poly_cleared_cb);
	ros::Subscriber a2  	= nh.subscribe("/tb_edto/poly_obstacles",1,poly_obstacles_cb);
	ros::Subscriber a3  	= nh.subscribe("/tb_edto/poly_unknown",1,poly_unknown_cb);
	ros::Subscriber os  	= nh.subscribe("/tb_edto/side",1,get_side_cb);
	ros::Subscriber o1  	= nh.subscribe("/tb_edto/sphere",1,get_sphere_cb);
	ros::Subscriber o1d  	= nh.subscribe("/tb_edto/down",1,get_down_cb);
	std_msgs::UInt8 mainstate;
	mainstate.data = 1;
	ros::Rate rate(2.0);
  ros::Time last_radialupdate = ros::Time::now();
	int count = 0;
	geometry_msgs::Point pos;
	ros::Time time_last = ros::Time::now();
	path_starsquare = create_gridpath(pos,200,10);
	while(ros::ok()){
    rate.sleep();
    ros::spinOnce();
		if(ready){
			ready = false;
			geometry_msgs::Point p = get_random_pnt(path_starsquare);
			p.z = 15;

			poly_size = 0;
			side_size = 0;
			sphere_size = 0;
			down_size = 0;
			dt_poly = 0.0;
			dt_side = 0.0;
			dt_sphere = 0.0;
			dt_down = 0.0;
			float z0 = 3;//get_random_i(0);
			float z1 = 30;// z0 + get_random_i(30);
			float r  = 75;//50+get_random_i(100);
			create_poly_volume(p,r,z0,z1);
			ROS_INFO("NEXT POINT: %.0f %.0f %.0f z0: %.0f z1: %.0f r: %.0f",p.x,p.y,p.z,z0,z1,r);
			count++;
			if(count == 1){
				pub_get_side.publish(poly_roi);
			}
			if(count == 2){
				pub_get_sphere.publish(poly_roi);
			}
			if(count == 3){
				pub_get_down.publish(poly_roi);
				count = 0;
			}
			request_time = ros::Time::now();
		}
		pub_get_poly.publish(poly_roi);
		pub_tb_fsm.publish(mainstate);
		pub_roi_poly.publish(poly_roi);
		pub_roi2d_poly.publish(poly_roi2d);
  }
  return 0;
}
