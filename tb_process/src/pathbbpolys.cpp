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
ros::Publisher pub_polygons,pub_polygon;
int par_num_points;
double par_extra_length;
bool par_auto;
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

std::vector<float> fix_ranges(std::vector<float> ranges){
	if(ranges[0] < ranges[1]/5 && ranges[0] < ranges[ranges.size()-1]/5)
		ranges[0] = (ranges[1] + ranges[ranges.size()-1])/2;

	for(int i = 1; i < ranges.size(); i++){
		if(ranges[i] < ranges[i+1]/5 && ranges[i] < ranges[i-1]/5)
			ranges[i] = (ranges[i+1] + ranges[i-1])/2;
	}
	return ranges;
}
std::vector<float> get_z_min_max_ave_path(nav_msgs::Path pathin){
	std::vector<float> zout;
	zout.resize(3);
	zout[0] = 200;
	zout[1] = -200;
	float zsum = 0;
	for(int i = 0; i < pathin.poses.size(); i++){
		zsum += pathin.poses[i].pose.position.z;
		if(pathin.poses[i].pose.position.z < zout[0])
			zout[0] = pathin.poses[i].pose.position.z;
	  if(pathin.poses[i].pose.position.z > zout[1])
      zout[1] = pathin.poses[i].pose.position.z;
  }
	zout[2] = zsum/pathin.poses.size();
  return zout;
}
geometry_msgs::Point get_ave_pnt(nav_msgs::Path pathin){
  geometry_msgs::Point pnt;
  for(int i = 0; i < pathin.poses.size(); i++){
    pnt.x += pathin.poses[i].pose.position.x;
    pnt.y += pathin.poses[i].pose.position.y;
    pnt.z += pathin.poses[i].pose.position.z;
  }
  pnt.x /= pathin.poses.size();
  pnt.y /= pathin.poses.size();
  pnt.z /= pathin.poses.size();
  return pnt;
}

geometry_msgs::PolygonStamped elevate_poly(geometry_msgs::PolygonStamped polyin,float z0,float z1){
	int n_size = polyin.polygon.points.size();
	for(int i = 0; i < n_size; i++){
		polyin.polygon.points[i].z = z0;
		polyin.polygon.points.push_back(polyin.polygon.points[i]);
		polyin.polygon.points[polyin.polygon.points.size()-1].z = z1;
	}
	return polyin;
}
geometry_msgs::PolygonStamped get_bounding_polygon(nav_msgs::Path pathin, int num_points, float extra_length){
	geometry_msgs::PolygonStamped polyout;
	polyout.header = hdr();
	polyout.polygon.points.resize(num_points);
	geometry_msgs::Point centroid = get_ave_pnt(pathin);
	//pathin = sort_path(pathin,centroid);
	float a0 = -M_PI;
	float da = 2*M_PI / num_points;
	float an = a0 + da;
	float dst_max = 0;
	std::vector<float> ranges;
	ranges.resize(num_points);
	for(int i = 0; i < pathin.poses.size(); i++){
		float hdng = get_hdng(pathin.poses[i].pose.position,centroid);
		float dst  = get_dst2d(pathin.poses[i].pose.position,centroid);
		int ai 		 = int(round((hdng + M_PI)/da));
		if(dst > ranges[ai])
			ranges[ai] = dst;
	}
	ranges = fix_ranges(ranges);
	for(int i = 0; i < ranges.size(); i++){
		float hdng = -M_PI + i * da;
		polyout.polygon.points[i].x = centroid.x + (ranges[i]+extra_length) * cos(hdng);
		polyout.polygon.points[i].y = centroid.y + (ranges[i]+extra_length) * sin(hdng);
	}
	std::vector<float> mn_mx_ave = get_z_min_max_ave_path(pathin);
	polyout = elevate_poly(polyout,mn_mx_ave[0],mn_mx_ave[1]);
	return polyout;
}
tb_msgsrv::PolygonsStamped get_bounding_polygons(tb_msgsrv::PathsStamped pathsin){
	tb_msgsrv::PolygonsStamped polysout;
	polysout.header = pathsin.header;
	for(int i = 0; i < pathsin.paths.size(); i++){
		polysout.polygons.push_back(get_bounding_polygon(pathsin.paths[i],par_num_points,par_extra_length));
	}
	return polysout;
}
void get_paths_bbpolys_cb(const tb_msgsrv::PathsStamped::ConstPtr& msg){
	pub_polygons.publish(get_bounding_polygons(*msg));
}
void get_path_bbpoly_cb(const nav_msgs::Path::ConstPtr& msg){
	pub_polygon.publish(get_bounding_polygon(*msg,par_num_points,par_extra_length));
}
void get_auto_paths_bbpoly_cb(const tb_msgsrv::PathsStamped::ConstPtr& msg){
	if(par_auto)
		pub_polygons.publish(get_bounding_polygons(*msg));
}
int main(int argc, char** argv)
{
  ros::init(argc, argv, "tb_process_bbpolys_node");
  ros::NodeHandle nh;
	ros::NodeHandle private_nh("~");
	// THIS NODE: Creates bounding polygons from path(s).
	private_nh.param("num_points_bbpolys",   par_num_points, 32);
  private_nh.param("extra_radius_bbpolys", par_extra_length, 3.0);//*2.0);
	private_nh.param("auto_process_edto", par_auto, true);//*2.0);

	ros::Subscriber os = nh.subscribe("/tb_process/get_paths_bbpolys",1,get_paths_bbpolys_cb);
	ros::Subscriber o1 = nh.subscribe("/tb_process/get_path_bbpoly",1,get_path_bbpoly_cb);
	ros::Subscriber o2 = nh.subscribe("/tb_process/paths_clusters",1,get_auto_paths_bbpoly_cb);
	pub_polygons = nh.advertise<tb_msgsrv::PolygonsStamped>("/tb_process/paths_bbpolys",10);
	pub_polygon  = nh.advertise<geometry_msgs::PolygonStamped>("/tb_process/path_bbpoly",10);
	ros::spin();
	return 0;
}
//
