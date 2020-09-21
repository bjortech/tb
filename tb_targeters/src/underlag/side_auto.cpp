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
shared_ptr<DynamicEDTOctomap> edf_ptr;
AbstractOcTree* abs_octree;
shared_ptr<OcTree> octree;
bool got_map = false;
ros::Publisher pub_path;
geometry_msgs::Point pos,last_pos;
int oct_xmin,oct_ymin,oct_zmin,oct_xmax,oct_ymax,oct_zmax,oct_range_x,oct_range_y,oct_range_z;
float last_yaw = 0.0;
int count = 0;
ros::Time last_sec;
double par_min_distance,par_maprad,par_dz;
double par_dst_target,par_dst_margin,par_dz_max,par_cluster_max_spacing;

geometry_msgs::PolygonStamped poly_cleared,poly_obstacles;
nav_msgs::Path path_raycast,path_visited;
std::vector<std::vector<geometry_msgs::Point32>> clusters;
std_msgs::Header hdr(){
  std_msgs::Header header;
  header.frame_id = "map";
  header.stamp    = ros::Time::now();
  return header;
}
float get_dst2d(geometry_msgs::Point p2, geometry_msgs::Point p1){
  return(sqrt(pow(p1.x-p2.x,2)+pow(p1.y-p2.y,2)));
}
float get_dst2d32(geometry_msgs::Point32 p2, geometry_msgs::Point32 p1){
  return(sqrt(pow(p1.x-p2.x,2)+pow(p1.y-p2.y,2)));
}
float get_dst2d32d(geometry_msgs::Point p2, geometry_msgs::Point32 p1){
  return(sqrt(pow(p1.x-p2.x,2)+pow(p1.y-p2.y,2)));
}
float get_dst3d(geometry_msgs::Point p2, geometry_msgs::Point p1){
  return(sqrt(pow(p1.x-p2.x,2)+pow(p1.y-p2.y,2)+pow(p1.z-p2.z,2)));
}
float get_hdng(geometry_msgs::Point p1,geometry_msgs::Point p0){
  float dx = p1.x - p0.x;
  float dy = p1.y - p0.y;
  return atan2(dy,dx);
}
double get_shortest(double target_hdng,double actual_hdng){
  double a = target_hdng - actual_hdng;
  if(a > M_PI)a -= M_PI*2;
  else if(a < -M_PI)a += M_PI*2;
  return a;
}
bool is_point32_not_visited(geometry_msgs::Point32 pin,float lim){
  float res,dst;
  if(path_visited.poses.size() == 0)
    return true;
  for(int i = 0; i < path_visited.poses.size(); i++){
     if(get_dst2d32d(path_visited.poses[i].pose.position,pin) < lim)
        return false;
  }
  return true;
}
float get_inclination(geometry_msgs::Point p2, geometry_msgs::Point p1){
  return atan2(p2.z - p1.z,get_dst2d(p2,p1));
}
bool update_edto_bbvec(std::vector<float> bbvec,float collision_radius){
	//Updates boundaries based on desired points (bbvec) and octomap extents
  geometry_msgs::Point bbmin_octree,bbmax_octree;
  octree.get()->getMetricMin(bbmin_octree.x,bbmin_octree.y,bbmin_octree.z);
  octree.get()->getMetricMax(bbmax_octree.x,bbmax_octree.y,bbmax_octree.z);
  octomap::point3d boundary_min(fmax(bbvec[0],bbmin_octree.x),fmax(bbvec[1],bbmin_octree.y),fmax(bbvec[2],bbmin_octree.z));
  octomap::point3d boundary_max(fmin(bbvec[3],bbmax_octree.x),fmin(bbvec[4],bbmax_octree.y),fmin(bbvec[5],bbmax_octree.z));

  edf_ptr.reset (new DynamicEDTOctomap(collision_radius,octree.get(),
          boundary_min,
          boundary_max,
          false));
  edf_ptr.get()->update();
  oct_xmin    = int(round(boundary_min.x()))+1;  oct_ymin    = int(round(boundary_min.y()))+1; oct_zmin    = int(round(boundary_min.z()))+1;
  oct_xmax    = int(round(boundary_max.x()))-1;  oct_ymax    = int(round(boundary_max.y()))-1; oct_zmax    = int(round(boundary_max.z()))-1;
  oct_range_x = oct_xmax - oct_xmin;           	 oct_range_y = oct_ymax - oct_ymin;    				oct_range_z = oct_zmax - oct_zmin;
	int vol = oct_range_x*oct_range_y*oct_range_z;
  if(vol <= 0)
		return false;
	return true;
}
geometry_msgs::Point32 p_to_pnt32(point3d p){
	geometry_msgs::Point32 pout;
	pout.x = p.x();
	pout.y = p.y();
	pout.z = p.z();
	return pout;
}
geometry_msgs::Point p_to_pnt(point3d p){
	geometry_msgs::Point pout;
	pout.x = p.x();
	pout.y = p.y();
	pout.z = p.z();
	return pout;
}
geometry_msgs::Point e_to_pnt(Eigen::Vector3f e){
	geometry_msgs::Point pout;
	pout.x = e.x();
	pout.y = e.y();
	pout.z = e.z();
	return pout;
}
bool outside_octomap(std::vector<float> bbvec){
	//If EDTO is updated with boundaries completely outside of octomap bounds it crashes
	//this wouldn't happen in a live test, but with limited maps it tends to happen now and then
	//												 axis-min > bbmax_octree.axis
	// 											   axis-min < bbmin_octree.axis

	geometry_msgs::Point bbmin_octree,bbmax_octree;
  octree.get()->getMetricMin(bbmin_octree.x,bbmin_octree.y,bbmin_octree.z);
  octree.get()->getMetricMax(bbmax_octree.x,bbmax_octree.y,bbmax_octree.z);
	if(bbvec[0] > bbmax_octree.x || bbvec[1] > bbmax_octree.y || bbvec[2] > bbmax_octree.z
	|| bbvec[3] < bbmin_octree.x || bbvec[4] < bbmin_octree.y || bbvec[5] < bbmin_octree.z)
		return true;
	else
		return false;
}
nav_msgs::Path get_side(){
	nav_msgs::Path pathout;
	geometry_msgs::PoseStamped ps;
	pathout.header.frame_id = ps.header.frame_id = "map";
	pathout.header.stamp = ps.header.stamp = ros::Time::now();
	for(int z = oct_zmin; z < oct_zmax; z++){
		for(int y = oct_ymin; y < oct_ymax; y++){
	    for(int x = oct_xmin; x < oct_xmax; x++){
	      octomap::point3d p(x,y,z);
	      octomap::point3d closestObst;
	      float d;
	      edf_ptr.get()->getDistanceAndClosestObstacle(p,d,closestObst);
	      if(abs(d - par_dst_target) < par_dst_margin){
	        ps.pose.orientation = tf::createQuaternionMsgFromYaw(atan2(closestObst.y() - p.y(),closestObst.x() - p.x()));
					ps.pose.position.x = x;					ps.pose.position.y = y;					ps.pose.position.z = z;
	        pathout.poses.push_back(ps);
	      }
	    }
		}
  }
  return pathout;
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
	float yaw = tf::getYaw(transformStamped.transform.rotation);
}

std::vector<float> get_zn(float z0, float z1, float z_intervals){
	geometry_msgs::Point bbmin_octree,bbmax_octree;
	octree.get()->getMetricMin(bbmin_octree.x,bbmin_octree.y,bbmin_octree.z);
	octree.get()->getMetricMax(bbmax_octree.x,bbmax_octree.y,bbmax_octree.z);
	float z0_actual = fmax(z0,bbmin_octree.z+1);
	float z1_actual = fmin(z1,bbmax_octree.z-1);
	float z_levels  = (z1_actual-z0_actual) / z_intervals;
	std::vector<float> zvec;
	float zval = z0_actual;
	while(zval < z1_actual){
		zval += z_intervals;
		if(zval < z1_actual)
			zvec.push_back(zval);
		else
			return zvec;
	}
	return zvec;
}

nav_msgs::Path get_side_path(std::vector<geometry_msgs::Point32> cluster){
	nav_msgs::Path pathout;
	std::vector<float> bbvec;
	bbvec.resize(6);
	bbvec[0] = bbvec[1] = bbvec[2] = 1000;
	bbvec[3] = bbvec[4] = bbvec[5] = -1000;
	float collision_radius = par_dst_target + par_dst_margin + 2.0;

	for(int i = 0; i < cluster.size(); i++){
		if(cluster[i].x < bbvec[0])
			bbvec[0] = cluster[i].x;
		if(cluster[i].y < bbvec[1])
			bbvec[1] = cluster[i].y;
		if(cluster[i].z < bbvec[2])
			bbvec[2] = cluster[i].z;
		if(cluster[i].x > bbvec[3])
			bbvec[3] = cluster[i].x;
		if(cluster[i].y > bbvec[4])
			bbvec[4] = cluster[i].y;
		if(cluster[i].z > bbvec[5])
			bbvec[5] = cluster[i].z;
	}
	bbvec[0] -= 10.0;
	bbvec[1] -= 10.0;
	bbvec[2] -= 10.0;
	bbvec[4] += 10.0;
	bbvec[2] += 10.0;
	bbvec[5] += 10.0;

	std::vector<nav_msgs::Path> paths;
	std::vector<float> zn = get_zn(bbvec[2],bbvec[5],par_dz_max*2);
	if(zn.size() >= 2){
		for(int i = 0; i < zn.size(); i++){
			bbvec[2] 	 = zn[i];
			bbvec[5] 	 = zn[i+1];
			if(bbvec[5] > bbvec[2] && update_edto_bbvec(bbvec,collision_radius))
				paths.push_back(get_side());
		}
	}
	else if(bbvec[5] > bbvec[2] && update_edto_bbvec(bbvec,collision_radius))
		paths.push_back(get_side());

	if(paths.size() > 0){
		pathout = paths[0];
		for(int i =0; i < paths.size(); i++){
			for(int k =0; k < paths[i].poses.size(); k++){
				pathout.poses.push_back(paths[i].poses[k]);
			}
		}
	}
	return pathout;
}
void octomap_callback(const octomap_msgs::Octomap& msg){
  abs_octree=octomap_msgs::fullMsgToMap(msg);
  octree.reset(dynamic_cast<octomap::OcTree*>(abs_octree));
  got_map = true;
}
std::vector<std::vector<geometry_msgs::Point32>> find_clusters(geometry_msgs::PolygonStamped polyin){
	std::vector<std::vector<geometry_msgs::Point32>> clustersout;
	std::vector<geometry_msgs::Point32> cluster;
	for(int i = 0; i < polyin.polygon.points.size(); i++){
		if(get_dst2d32(polyin.polygon.points[i],polyin.polygon.points[i-1]) < par_cluster_max_spacing)
			cluster.push_back(polyin.polygon.points[i]);
		else if(cluster.size() > 0){
			ROS_INFO("Cluster[%i]: %i pnts",clustersout.size(),cluster.size());
			clustersout.push_back(cluster);
			cluster.resize(0);
		}
	}
	ROS_INFO("Returning %i",clustersout.size());
	return clustersout;
}

geometry_msgs::PolygonStamped remove_visited(geometry_msgs::PolygonStamped polyin){
	geometry_msgs::PolygonStamped polyout;
	polyout.header = hdr();
	for(int i = 0; i < polyin.polygon.points.size(); i++){
		if(is_point32_not_visited(polyin.polygon.points[i],7)){
			polyout.polygon.points.push_back(polyin.polygon.points[i]);
		}
	}
	return polyout;
}

void poly_obstacles_cb(const geometry_msgs::PolygonStamped::ConstPtr& msg){
	poly_obstacles = remove_visited(*msg);
	if(poly_obstacles.polygon.points.size() > 0)
		clusters = find_clusters(poly_obstacles);
}
void path_raycast_cb(const nav_msgs::Path::ConstPtr& msg){
	path_raycast = *msg;
}
void path_visited_cb(const nav_msgs::Path::ConstPtr& msg){
	path_visited = *msg;
}
int main(int argc, char** argv)
{
  ros::init(argc, argv, "tb_edto_side_auto_node");
	ros::NodeHandle nh;
	ros::NodeHandle private_nh("~");
	private_nh.param("update_area_sidelength", par_maprad, 30.0);
	private_nh.param("update_area_z_radius", par_dz, 1.5);
	private_nh.param("obstacle_distance_cutoff", par_min_distance, 3.0);
	private_nh.param("cluster_spacing", par_cluster_max_spacing, 7.0);

	private_nh.param("dst_target",par_dst_target, 6.0);
	private_nh.param("dst_margin",par_dst_margin, 1.0);
	private_nh.param("dz_max",par_dz_max, 2.0);
	poly_obstacles.header = hdr();
	poly_cleared.header 	= hdr();
	path_raycast.header   = hdr();
	tf2_ros::TransformListener tf2_listener(tfBuffer);

	ros::Subscriber s1  = nh.subscribe("/octomap_full",1,octomap_callback);
	ros::Subscriber s2  = nh.subscribe("/tb_edto/poly_obstacles",1,poly_obstacles_cb);
	ros::Subscriber s3  = nh.subscribe("/tb_edto/path_raycast",1,path_raycast_cb);
	ros::Subscriber s4  = nh.subscribe("/tb_world/path_visited",1,path_visited_cb);
	pub_path  					= nh.advertise<nav_msgs::Path>("/tb_edto/path_side",100);
	ros::Rate rate(1.0);
	int cluster_count = 0;
	nav_msgs::Path path_side;
	while(ros::ok()){
		update_pos();
		if(got_map){
			if(clusters.size() > 0){
				if(cluster_count < clusters.size()){
					ROS_INFO("clusters[%i],cluster_count: %i pathout: %i",clusters.size(),cluster_count,path_side.poses.size());
					path_side = get_side_path(clusters[cluster_count]);
					cluster_count++;
					pub_path.publish(path_side);
				}
				else
					cluster_count = 0;
			}
		}
		rate.sleep();
		ros::spinOnce();
	}
	return 0;
}
