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
int g_oct_xmin,g_oct_ymin,g_oct_zmin,g_oct_xmax,g_oct_ymax,g_oct_zmax,g_oct_range_x,g_oct_range_y,g_oct_range_z;
double par_collision_radius,par_dz,par_maprad,par_min_distance;
int par_num_rays;
bool ready;
std_msgs::Header hdr(){
  std_msgs::Header header;
  header.frame_id = "map";
  header.stamp    = ros::Time::now();
  return header;
}

float get_dst2d(geometry_msgs::Point p2, geometry_msgs::Point p1){
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
float get_inclination(geometry_msgs::Point p2, geometry_msgs::Point p1){
  return atan2(p2.z - p1.z,get_dst2d(p2,p1));
}
std::vector<float> getBoundingBoxAroundPoint(geometry_msgs::Point midpoint,float max_ray_len){
	std::vector<float> bbvec;
	bbvec.resize(6);
	bbvec[0] = midpoint.x - max_ray_len;
	bbvec[1] = midpoint.y - max_ray_len;
	bbvec[2] = midpoint.z - par_dz;
	bbvec[3] = midpoint.x + max_ray_len;
	bbvec[4] = midpoint.y + max_ray_len;
	bbvec[5] = midpoint.z +	par_dz;
	return bbvec;
}
bool updateOctomapEDTO(std::vector<float> bbvec,float collision_radius){
	//Updates boundaries based on desired points (bbvec) and octomap extents
  geometry_msgs::Point bbmin_octree,bbmax_octree;
  octree.get()->getMetricMin(bbmin_octree.x,bbmin_octree.y,bbmin_octree.z);
  octree.get()->getMetricMax(bbmax_octree.x,bbmax_octree.y,bbmax_octree.z);
  octomap::point3d boundary_min(fmax(bbvec[0],bbmin_octree.x),fmax(bbvec[1],bbmin_octree.y),fmax(bbvec[2],bbmin_octree.z));
  octomap::point3d boundary_max(fmin(bbvec[3],bbmax_octree.x),fmin(bbvec[4],bbmax_octree.y),fmin(bbvec[5],bbmax_octree.z));
	ROS_INFO("DistancePoly2D-bbvec: %.0f %.0f %.0f %.0f %.0f %.0f",bbvec[0],bbvec[1],bbvec[2],bbvec[3],bbvec[4],bbvec[5]);
	ROS_INFO("DistancePoly2D-bboct: %.0f %.0f %.0f %.0f %.0f %.0f",bbmin_octree.x,bbmin_octree.y,bbmin_octree.z,bbmax_octree.x,bbmax_octree.y,bbmax_octree.z);
	ROS_INFO("DistancePoly2D-bbout: %.0f %.0f %.0f %.0f %.0f %.0f",boundary_min.x(),boundary_min.y(),boundary_min.z(),boundary_max.x(),boundary_max.y(),boundary_max.z());
  edf_ptr.reset (new DynamicEDTOctomap(collision_radius,octree.get(),
          boundary_min,
          boundary_max,
          false));
  edf_ptr.get()->update();
  g_oct_xmin    = int(round(boundary_min.x()))+1;  g_oct_ymin    = int(round(boundary_min.y()))+1; g_oct_zmin    = int(round(boundary_min.z()))+1;
  g_oct_xmax    = int(round(boundary_max.x()))-1;  g_oct_ymax    = int(round(boundary_max.y()))-1; g_oct_zmax    = int(round(boundary_max.z()))-1;
  g_oct_range_x = g_oct_xmax - g_oct_xmin;           	 g_oct_range_y = g_oct_ymax - g_oct_ymin;    				g_oct_range_z = g_oct_zmax - g_oct_zmin;
	int vol = g_oct_range_x*g_oct_range_y*g_oct_range_z;
  if(vol <= 0)
		return false;
	return true;
}

bool isPointInOctomap(Eigen::Vector3f vec){
	if(vec.x() > float(g_oct_xmin) && vec.y() > float(g_oct_ymin)
	&& vec.x() < float(g_oct_xmax) && vec.y() < float(g_oct_ymax))
		return true;
	else
		return false;
}
geometry_msgs::PolygonStamped raycastPolyClearedFromMidPoint(geometry_msgs::Point midpoint,int num_rays,float max_ray_len,float obstacle_distance_cutoff){
	float rads_pr_i = 2*M_PI / num_rays;
	/*
	* Create cleared polygon around point
	*/
	geometry_msgs::PolygonStamped polyout;
	polyout.polygon.points.resize(num_rays);
	polyout.header 	= hdr();
	for(int i  = 0; i < num_rays; i++){
		Eigen::Vector3f pnt1_vec(midpoint.x,midpoint.y,midpoint.z);
		Eigen::Vector3f pnt2_vec(midpoint.x+max_ray_len*cos(i*rads_pr_i),
													   midpoint.y+max_ray_len*sin(i*rads_pr_i),
														 midpoint.z);
		Eigen::Vector3f cur_vec = pnt1_vec;
		Eigen::Vector3f stride_vec = (pnt2_vec - pnt1_vec).normalized();
		float cur_ray_len = 0;
		float distance	  = obstacle_distance_cutoff+5;
		//RAYCAST LOOP
		while(cur_ray_len < max_ray_len && distance > obstacle_distance_cutoff){
			if(cur_vec.x() > float(g_oct_xmin) && cur_vec.y() > float(g_oct_ymin)
			&& cur_vec.x() < float(g_oct_xmax) && cur_vec.y() < float(g_oct_ymax)){
				//	if(isPointInOctomap(pnt2_vec)){
				cur_vec 		= cur_vec + stride_vec;
				cur_ray_len = (cur_vec-pnt1_vec).norm();
				point3d stridep(cur_vec.x(),cur_vec.y(),cur_vec.z());
				distance = edf_ptr.get()->getDistance(stridep);
			}
			else
				break;
		}
		polyout.polygon.points[i].x = cur_vec.x();
		polyout.polygon.points[i].y = cur_vec.y();
		polyout.polygon.points[i].z = cur_vec.z();
	}
	return polyout;
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
geometry_msgs::Point getPointInMap(std::string frame){
	geometry_msgs::TransformStamped transformStamped;
	try{
		transformStamped = tfBuffer.lookupTransform("map",frame,
														 ros::Time(0));
	}
	catch (tf2::TransformException &ex) {
		ROS_WARN("%s",ex.what());
		ros::Duration(1.0).sleep();
	}
	geometry_msgs::Point pout;
	pout.x = transformStamped.transform.translation.x;
	pout.y = transformStamped.transform.translation.y;
	pout.z = transformStamped.transform.translation.z;
	return pout;
}
void octomap_callback(const octomap_msgs::Octomap& msg){
  abs_octree=octomap_msgs::fullMsgToMap(msg);
  octree.reset(dynamic_cast<octomap::OcTree*>(abs_octree));
  got_map = true;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "tb_distancepoly2d_node");
	ros::NodeHandle nh;
	ros::NodeHandle private_nh("~");
	private_nh.param("update_area_z_radius", par_dz, 1.5);
	private_nh.param("maximum_raylength", par_maprad, 50.0);
	private_nh.param("minimum_obstacle_clearing", par_min_distance, 3.0);
	private_nh.param("number_of_rays", par_num_rays, 32);

	tf2_ros::TransformListener tf2_listener(tfBuffer);

	ros::Subscriber s1  = nh.subscribe("/octomap_full",1,octomap_callback);
	ros::Publisher pub_poly_cleared	= nh.advertise<geometry_msgs::PolygonStamped>("/tb_distancecloud/poly_boundary",100);
	ros::Rate rate(1.0);
	float collision_radius = par_dz +	par_min_distance;
	int z_count = 0;
	ready = true;
	while(ros::ok()){
		if(got_map){
			geometry_msgs::Point midpoint = getPointInMap("base_perfect_alt");
			std::vector<float> bbvec = getBoundingBoxAroundPoint(midpoint,par_maprad+collision_radius);
			if(!outside_octomap(bbvec)){
				if(updateOctomapEDTO(bbvec,collision_radius))
					pub_poly_cleared.publish(raycastPolyClearedFromMidPoint(midpoint,par_num_rays,par_maprad,collision_radius));
			}
		}
		rate.sleep();
		ros::spinOnce();
	}
	return 0;
}
