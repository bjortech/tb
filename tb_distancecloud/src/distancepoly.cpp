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
ros::Publisher pub_path,pub_poly_cleared,pub_poly_obstacles;
geometry_msgs::Point pos,last_pos;
int oct_xmin,oct_ymin,oct_zmin,oct_xmax,oct_ymax,oct_zmax,oct_range_x,oct_range_y,oct_range_z;
float last_yaw = 0.0;
int count = 0;
ros::Time last_sec;
double par_min_distance,par_maprad,par_dz;
geometry_msgs::PolygonStamped poly_cleared,poly_obstacles;
nav_msgs::Path path_raycast;
bool par_track_only_last;
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

bool isPointInOctomap(Eigen::Vector3f vec){
	if(vec.x() > float(oct_xmin) && vec.y() > float(oct_ymin)
	&& vec.x() < float(oct_xmax) && vec.y() < float(oct_ymax))
		return true;
	else
		return false;
}
geometry_msgs::PolygonStamped raycastPolyClearedFromMidPoint(geometry_msgs::Point midpoint,float collision_radius){
	ROS_INFO("raycast_circle_edto");
	//This function takes origo and collision_radius
	int num_rays 		= 32;
	float rads_pr_i = 2*M_PI / num_rays;
	/*Poly_cleared will create the polygon that displays free space at fixed size (like sensor_msgs::LaserScan)
	* Poly_obstacles illustrate the points that resulted in breaking from the raycast function
	* Path displays points along the raycast and the Quaternion represents the rotation from point_of_ray(stridep) to obstacle_point
	*/
	geometry_msgs::PolygonStamped polyout;
	polyout.polygon.points.resize(num_rays);
	polyout.header 	= hdr();
	ps.header = hdr();
	geometry_msgs::Point32 closest_obstacle;
	for(int i  = 0; i < num_rays; i++){
		Eigen::Vector3f pnt1_vec(midpoint.x,midpoint.y,midpoint.z);
		Eigen::Vector3f pnt2_vec(midpoint.x+par_maprad*cos(i*rads_pr_i),
													   midpoint.y+par_maprad*sin(i*rads_pr_i),
														 midpoint.z);
		if(isPointInOctomap(pnt2_vec)){
			Eigen::Vector3f cur_vec = pnt1_vec;
			Eigen::Vector3f stride_vec = (pnt2_vec - pnt1_vec).normalized();
			float cur_ray_len = 0;
			float distance	  = collision_radius;
			//RAYCAST LOOP
			while(cur_ray_len < par_maprad && distance > par_min_distance){
				cur_vec 		= cur_vec + stride_vec;
				cur_ray_len = (cur_vec-pnt1_vec).norm();
				//CHECK that point is within octomap boundaries - looking up a point that's not available causes crash

					point3d stridep(cur_vec.x(),cur_vec.y(),cur_vec.z());
					distance = edf_ptr.get()->getDistance(stridep);
				}
				else
					break;
			}
		}

		polyout.polygon.points[i].x = cur_vec.x();
		polyout.polygon.points[i].y = cur_vec.y();
		polyout.polygon.points[i].z = midpoint.z;
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

void octomap_callback(const octomap_msgs::Octomap& msg){
  abs_octree=octomap_msgs::fullMsgToMap(msg);
  octree.reset(dynamic_cast<octomap::OcTree*>(abs_octree));
  got_map = true;
}
int main(int argc, char** argv)
{
  ros::init(argc, argv, "tb_edto_poly_auto2_node");
	ros::NodeHandle nh;
	ros::NodeHandle private_nh("~");
	private_nh.param("only_track_last_obstacle", par_track_only_last, false);
	private_nh.param("update_area_sidelength", par_maprad, 30.0);
	private_nh.param("update_area_z_radius", par_dz, 1.5);
	private_nh.param("obstacle_distance_cutoff", par_min_distance, 3.0);
	poly_obstacles.header = hdr();
	poly_cleared.header 	= hdr();
	path_raycast.header   = hdr();
	tf2_ros::TransformListener tf2_listener(tfBuffer);

	ros::Subscriber s1  = nh.subscribe("/octomap_full",1,octomap_callback);
	pub_poly_cleared   	= nh.advertise<geometry_msgs::PolygonStamped>("/tb_edto/poly_cleared",100);
	pub_poly_obstacles  = nh.advertise<geometry_msgs::PolygonStamped>("/tb_edto/poly_obstacles",100);
	pub_path  					= nh.advertise<nav_msgs::Path>("/tb_edto/path_raycast",100);
	ros::Rate rate(1.0);
	while(ros::ok()){

		float yaw = tf::getYaw(transformStamped.transform.rotation);
		if(got_map){
				std::vector<float> bbvec;
				bbvec.resize(6);
				bbvec[0] = pos.x - par_maprad;
				bbvec[1] = pos.y - par_maprad;
				bbvec[2] = altlvl * par_dz - par_dz;
				bbvec[3] = pos.x + par_maprad;
				bbvec[4] = pos.y + par_maprad;
				bbvec[5] = altlvl * par_dz + par_dz;
				float collision_radius = 10;
				if(!outside_octomap(bbvec)){
					if(update_edto_bbvec(bbvec,collision_radius))
							raycast_circle_edto(pos,collision_radius);
				}
			}
		}
	}
		update_pos();
		if(poly_cleared.polygon.points.size() > 0){
			pub_poly_obstacles.publish(poly_obstacles);
		}

		rate.sleep();
		ros::spinOnce();
	}
	return 0;
}
