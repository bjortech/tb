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
double par_min_distance,par_maprad,par_dz;
double par_dst_target,par_dst_margin,par_dz_max,par_cluster_max_spacing;

nav_msgs::Path path_visited;
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
	pathout.header = ps.header = hdr();
	float z = (oct_zmin+oct_zmax)/2;
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
  return pathout;
}
std::vector<float> get_bbvec_from_midpoint(geometry_msgs::Point midpoint,float rad){
	std::vector<float> bbvec;
	bbvec.resize(6);
	bbvec[0] = midpoint.x - rad;
	bbvec[1] = midpoint.y - rad;
	bbvec[2] = midpoint.z - par_dz;
	bbvec[3] = midpoint.x + rad;
	bbvec[4] = midpoint.y + rad;
	bbvec[5] = midpoint.z + par_dz;
	return bbvec;
}
void get_side_path2d_large_cb(const geometry_msgs::PointStamped::ConstPtr& msg){
	if(got_map){
		ros::Time t0 = ros::Time::now();
		float collision_radius = par_dst_target + par_dst_margin + 2.0;
		std::vector<float> bbvec = get_bbvec_from_midpoint(msg->point,(par_maprad+collision_radius));
		if(!outside_octomap(bbvec) && update_edto_bbvec(bbvec,collision_radius)){
			nav_msgs::Path path_side2d = get_side();
			float dt = (ros::Time::now()-t0).toSec();
			ROS_INFO("PathSide2D[%.4f sec]: %i poses found within %.0fx%.0f around %.0f %.0f %.0f ",dt,path_side2d.poses.size(),oct_range_x,oct_range_y,msg->point.x,msg->point.y,msg->point.z);
			pub_path.publish(path_side2d);
		}
	}
}
void get_side_path2d_small_cb(const geometry_msgs::PointStamped::ConstPtr& msg){
	if(got_map){
		ros::Time t0 = ros::Time::now();
		float collision_radius = par_dst_target + par_dst_margin + 2.0;
		std::vector<float> bbvec = get_bbvec_from_midpoint(msg->point,(10+collision_radius));
		if(!outside_octomap(bbvec) && update_edto_bbvec(bbvec,collision_radius)){
			nav_msgs::Path path_side2d = get_side();
			float dt = (ros::Time::now()-t0).toSec();
			ROS_INFO("PathSide2D[%.4f sec]: %i poses found within %.0fx%.0f around %.0f %.0f %.0f ",dt,path_side2d.poses.size(),oct_range_x,oct_range_y,msg->point.x,msg->point.y,msg->point.z);
			pub_path.publish(path_side2d);
		}
	}
}
void octomap_callback(const octomap_msgs::Octomap& msg){
  abs_octree=octomap_msgs::fullMsgToMap(msg);
  octree.reset(dynamic_cast<octomap::OcTree*>(abs_octree));
  got_map = true;
}
int main(int argc, char** argv)
{
  ros::init(argc, argv, "tb_targeters_side2d_node");
	ros::NodeHandle nh;
	ros::NodeHandle private_nh("~");
	private_nh.param("update_area_sidelength", par_maprad, 30.0);
	private_nh.param("update_area_z_radius", par_dz, 1.5);

	private_nh.param("dst_target",par_dst_target, 6.0);
	private_nh.param("dst_margin",par_dst_margin, 1.0);

	ros::Subscriber s1  = nh.subscribe("/octomap_full",1,octomap_callback);
	ros::Subscriber s2  = nh.subscribe("/tb_edto/get_side2d",1,get_side_path2d_large_cb);
	ros::Subscriber s3  = nh.subscribe("/tb_edto/get_side2d_small",1,get_side_path2d_small_cb);
	pub_path  					= nh.advertise<nav_msgs::Path>("/tb_edto/path2d_side",100);
	ros::spin();
	return 0;
}
