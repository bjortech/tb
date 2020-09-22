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
#include <tb_msgsrv/PolygonsStamped.h>


tf2_ros::Buffer tfBuffer;
using namespace octomap;
using namespace std;
shared_ptr<DynamicEDTOctomap> edf_ptr;
AbstractOcTree* abs_octree;
shared_ptr<OcTree> octree;
bool got_map = false;
int oct_xmin,oct_ymin,oct_zmin,oct_xmax,oct_ymax,oct_zmax,oct_range_x,oct_range_y,oct_range_z;
double par_poly_radius;
geometry_msgs::PolygonStamped poly_cleared,poly_obstacles,poly_unknown;
geometry_msgs::PolygonStamped master_poly_cleared,master_poly_unknown,master_poly_obstacles;
tb_msgsrv::PolygonsStamped polys_obstacles,polys_unknown,polys_cleared;
double par_dst_target,par_dst_margin,par_dz_max;
ros::Publisher pub_polys_cleared,pub_polys_unknown,pub_polys_obstacles;
int pub_count = 0;
std_msgs::Header hdr(){
  std_msgs::Header header;
  header.frame_id = "map";
  header.stamp    = ros::Time::now();
  return header;
}
bool update_edto_bbvec(std::vector<float> bbvec,float collision_radius){
  geometry_msgs::Point bbmin_octree,bbmax_octree;
  octree.get()->getMetricMin(bbmin_octree.x,bbmin_octree.y,bbmin_octree.z);
  octree.get()->getMetricMax(bbmax_octree.x,bbmax_octree.y,bbmax_octree.z);

  octomap::point3d boundary_min(fmax(bbvec[0],bbmin_octree.x),fmax(bbvec[1],bbmin_octree.y),fmax(bbvec[2],bbmin_octree.z));
  octomap::point3d boundary_max(fmin(bbvec[3],bbmax_octree.x),fmin(bbvec[4],bbmax_octree.y),fmin(bbvec[5],bbmax_octree.z));
	ROS_INFO("POLY bbvec: %.0f %.0f %.0f %.0f %.0f %.0f",bbvec[0],bbvec[1],bbvec[2],bbvec[3],bbvec[4],bbvec[5]);
	ROS_INFO("POLY bboct: %.0f %.0f %.0f %.0f %.0f %.0f",bbmin_octree.x,bbmin_octree.y,bbmin_octree.z,bbmax_octree.x,bbmax_octree.y,bbmax_octree.z);
	ROS_INFO("POLY bbout: %.0f %.0f %.0f %.0f %.0f %.0f",boundary_min.x(),boundary_min.y(),boundary_min.z(),boundary_max.x(),boundary_max.y(),boundary_max.z());
  edf_ptr.reset (new DynamicEDTOctomap(collision_radius,octree.get(),
          boundary_min,
          boundary_max,
          false));
  edf_ptr.get()->update();
  oct_xmin    = int(round(boundary_min.x()))+1;  oct_ymin    = int(round(boundary_min.y()))+1; oct_zmin    = int(round(boundary_min.z()))+1;
  oct_xmax    = int(round(boundary_max.x()))-1;  oct_ymax    = int(round(boundary_max.y()))-1; oct_zmax    = int(round(boundary_max.z()))-1;
  oct_range_x = oct_xmax - oct_xmin;                     oct_range_y = oct_ymax - oct_ymin;                    oct_range_z = oct_zmax - oct_zmin;
  int vol = oct_range_x*oct_range_y*oct_range_z;
  if(vol <= 0)
		return false;
	return true;
}

void octomap_callback(const octomap_msgs::Octomap& msg){
  abs_octree=octomap_msgs::fullMsgToMap(msg);
  octree.reset(dynamic_cast<octomap::OcTree*>(abs_octree));
  got_map = true;
}
void reset_polygons(int num_rays){
	poly_obstacles.header = hdr();
	poly_obstacles.polygon.points.resize(0);
	poly_obstacles.polygon.points.resize(num_rays);
	poly_cleared = poly_unknown = poly_obstacles;
	poly_unknown.polygon.points.resize(num_rays*2);
}
void get_poly(geometry_msgs::Point midpoint,float collision_radius){
	int num_rays = 32;
	float rads_pr_i = 2*M_PI / num_rays;
	reset_polygons(num_rays);
	for(int i  = 0; i < num_rays; i++){
		Eigen::Vector3f pnt1_vec(midpoint.x,midpoint.y,midpoint.z);
		Eigen::Vector3f pnt2_vec(midpoint.x+par_poly_radius*cos(i*rads_pr_i),par_poly_radius*sin(i*rads_pr_i)+midpoint.y,midpoint.z);

		poly_unknown.polygon.points[i].x = pnt1_vec.x();
		poly_unknown.polygon.points[i].y = pnt1_vec.y();
		poly_unknown.polygon.points[i].z = midpoint.z;

		poly_unknown.polygon.points[poly_unknown.polygon.points.size()-1-i].x = pnt2_vec.x();
		poly_unknown.polygon.points[poly_unknown.polygon.points.size()-1-i].y = pnt2_vec.y();
		poly_unknown.polygon.points[poly_unknown.polygon.points.size()-1-i].z = midpoint.z;
		poly_obstacles.polygon.points[i] = poly_unknown.polygon.points[poly_unknown.polygon.points.size()-1-i];

		Eigen::Vector3f cur_vec = pnt1_vec;
		Eigen::Vector3f stride_vec;
		Eigen::Vector3f last_valid;
		stride_vec = (pnt2_vec - pnt1_vec).normalized();
		float cur_ray_len = 0;
		float distance = collision_radius;

		while(cur_ray_len < par_poly_radius){
			cur_vec 		= cur_vec + stride_vec;
			cur_ray_len = (cur_vec-pnt1_vec).norm();
			if(cur_vec.x() > float(oct_xmin) && cur_vec.y() > float(oct_ymin)
			&& cur_vec.x() < float(oct_xmax) &&  cur_vec.y() < float(oct_ymax)){
				point3d stridep(cur_vec.x(),cur_vec.y(),cur_vec.z());
				point3d closestObst;
				//distance = edf_ptr.get()->getDistance(stridep);
				edf_ptr.get()->getDistanceAndClosestObstacle(stridep,distance,closestObst);
				if(distance < par_dst_margin){
					poly_obstacles.polygon.points[i].x = closestObst.x();
					poly_obstacles.polygon.points[i].y = closestObst.y();
					poly_obstacles.polygon.points[i].z = closestObst.z();
					break;
				}
				poly_unknown.polygon.points[i].x = cur_vec.x();
				poly_unknown.polygon.points[i].y = cur_vec.y();
				poly_unknown.polygon.points[i].z = midpoint.z;
			}
			else
				break;
		}

		poly_cleared.polygon.points[i].x = cur_vec.x();
		poly_cleared.polygon.points[i].y = cur_vec.y();
		poly_cleared.polygon.points[i].z = midpoint.z;
	}
	polys_obstacles.polygons.push_back(poly_obstacles);
	polys_unknown.polygons.push_back(poly_unknown);
	polys_cleared.polygons.push_back(poly_cleared);
}

void display(std::vector<float> vec){
	for(int i = 1; i < vec.size(); i++){
		ROS_INFO("Vec[%i->%i][%.0f->%.0f]: %.0f",i-1,i,vec[i-1],vec[i],(vec[i]+vec[i-1])/2);
	}
}
std::vector<float> get_zn(float z0, float z1, float z_intervals){
	geometry_msgs::Point bbmin_octree,bbmax_octree;
	octree.get()->getMetricMin(bbmin_octree.x,bbmin_octree.y,bbmin_octree.z);
	octree.get()->getMetricMax(bbmax_octree.x,bbmax_octree.y,bbmax_octree.z);
	float z0_actual = fmax(z0,bbmin_octree.z);
	float z1_actual = fmin(z1,bbmax_octree.z);
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
void get_poly_cb(const geometry_msgs::PolygonStamped::ConstPtr& msg){
	if(got_map){
		std::vector<float> bbvec;
		bbvec.resize(6);
		bbvec[0] = bbvec[1] = bbvec[2] = 1000;
		bbvec[3] = bbvec[4] = bbvec[5] = -1000;
		float collision_radius = 7.0;
		polys_cleared.polygons.resize(0);
		polys_unknown.polygons.resize(0);
		polys_obstacles.polygons.resize(0);
		for(int i = 0; i < msg->polygon.points.size(); i++){
			if(msg->polygon.points[i].x < bbvec[0])
				bbvec[0] = msg->polygon.points[i].x;
			if(msg->polygon.points[i].y < bbvec[1])
				bbvec[1] = msg->polygon.points[i].y;
			if(msg->polygon.points[i].z < bbvec[2])
				bbvec[2] = msg->polygon.points[i].z;
			if(msg->polygon.points[i].x > bbvec[3])
				bbvec[3] = msg->polygon.points[i].x;
			if(msg->polygon.points[i].y > bbvec[4])
				bbvec[4] = msg->polygon.points[i].y;
			if(msg->polygon.points[i].z > bbvec[5])
				bbvec[5] = msg->polygon.points[i].z;
		}
		geometry_msgs::Point midpoint;
		midpoint.x = (bbvec[0]+bbvec[3])/2;
		midpoint.y = (bbvec[1]+bbvec[4])/2;
		par_poly_radius = fmax(abs(bbvec[3]-bbvec[0]),abs(bbvec[4]-bbvec[1]))/2+10;
		ROS_INFO("Midpoint: %.0f %.0f %.0f - radius: %.0f",midpoint.x,midpoint.y,midpoint.z,par_poly_radius);

		std::vector<nav_msgs::Path> paths;
		std::vector<float> zn = get_zn(bbvec[2],bbvec[5],par_dz_max);
		display(zn);
		if(zn.size() >= 2){
			for(int i = 0; i < zn.size(); i++){
				bbvec[2] 	 = zn[i];
				bbvec[5] 	 = zn[i+1];
				midpoint.z = (zn[i+1] + zn[i])/2.0;
				if(bbvec[5] > bbvec[2] && update_edto_bbvec(bbvec,collision_radius))
					get_poly(midpoint,collision_radius);
			}
		}
		else if(bbvec[5] > bbvec[2] && update_edto_bbvec(bbvec,collision_radius))
			get_poly(midpoint,collision_radius);

		polys_obstacles.header = hdr();
		polys_unknown.header = hdr();
		polys_cleared.header = hdr();
		pub_polys_cleared.publish(polys_cleared);
	}
}
void mainstate_cb(const std_msgs::UInt8::ConstPtr& msg){
	pub_count++;
	if(pub_count == 1){
		pub_polys_cleared.publish(polys_cleared);
	}
	else if(pub_count == 2){
		pub_polys_unknown.publish(polys_unknown);
	}
	else if(pub_count == 3){
		pub_polys_obstacles.publish(polys_obstacles);
	}
	else
		pub_count = 0;
}
int main(int argc, char** argv)
{
  ros::init(argc, argv, "tb_edto_poly_node");
  ros::NodeHandle nh;
	ros::NodeHandle private_nh("~");
	private_nh.param("poly_radius",par_poly_radius, 6.0);
	private_nh.param("dst_margin",par_dst_margin, 2.0);
	private_nh.param("dz_max",par_dz_max, 3.0);
	ros::Subscriber s1  = nh.subscribe("/octomap_full",1,octomap_callback);
	ros::Subscriber s2  = nh.subscribe("/tb_edto/get_poly",1,get_poly_cb);
	ros::Subscriber s3  = nh.subscribe("/tb_fsm/main_state",1,mainstate_cb);
	pub_polys_cleared    = nh.advertise<tb_msgsrv::PolygonsStamped>("/tb_edto/polys_cleared",10);
	pub_polys_unknown 	  = nh.advertise<tb_msgsrv::PolygonsStamped>("/tb_edto/polys_obstacles",10);
	pub_polys_obstacles  = nh.advertise<tb_msgsrv::PolygonsStamped>("/tb_edto/polys_unknown",10);
	ros::spin();
  return 0;
}
