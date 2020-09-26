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


tf2_ros::Buffer tfBuffer;
using namespace octomap;
using namespace std;
shared_ptr<DynamicEDTOctomap> edf_ptr;
AbstractOcTree* abs_octree;
shared_ptr<OcTree> octree;
bool got_map = false;
int oct_xmin,oct_ymin,oct_zmin,oct_xmax,oct_ymax,oct_zmax,oct_range_x,oct_range_y,oct_range_z;
ros::Publisher pub_path,pub_path_cleared,pub_poly_obs;
double par_sphere_radius;
int cnt = 0;
ros::Time last_sec;
geometry_msgs::Point pos,last_pos;
float last_yaw;
nav_msgs::Path pathout,pathout_clear;

geometry_msgs::PolygonStamped poly_obs;

float get_dst3d(geometry_msgs::Point p2, geometry_msgs::Point p1){
  return(sqrt(pow(p1.x-p2.x,2)+pow(p1.y-p2.y,2)+pow(p1.z-p2.z,2)));
}
bool update_edto_bbvec(std::vector<float> bbvec,float collision_radius){
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
void get_sphere(geometry_msgs::Point midpoint,float collision_radius){
	int num_rays = 32;
	float rads_pr_i = 2*M_PI / num_rays;
	geometry_msgs::PoseStamped ps;
	ps.header.frame_id = "map";
	for(int k = 0; k < num_rays/2; k++){
		for(int i  = 0; i < num_rays; i++){
			Eigen::Vector3f pnt1_vec(midpoint.x,midpoint.y,midpoint.z);
			Eigen::Vector3f pnt2_vec(midpoint.x+par_sphere_radius*sin(k*rads_pr_i)*cos(i*rads_pr_i),par_sphere_radius*sin(k*rads_pr_i)*sin(i*rads_pr_i)+midpoint.y,midpoint.z+par_sphere_radius*cos(k*rads_pr_i));
			Eigen::Vector3f cur_vec = pnt1_vec;
			Eigen::Vector3f stride_vec;
			Eigen::Vector3f last_valid;
			stride_vec = (pnt2_vec - pnt1_vec).normalized();

			float tot_length = (pnt2_vec - pnt1_vec).norm();
			float cur_ray_len=0;
			float distance = collision_radius;
			geometry_msgs::Point32 last_obs;
			geometry_msgs::Point last_cand,last_clear;

			while(distance > 5 && cur_ray_len < par_sphere_radius){
				cur_vec 		= cur_vec + stride_vec;
				cur_ray_len = (cur_vec-pnt1_vec).norm();
				if(cur_vec.x() > float(oct_xmin) && cur_vec.y() > float(oct_ymin) && cur_vec.z() > float(oct_zmin)
				&& cur_vec.x() < float(oct_xmax) && cur_vec.y() < float(oct_ymax) && cur_vec.z() < float(oct_zmax)){
					point3d stridep(cur_vec.x(),cur_vec.y(),cur_vec.z());
					distance = edf_ptr.get()->getDistance(stridep);
					if(distance < collision_radius-1 && distance > 2){
						point3d closestObst;
						//distance = edf_ptr.get()->getDistance(stridep);
						edf_ptr.get()->getDistanceAndClosestObstacle(stridep,distance,closestObst);
						last_valid.x() = stridep.x();
						last_valid.y() = stridep.y();
						last_valid.z() = stridep.z();
						last_obs.x =	closestObst.x();
						last_obs.y =	closestObst.y();
						last_obs.z =	closestObst.z();
					}
					else{
						last_clear.x = stridep.x();
						last_clear.y = stridep.y();
						last_clear.z = stridep.z();
					}
				}
				else
					break;
			}
			if((last_valid-pnt1_vec).norm() > 5 && abs(last_valid.x() - midpoint.x) > 5 && abs(last_valid.y() - midpoint.y) > 5){
				ps.pose.position.x 	= last_valid.x();
				ps.pose.position.y 	= last_valid.y();
				ps.pose.position.z 	= last_valid.z();
				ps.pose.orientation = tf::createQuaternionMsgFromYaw(atan2(last_obs.y-midpoint.y,last_obs.x-midpoint.x));
				poly_obs.polygon.points.push_back(last_obs);
				pathout.poses.push_back(ps);
			}
			if(last_clear.x != 0){
				ps.pose.position 	= last_clear;
				ps.pose.orientation = tf::createQuaternionMsgFromYaw(atan2(ps.pose.position.y-midpoint.y,ps.pose.position.x-midpoint.x));
				pathout_clear.poses.push_back(ps);
			}
		}
	}

	ROS_INFO("Got: %i-%i pathout-obstacles vs %i cleared",pathout.poses.size(),poly_obs.polygon.points.size(),pathout_clear.poses.size());
	pathout.header.frame_id = pathout_clear.header.frame_id = poly_obs.header.frame_id = "map";
	pathout.header.stamp = pathout_clear.header.stamp = poly_obs.header.stamp = ros::Time::now();
	if(cnt == 0){
		cnt = 2;
		pub_path.publish(pathout);

	}
	if(cnt == 1){
		cnt = 0;
		pub_path_cleared.publish(pathout_clear);

	}
	if(cnt == 2){
		cnt = 1;
		pub_poly_obs.publish(poly_obs);
	}
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
		transformStamped = tfBuffer.lookupTransform("map","base_future",
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
	if(got_map){
		if((get_dst3d(pos,last_pos) > 5.0) || ((ros::Time::now()-last_sec).toSec() > 5.0)){
			std::vector<float> bbvec;
			bbvec.resize(6);
			ros::Time t0 = ros::Time::now();
			bbvec[0] = pos.x - par_sphere_radius;
			bbvec[1] = pos.y - par_sphere_radius;
			bbvec[2] = pos.z - par_sphere_radius;
			bbvec[3] = pos.x + par_sphere_radius;
			bbvec[4] = pos.y + par_sphere_radius;
			bbvec[5] = pos.z + par_sphere_radius;
			last_pos = pos;
			last_yaw = yaw;
			last_sec = ros::Time::now();
			float collision_radius = 15;
			ROS_INFO("updating poly ");
			if(!outside_octomap(bbvec)){
				if(update_edto_bbvec(bbvec,collision_radius))
					get_sphere(pos,collision_radius);
			}
		}
	}
}
int main(int argc, char** argv)
{
  ros::init(argc, argv, "tb_edto_sphere_node");
  ros::NodeHandle nh;
	ros::NodeHandle private_nh("~");
	private_nh.param("sphere_radius",par_sphere_radius, 30.0);
	ros::Subscriber s1  = nh.subscribe("/octomap_full",1,octomap_callback);
	pub_path 					  = nh.advertise<nav_msgs::Path>("/tb_edto/sphere",10);
	pub_path_cleared    = nh.advertise<nav_msgs::Path>("/tb_edto/sphere_cleared",10);
	pub_poly_obs   		  = nh.advertise<geometry_msgs::PolygonStamped>("/tb_edto/sphere_poly_obst",10);
	ros::Rate rate(1.0);
	int cluster_count = 0;
	nav_msgs::Path path_side;
	while(ros::ok()){
		update_pos();
		pathout.header.frame_id = "map";
		pathout.header.stamp = ros::Time::now();
		pathout_clear.header.frame_id = "map";
		pathout_clear.header.stamp = ros::Time::now();
		poly_obs.header.frame_id = "map";
		poly_obs.header.stamp = ros::Time::now();
		if(pathout.poses.size() > 0){
			pub_path.publish(pathout);
			pathout.poses.resize(0);
		}
		if(pathout_clear.poses.size() > 0){
			pub_path_cleared.publish(pathout_clear);
			pathout.poses.resize(0);
		}
		if(poly_obs.polygon.points.size() > 0){
			pub_poly_obs.publish(poly_obs);
			poly_obs.polygon.points.resize(0);
		}
		rate.sleep();
		ros::spinOnce();
	}
	return 0;
}
