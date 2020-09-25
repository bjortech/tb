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

shared_ptr<DynamicEDTOctomap> edf_ptr;
AbstractOcTree* abs_octree;
shared_ptr<OcTree> octree;
bool got_map = false;
geometry_msgs::Point pos,cmd_pos,pos_future;
int oct_xmin,oct_ymin,oct_zmin,oct_xmax,oct_ymax,oct_zmax,oct_range_x,oct_range_y,oct_range_z;
double par_dz,par_collision_radius;


bool update_edto_bbvec(std::vector<float> bbvec, float collrad){
	//Updates boundaries based on desired points (bbvec) and octomap extents
  geometry_msgs::Point bbmin_octree,bbmax_octree;
  octree.get()->getMetricMin(bbmin_octree.x,bbmin_octree.y,bbmin_octree.z);
  octree.get()->getMetricMax(bbmax_octree.x,bbmax_octree.y,bbmax_octree.z);
  octomap::point3d boundary_min(fmax(bbvec[0],bbmin_octree.x),fmax(bbvec[1],bbmin_octree.y),fmax(bbvec[2],bbmin_octree.z));
  octomap::point3d boundary_max(fmin(bbvec[3],bbmax_octree.x),fmin(bbvec[4],bbmax_octree.y),fmin(bbvec[5],bbmax_octree.z));

  edf_ptr.reset (new DynamicEDTOctomap(collrad,octree.get(),
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
std::vector<float> get_bbvec_from_midpoint(geometry_msgs::Point midpoint,float rad, bool seek_down){
	std::vector<float> bbvec;
	bbvec.resize(6);
//int zlvl = floor(midpoint.z / par_dz);
	if(!seek_down){
		bbvec[0] = midpoint.x - rad;
		bbvec[1] = midpoint.y - rad;
		bbvec[2] = midpoint.z - par_dz;
		bbvec[3] = midpoint.x + rad;
		bbvec[4] = midpoint.y + rad;
		bbvec[5] = midpoint.z +	par_dz;
	}
	else{
		bbvec[0] = midpoint.x - rad/2;
		bbvec[1] = midpoint.y - rad/2;
		bbvec[2] = midpoint.z - rad;
		bbvec[3] = midpoint.x + rad/2;
		bbvec[4] = midpoint.y + rad/2;
		bbvec[5] = midpoint.z +	par_dz;
	}
	return bbvec;
}
geometry_msgs::PointStamped find_closest_obstacle(geometry_msgs::Point pnt){
	octomap::point3d p(pnt.x,pnt.y,pnt.z);
	octomap::point3d closestObst;
	float d;
	edf_ptr.get()->getDistanceAndClosestObstacle(p,d,closestObst);
	geometry_msgs::PointStamped pobs;
	pobs.header.frame_id = "no_obstacle";
	pobs.header.stamp    = ros::Time::now();
	if(d < 20 && d > 0){
		pobs.header.frame_id = "map";
		pobs.point.x = closestObst.x();
		pobs.point.y = closestObst.y();
		pobs.point.z = closestObst.z();
	}
	return pobs;
}
geometry_msgs::PointStamped update_edto_and_get_closest_obstacle(geometry_msgs::Point pnt,bool seek_down){
	geometry_msgs::PointStamped pobs;
	pobs.header.frame_id = "no_data";
	std::vector<float> bbvec = get_bbvec_from_midpoint(pnt,par_collision_radius,seek_down);
	if(!outside_octomap(bbvec) && update_edto_bbvec(bbvec,par_collision_radius))
		return find_closest_obstacle(pnt);
	else
		return pobs;
}

void update_midpoint(){
	geometry_msgs::TransformStamped transformStamped;

	try{
			transformStamped = tfBuffer.lookupTransform("map","base_future",
															 ros::Time(0));
	}
	catch (tf2::TransformException &ex) {
		ROS_WARN("%s",ex.what());
		ros::Duration(1.0).sleep();
	}
	 pos_future.x = transformStamped.transform.translation.x;
	 pos_future.y = transformStamped.transform.translation.y;
	 pos_future.z = transformStamped.transform.translation.z;
}

void octomap_callback(const octomap_msgs::Octomap& msg){
  abs_octree=octomap_msgs::fullMsgToMap(msg);
  octree.reset(dynamic_cast<octomap::OcTree*>(abs_octree));
  got_map = true;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "tb_autoevade_obstaclefinder_node");
	ros::NodeHandle nh;
	ros::NodeHandle private_nh("~");
	private_nh.param("update_area_sidelength", par_collision_radius, 30.0);
	private_nh.param("update_area_z_radius", par_dz, 1.5);
	tf2_ros::TransformListener tf2_listener(tfBuffer);

	ros::Subscriber s1  = nh.subscribe("/octomap_full",1,octomap_callback);
	ros::Publisher pub_obstacle_xy  = nh.advertise<geometry_msgs::PointStamped>("/tb_autoevade/obstacle_xy",10);
	ros::Publisher pub_obstacle_z   = nh.advertise<geometry_msgs::PointStamped>("/tb_autoevade/obstacle_z",10);

	ros::Rate rate(2.0);
	bool everyother;
	while(ros::ok()){
		update_midpoint();
		if(got_map){
			ros::Time t0 = ros::Time::now();
			if(everyother){
				everyother = false;
				geometry_msgs::PointStamped obs_xy = update_edto_and_get_closest_obstacle(pos_future,false);
				if(obs_xy.header.frame_id == "map"){
					pub_obstacle_xy.publish(obs_xy);
				}
				ROS_INFO("finding obstacle xy: %.4f sec",(ros::Time::now()-t0).toSec());
			}
			else{
				everyother = true;
				geometry_msgs::PointStamped obs_z = update_edto_and_get_closest_obstacle(pos_future,true);
				if(obs_z.header.frame_id == "map"){
					pub_obstacle_z.publish(obs_z);
				}
				ROS_INFO("finding obstacle z: %.4f sec",(ros::Time::now()-t0).toSec());
			}
		}
		rate.sleep();
		ros::spinOnce();
	/*		geometry_msgs::Point pos_forw;
			pos_forw.x = (cmd_pos.x + pos.x)/2;
			pos_forw.y = (cmd_pos.y + pos.y)/2;
			pos_forw.z = (cmd_pos.z + pos.z)/2;
			geometry_msgs::PointStamped obs_cmd = update_edto_and_get_closest_obstacle(cmd_pos);
			geometry_msgs::PointStamped obs_pos = update_edto_and_get_closest_obstacle(pos);
			geometry_msgs::PointStamped obs_mid = update_edto_and_get_closest_obstacle(pos_forw);
			if(obs_cmd.header.frame_id == "map"){
				pub_obstacle.publish(obs_cmd);
			}
			if(obs_pos.header.frame_id == "map"){
				pub_obstacle.publish(obs_pos);
			}
			if(obs_mid.header.frame_id == "map"){
				pub_obstacle.publish(obs_mid);
			}*/

	}
	return 0;
}
