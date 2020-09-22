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
ros::Publisher pub_path;
double par_sphere_radius;
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
	ROS_INFO("DOWN bbvec: %.0f %.0f %.0f %.0f %.0f %.0f",bbvec[0],bbvec[1],bbvec[2],bbvec[3],bbvec[4],bbvec[5]);
	ROS_INFO("DOWN bboct: %.0f %.0f %.0f %.0f %.0f %.0f",bbmin_octree.x,bbmin_octree.y,bbmin_octree.z,bbmax_octree.x,bbmax_octree.y,bbmax_octree.z);
	ROS_INFO("DOWN bbout: %.0f %.0f %.0f %.0f %.0f %.0f",boundary_min.x(),boundary_min.y(),boundary_min.z(),boundary_max.x(),boundary_max.y(),boundary_max.z());
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

nav_msgs::Path get_down(float collision_radius){
	ROS_INFO("Get down");
	nav_msgs::Path pathout;
	geometry_msgs::PoseStamped ps;
	pathout.header.frame_id = ps.header.frame_id = "map";
	pathout.header.stamp = ps.header.stamp = ros::Time::now();
	ps.pose.orientation.x = ps.pose.orientation.z = 0.0;
	ps.pose.orientation.y = ps.pose.orientation.w = 0.7071;
  for(int y = oct_ymin; y < oct_ymax; y++){
    for(int x = oct_xmin; x < oct_xmax; x++){
			float z = oct_zmax;
			float dst = collision_radius;
		//	while(dst > (collision_radius-1) && z > oct_zmin){
			while(z > oct_zmin){
				octomap::point3d p(x,y,z);
				octomap::point3d closestObst;
				edf_ptr.get()->getDistanceAndClosestObstacle(p,dst,closestObst);
				if(round(dst) == 1 && dst < collision_radius && closestObst.z() < z){
			//	if(d < collision_radius && d >= 0){
					ps.pose.position.x = x;					ps.pose.position.y = y;					ps.pose.position.z = z;
					pathout.poses.push_back(ps);
					break;
				}
				z--;
			}
    }
  }
	ROS_INFO("pathout %i ",pathout.poses.size());

  return pathout;
}
/*	while(dst > (collision_radius-1) && z > oct_zmin){
		octomap::point3d p(x,y,z);
		dst = edf_ptr.get()->getDistance(p);
		z--;
	}
	octomap::point3d p(x,y,z);
	octomap::point3d closestObst;
	float d;
	edf_ptr.get()->getDistanceAndClosestObstacle(p,d,closestObst);
	if(d < collision_radius){
		ps.pose.position.x = x;					ps.pose.position.y = y;					ps.pose.position.z = z;
		pathout.poses.push_back(ps);
	}*/
void get_down_cb(const geometry_msgs::PolygonStamped::ConstPtr& msg){
	if(got_map){
		std::vector<float> bbvec;
		bbvec.resize(6);
		bbvec[0] = bbvec[1] = bbvec[2] = 1000;
		bbvec[3] = bbvec[4] = bbvec[5] = -1000;
		float collision_radius = 1.5;
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
		if(update_edto_bbvec(bbvec,collision_radius))
			pub_path.publish(get_down(collision_radius));
	}
}
int main(int argc, char** argv)
{
  ros::init(argc, argv, "tb_edto_down_node");
  ros::NodeHandle nh;
	ros::NodeHandle private_nh("~");
	private_nh.param("sphere_radius",par_sphere_radius, 30.0);
	ros::Subscriber s1  = nh.subscribe("/octomap_full",1,octomap_callback);
	ros::Subscriber s2  = nh.subscribe("/tb_edto/get_down",1,get_down_cb);
  pub_path 					  = nh.advertise<nav_msgs::Path>("/tb_edto/down",10);
	ros::spin();
  return 0;
}
