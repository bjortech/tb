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
std::vector<float> getinpath_boundingbox(nav_msgs::Path pathin){
  std::vector<float> bbvec;
  bbvec.resize(6);
  bbvec[3] = bbvec[4] = bbvec[5] = -100;
  bbvec[0] = bbvec[1] = bbvec[2] = 100;
  for(int i = 0; i < pathin.poses.size(); i++){
    if(pathin.poses[i].pose.position.x > bbvec[3])bbvec[3] = pathin.poses[i].pose.position.x;
    if(pathin.poses[i].pose.position.y > bbvec[4])bbvec[4] = pathin.poses[i].pose.position.y;
    if(pathin.poses[i].pose.position.z > bbvec[5])bbvec[5] = pathin.poses[i].pose.position.z;
    if(pathin.poses[i].pose.position.x < bbvec[0])bbvec[0] = pathin.poses[i].pose.position.x;
    if(pathin.poses[i].pose.position.y < bbvec[1])bbvec[1] = pathin.poses[i].pose.position.y;
    if(pathin.poses[i].pose.position.z < bbvec[2])bbvec[2] = pathin.poses[i].pose.position.z;
  }
  return bbvec;
}
std::vector<float> inflate_bbvec(std::vector<float> bbvec,float inflation_xy,float inflation_z){
  bbvec[0] = bbvec[0] - inflation_xy;
  bbvec[3] = bbvec[3] + inflation_z;
  bbvec[1] = bbvec[1] - inflation_xy;
  bbvec[4] = bbvec[4] + inflation_xy;
  bbvec[2] = bbvec[2] - inflation_xy;
  bbvec[5] = bbvec[5] + inflation_z;
  return bbvec;
}
void check_point_edto(geometry_msgs::Point pnt,float collision_radius,float min_dst, bool get_point){
	geometry_msgs::PointStamped closest_obst;
	std_msgs::Float64 closest_dst;
	std::vector<float> bbvec = inflate_bbvec(getinpath_boundingbox(pathin),7.0,3.0);
	closest_obst.header.frame_id = "no_obstacle";
	closest_obst.header.stamp 	 = ros::Time::now();
  if(update_edto_bbvec(bbvec,collision_radius)){
    for(int i = 0; i < pathin.poses.size(); i++){
      geometry_msgs::Point pnt = pathin.poses[i].pose.position;
      if(pnt.x < float(oct_xmax) && pnt.x > float(oct_xmin)
      && pnt.y < float(oct_ymax) && pnt.y > float(oct_ymin)
      && pnt.z < float(oct_zmax) && pnt.z > float(oct_zmin)){
        point3d closestObst;
        point3d p(pnt.x,pnt.y,pnt.z);
        float dst = collision_radius;
        edf_ptr.get()->getDistanceAndClosestObstacle(p, dst, closestObst);
				if(dst < closest_dst.data && dst > 0){
					closest_obst.point.x = closestObst.x();
					closest_obst.point.y = closestObst.y();
					closest_obst.point.z = closestObst.z();
					closest_obst.header.frame_id = "map";
					closest_dst.data = dst;
				}
      }
    }
  }
	if(get_point)
		pub_path_closest_point.publish(closest_obst);
	else
		pub_path_closest_dst.publish(closest_obst);
}
void check_point_edto(geometry_msgs::Point pnt,float collision_radius,float min_dst, bool get_point){
	geometry_msgs::PointStamped closest_obst;
	std_msgs::Float64 closest_dst;
	closest_obst.header.frame_id = "no_obstacle";
	closest_obst.header.stamp 	 = ros::Time::now();
	std::vector<float> bbvec;
	bbvec.resize(6);
	bbvec[3] = pnt.x;	bbvec[4] = pnt.y;	bbvec[5] = pnt.z;
	bbvec[0] = pnt.x;	bbvec[1] = pnt.y;	bbvec[2] = pnt.z;
  bbvec = inflate_bbvec(bbvec,15.0,10.0);
  if(update_edto_bbvec(bbvec,collision_radius)){
    if(pnt.x < float(oct_xmax) && pnt.x > float(oct_xmin)
    && pnt.y < float(oct_ymax) && pnt.y > float(oct_ymin)
    && pnt.z < float(oct_zmax) && pnt.z > float(oct_zmin)){
      point3d closestObst;
      point3d p(pnt.x,pnt.y,pnt.z);
      float dst = collision_radius;
      edf_ptr.get()->getDistanceAndClosestObstacle(p, dst, closestObst);
      if(dst < closest_dst.data && dst > 0){
				closest_obst.point.x = closestObst.x();
				closest_obst.point.y = closestObst.y();
				closest_obst.point.z = closestObst.z();
				closest_obst.header.frame_id = "map";
				closest_dst.data = dst;
    	}
		}
  }
	if(get_point)
		pub_point_closest_dst.publish(closest_obst);
	else
		pub_point_closest_dst.publish(closest_obst);
}
void get_path_closest_ds(const nav_msgs::Path::ConstPtr& msg){
	if(got_map){
		check_point_edto(*msg,10,5,false);
	}
}

void get_point_closest_ds(const geometry_msgs::PointStamped::ConstPtr& msg){
	if(got_map){
		check_point_edto(*msg,10,5,false);
	}
}
void get_down_cb(const geometry_msgs::PolygonStamped::ConstPtr& msg){

}
int main(int argc, char** argv)
{
  ros::init(argc, argv, "tb_edto_down_node");
  ros::NodeHandle nh;
	ros::NodeHandle private_nh("~");
	private_nh.param("sphere_radius",par_sphere_radius, 30.0);
	ros::Subscriber s1  = nh.subscribe("/octomap_full",1,octomap_callback);
	ros::Subscriber s2  = nh.subscribe("/tb_obstacles/get_path_closest_dst",1,get_path_closest_ds);
	ros::Subscriber s3  = nh.subscribe("/tb_obstacles/get_point_closest_dst",1,get_point_closest_ds);
	pub_path_closest_dst 	 = nh.advertise<std_msgs::Float64>("/tb_obstacles/path_closest_dst",10);
	pub_point_closest_dst  = nh.advertise<std_msgs::Float64>("/tb_obstacles/point_closest_dst",10);
	ros::spin();
  return 0;
}
