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
geometry_msgs::Point pos,last_pos;
geometry_msgs::PointStamped pnt_down;
int oct_xmin,oct_ymin,oct_zmin,oct_xmax,oct_ymax,oct_zmax,oct_range_x,oct_range_y,oct_range_z;
float last_yaw = 0.0;
ros::Time last_sec;
double par_min_distance,par_maprad,par_dz,par_maprad0;
geometry_msgs::PolygonStamped poly_obs,poly_cleared,poly_obstacles;

nav_msgs::Path path_down;
float get_dst2d32(geometry_msgs::Point32 p2, geometry_msgs::Point32 p1){
  return(sqrt(pow(p1.x-p2.x,2)+pow(p1.y-p2.y,2)));
}
bool is_point32_in_poly(geometry_msgs::PolygonStamped poly_to_check,geometry_msgs::Point32 pin,float lim){
  float res,dst;
  if(poly_to_check.polygon.points.size() == 0)
    return false;
  for(int i = 0; i < poly_to_check.polygon.points.size(); i++){
     if(get_dst2d32(poly_to_check.polygon.points[i],pin) < lim)
        return false;
  }
  return true;
}
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
geometry_msgs::Quaternion get_quat(geometry_msgs::Point to, geometry_msgs::Point from){
	float incl = atan2(to.z - from.z,get_dst2d(to,from));
	float hdng = get_hdng(to,from);
	return tf::createQuaternionMsgFromRollPitchYaw(0,-incl,hdng);
}
nav_msgs::Path get_down(float collision_radius){
	nav_msgs::Path pathout;
	geometry_msgs::PoseStamped ps;

	poly_obs.header.frame_id = pathout.header.frame_id = ps.header.frame_id = "map";
	poly_obs.header.stamp = pathout.header.stamp = ps.header.stamp = ros::Time::now();
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
					geometry_msgs::Point32 pobs;
					pobs.x = closestObst.x();
					pobs.y = closestObst.y();
					pobs.z = closestObst.z();
					//if(!is_point32_in_poly(poly_obs,pobs,1.0))
					poly_obs.polygon.points.push_back(pobs);
			//	if(d < collision_radius && d >= 0){
					ps.pose.position.x = x;					ps.pose.position.y = y;					ps.pose.position.z = z;
					pathout.poses.push_back(ps);
					break;
				}
				z--;
			}
    }
  }
  return pathout;
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
	float yaw = tf::getYaw(transformStamped.transform.rotation);
	if(got_map){
		if((get_dst3d(pos,last_pos) > 5.0) || (abs(get_shortest(yaw,last_yaw)) > 1.0) || ((ros::Time::now()-last_sec).toSec() > 5.0)){
			std::vector<float> bbvec;
			bbvec.resize(6);
			geometry_msgs::Point bbmin_octree,bbmax_octree;
			octree.get()->getMetricMin(bbmin_octree.x,bbmin_octree.y,bbmin_octree.z);
			octree.get()->getMetricMax(bbmax_octree.x,bbmax_octree.y,bbmax_octree.z);
			bbvec[0] = pos.x - par_maprad0;
			bbvec[1] = pos.y - par_maprad0;
			bbvec[2] = bbmin_octree.z;
			bbvec[3] = pos.x + par_maprad0;
			bbvec[4] = pos.y + par_maprad0;
			bbvec[5] = pos.z + par_dz;
			last_pos = pos;
			last_yaw = yaw;
			last_sec = ros::Time::now();
			float collision_radius = pos.z + par_maprad;
			if(!outside_octomap(bbvec)){
				if(update_edto_bbvec(bbvec,collision_radius)){
					point3d stridep(pos.x,pos.y,pos.z);
					point3d closestObst;
					float distance;
					edf_ptr.get()->getDistanceAndClosestObstacle(stridep,distance,closestObst);
					pnt_down.point = p_to_pnt(closestObst);
					pnt_down.header = hdr();
					bbvec[0] = pos.x - par_maprad;
					bbvec[1] = pos.y - par_maprad;
					bbvec[2] = pos.z - par_dz;
					bbvec[3] = pos.x + par_maprad;
					bbvec[4] = pos.y + par_maprad;
					bbvec[5] = pos.z + par_dz;
					if(update_edto_bbvec(bbvec,collision_radius)){
						path_down = get_down(1.5);
					}
				}
			}
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
  ros::init(argc, argv, "tb_edto_down_auto_node");
	ros::NodeHandle nh;
	ros::NodeHandle private_nh("~");
	private_nh.param("prescreen_area_sidelength", par_maprad0, 7.0);
	private_nh.param("update_area_sidelength", par_maprad, 30.0);
	private_nh.param("update_area_z_radius", par_dz, 5.0);
	tf2_ros::TransformListener tf2_listener(tfBuffer);
	ros::Subscriber s1  = nh.subscribe("/octomap_full",1,&octomap_callback);
	ros::Publisher pub_point 	= nh.advertise<geometry_msgs::PointStamped>("/tb_edto/down_auto_point",100);
	ros::Publisher pub_poly = nh.advertise<geometry_msgs::PolygonStamped>("/tb_edto/downauto_obstacles",100);
	ros::Publisher pub_path 	= nh.advertise<nav_msgs::Path>("/tb_edto/downauto_path",100);
	ros::Rate rate(6.0);
	while(ros::ok()){
		if(poly_obs.polygon.points.size() > 0){
			pub_poly.publish(poly_obs);
			pub_path.publish(path_down);
			pub_point.publish(pnt_down);
			poly_obs.polygon.points.resize(0);
		}
		update_pos();
		rate.sleep();
		ros::spinOnce();
	}
	return 0;
}
