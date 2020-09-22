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
ros::Publisher pub_map_updates;
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
void update_mbmap_edto(float collision_radius){
	map_msgs::OccupancyGridUpdate update;
	update.header.stamp = ros::Time::now();
	update.header.frame_id = "map";
	update.x = 500+oct_xmin;
	update.y = 500+oct_ymin;
	update.width  = oct_xmax-oct_xmin;
	update.height = oct_ymax-oct_ymin;
	update.data.resize(update.width * update.height);
	unsigned int i = 0;
	for(int y = oct_ymin; y < oct_ymax; y++){
		for(int x = oct_xmin; x < oct_xmax; x++){
			float z = oct_zmax;
			float dst = collision_radius;
			while(dst > (collision_radius-1) && z > oct_zmin){
				octomap::point3d p(x,y,z);
				dst = edf_ptr.get()->getDistance(p);
				z--;
			}
      octomap::point3d p(x,y,z);
      octomap::point3d closestObst;
      float d;
      edf_ptr.get()->getDistanceAndClosestObstacle(p,d,closestObst);
			if(d < collision_radius)
				update.data[i++] = 100;
			else
				update.data[i++] = 0;
    }
	}
	pub_map_updates.publish(update);
}
void update_mbmap_cleared(int x0, int y0, int x1, int y1){
	map_msgs::OccupancyGridUpdate update;
	update.header.stamp = ros::Time::now();
	update.header.frame_id = "map";
	update.x = 500+x0;
	update.y = 500+y0;
	update.width  = x1-x0;
	update.height = y1-y0;
	update.data.resize(update.width * update.height);
	unsigned int i = 0;
	for(int y = y0; y < y1; y++){
		for(int x = x0; x < x1; x++){
			update.data[i++] = 0;
    }
	}
	pub_map_updates.publish(update);
}

bool outside_octomap(std::vector<float> bbvec){
  geometry_msgs::Point bbmin_octree,bbmax_octree;
  octree.get()->getMetricMin(bbmin_octree.x,bbmin_octree.y,bbmin_octree.z);
  octree.get()->getMetricMax(bbmax_octree.x,bbmax_octree.y,bbmax_octree.z);
	if(bbvec[2] > bbmax_octree.z)
		return true;
	return false;
}
void update_mbmap_edto_cb(const geometry_msgs::PolygonStamped::ConstPtr& msg){
	if(got_map){
		std::vector<float> bbvec;
		bbvec.resize(6);
		bbvec[0] = bbvec[1] = bbvec[2] = 100;
		bbvec[3] = bbvec[4] = bbvec[5] = -100;
		float collision_radius = 1.1;
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
		if(!outside_octomap(bbvec)){
			if(update_edto_bbvec(bbvec,collision_radius))
					update_mbmap_edto(collision_radius);
		}
		else
			update_mbmap_cleared(int(bbvec[0]),int(bbvec[1]),int(bbvec[3]),int(bbvec[4]));
	}
}
int main(int argc, char** argv)
{
  ros::init(argc, argv, "tb_costmap_edto2_node");
  ros::NodeHandle nh;
	ros::NodeHandle private_nh("~");
	private_nh.param("sphere_radius",par_sphere_radius, 30.0);
	ros::Subscriber s1  = nh.subscribe("/octomap_full",1,octomap_callback);
	ros::Subscriber s2  = nh.subscribe("/tb_costmap/update_EDTO",1,update_mbmap_edto_cb);
	pub_map_updates   = nh.advertise<map_msgs::OccupancyGridUpdate>("/map_updates",100);
	ros::spin();
  return 0;
}
