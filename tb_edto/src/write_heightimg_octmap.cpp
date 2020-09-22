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

cv::Mat img(1000,1000,CV_8UC3,cv::Scalar(0, 0, 0)); //create image, set encoding and size, init pixels to default val
cv::Mat img2(1000,1000,CV_8UC3,cv::Scalar(0, 0, 0)); //create image, set encoding and size, init pixels to default val

double par_sphere_radius;
float y2r(float y, float rows,float res){
  return (rows / 2 - y / res);
}
float x2c(float x, float cols,float res){
  return (x / res + cols/2);
}
int r2y(float r, float rows,float res){
  return int((rows / 2 - r) * res);
}
int c2x(float c, float cols,float res){
  return int((c - cols / 2) * res);
}
bool update_edto_bbvec(float collision_radius){
  geometry_msgs::Point bbmin_octree,bbmax_octree;
  octree.get()->getMetricMin(bbmin_octree.x,bbmin_octree.y,bbmin_octree.z);
  octree.get()->getMetricMax(bbmax_octree.x,bbmax_octree.y,bbmax_octree.z);

  octomap::point3d boundary_min(bbmin_octree.x,bbmin_octree.y,bbmin_octree.z);
  octomap::point3d boundary_max(bbmax_octree.x,bbmax_octree.y,bbmax_octree.z);
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

void get_down(){
	 for(int z = oct_zmin; z < oct_zmax; z++){
    for(int y = oct_ymin; y < oct_ymax; y++){
      for(int x = oct_xmin; x < oct_xmax; x++){
        octomap::point3d p(x,y,z);
        octomap::point3d closestObst;
        float d;
        edf_ptr.get()->getDistanceAndClosestObstacle(p,d,closestObst);
				if(round(d) == 1 && d < 1.1 && closestObst.z() <= z){
					if(img.at<cv::Vec3b>( y2r(y,img.rows,1),x2c(x,img.cols,1))[0] < z*10)
						img.at<cv::Vec3b>( y2r(y,img.rows,1),x2c(x,img.cols,1))[0] = z*10;
        }
      }
    }
  }
}
void get_down2(float collision_radius){
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
			if(d < collision_radius){
				if(img2.at<cv::Vec3b>( y2r(y,img.rows,1),x2c(x,img.cols,1))[0] < z*10)
					img2.at<cv::Vec3b>( y2r(y,img.rows,1),x2c(x,img.cols,1))[0] = z*10;
      }
    }
  }
}
int main(int argc, char** argv)
{
  ros::init(argc, argv, "tb_write_heightimg_node");
  ros::NodeHandle nh;
	ros::NodeHandle private_nh("~");
	ros::Subscriber s1  = nh.subscribe("/octomap_full",1,octomap_callback);
  pub_path 					  = nh.advertise<nav_msgs::Path>("/tb_edto/down",10);
	ros::Rate rate(1.0);
	bool first = true;
	while(ros::ok()){
		if(got_map && first){
			float collision_radius = 1.1;
			ros::Time t0 = ros::Time::now();
			update_edto_bbvec(collision_radius);
			float dt = (ros::Time::now()-t0).toSec();
			ros::Time t1 = ros::Time::now();
			get_down();
			float dt1 = (ros::Time::now()-t1).toSec();
			cv::imwrite("/home/nuc/brain/d1.png",img);
			ros::Time t2 = ros::Time::now();
			get_down2(collision_radius);
			float dt2 = (ros::Time::now()-t2).toSec();
			ROS_INFO("Get update: %.3f - d1: %.3f - d2: %.3f",dt,dt1,dt2);
			cv::imwrite("/home/nuc/brain/d2.png",img2);
			first = false;
		}
		rate.sleep();
		ros::spinOnce();
	}
	return 0;
}
