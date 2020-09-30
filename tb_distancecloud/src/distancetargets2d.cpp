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
double par_grid_sidelength,par_area_sidelength;
ros::Publisher pub_targets_cleared;
std_msgs::Header hdr(){
  std_msgs::Header header;
  header.frame_id = "map";
  header.stamp    = ros::Time::now();
  return header;
}
bool isXYinPoly(geometry_msgs::PolygonStamped polyin, float x, float y)
{
  int cross = 0;
	geometry_msgs::Point point;
	point.x = x;
	point.y = y;
  for(int i = 0,j = polyin.polygon.points.size()-1; i < polyin.polygon.points.size(); j=i++){//  for (int i = 0, j = vertices_.size() - 1; i < vertices_.size(); j = i++) {
    if ( ((polyin.polygon.points[i].y > point.y) != (polyin.polygon.points[j].y > point.y))
           && (point.x < (polyin.polygon.points[j].x - polyin.polygon.points[i].x) * (point.y - polyin.polygon.points[i].y) /
            (polyin.polygon.points[j].y - polyin.polygon.points[i].y) + polyin.polygon.points[i].x) )
    {
      cross++;
    }
  }
  return bool(cross % 2);
}
std::vector<float> getPolyBoundingBox(geometry_msgs::PolygonStamped polyin)
{
	std::vector<float> bbvec;
	bbvec.resize(6);
	bbvec[0] = bbvec[1] = bbvec[2] = 1000;
	bbvec[3] = bbvec[4] = bbvec[5] = -1000;

	for(int i = 0; i < polyin.polygon.points.size(); i++){
		if(polyin.polygon.points[i].x < bbvec[0])
			bbvec[0] = polyin.polygon.points[i].x;
		if(polyin.polygon.points[i].y < bbvec[1])
			bbvec[1] = polyin.polygon.points[i].y;
		if(polyin.polygon.points[i].z < bbvec[2])
			bbvec[2] = polyin.polygon.points[i].z;

		if(polyin.polygon.points[i].x > bbvec[3])
			bbvec[3] = polyin.polygon.points[i].x;
		if(polyin.polygon.points[i].y > bbvec[4])
			bbvec[4] = polyin.polygon.points[i].y;
		if(polyin.polygon.points[i].z > bbvec[5])
			bbvec[5] = polyin.polygon.points[i].z;
	}
	return bbvec;
}

nav_msgs::Path getGridsInWorld(geometry_msgs::PolygonStamped poly_boundary)
{
  nav_msgs::Path pathout;
  ////ROS_INFO("Area side: %.2f radlen: %.2f num_grids: %i centroid_sides: %.2f",area_sidelength,radlen_xy,num_grids,centroid_sides);
  int i = 0;
	int num_grids = par_area_sidelength / par_grid_sidelength;
	std::vector<float> bbvec = getPolyBoundingBox(poly_boundary);
  for (int y = 0; y < num_grids; y++)
  {
    for (int x = 0; x < num_grids; x++)
    {
			geometry_msgs::Point p;
			p.x      = x * par_grid_sidelength - par_area_sidelength * 0.5 + par_grid_sidelength*0.5;
			p.y      = y * par_grid_sidelength - par_area_sidelength * 0.5 + par_grid_sidelength*0.5;
			p.z  		 = poly_boundary.polygon.points[0].z;
			if(p.x > bbvec[0] && p.y > bbvec[1]
			&& p.x < bbvec[3] && p.y < bbvec[4])
			{
				if(isXYinPoly(poly_boundary,p.x,p.y))
				{
				 geometry_msgs::PoseStamped ps;
				 ps.header 						 = hdr();
				 ps.pose.position 		 = p;
				 ps.pose.orientation.w = 1.0;
				 pathout.poses.push_back(ps);
				}
			}
		}
	}
	pathout.header = hdr();
  return pathout;
}

void polyBoundaryCallback(const geometry_msgs::PolygonStamped::ConstPtr& msg)
{
	if(msg->polygon.points.size() > 0)
	{
		pub_targets_cleared.publish(getGridsInWorld(*msg));
	}
}
int main(int argc, char** argv)
{
  ros::init(argc, argv, "tb_distancetargets2d_node");
	ros::NodeHandle nh;
	ros::NodeHandle private_nh("~");
	private_nh.param("grid_sidelength", par_grid_sidelength, 5.0);
	private_nh.param("area_sidelength", par_area_sidelength, 1000.0);

	tf2_ros::TransformListener tf2_listener(tfBuffer);
	ros::Subscriber s1  = nh.subscribe("/tb_distancecloud/poly_boundary",1,polyBoundaryCallback);
	pub_targets_cleared	= nh.advertise<nav_msgs::Path>("/tb_distancecloud/grids_active",100);
	ros::spin();
	return 0;
}
