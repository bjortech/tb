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
bool par_normalize_rgb;
int g_oct_xmin,g_oct_ymin,g_oct_zmin,g_oct_xmax,g_oct_ymax,g_oct_zmax,g_oct_range_x,g_oct_range_y,g_oct_range_z;
double par_collision_radius,par_res,par_dz;
cv::Mat img_height(1000,1000,CV_8U,cv::Scalar(0)); //create image, set encoding and size, init pixels to default val
ros::Publisher pub_distance_cloud;
geometry_msgs::PolygonStamped poly_obs,poly_cleared,poly_obstacles;
nav_msgs::Path g_trajectory_visited;
ros::Time g_process_start;
std::string g_active_process;

std_msgs::Header hdr(){
  std_msgs::Header header;
  header.frame_id = "map";
  header.stamp    = ros::Time::now();
  return header;
}
void start_process(std::string name){
  float dt = (ros::Time::now() - g_process_start).toSec();
  if(g_active_process != "")
    ROS_INFO("DistanceCloud2D: Process: %s took %.5f sec",g_active_process.c_str(),dt);
  g_active_process = name;
  g_process_start  = ros::Time::now();
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
float y2r(float y){
  return (img_height.rows / 2 - y / par_res);
}
float x2c(float x){
  return (x / par_res + img_height.cols/2);
}
int r2y(float r){
  return int((img_height.rows / 2 - r) * par_res);
}
int c2x(float c){
  return int((c - img_height.cols / 2) * par_res);
}

std::vector<std::tuple<int,int,int,int,int,int,int>> normalizeRGB(std::vector<std::tuple<int,int,int,int,int,int,int>> vector_tuple)
{
	int rmx = 0;	int gmx = 0;	int bmx = 0;
	int rmn = 255;int gmn = 255;int bmn = 255;
	for(int i = 0; i < vector_tuple.size(); i++){
		if(std::get<4>(vector_tuple[i]) < rmn)
			rmn = std::get<4>(vector_tuple[i]);
		if(std::get<5>(vector_tuple[i]) < gmn)
			gmn = std::get<5>(vector_tuple[i]);
		if(std::get<6>(vector_tuple[i]) < bmn)
			bmn = std::get<6>(vector_tuple[i]);
		if(std::get<4>(vector_tuple[i]) > rmx)
			rmx = std::get<4>(vector_tuple[i]);
		if(std::get<5>(vector_tuple[i]) > gmx)
			gmx = std::get<5>(vector_tuple[i]);
		if(std::get<6>(vector_tuple[i]) > bmx)
			bmx = std::get<6>(vector_tuple[i]);
	}
	ROS_INFO("DistanceCloud2D-Normalizing %i tuples-RGB (b->elevation_at_xy: 			%i -> %i (range: %i)",vector_tuple.size(),bmn,bmx,bmx-bmn);
	ROS_INFO("DistanceCloud2D-Normalizing %i tuples-RGB (g->dst_nearest_visited:  %i -> %i (range: %i)",vector_tuple.size(),gmn,gmx,gmx-gmn);
	ROS_INFO("DistanceCloud2D-Normalizing %i tuples-RGB (r->dst_nearest_obstacle: %i -> %i (range: %i)",vector_tuple.size(),rmn,rmx,rmx-rmn);
	for(int i = 0; i < vector_tuple.size(); i++){
		float r_rel = float(std::get<4>(vector_tuple[i]) - rmn)/float(rmx - rmn);
		float g_rel = float(std::get<5>(vector_tuple[i]) - gmn)/float(gmx - gmn);
		float b_rel = float(std::get<6>(vector_tuple[i]) - bmn)/float(bmx - bmn);
		std::get<4>(vector_tuple[i]) = int(round(r_rel * 255.0));
		std::get<5>(vector_tuple[i]) = int(round(g_rel * 255.0));
		std::get<6>(vector_tuple[i]) = int(round(b_rel * 255.0));
	}
	return vector_tuple;
}

bool updateOctomapEDTO(std::vector<float> bbvec){
	//Updates boundaries based on desired points (bbvec) and octomap extents
  geometry_msgs::Point bbmin_octree,bbmax_octree;
  octree.get()->getMetricMin(bbmin_octree.x,bbmin_octree.y,bbmin_octree.z);
  octree.get()->getMetricMax(bbmax_octree.x,bbmax_octree.y,bbmax_octree.z);
  octomap::point3d boundary_min(fmax(bbvec[0],bbmin_octree.x),fmax(bbvec[1],bbmin_octree.y),fmax(bbvec[2],bbmin_octree.z));
  octomap::point3d boundary_max(fmin(bbvec[3],bbmax_octree.x),fmin(bbvec[4],bbmax_octree.y),fmin(bbvec[5],bbmax_octree.z));
	ROS_INFO("DistanceCloud2D-bbvec: %.0f %.0f %.0f %.0f %.0f %.0f",bbvec[0],bbvec[1],bbvec[2],bbvec[3],bbvec[4],bbvec[5]);
	ROS_INFO("DistanceCloud2D-bboct: %.0f %.0f %.0f %.0f %.0f %.0f",bbmin_octree.x,bbmin_octree.y,bbmin_octree.z,bbmax_octree.x,bbmax_octree.y,bbmax_octree.z);
	ROS_INFO("DistanceCloud2D-bbout: %.0f %.0f %.0f %.0f %.0f %.0f",boundary_min.x(),boundary_min.y(),boundary_min.z(),boundary_max.x(),boundary_max.y(),boundary_max.z());
  edf_ptr.reset (new DynamicEDTOctomap(par_collision_radius,octree.get(),
          boundary_min,
          boundary_max,
          false));
  edf_ptr.get()->update();
  g_oct_xmin    = int(round(boundary_min.x()))+1;  g_oct_ymin    = int(round(boundary_min.y()))+1; g_oct_zmin    = int(round(boundary_min.z()))+1;
  g_oct_xmax    = int(round(boundary_max.x()))-1;  g_oct_ymax    = int(round(boundary_max.y()))-1; g_oct_zmax    = int(round(boundary_max.z()))-1;
  g_oct_range_x = g_oct_xmax - g_oct_xmin;           	 g_oct_range_y = g_oct_ymax - g_oct_ymin;    				g_oct_range_z = g_oct_zmax - g_oct_zmin;
	int vol = g_oct_range_x*g_oct_range_y*g_oct_range_z;
  if(vol <= 0)
		return false;
	return true;
}
bool isOutsideOctomap(std::vector<float> bbvec){
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

std::vector<float> getPolyBoundingBox(geometry_msgs::PolygonStamped polyin){
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
	float range_z = bbvec[5]-bbvec[2];
	if(range_z < par_dz*2.0){
		float z  = (bbvec[5]+bbvec[2])/2;
		bbvec[2] = z - par_dz;
		bbvec[5] = z + par_dz;
	}
	return bbvec;
}

int getDistanceClosestPointVisited(float x,float y, float z)
{
  float min_dst_squared = 1000000000;

  if(g_trajectory_visited.poses.size() == 0)
    return 0;

  for(int i = 0; i < g_trajectory_visited.poses.size(); i++)
	{
    float dst_squared =
		pow((g_trajectory_visited.poses[i].pose.position.x-x),2) +
		pow((g_trajectory_visited.poses[i].pose.position.y-y),2) +
		pow((g_trajectory_visited.poses[i].pose.position.z-z),2);

		if(dst_squared < min_dst_squared)
      min_dst_squared = dst_squared;
  }
	float min_dst = sqrt(min_dst_squared);
	return int(round(min_dst));
}
bool isXYinPoly(geometry_msgs::PolygonStamped polyin, float x, float y){
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
sensor_msgs::PointCloud2 pointcloudFromVector(std::vector<std::tuple<int,int,int,int,int,int,int>> vec_xyz_rgb)
{
	sensor_msgs::PointCloud2 cloud_out;
	cloud_out.header = hdr();
	sensor_msgs::PointCloud2Modifier cloud_out_modifier(cloud_out);
	cloud_out_modifier.setPointCloud2FieldsByString(2, "xyz", "rgb");
	cloud_out_modifier.resize(vec_xyz_rgb.size());

	sensor_msgs::PointCloud2Iterator<float> iter_x(cloud_out, "x");
	sensor_msgs::PointCloud2Iterator<float> iter_y(cloud_out, "y");
	sensor_msgs::PointCloud2Iterator<float> iter_z(cloud_out, "z");
	sensor_msgs::PointCloud2Iterator<uint8_t> iter_rgb(cloud_out, "rgb");

	for (unsigned int i = 0; i < vec_xyz_rgb.size();
			 ++i, ++iter_x, ++iter_y, ++iter_z, ++iter_rgb)
	{
		iter_x[0] = std::get<1>(vec_xyz_rgb[i]);
		iter_x[1] = std::get<2>(vec_xyz_rgb[i]);
		iter_x[2] = std::get<3>(vec_xyz_rgb[i]);
		iter_rgb[0] = std::get<4>(vec_xyz_rgb[i]);
		iter_rgb[1] = std::get<5>(vec_xyz_rgb[i]);
		iter_rgb[2] = std::get<6>(vec_xyz_rgb[i]);
	}
	return cloud_out;
}
std::vector<std::tuple<int,int,int,int,int,int,int>> getXYZRGB(geometry_msgs::PolygonStamped poly_cleared)
{
	std::vector<std::tuple<int,int,int,int,int,int,int>> vec_pntxyz_dstrgb;
	int i = 0;
	int z = (g_oct_zmin+g_oct_zmax)/2;
	for(int y = g_oct_ymin; y < g_oct_ymax; y++)
	{
    for(int x = g_oct_xmin; x < g_oct_xmax; x++)
		{
			if(isXYinPoly(poly_cleared,float(x),float(y)))
			{
				octomap::point3d p(x,y,z);
				int elevation_at_xy      = img_height.at<uchar>(y2r(y),x2c(x));
				int dst_nearest_visited  = getDistanceClosestPointVisited(float(x),float(y),float(z));
				int dst_nearest_obstacle = int(round(edf_ptr.get()->getDistance(p)));
				vec_pntxyz_dstrgb.push_back(std::make_tuple(i,x,y,z,elevation_at_xy,dst_nearest_visited,dst_nearest_obstacle));
				i++;
			}
		}
	}
	return vec_pntxyz_dstrgb;
}

void poly_cleared_cb(const geometry_msgs::PolygonStamped::ConstPtr& msg){
	/*
	1) Get polygon cleared minmax values,
	2) check that they are (at least partially) inside octomap minmax values,
	3) go through points in range:
		- check that they are within poly cleared
			B: elevation_at_xy
		 	G: dst_nearest_visited
			R: dst_nearest_obstacle
	*/
	if(got_map)
	{
		start_process("getPolyBoundingBox");
		geometry_msgs::PolygonStamped poly_lineofsight = *msg;
		std::vector<float> minmax_poly = getPolyBoundingBox(poly_lineofsight);

		if(!isOutsideOctomap(minmax_poly))
		{
			start_process("updateOctomapEDTO");

			if(updateOctomapEDTO(minmax_poly))
			{
				start_process("getXYZRGB");
				std::vector<std::tuple<int,int,int,int,int,int,int>> vt_xyz_rgb = getXYZRGB(poly_lineofsight);
				if(par_normalize_rgb){
					start_process("normalizeRGB");
					vt_xyz_rgb = normalizeRGB(vt_xyz_rgb);
				}
				start_process("pointcloudFromVector");
				sensor_msgs::PointCloud2 distance_cloud_2d 										  = pointcloudFromVector(vt_xyz_rgb);
				start_process("Downtime");
				pub_distance_cloud.publish(distance_cloud_2d);
			}
		}
	}
}
void octomapCallback(const octomap_msgs::Octomap& msg){
  abs_octree = octomap_msgs::fullMsgToMap(msg);
  octree.reset(dynamic_cast<octomap::OcTree*>(abs_octree));
  got_map = true;
}
void assembledCloud_cb(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
	if(msg->data.size() > 10 && msg->header.frame_id == "map")
	{
		sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x");
		sensor_msgs::PointCloud2ConstIterator<float> iter_y(*msg, "y");
		sensor_msgs::PointCloud2ConstIterator<float> iter_z(*msg, "z");
		for(int i = 0; iter_x != iter_x.end(); ++i, ++iter_x, ++iter_y, ++iter_z)
		{
			if(!std::isnan(*iter_x) && !std::isnan(*iter_y) && !std::isnan(*iter_z))
	    {
	      int r = y2r(*iter_y);
	      int c = x2c(*iter_x);
	      int z = fmax(*iter_z,1);
				if(z > img_height.at<uchar>(r,c))
					img_height.at<uchar>(r,c) = z;
	    }
		}
	}
}
void trajectoryVisitedCallback(const nav_msgs::Path::ConstPtr& msg){
	g_trajectory_visited = *msg;
}
int main(int argc, char** argv)
{
  ros::init(argc, argv, "tb_distancecloud2d_node");
	ros::NodeHandle nh;
	ros::NodeHandle private_nh("~");
	private_nh.param("update_area_z_radius", par_dz, 1.5);
	private_nh.param("resolution", par_res, 1.0);
	private_nh.param("collision_radius", par_collision_radius, 15.0);
	private_nh.param("normalize_RGB", par_normalize_rgb, true);

	tf2_ros::TransformListener tf2_listener(tfBuffer);
	/*
	Need to get the following topics:
	/tb_edto/poly_cleared - defines the current line of sight in 2D at target position
	/tb_trajectorywriter/visited_poses - trajectory points visited
	/assembled_cloud2 - all points seen over an interval of time
	*/
	ros::Subscriber s1  = nh.subscribe("/octomap_full",1,octomapCallback);
	ros::Subscriber s2  = nh.subscribe("/tb_edto/poly_cleared",1,poly_cleared_cb);
	ros::Subscriber s3  = nh.subscribe("/tb_trajectorywriter/visited_poses",1,trajectoryVisitedCallback);
	ros::Subscriber s4  = nh.subscribe("/assembled_cloud2",1,assembledCloud_cb);

	pub_distance_cloud = nh.advertise<sensor_msgs::PointCloud2>("/tb_distancecloud/generated_cloud_2d",100);

	ros::spin();
	return 0;
}
