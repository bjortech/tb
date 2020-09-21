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
ros::Publisher pub_path;
geometry_msgs::Point pos,last_pos;
int oct_xmin,oct_ymin,oct_zmin,oct_xmax,oct_ymax,oct_zmax,oct_range_x,oct_range_y,oct_range_z;
float last_yaw = 0.0;
int cnt = 0;
ros::Time last_sec;
double par_min_distance,par_maprad,par_dz;
double par_dst_target,par_dst_margin,par_cluster_max_spacing;
cv::Mat imb_blank_mono(1000,1000,CV_8U,cv::Scalar(0)); //create image, set encoding and size, init pixels to default val
cv::Mat img_paths(1000,1000,CV_8U,cv::Scalar(0)); //create image, set encoding and size, init pixels to default val
cv::Mat img(1000,1000,CV_8UC3,cv::Scalar(0,0,0)); //create image, set encoding and size, init pixels to default val
cv::Mat img_blank(1000,1000,CV_8UC3,cv::Scalar(0,0,0)); //create image, set encoding and size, init pixels to default val
geometry_msgs::Point last_endpoint;

geometry_msgs::PolygonStamped poly_cleared,poly_obstacles;
nav_msgs::Path path_raycast,path_visited;
std::vector<std::vector<geometry_msgs::Point32>> clusters;
std_msgs::Header hdr(){
  std_msgs::Header header;
  header.frame_id = "map";
  header.stamp    = ros::Time::now();
  return header;
}
float y2r(float y){
  return (img_paths.rows / 2 - y);
}
float x2c(float x){
  return (x + img_paths.cols/2);
}
int r2y(float r){
  return int((img_paths.rows / 2 - r));
}
int c2x(float c){
  return int((c - img_paths.cols / 2));
}
cv::Point pnt322cv(geometry_msgs::Point32 pin){
	int c = x2c(pin.x);
	int r = y2r(pin.y);
	return cv::Point(c,r);
}
cv::Point pnt2cv(geometry_msgs::Point pin){
	int c = x2c(pin.x);
	int r = y2r(pin.y);
	return cv::Point(c,r);
}
float get_dst2d(geometry_msgs::Point p2, geometry_msgs::Point p1){
  return(sqrt(pow(p1.x-p2.x,2)+pow(p1.y-p2.y,2)));
}
float get_dst2d32(geometry_msgs::Point32 p2, geometry_msgs::Point32 p1){
  return(sqrt(pow(p1.x-p2.x,2)+pow(p1.y-p2.y,2)));
}
float get_dst2d3(geometry_msgs::Point32 p2, geometry_msgs::Point p1){
  return(sqrt(pow(p1.x-p2.x,2)+pow(p1.y-p2.y,2)));
}
float get_dst2d32d(geometry_msgs::Point p2, geometry_msgs::Point32 p1){
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
bool is_point32_not_visited(geometry_msgs::Point32 pin,float lim){
  float res,dst;
  if(path_visited.poses.size() == 0)
    return true;
  for(int i = 0; i < path_visited.poses.size(); i++){
     if(get_dst2d32d(path_visited.poses[i].pose.position,pin) < lim)
        return false;
  }
  return true;
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
geometry_msgs::PolygonStamped remove_visited(geometry_msgs::PolygonStamped polyin){
	geometry_msgs::PolygonStamped polyout;
	polyout.header = hdr();
	for(int i = 0; i < polyin.polygon.points.size(); i++){
		if(is_point32_not_visited(polyin.polygon.points[i],7)){
			polyout.polygon.points.push_back(polyin.polygon.points[i]);
		}
	}
	return polyout;
}
geometry_msgs::Point get_next_endpoint(geometry_msgs::Point endpoint){
	geometry_msgs::Point new_endpoint;
	geometry_msgs::Point32 best_p;
	int r = y2r(endpoint.y);
	int c = x2c(endpoint.x);
	int ffillMode = 1;  int connectivity = 8;  int newMaskVal = 255;
	int flags = connectivity + (newMaskVal << 8) +
			 (ffillMode == 1 ? cv::FLOODFILL_FIXED_RANGE : 0);
	cv::Rect ccomp;
	int area = cv::floodFill(img_paths,cv::Point(c,r),cv::Scalar(150), &ccomp, cv::Scalar(1),cv::Scalar(1), flags);
	for(int x = oct_xmin; x < oct_xmax; x++){
		for(int y = oct_xmin; y < oct_xmax; y++){
			if(img_paths.at<uchar>(y2r(y),x2c(x)) == 150){
				geometry_msgs::Point32 p;
				p.x = x; p.y = y; p.z = endpoint.z;
				if(!is_point32_not_visited(p,7) && get_dst2d3(p,endpoint)){
					best_p = p;
				}
			}
		}
	}
	cv::circle(img_paths,pnt322cv(best_p),1,cv::Scalar(0,0,200),1);
	cv::line (img_paths, pnt2cv(endpoint), pnt322cv(best_p), cv::Scalar(0,200,0),1,cv::LINE_8,0);
	new_endpoint.x = best_p.x;
	new_endpoint.y = best_p.y;
	new_endpoint.z = best_p.z;
	return new_endpoint;
}
void get_side(){
	imb_blank_mono.copyTo(img_paths);
		for(int y = oct_ymin; y < oct_ymax; y++){
	    for(int x = oct_xmin; x < oct_xmax; x++){
	      octomap::point3d p(x,y,last_endpoint.z);
	      if(abs(edf_ptr.get()->getDistance(p) - par_dst_target) < par_dst_margin)
					img_paths.at<uchar>(y2r(y),x2c(x)) = 100;
		}
  }
	cv::imwrite("/home/nuc/brain/cluster/"+std::to_string(cnt)+"clusters.png",img_paths);
}
geometry_msgs::Point get_endpoint(geometry_msgs::Point pin,float collision_radius){
	imb_blank_mono.copyTo(img_paths);
	geometry_msgs::Point pout;
	for(int y = oct_ymin; y < oct_ymax; y++){
    for(int x = oct_xmin; x < oct_xmax; x++){
      octomap::point3d p(x,y,pin.z);
			octomap::point3d closestObst;
			float distance;
			edf_ptr.get()->getDistanceAndClosestObstacle(p,distance,closestObst);
			if(distance < collision_radius){
				img.at<cv::Vec3b>(y2r(p.y()),x2c(p.x()))[0] = distance;
				img.at<cv::Vec3b>(y2r(closestObst.y()),x2c(closestObst.x()))[2] = closestObst.z()*5;
			}
		}
	}
	cv::imwrite("/home/nuc/brain/cluster/"+std::to_string(cnt)+"distance_field.png",img);

	ROS_INFO("Get endpoint: %.0f %.0f %.0f out: %.0f %.0f %.0f",pin.x,pin.y,pin.z,pout.x,pout.y,pout.z);
	return pout;
}
std::vector<float> get_bbvec_from_endpoint(geometry_msgs::Point endpoint){
	std::vector<float> bbvec;
	bbvec.resize(6);
	bbvec[0] = endpoint.x - par_maprad;
	bbvec[1] = endpoint.y - par_maprad;
	bbvec[2] = endpoint.z - par_dz;
	bbvec[3] = endpoint.x + par_maprad;
	bbvec[4] = endpoint.y + par_maprad;
	bbvec[5] = endpoint.z + par_dz;
	return bbvec;
}
bool get_endpnt(geometry_msgs::Point pointin){
	ROS_INFO("Get endpoint: %.0f %.0f %.0f",pointin.x,pointin.y,pointin.z);
	float collision_radius = par_dst_target+par_dst_margin;
	std::vector<float> bbvec = get_bbvec_from_endpoint(pointin);
	if(!outside_octomap(bbvec)){
		if(update_edto_bbvec(bbvec,collision_radius)){
			geometry_msgs::Point p_temp = get_endpoint(pointin,collision_radius);
			if(p_temp.z == 0){
				ROS_INFO("did not find new endpoint");
				return false;
			}
			else{
				ROS_INFO("did find new endpoint: %.0f %.0f %.0f",p_temp.x,p_temp.y,p_temp.z);
				last_endpoint = p_temp;
				return true;
			}
		}
	}
}

void octomap_callback(const octomap_msgs::Octomap& msg){
  abs_octree=octomap_msgs::fullMsgToMap(msg);
  octree.reset(dynamic_cast<octomap::OcTree*>(abs_octree));
  got_map = true;
}

void path_visited_cb(const nav_msgs::Path::ConstPtr& msg){
	path_visited = *msg;
}
void test_cb(const geometry_msgs::Point::ConstPtr& msg){

}

nav_msgs::Path create_gridpath(float area_sidelength,float radlen_xy){
	ROS_INFO("Create gridpath: %.0f %.0f",area_sidelength,radlen_xy);
  float centroid_sides = 2*radlen_xy;
  int num_grids = area_sidelength / centroid_sides;
  nav_msgs::Path pathout;
  pathout.header = hdr();
  int i = 0;
  for (int y = 0; y < num_grids; y++)
  {
    for (int x = 0; x < num_grids; x++)
    {
			for(int zn = 0; zn < 10; zn++){
				geometry_msgs::PoseStamped ps;
				ps.pose.position.x = float(area_sidelength*-0.5 + x * centroid_sides+centroid_sides*0.5);
				ps.pose.position.y = float(area_sidelength*-0.5 + y * centroid_sides+centroid_sides*0.5);
				ps.pose.position.z = zn * 3;
				ps.pose.orientation.w = 1.0;
				ps.header = hdr();
				pathout.poses.push_back(ps);
			}
    }
  }
  return pathout;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "tb_edto_side_auto_node");
	ros::NodeHandle nh;
	ros::NodeHandle private_nh("~");
	private_nh.param("update_area_sidelength", par_maprad, 15.0);
	private_nh.param("update_area_z_radius", par_dz, 1.5);
	private_nh.param("dst_target",par_dst_target, 8.0);
	private_nh.param("dst_margin",par_dst_margin, 2.0);
	poly_obstacles.header = hdr();
	poly_cleared.header 	= hdr();
	path_raycast.header   = hdr();
	tf2_ros::TransformListener tf2_listener(tfBuffer);

	ros::Subscriber s1  = nh.subscribe("/octomap_full",1,octomap_callback);
	ros::Subscriber s4  = nh.subscribe("/tb_world/path_visited",1,path_visited_cb);
	ros::Subscriber s2  = nh.subscribe("/tb_test",1,test_cb);
	pub_path  					= nh.advertise<nav_msgs::Path>("/tb_edto/path_side",100);
	ros::Publisher pub_pnt = nh.advertise<geometry_msgs::PointStamped>("/tb_test/endpoint",100);
	ros::Rate rate(1.0);
	geometry_msgs::Point p0;
	nav_msgs::Path gridpath = create_gridpath(200,10);
	geometry_msgs::PointStamped endpoint_stamped;
	while(ros::ok()){
		endpoint_stamped.point = last_endpoint;
		endpoint_stamped.header = hdr();
		pub_pnt.publish(endpoint_stamped);
		if(got_map){
			if(last_endpoint.z > 0){
				std::vector<float> bbvec = get_bbvec_from_endpoint(last_endpoint);
				if(!outside_octomap(bbvec)){
					if(update_edto_bbvec(bbvec,par_dst_target+par_dst_margin)){
						get_side();
						last_endpoint = get_next_endpoint(last_endpoint);
						cnt++;
						cv::imwrite("/home/nuc/brain/cluster/"+std::to_string(cnt)+"clusters_down.png",img_paths);
					}
				}
			}
			else{
				if(cnt < gridpath.poses.size()){
					cnt++;
					bool got_new_endpoint = get_endpnt(gridpath.poses[cnt].pose.position);		
				}


			//	bool got_new_endpoint = false;
			//	int i = 0;
			//	while(!got_new_endpoint && i < gridpath.poses.size()-1){
			//		got_new_endpoint = get_endpnt(gridpath.poses[i++].pose.position);
			//	}
			}
		}
	//	rate.sleep();
		ros::spinOnce();
	}
	return 0;
}
