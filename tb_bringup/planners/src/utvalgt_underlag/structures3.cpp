#include <ros/ros.h>
#include <octomap_msgs/conversions.h>
#include <octomap_msgs/Octomap.h>
#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <octomap/OcTreeBase.h>
#include <octomap/octomap_types.h>
#include <tf/transform_datatypes.h>
#include <dynamicEDT3D/dynamicEDTOctomap.h>
#include <chrono>
#include <std_msgs/UInt8.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_listener.h>
#include "tf2_ros/message_filter.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include <nav_msgs/Path.h>
#include <nav_msgs/OccupancyGrid.h>
#include <map_msgs/OccupancyGridUpdate.h>
#include <std_msgs/UInt8.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Point32.h>
#include <eigen3/Eigen/Core>
#include <geometry_msgs/Vector3Stamped.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <image_transport/image_transport.h>

using namespace octomap;
using namespace std;
shared_ptr<DynamicEDTOctomap> edf_ptr;
AbstractOcTree* abs_octree;
shared_ptr<OcTree> octree;
ros::Publisher img_pub;
bool got_map,par_inspect_top;
double par_num_levels,par_maprad,par_hiabs,par_loabs,par_lookahead_distance,par_takeoffaltitude;
tf2_ros::Buffer tfBuffer;
int xmin,ymin,zmin,xmax,ymax,zmax,range_x,range_y,range_z,zmin_global,zmax_global;
geometry_msgs::PointStamped closest_obstacle,closest_obstacle_plane;
float closest_obstacle_plane_dist,closest_obstacle_dist;
geometry_msgs::Point pos,enter_slope;
geometry_msgs::PointStamped temp_target_origo,area_centroid;
geometry_msgs::Point bbmin_custom,bbmax_custom,bbmin_octree,bbmax_octree;
int missionstate,mainstate;
nav_msgs::Path building_path_visited,path_candidates,visited_path,path_active;
std::vector<nav_msgs::Path>building_paths;
ros::Publisher pub_tarpose,visited_path_pub,invoke_pub,targetalt_pub;
geometry_msgs::PoseStamped last_pose,target;
std_msgs::Float64 cmdarm_msg;
std_msgs::Float64 target_alt;
float area_range,delta_z;
std_msgs::UInt8 state_msg;

cv::Mat mapimg(1000,1000,CV_8UC3,cv::Scalar(0, 0, 0)); //create image, set encoding and size, init pixels to default val
cv::Mat mapimg_track(1000,1000,CV_8U,cv::Scalar(0)); //create image, set encoding and size, init pixels to default val
cv::Mat mapimg_mono_copy(1000,1000,CV_8U,cv::Scalar(0)); //create image, set encoding and size, init pixels to default val
cv::Mat mapimg_mono(1000,1000,CV_8U,cv::Scalar(0)); //create image, set encoding and size, init pixels to default val
cv::Mat mapimg_track_copy(1000,1000,CV_8U,cv::Scalar(0)); //create image, set encoding and size, init pixels to default val
cv::Mat mapimg_track_used(1000,1000,CV_8U,cv::Scalar(0)); //create image, set encoding and size, init pixels to default val
cv::Mat mapimg_visited(1000,1000,CV_8U,cv::Scalar(0)); //create image, set encoding and size, init pixels to default val
cv::Mat mapimg_copy(1000,1000,CV_8UC3,cv::Scalar(0, 0, 0)); //create image, set encoding and size, init pixels to default val
bool use_mono = true;;
float zneg = -10;
float zpos = 100;
float collision_radius = 1.1;
std::string par_workdir;
bool idle,par_live;
ros::Time last_twist,last_rosinfo;

cv::Mat slopeimage_mono(1000,1000,CV_8U,cv::Scalar(0)); //create image, set encoding and size, init pixels to default val
cv::Mat slopeimage_mono_mid(1000,1000,CV_8U,cv::Scalar(0)); //create image, set encoding and size, init pixels to default val
cv::Mat slopeimage_mono_mid_copy(1000,1000,CV_8U,cv::Scalar(0)); //create image, set encoding and size, init pixels to default val
cv::Mat slopeimage_mono_copy(1000,1000,CV_8U,cv::Scalar(0)); //create image, set encoding and size, init pixels to default val
cv::Mat building_mono(1000,1000,CV_8U,cv::Scalar(0)); //create image, set encoding and size, init pixels to default val
cv::Mat building_mono_copy(1000,1000,CV_8U,cv::Scalar(0)); //create image, set encoding and size, init pixels to default val
cv::Mat building_pixels(1000,1000,CV_8UC3,cv::Scalar(0, 0, 0)); //create image, set encoding and size, init pixels to default val
std::vector<std::tuple<int,int,int>>vt_i_c_r;
bool building_active,building_inspected_sides,building_inspected_top,building_post_processed;
int building_number_active = 0;
int building_visited_path_activation_start = 0;
int building_visited_path_activation_end = 0;
int area_building,area_slope;

geometry_msgs::PolygonStamped building_polygon;
geometry_msgs::PointStamped building_centroid,building_entry_point,building_closest_point;
geometry_msgs::PointStamped building_centroid1,building_centroid2;
nav_msgs::Path building_path,path_targets;
std::vector<int>buildings_completed;
std::vector<float>building_ranges_at_rad;
std::vector<geometry_msgs::Point>buildings_entry_points;
std::vector<geometry_msgs::Point>buildings_centroids;
std::vector<geometry_msgs::Point>buildings_closest_points;
std::vector<geometry_msgs::PolygonStamped>buildings_polygons;
std::vector<std::vector<geometry_msgs::Point>>buildings_min_max;

int last_i;
bool sort_i(const std::tuple<int,float,float,float>& a,
               const std::tuple<int,float,float,float>& b)
{
    return (std::get<0>(a) < std::get<0>(b));
}
bool sort_hdng_pair(const std::tuple<int,float>& a,
               			const std::tuple<int,float>& b)
{
    return (std::get<1>(a) < std::get<1>(b));
}
bool sort_hdng(const std::tuple<int,float,float,float>& a,
               const std::tuple<int,float,float,float>& b)
{
    return (std::get<1>(a) < std::get<1>(b));
}
bool sort_dst_visited(const std::tuple<int,float,float,float>& a,
               const std::tuple<int,float,float,float>& b)
{
    return (std::get<2>(a) < std::get<2>(b));
}

bool sort_dst_building(const std::tuple<int,float,float,float>& a,
               const std::tuple<int,float,float,float>& b)
{
    return (std::get<3>(a) < std::get<3>(b));
}
double get_shortest(double target_hdng,double actual_hdng){
  double a = target_hdng - actual_hdng;
  if(a > M_PI)a -= M_PI*2;
  else if(a < -M_PI)a += M_PI*2;
  return a;
}
void octomap_callback(const octomap_msgs::Octomap& msg){
  abs_octree=octomap_msgs::fullMsgToMap(msg);
  octree.reset(dynamic_cast<octomap::OcTree*>(abs_octree));
  got_map = true;
}

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
float get_dst2d(geometry_msgs::Point p1, geometry_msgs::Point p2){
  return(sqrt(pow(p1.x,2)+pow(p2.y,2)));
}
float get_dst3d(geometry_msgs::Point p1, geometry_msgs::Point p2){
  return(sqrt(pow(p1.x-p2.x,2)+pow(p1.y-p2.y,2)+pow(p1.z-p2.z,2)));
}
float get_hdng(geometry_msgs::Point p1,geometry_msgs::Point p0){
  float dx = p1.x - p0.x;
  float dy = p1.y - p0.y;
  return atan2(dy,dx);
}

bool in_poly(geometry_msgs::PolygonStamped polyin, geometry_msgs::Point32 point)
{
  int cross = 0;
  for(int i = 0,j = polyin.polygon.points.size()-1; i < polyin.polygon.points.size(); j=i++){
//  for (int i = 0, j = vertices_.size() - 1; i < vertices_.size(); j = i++) {
    if ( ((polyin.polygon.points[i].y > point.y) != (polyin.polygon.points[j].y > point.y))
           && (point.x < (polyin.polygon.points[j].x - polyin.polygon.points[i].x) * (point.y - polyin.polygon.points[i].y) /
            (polyin.polygon.points[j].y - polyin.polygon.points[i].y) + polyin.polygon.points[i].x) )
    {
      cross++;
    }
  }
  return bool(cross % 2);
}
bool in_path(nav_msgs::Path pathin,geometry_msgs::PoseStamped ps){
  if(pathin.poses.size() == 0)
    return false;
  for(int i = 0; i < pathin.poses.size(); i++){
    if( pathin.poses[i].pose.position.x == ps.pose.position.x
     && pathin.poses[i].pose.position.y == ps.pose.position.y
     && pathin.poses[i].pose.position.z == ps.pose.position.z)
      return true;
  }
  return false;
}
float dst_point_in_path_zfac(nav_msgs::Path pathin,geometry_msgs::Point pin,float zfac){
  float res,dst;
  res = 1000;
  if(pathin.poses.size() == 0)
    return res;
  for(int i = 0; i < pathin.poses.size(); i++){
    dst = sqrt((pathin.poses[i].pose.position.x-pin.x)*(pathin.poses[i].pose.position.x-pin.x)+
               (pathin.poses[i].pose.position.y-pin.y)*(pathin.poses[i].pose.position.y-pin.y)+
               zfac*(pathin.poses[i].pose.position.z-pin.z)*(pathin.poses[i].pose.position.z-pin.z));
    if(dst < res)
      res = dst;
  }
  return res;
}
float dst_point_in_path(nav_msgs::Path pathin,geometry_msgs::Point pin){
  float res,dst;
  res = 1000;
  if(pathin.poses.size() == 0)
    return res;
  for(int i = 0; i < pathin.poses.size(); i++){
    dst = sqrt((pathin.poses[i].pose.position.x-pin.x)*(pathin.poses[i].pose.position.x-pin.x)+
               (pathin.poses[i].pose.position.y-pin.y)*(pathin.poses[i].pose.position.y-pin.y)+
               (pathin.poses[i].pose.position.z-pin.z)*(pathin.poses[i].pose.position.z-pin.z));
    if(dst < res)
      res = dst;
  }
  return res;
}

bool in_building_completeded_list(int itarget){
  for(int i = 0; i < buildings_completed.size(); i++){
    if(buildings_completed[i] == itarget)
      return true;
  }
  return false;
}
bool in_blacklist(int itarget,std::vector<int>blacklist){
  for(int i = 0; i < blacklist.size(); i++){
    if(blacklist[i] == itarget)
      return true;
  }
  return false;
}
void activate_building(){
  building_visited_path_activation_start = visited_path.poses.size();
  building_active = true;
  ROS_INFO("Distance to Slope: %.2f; activating building %i at visited_path %i",building_number_active,building_visited_path_activation_start);
}
void deactivate_building(){
  building_active = false;
  building_visited_path_activation_end = visited_path.poses.size();
  ROS_INFO("Distance to Slope: %.2f; DEactivating building %i at visited_path %i",building_number_active,building_visited_path_activation_end);
}
std::vector<geometry_msgs::Point> get_path_limits(nav_msgs::Path pathin){
  std::vector<geometry_msgs::Point>out;
  double ltot,dx,dy,dz;
  geometry_msgs::Point lim0,lim1,dtot;
  for(int i = 0; i < (pathin.poses.size()-1); i++){
    if(pathin.poses[i+1].pose.position.x < lim0.x)
      lim0.x = pathin.poses[i+1].pose.position.x;
    if(pathin.poses[i+1].pose.position.y < lim0.y)
      lim0.y = pathin.poses[i+1].pose.position.y;
    if(pathin.poses[i+1].pose.position.z < lim0.z)
      lim0.z = pathin.poses[i+1].pose.position.z;
    if(pathin.poses[i+1].pose.position.x > lim1.x)
      lim1.x = pathin.poses[i+1].pose.position.x;
    if(pathin.poses[i+1].pose.position.y > lim1.y)
      lim1.y = pathin.poses[i+1].pose.position.y;
    if(pathin.poses[i+1].pose.position.z > lim1.z)
      lim1.z = pathin.poses[i+1].pose.position.z;
  }
  float dst_x,dst_y,dst_z;
  dst_x = lim1.x - lim0.x;
  dst_y = lim1.y - lim0.y;
  dst_z = lim1.z - lim0.z;
  ROS_INFO("PathWorker: boundingbox(min,max):  x: %.2f -> %.2f,y: %.2f -> %.2f,z: %.2f -> %.2f",lim0.x, lim0.y, lim0.z, lim1.x, lim1.y, lim1.z);
  ROS_INFO("PathWorker: range axes:            x: %.2f         y: %.2f         z: %.2f",dst_x, dst_y, dst_z);
  out.push_back(lim0);
  out.push_back(lim1);
  return out;
}
void post_process(){
  deactivate_building();
  nav_msgs::Path bpath;
  bpath.header = visited_path.header;
  for(int i = building_visited_path_activation_start; i < building_visited_path_activation_end; i++){
    bpath.poses.push_back(visited_path.poses[i]);
  }
  ROS_INFO("GetNextTarget: Building[%i] processed: %i poses",building_number_active,bpath.poses.size());
  building_paths.push_back(bpath);
  buildings_completed.push_back(building_number_active);
  buildings_min_max.push_back(get_path_limits(bpath));
  buildings_polygons.push_back(building_polygon);
  buildings_centroids.push_back(building_centroid.point);
  ROS_INFO("GetNextTarget: BUILDING[%i] POSTPROCESSING: From logged pos %i -> %i, %i poses",building_visited_path_activation_start,building_visited_path_activation_end,bpath.poses.size());
}
void pub_target(){
  if(path_candidates.poses.size() == 0 && pos.z > building_centroid.point.z){
    building_inspected_sides = true;
    post_process();
  }
  target.header.stamp  = ros::Time::now();
  ROS_INFO("TargetPUB: %.2f %.2f %.2f",target.pose.position.x,target.pose.position.y,target.pose.position.z);
  if(building_active)
    building_path_visited.poses.push_back(target);
  path_targets.poses.push_back(target);
  pub_tarpose.publish(target);
}
bool update_edto(geometry_msgs::Point midpoint,float collision_radius,float maprad,float z0,float z1,bool unknownAsOccupied,bool sides){
  if(got_map){
    ros::Time t0 = ros::Time::now();
    int zmid = (z1+z0)/2;


    octree.get()->getMetricMin(bbmin_octree.x,bbmin_octree.y,bbmin_octree.z);
    octree.get()->getMetricMax(bbmax_octree.x,bbmax_octree.y,bbmax_octree.z);
    zmin_global = int(round(bbmin_octree.z));
    zmax_global = int(round(bbmax_octree.z));

    z0 = fmin(z0,bbmax_octree.z-2);
    z1 = fmin(z1,bbmax_octree.z);
    bbmin_custom.x = midpoint.x-maprad;
    bbmin_custom.y = midpoint.y-maprad;
    bbmin_custom.z = z0;

    bbmax_custom.x = midpoint.x+maprad;
    bbmax_custom.y = midpoint.y+maprad;
    bbmax_custom.z = z1;
    float zmin_touse = fmax(bbmin_custom.z,bbmin_octree.z);
    float zmax_touse = fmin(bbmax_custom.z,bbmax_octree.z);
    if(zmax_touse < zmin_touse){
      zmax_touse = fmax(bbmin_custom.z,bbmin_octree.z);
      zmin_touse = fmin(bbmax_custom.z,bbmax_octree.z);
    }

    octomap::point3d boundary_min(fmax(bbmin_custom.x,bbmin_octree.x),fmax(bbmin_custom.y,bbmin_octree.y),zmin_touse);
    octomap::point3d boundary_max(fmin(bbmax_custom.x,bbmax_octree.x),fmin(bbmax_custom.y,bbmax_octree.y),zmax_touse);
    /*ROS_INFO("update_edto: bb_octre_min:  x: %.2f y: %.2f z: %.2f",bbmin_octree.x,bbmin_octree.y,bbmin_octree.z);
    ROS_INFO("update_edto: bb_octre_max:  x: %.2f y: %.2f z: %.2f",bbmax_octree.x,bbmax_octree.y,bbmax_octree.z);
    ROS_INFO("update_edto: bb_custom_min: x: %.2f y: %.2f z: %.2f",bbmin_custom.x,bbmin_custom.y,bbmin_custom.z);
    ROS_INFO("update_edto: bb_custom_max: x: %.2f y: %.2f z: %.2f",bbmax_custom.x,bbmax_custom.y,bbmax_custom.z);
    ROS_INFO("update_edto: bb_final_min:  x: %.2f y: %.2f z: %.2f",boundary_min.x(),boundary_min.y(),boundary_min.z());
    ROS_INFO("update_edto: bb_final_max:  x: %.2f y: %.2f z: %.2f",boundary_max.x(),boundary_max.y(),boundary_max.z());*/

    edf_ptr.reset (new DynamicEDTOctomap(collision_radius,octree.get(),
            boundary_min,
            boundary_max,
            unknownAsOccupied));
    edf_ptr.get()->update();
    xmin    = int(round(boundary_min.x()))+1;  ymin    = int(round(boundary_min.y()))+1; zmin    = int(round(boundary_min.z()));
    xmax    = int(round(boundary_max.x()))-1;  ymax    = int(round(boundary_max.y()))-1; zmax    = int(round(boundary_max.z()));
    range_x = xmax - xmin;                     range_y = ymax - ymin;                    range_z = zmax - zmin;
    int vol = range_x*range_y*range_z;
    if(vol <= 0)
      return false;
    octomap::point3d p(midpoint.x,midpoint.y,midpoint.z);
    octomap::point3d closestObst;
		float dst;
    edf_ptr.get()->getDistanceAndClosestObstacle(p, dst, closestObst);

		if(sides){
			closest_obstacle_plane.point.x=closestObst.x();
			closest_obstacle_plane.point.y=closestObst.y();
			closest_obstacle_plane.point.z=closestObst.z();
			closest_obstacle_plane_dist = dst;
		}
		else{
			closest_obstacle.point.x=closestObst.x();
			closest_obstacle.point.y=closestObst.y();
			closest_obstacle.point.z=closestObst.z();
			closest_obstacle_dist = dst;
		}
  float dt = (ros::Time::now() - t0).toSec();
    ROS_INFO("update_edto[%i points in %.3f sec]: xmin: %i, ymin: %i, zmin: %i, xmax: %i, ymax: %i, zmax: %i, range_x: %i, range_y: %i, range_z: %i ",vol,dt,xmin,ymin,zmin,xmax,ymax,zmax,range_x,range_y,range_z);
    return true;
  }
  else
    return false;
}

geometry_msgs::Point lengthedit_ray(geometry_msgs::Point p0,geometry_msgs::Point p1,float meters_to_add){
  Eigen::Vector3f stride_vec;
	Eigen::Vector3f pnt1_vec(p0.x,p0.y,p1.z);
	Eigen::Vector3f pnt2_vec(p1.x,p1.y,p1.z);
	float tot_length = (pnt2_vec - pnt1_vec).norm();
  stride_vec = (pnt2_vec - pnt1_vec).normalized();
  Eigen::Vector3f cur_vec = pnt1_vec + stride_vec * (tot_length+meters_to_add);
	geometry_msgs::Point new_endpoint;
	new_endpoint.x = cur_vec.x();
	new_endpoint.y = cur_vec.y();
	new_endpoint.z = cur_vec.z();
	ROS_INFO("P0 %.2f %.2f %.2f -> P1 %.2f %.2f %.2f:  [%.2f m], adding: %.2f m, endpoint: %.2f %.2f %.2f",p0.x,p0.y,p0.z,p1.x,p1.y,p1.z,tot_length,meters_to_add,new_endpoint.x,new_endpoint.y,new_endpoint.z);
  return new_endpoint;
}

void initialscan(geometry_msgs::Point midpoint, float collision_radius, float maprad, int num_rays){
    int xpos = int(round(midpoint.x));
    int ypos = int(round(midpoint.y));
    int zpos = int(round(midpoint.z));
		geometry_msgs::Point current_object;
		std::vector<geometry_msgs::Point>current_objects;
		std::vector<std::vector<geometry_msgs::Point>>objects_clusters;
    float rads_pr_i = 2*M_PI / num_rays;
		current_object.z = zpos;
    for(int i  = 0; i < num_rays; i++){
			float current_angle = -M_PI + i*rads_pr_i;
      Eigen::Vector3f pnt1_vec(xpos,ypos,zpos);
      Eigen::Vector3f pnt2_vec(xpos+maprad*cos(current_angle),maprad*sin(current_angle)+ypos,zpos);
      Eigen::Vector3f cur_vec = pnt1_vec;
      float tot_length = (pnt2_vec - pnt1_vec).norm();
      Eigen::Vector3f stride_vec;
      stride_vec = (pnt2_vec - pnt1_vec).normalized();
      float cur_ray_len=0;
			bool clear = true;
			point3d closestObst;
      while(cur_ray_len < maprad){
        cur_vec = cur_vec + stride_vec;
        cur_ray_len = (cur_vec-pnt1_vec).norm();
        point3d stridep(cur_vec.x(),cur_vec.y(),cur_vec.z());
				if(stridep.x() > bbmax_octree.x || stridep.y() > bbmax_octree.y || stridep.x() < bbmin_octree.x || stridep.y() < bbmin_octree.y){
					break;
				}
				float distance = collision_radius;
        edf_ptr.get()->getDistanceAndClosestObstacle(stridep,distance,closestObst);
				if(distance < collision_radius){
					clear = false;
					break;
				}
     	}
			if(!clear){
				current_object.x = closestObst.x();
				current_object.y = closestObst.y();
				current_objects.push_back(current_object);
				ROS_INFO("InitialScan[rads: %.2f]: Object[#%i] found at %.0f %.0f",current_angle,current_objects.size(),current_object.x,current_object.y);
			}
			else{
				if(current_objects.size() > 1){
					objects_clusters.push_back(current_objects);
					ROS_INFO("InitialScan[rads: %.2f]: CLEAR!,objects_cluster[#%i] had %i points",current_angle,objects_clusters.size(),current_objects.size());
				}
				if(current_objects.size() == 1){
					ROS_INFO("InitialScan[rads: %.2f]: CLEAR!, not appended as cluster due to size == 1",current_angle);
				}
				current_objects.resize(0);
			}
	 	}
		mapimg.copyTo(mapimg_copy);
		if(objects_clusters.size() > 0){
		for(int i = 0; i < objects_clusters.size(); i++){
			int cluster_color = objects_clusters.size() - i / objects_clusters.size();
			float min_dst = maprad;
			float min_dst_hdng;
			float min_hdng = M_PI;
			float max_hdng = -M_PI;
			geometry_msgs::Point entry_point;
			int max_hdng_i,min_hdng_i;
			for(int k = 0; k < objects_clusters[i].size(); k++){
				ROS_INFO("ObjectsCluster[%i]: point[%i]",i,k,objects_clusters[i][k].x,objects_clusters[i][k].y);
				float object_hdng = get_hdng(objects_clusters[i][k],midpoint);
				float object_dst  = get_dst2d(objects_clusters[i][k],midpoint);
				if(object_dst < min_dst){
					min_dst			 = object_dst;
					min_dst_hdng = object_hdng;
					entry_point  = objects_clusters[i][k];
				}
				if(object_hdng < min_hdng){
					min_hdng = object_hdng;
					min_hdng_i = k;
				}
				if(object_hdng > max_hdng){
					max_hdng = object_hdng;
					max_hdng_i = k;
				}
				int r = y2r(objects_clusters[i][k].y,mapimg.rows,1);
				int c = x2c(objects_clusters[i][k].x,mapimg.cols,1);
				cv::circle(mapimg_copy,cv::Point(c,r),5,cv::Scalar(cluster_color,cluster_color,cluster_color),-1);
			}
			int r0 = y2r(midpoint.y,mapimg.rows,1);
			int c0 = x2c(midpoint.x,mapimg.cols,1);
			int r1 = y2r(objects_clusters[i][min_hdng_i].y,mapimg.rows,1);
			int c1 = x2c(objects_clusters[i][min_hdng_i].x,mapimg.cols,1);
			cv::line (mapimg_copy, cv::Point(c0,r0), cv::Point(c1,r1), cv::Scalar(cluster_color,cluster_color,cluster_color),1,cv::LINE_8,0);
			r1 = y2r(objects_clusters[i][max_hdng_i].y,mapimg.rows,1);
		  c1 = x2c(objects_clusters[i][max_hdng_i].x,mapimg.cols,1);
			cv::line (mapimg_copy, cv::Point(c0,r0), cv::Point(c1,r1), cv::Scalar(cluster_color,cluster_color,cluster_color),1,cv::LINE_8,0);
			ROS_INFO("ObjectsCluster[%i]: hdng min->max: %.2f -> %.2f, entry_hdng: %.2f",i,min_hdng,max_hdng,min_dst_hdng,min_dst_hdng);
			buildings_closest_points.push_back(entry_point);
			entry_point = lengthedit_ray(midpoint,entry_point,-10);
			r1 = y2r(entry_point.y,mapimg.rows,1);
			c1 = x2c(entry_point.x,mapimg.cols,1);
			cv::circle(mapimg_copy,cv::Point(c1,r1),10,cv::Scalar(0,0,255),-1);
			cv::imwrite(par_workdir+"/initialscan.png",mapimg_copy);
			buildings_entry_points.push_back(entry_point);
			mapimg.copyTo(mapimg_copy);
    }
  }
}

void get_zdown(float collision_radius){
  octomap::point3d closestObst;
  for(int y = ymin; y < ymax; y++){
    for(int x = xmin; x < xmax; x++){
      int z = zmax;
      float dst = collision_radius;
      while(dst > (collision_radius-1) && z > bbmin_octree.z){
        octomap::point3d p(x,y,z);
        edf_ptr.get()->getDistanceAndClosestObstacle(p,dst,closestObst);
        z--;
      }
			int r = y2r(y,mapimg.rows,1);
			int c = x2c(x,mapimg.cols,1);
			mapimg.at<cv::Vec3b>(r,c)[2] = (z - bbmin_octree.z);
      mapimg_mono.at<uchar>(r,c) = (z - bbmin_octree.z);
    }
  }
	ROS_INFO("get_zdown complete");
}

void get_zsideways(float collision_radius,float zval){
	nav_msgs::Path path_candidates_backup;
	path_candidates_backup = path_candidates;
	ROS_INFO("Get z sideways %i candidates backup size",path_candidates.poses.size());
	path_candidates.poses.resize(0);
  for(int y = ymin; y < ymax; y++){
    for(int x = xmin; x < xmax; x++){

	    octomap::point3d p(x,y,zval);
			octomap::point3d closestObst;
	    float dst; //= edf_ptr.get()->getDistance(p);
			edf_ptr.get()->getDistanceAndClosestObstacle(p,dst,closestObst);
		//	int bnum = get_building(closestObst.x(),closestObst.y(),closestObst.z());
			int r = y2r(y,mapimg.rows,1); int c = x2c(x,mapimg.cols,1);
      slopeimage_mono.at<uchar>(r,c)     = 0;
			slopeimage_mono_mid.at<uchar>(r,c) = 0;
			int ro = y2r(closestObst.y(),mapimg.rows,1); int co = x2c(closestObst.x(),mapimg.cols,1);
			if(dst < collision_radius && dst > 0){
				slopeimage_mono.at<uchar>(r,c) = dst;
				mapimg.at<cv::Vec3b>(r,c)[1] 	 = dst;
			}
			if(abs(dst - collision_radius/2) < 1){
				float dst_pos = sqrt(pow(pos.x - x,2)+pow(pos.y-y,2));
				geometry_msgs::PoseStamped ps;
				float yaw  = atan2(closestObst.y() - y,closestObst.x()-x);
				float hdng = atan2(y - pos.y,x-pos.x);
				ps.pose.position.x = x;
				ps.pose.position.y = y;
				ps.pose.position.z = zval;
				ps.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);
				ps.header.frame_id = "map";
				ps.header.stamp    = ros::Time::now();
				slopeimage_mono_mid.at<uchar>(r,c) = 200;
				if(dst_pos < par_lookahead_distance){
					if(slopeimage_mono_mid_copy.at<uchar>(r,c) >= 200
            && !(mapimg_visited.at<uchar>(r,c) > pos.z-3)
						&& dst_point_in_path_zfac(visited_path,ps.pose.position,3) > 15
            //&& dst_point_in_path(path_candidates,ps.pose.position) > 7
						&& get_shortest(yaw,hdng) > 0)

							path_candidates.poses.push_back(ps);
				}
			}
			if(mapimg.at<cv::Vec3b>(ro,co)[0] != 0)
				mapimg.at<cv::Vec3b>(r,c)[0] = mapimg.at<cv::Vec3b>(ro,co)[0];
		}
	}
	ROS_INFO("Get z sideways %i candidates output size",path_candidates.poses.size());
	if(path_candidates.poses.size() == 0 && path_candidates_backup.poses.size() > 1)
		path_candidates = path_candidates_backup;
}


geometry_msgs::Point get_closest_point_in_slope(geometry_msgs::Point seed_point,int maprad){
	geometry_msgs::Point closest_point;
	int rmid = y2r(seed_point.y,mapimg.rows,1);
	int cmid = x2c(seed_point.x,mapimg.cols,1);
	int rmin = fmax(rmid - maprad,0);
	int cmin = fmax(cmid - maprad,0);
	int rmax = fmin(rmid + maprad,mapimg.rows);
	int cmax = fmin(cmid + maprad,mapimg.cols);
	float mindst = 1000;
	ROS_INFO("cmin %i rmin %i,cmax %i rmax %i ",cmin,rmin,cmax,rmax);

	ROS_INFO("Closest point in slope orig_point: %.0f %.0f",seed_point.x,seed_point.y);
	for(int r = rmin; r < rmax; r++){
		for(int c = cmin; c < cmax; c++){
			if(slopeimage_mono_mid.at<uchar>(r,c) == 200){
				float dst = sqrt(pow(r - rmid,2) + pow(c - cmid,2));
				if(dst < mindst){
					mindst = dst;
					closest_point.x = c2x(c,mapimg.rows,1);
					closest_point.y = r2y(r,mapimg.cols,1);
				}
			}
		}
	}
  if(!in_building_completeded_list(building_number_active)){
  	if(!building_active && mindst < 5)
      activate_building();
  	else if(building_active && mindst > 10)
  		deactivate_building();
  	else
  		return closest_point;
  }

	return closest_point;
}

geometry_msgs::Point colorize_area(int r, int c,int maprad){
	int rmin = fmax(r - maprad,0);
	int cmin = fmax(c - maprad,0);
	int rmax = fmin(r + maprad,mapimg.rows);
	int cmax = fmin(c + maprad,mapimg.cols);
	for(int r = rmin; r < rmax; r++){
		for(int c = cmin; c < cmax; c++){
			mapimg_visited.at<uchar>(r,c) = int(pos.z);
		}
	}
}

void alternate_centroid(){
  std::vector<geometry_msgs::Point>bb01;
  bb01 = get_path_limits(building_path_visited);
  int	cmin = x2c(bb01[0].x,building_mono_copy.cols,1);
  int	cmax = x2c(bb01[1].x,building_mono_copy.cols,1);
  int	rmin = y2r(bb01[1].y,building_mono_copy.rows,1);
  int	rmax = y2r(bb01[0].y,building_mono_copy.rows,1);
  building_centroid1.point.x = round((bb01[0].x+bb01[1].x)/2);
  building_centroid1.point.y = round((bb01[0].y+bb01[1].y)/2);
  building_centroid1.point.z = round(bb01[1].z);
  ROS_INFO("area_centroid3 %i counts %.0f,%.0f",building_centroid1.point.x,building_centroid1.point.y,building_centroid1.point.z);
}

void update_object_info(){
	float xsum = 0;
	float ysum = 0;
	int xy_count = 0;
	std::vector<float> zvals;
	std::vector<geometry_msgs::Point>bb01;
	bb01 = get_path_limits(building_path_visited);
	int	cmin = x2c(bb01[0].x,mapimg_track.cols,1);
	int	cmax = x2c(bb01[1].x,mapimg_track.cols,1);
	int	rmin = y2r(bb01[1].y,mapimg_track.rows,1);
	int	rmax = y2r(bb01[0].y,mapimg_track.rows,1);
	area_range = fmax(sqrt(pow(cmax - cmin,2)+pow(rmax-rmin,2)),50)/2;
	for(int r = rmin; r < rmax; r++){
		for(int c = cmin; c < cmax; c++){
      if(building_mono_copy.at<uchar>(r,c) == 255){
				xsum += c2x(c,mapimg_track.cols,1);
				ysum += r2y(r,mapimg_track.rows,1);
				xy_count++;
				zvals.push_back(mapimg.at<cv::Vec3b>(r,c)[2]);
			}
		}
	}
	if(xy_count > 0){
		sort(zvals.begin(), zvals.end());
		int percent10 = int(round(zvals.size()*0.1));
		ROS_INFO("Zvals: 2/10: %.0f, 5/10: %.0f, 8/10: %.0f",zvals[percent10*2],zvals[percent10*5],zvals[percent10*8]);
    building_centroid2.point.x = round(xsum / xy_count);
		building_centroid2.point.y = round(ysum / xy_count);
		building_centroid2.point.z = round(zvals[percent10*9]);
		ROS_INFO("area_centroid %i counts %.0f,%.0f",xy_count,building_centroid.point.x,building_centroid.point.y,building_centroid.point.z);
	}
}

void color_building(int building_number){
	int rmax = y2r(ymin,mapimg.rows,1);
	int rmin = y2r(ymax,mapimg.rows,1);
	int cmin = x2c(xmin,mapimg.cols,1);
	int cmax = x2c(xmax,mapimg.cols,1);
	int rsum,csum;
	int count = 0;
	std::vector<int> zvals;
	ROS_INFO("Color building %i",building_number);
	for(int r = rmin; r < rmax; r++){
		for(int c = cmin; c < cmax; c++){
			if(building_mono_copy.at<uchar>(r,c) == 255){
				zvals.push_back(mapimg_mono.at<uchar>(r,c));
				//vt_i_c_r.push_back(std::make_tuple(vt_i_c_r.size(),c,r));
				count++;
				rsum += r;
				csum += c;
				mapimg.at<cv::Vec3b>(r,c)[0] = building_number*10;
				mapimg_mono_copy.at<uchar>(r,c) = 0;
			}
		}
	}
	if(count > 0){
		sort(zvals.begin(), zvals.end());
		int percent10 = int(round(zvals.size()*0.1));
		ROS_INFO("Zvals: 2/10: %.0f, 5/10: %.0f, 8/10: %.0f",zvals[percent10*2],zvals[percent10*5],zvals[percent10*8]);
		building_centroid.point.x = c2x(int(round(csum/count)),mapimg.cols,1);
		building_centroid.point.y = r2y(int(round(rsum/count)),mapimg.rows,1);
		building_centroid.point.z = round(zvals[percent10*9]);
		ROS_INFO("building_centroid %i counts %.0f %.0f,%.0f",count,building_centroid.point.x,building_centroid.point.y,building_centroid.point.z);
	}
}

bool check_if_inside_bb(std::vector<geometry_msgs::Point> l0l1,geometry_msgs::Point p){
	if(l0l1[0].x < p.x && l0l1[0].y < p.y && l0l1[1].x > p.x && l0l1[1].y > p.y)
		return true;
	else
		return false;
}
void update_edtos_down(){
	collision_radius = 1.1;
	float maprad = par_maprad;
	update_edto(pos,collision_radius,maprad,pos.z-10,pos.z+50,false,false);
	get_zdown(collision_radius);
}
void update_edtos_side(){
	float collision_radius_sides = 20;
	float maprad = par_maprad;
	update_edto(pos,collision_radius_sides,maprad,pos.z-1,pos.z,false,true);
	get_zsideways(collision_radius_sides,pos.z);
}


int request_floodfill_building(geometry_msgs::Point seed_point,int roi_rad){
	int seed_c = x2c(seed_point.x,mapimg.cols,1);
	int seed_r = y2r(seed_point.y,mapimg.rows,1);
	cv::Point seed_pnt(seed_c,seed_r);
  int ffillMode = 0;
  int connectivity = 8;
  int newMaskVal = 255;
  int flags = connectivity + (newMaskVal << 8) +
       (ffillMode == 1 ? cv::FLOODFILL_FIXED_RANGE : 0);
  cv::Rect ccomp;
//	mapimg_mono.copyTo(building_mono_copy);

	cv::Rect rect_roi(seed_c-roi_rad,seed_r-roi_rad,roi_rad*2,roi_rad*2);
  int area = cv::floodFill(building_mono_copy,seed_pnt,cv::Scalar(255), &rect_roi, cv::Scalar(2),cv::Scalar(2), flags);
	cv::imwrite(par_workdir + "/building_mono.png", building_mono);
	cv::imwrite(par_workdir + "/building_mono_copy.png", building_mono_copy);
	ROS_INFO("FLOODFILL[x %.2f y %.2f] ABS area: %i",seed_point.x,seed_point.y,area);
  return area;
}

int request_floodfill_slope(geometry_msgs::Point seed_point,int roi_rad){
  int seed_c = x2c(seed_point.x,mapimg.cols,1);
  int seed_r = x2c(seed_point.y,mapimg.cols,1);
 	int dst0 = slopeimage_mono.at<uchar>(seed_r,seed_c);
	cv::Point seed_pnt(seed_c,seed_r);

  int ffillMode = 0;
  int connectivity = 8;
  int newMaskVal = 255;
  int flags = connectivity + (newMaskVal << 8) +
       (ffillMode == 1 ? cv::FLOODFILL_FIXED_RANGE : 0);
  cv::Rect ccomp;
	slopeimage_mono.copyTo(slopeimage_mono_copy);
	slopeimage_mono_mid.copyTo(slopeimage_mono_mid_copy);

	cv::Rect rect_roi(seed_c-roi_rad,seed_r-roi_rad,roi_rad*2,roi_rad*2);
	int lo = fmax(dst0 - 2,1);
	int hi = fmax(20 - dst0,1);

	int area  = cv::floodFill(slopeimage_mono_copy,seed_pnt,cv::Scalar(255), &rect_roi, cv::Scalar(5),cv::Scalar(5), flags);
	ROS_INFO("FLOODFILL_MONO[x %.2f y %.2f] ABS area: %i",seed_point.x,seed_point.y,area);
	int area2 = cv::floodFill(slopeimage_mono_mid_copy,seed_pnt,cv::Scalar(255), &rect_roi, cv::Scalar(1),cv::Scalar(1), flags);
	ROS_INFO("FLOODFILL_MONO_MID[x %.2f y %.2f] ABS area: %i",seed_point.x,seed_point.y,area2);

	cv::imwrite(par_workdir + "/slopeimage_mono.png", slopeimage_mono);
	cv::imwrite(par_workdir + "/slopeimage_mono_copy.png", slopeimage_mono_copy);
	cv::imwrite(par_workdir + "/slopeimage_mono_mid.png", slopeimage_mono_mid);
	cv::imwrite(par_workdir + "/slopeimage_mono_mid_copy.png", slopeimage_mono_mid_copy);
  return area;
}

void find_closest_building(){
	int closest_building;
	float closest_dst = 1000;
	if(buildings_entry_points.size() > 1){
		for(int i = 0; i < buildings_entry_points.size(); i++){
      if(!in_building_completeded_list(i)){
  			float dst_0 = sqrt(pow(buildings_entry_points[i].x,2)+pow(buildings_entry_points[i].y,2));
  			if(dst_0 < closest_dst){
  				ROS_INFO("ObjectsCluster[%i]: new closest building[%i]: %.2f -> %.2f",closest_building,i,closest_dst,dst_0);
  				closest_building = i;
  				closest_dst      = dst_0;
  			}
  		}
    }
	}
  building_post_processed = false;
  building_inspected_sides = false;
  building_active = false;
  building_inspected_top = false;
	building_number_active = closest_building;
	int	c = x2c(building_closest_point.point.x,mapimg_track.cols,1);
	int	r = y2r(building_closest_point.point.y,mapimg_track.rows,1);
	building_closest_point.point = buildings_closest_points[closest_building];
	building_centroid.point      = buildings_closest_points[closest_building];
	building_entry_point.point   = buildings_entry_points[closest_building];
	target.pose.position 	 			 = buildings_entry_points[building_number_active];
  pub_target();
}
void checktf(){
  geometry_msgs::TransformStamped transformStamped;
  try{
    transformStamped = tfBuffer.lookupTransform("map","base_stabilized",
                             ros::Time(0));
  }
  catch (tf2::TransformException &ex) {
    ROS_WARN("%s",ex.what());
    ros::Duration(1.0).sleep();
  }

	pos.x = transformStamped.transform.translation.x;
	pos.y = transformStamped.transform.translation.y;
	pos.z = transformStamped.transform.translation.z;
	if(building_active){
		float rel_hdng  = get_hdng(pos,building_centroid.point);
		float rel_dist  = get_dst2d(pos,building_centroid.point);
		float rads_pr_i = 2*M_PI / 72;
		int 	i   	    = int(round((rel_hdng + M_PI) / rads_pr_i));
    if(abs(i - last_i) > 1){
      int delta_i = abs(i - last_i);
      float d_x = pos.x - building_polygon.polygon.points[last_i].x;
      float d_y = pos.y - building_polygon.polygon.points[last_i].y;
      float d_z = pos.z - building_polygon.polygon.points[last_i].z;
      for(int ii = 0; ii < delta_i; ii++){
        building_polygon.polygon.points[last_i+i].x = building_polygon.polygon.points[last_i].x + d_x / delta_i * i;
        building_polygon.polygon.points[last_i+i].y = building_polygon.polygon.points[last_i].y + d_y / delta_i * i;
        building_polygon.polygon.points[last_i+i].z = building_polygon.polygon.points[last_i].z + d_z / delta_i * i;
        building_ranges_at_rad[last_i+i] = sqrt(pow(building_polygon.polygon.points[last_i+i].x - building_centroid.point.x,2)+
                                                pow(building_polygon.polygon.points[last_i+i].y - building_centroid.point.y,2));
      }
    }
    last_i = i;
		building_polygon.polygon.points[i].x = pos.x;
		building_polygon.polygon.points[i].y = pos.y;
		building_polygon.polygon.points[i].z = pos.z;
		if((ros::Time::now()-last_rosinfo).toSec() > 2){
			last_rosinfo = ros::Time::now();
			ROS_INFO("CheckTf: Pos %.2f %.2f %.2f, centroid %.2f %.2f %.2f relative %.2f %.2f %.2f",pos.x,pos.y,pos.z,building_centroid.point.x,building_centroid.point.y,building_centroid.point.z,pos.x-building_centroid.point.x,pos.y-building_centroid.point.y,pos.z-building_centroid.point.z);
			ROS_INFO("CheckTf: rel_hdng %.2f rel_dist %.2f rads_pr_i %.2f i: %i",rel_hdng,rel_dist,rads_pr_i,i);
		}
	}
  if(mainstate == 0){
    if(transformStamped.transform.translation.z >= 1.0)
      mainstate = 1;
  }
  else if(mainstate == 2){
    if(sqrt(pow(transformStamped.transform.translation.x,2)+pow(transformStamped.transform.translation.y,2)) < 3){
      if(target_alt.data > 5)
        target_alt.data = 5;
      else if(round(transformStamped.transform.translation.z<= 5))
        mainstate = 3;
    }
    else if((ros::Time::now() - visited_path.header.stamp).toSec() > 5){
      target.pose.position.x = 0;
      target.pose.position.y = 0;
      target.pose.position.z = 30;
      pub_target();
    }
  }
  else if(sqrt(pow(transformStamped.transform.translation.x - last_pose.pose.position.x,2)+ pow(transformStamped.transform.translation.y - last_pose.pose.position.y,2)+
  pow(transformStamped.transform.translation.z - last_pose.pose.position.z,2)) >= 1.00){
    last_pose.pose.position.x  = transformStamped.transform.translation.x;

	  last_pose.pose.position.y  = transformStamped.transform.translation.y;
    last_pose.pose.position.z  = transformStamped.transform.translation.z;
    last_pose.pose.orientation = transformStamped.transform.rotation;
    last_pose.header           = transformStamped.header;
		int r = y2r(last_pose.pose.position.y,mapimg.rows,1);
		int c = x2c(last_pose.pose.position.x,mapimg.cols,1);

    visited_path.poses.push_back(last_pose);
    visited_path.header.stamp  = ros::Time::now();
		colorize_area(r,c,5);
	}
}


float get_danger_to_pos(geometry_msgs::Point target_point){
  float maprad = get_dst2d(target_point,pos) * 1.2;
  geometry_msgs::Point midpoint;
  float collision_radius = 15;
  midpoint.x = (pos.x + target_point.x)/2;
  midpoint.y = (pos.y + target_point.y)/2;
  midpoint.z = (pos.z + target_point.z)/2;
  update_edto(midpoint,collision_radius,maprad,midpoint.z-1,midpoint.z+1,false,true);
  Eigen::Vector3f pnt1_vec(pos.x,pos.y,pos.z);
  Eigen::Vector3f pnt2_vec(target_point.x,target_point.y,target_point.z);
  Eigen::Vector3f stride_vec;
  Eigen::Vector3f cur_vec = pnt1_vec;
  float cur_ray_len = 0;
  float distance    = collision_radius;
  stride_vec        = (pnt2_vec - pnt1_vec).normalized();
  float tot_length  = (pnt2_vec - pnt1_vec).norm();
  update_edto(pos,collision_radius,tot_length+collision_radius,pos.z-1.5,pos.z+1,false,true);
  float stride_len  = 1;
  float lowest_dst  = 100;
  for(int k = 0; k < tot_length / stride_len; k++){
    cur_vec = cur_vec + stride_vec * stride_len;
    point3d stridep(cur_vec.x(),cur_vec.y(),cur_vec.z());
    distance = edf_ptr.get()->getDistance(stridep);
    if(distance < lowest_dst)
      lowest_dst = distance;
  }
  return lowest_dst;
}

void refresh_buildings_initscan(float collision_radius,float maprad,geometry_msgs::Point midpoint){
  update_edto(midpoint,collision_radius,maprad,midpoint.z-1,midpoint.z+1,false,true);
  initialscan(midpoint,collision_radius,maprad,36);
  find_closest_building();
}


void get_next_target_from_candidates(){
  if(!building_inspected_sides && building_active){
    if(path_candidates.poses.size() == 0){
      target_alt.data += 5;
      target = building_path_visited.poses[0];
      target.pose.position.z = target_alt.data;
      building_path_visited.poses.push_back(target);
      float dst_target = get_dst2d(target.pose.position,pos);
      ROS_INFO("GetNextTarget: Dst to next target: %.2f %.2f %.2f %.2f",dst_target,target.pose.position.x,target.pose.position.y,target.pose.position.z);
      pub_target();
    }
    else{
      float best_danger_to_i = -10;
      int best_cand = 0;
      ROS_INFO("GetNextTarget: comparing %i poses",path_candidates.poses.size());
      for(int i = 0; i< path_candidates.poses.size(); i++){
        float rel_hdng  = get_hdng(path_candidates.poses[i].pose.position,building_centroid.point);
        float rel_hdng0 = get_hdng(pos,building_centroid.point);
        float rel_dist  = get_dst2d(pos,path_candidates.poses[i].pose.position);
        float delta_hdng= get_shortest(rel_hdng,rel_hdng0);
        float hdng_pr_dst = delta_hdng / (rel_dist+0.000001);
        float danger_to_i = get_danger_to_pos(path_candidates.poses[i].pose.position);

        ROS_INFO("GetNextTarget: danger_to_i: %.2f hdng_pr_dst: %.2f delta_hdng: %.2f rel_hdng (%.2f -> %.2f) rel_dist %.2f i: %i",danger_to_i,hdng_pr_dst,delta_hdng,rel_hdng0,rel_hdng,rel_dist,best_cand);

        if(danger_to_i*rel_dist > best_danger_to_i && delta_hdng > 0 && rel_dist < 50){
          best_danger_to_i = danger_to_i*rel_dist;
          best_cand = i;
          ROS_INFO("GetNextTarget: hdng_pr_dst: %.2f delta_hdng: %.2f rel_hdng (%.2f -> %.2f) rel_dist %.2f i: %i",hdng_pr_dst,delta_hdng,rel_hdng0,rel_hdng,rel_dist,best_cand);
        }
      }
      target = path_candidates.poses[best_cand];
      pub_target();
    }
  }
}

void result_cb(const std_msgs::Bool::ConstPtr& msg){
  idle = true;
}

cv::Mat get_element(int dilation_elem, int dilation_size){
	int dilation_type;
	if( dilation_elem == 0 ){ dilation_type = cv::MORPH_RECT; }
	else if( dilation_elem == 1 ){ dilation_type = cv::MORPH_CROSS; }
	else if( dilation_elem == 2) { dilation_type = cv::MORPH_ELLIPSE; }
	cv::Mat element = cv::getStructuringElement( dilation_type,
																	 cv::Size( 2*dilation_size + 1, 2*dilation_size+1 ),
																	 cv::Point( dilation_size, dilation_size ) );
	return element;
}

void cmdvel_cb(const geometry_msgs::Twist::ConstPtr& msg){
	//if(sqrt(pow(msg->linear.x,2) + pow(msg->angular.z,2)) > 0.3){
	if(msg->linear.x + msg->angular.z != 0.00){
		last_twist = ros::Time::now();
		idle = false;
	}
}
int main(int argc, char** argv)
{
  ros::init(argc, argv, "tb_structures_node");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");
  private_nh.getParam("workdir_path", par_workdir);
	private_nh.param("map_sidelength_min",  par_maprad, 150.0);
	private_nh.param("ffill_hiabs",  par_hiabs, 2.0);
  private_nh.param("ffill_loabs",  par_loabs, 3.0);
	private_nh.param("takeoff_altitude",		par_takeoffaltitude, 15.0);
	private_nh.param("lookahead_distance",  par_lookahead_distance, 50.0);
  private_nh.param("is_live",		par_live, false);
  private_nh.param("inspect_top",		par_inspect_top, false);
  tf2_ros::TransformListener tf2_listener(tfBuffer);
	visited_path.header.frame_id = "map";

	target.header.frame_id = "map";
	target.pose.orientation.w = 1;
	target.header.stamp    = ros::Time::now();
	building_polygon.polygon.points.resize(72);
	building_ranges_at_rad.resize(72);

path_candidates.header = path_targets.header = building_path_visited.header = building_polygon.header = building_entry_point.header = building_closest_point.header
= building_centroid.header = building_centroid1.header = building_centroid2.header = target.header;


ros::Subscriber s1 = nh.subscribe("/octomap_full",1,octomap_callback);
ros::Subscriber s2 = nh.subscribe("/cmd_vel",1,&cmdvel_cb);
	ros::Subscriber sa = nh.subscribe("/tb_cmdmb/success",  100,&result_cb);

	targetalt_pub       = nh.advertise<std_msgs::Float64>("/tb_cmd/alt_target",100);
	pub_tarpose         = nh.advertise<geometry_msgs::PoseStamped>("/tb_cmdmb/target_pose",100);

	visited_path_pub    					= nh.advertise<nav_msgs::Path>("/tb_nav/visited_path",10);
	ros::Publisher cpath_pub   	  = nh.advertise<nav_msgs::Path>("/tb_path/path_candidates",100);

	ros::Publisher building_polygon_pub       = nh.advertise<geometry_msgs::PolygonStamped>("/tb_bld/building_polygon",100);
  ros::Publisher building_centroid_pub      = nh.advertise<geometry_msgs::PointStamped>("/tb_bld/building_centroid",100);
  ros::Publisher building_centroid_pub1     = nh.advertise<geometry_msgs::PointStamped>("/tb_bld/building_centroid1",100);
  ros::Publisher building_centroid_pub2     = nh.advertise<geometry_msgs::PointStamped>("/tb_bld/building_centroid2",100);
	ros::Publisher building_entry_point_pub   = nh.advertise<geometry_msgs::PointStamped>("/tb_bld/building_entry_point",100);
	ros::Publisher building_closest_point_pub = nh.advertise<geometry_msgs::PointStamped>("/tb_bld/building_closest_point",100);
	ros::Publisher building_path_pub          = nh.advertise<nav_msgs::Path>("/tb_bld/building_path",100);
	ros::Publisher building_path_visited_pub  = nh.advertise<nav_msgs::Path>("/tb_bld/building_path_visited",100);
  ros::Publisher pub 											  = nh.advertise<sensor_msgs::Image>("/tb_bld/mapimg", 1);

	ros::Publisher state_pub   = nh.advertise<std_msgs::UInt8>("/tb_fsm/main_state",10);
	ros::Publisher invoke_pub  = nh.advertise<std_msgs::String>("/tb_invoke",10);
	ros::Publisher cmdarm_pub  = nh.advertise<std_msgs::Float64>("/tb_cmd/arm_pitch",10);

	ros::Rate rate(1);
	bool done = true;
	ros::Time start = ros::Time::now();
	ros::Time last_check = ros::Time::now();
	bool building_base_mapped;
	int pathnum;
	bool invoke_published;
	bool bag_published;
	target_alt.data = par_takeoffaltitude;
	int zval = round(target_alt.data);
  bool first = true;
  while(ros::ok()){
		checktf();

		if(mainstate != state_msg.data) {
			ROS_INFO("MAINSTATE: %i -> %i",state_msg.data,mainstate);
			state_msg.data      = mainstate;
		}
		if(par_live){
			if((ros::Time::now() - start).toSec() > 5 && !invoke_published){
				std_msgs::String invoke_msg;
				invoke_msg.data = "roslaunch tb_nxtgen m600.launch";
				invoke_published = true;
				invoke_pub.publish(invoke_msg);
			}
			else if((ros::Time::now() - start).toSec() > 10 && !bag_published){
				std_msgs::String invoke_msg;
				invoke_msg.data = "rosbag record -O /home/nuc/bag.bag -a";
				bag_published = true;
				invoke_pub.publish(invoke_msg);
			}
			else if (mainstate < 2 && (ros::Time::now() - start).toSec() > 700)
				mainstate = 2;
		}
		cv::Mat element = get_element(0,3);

		if((ros::Time::now() - last_check).toSec() > 5 && got_map){
			last_check = ros::Time::now();
			update_edtos_down();
			cv::erode(mapimg_mono,mapimg_mono,element);
			cv::dilate(mapimg_mono,mapimg_mono,element);
			if(buildings_entry_points.size() > building_number_active)
				color_building(building_number_active);
      if(building_path_visited.poses.size() > 2){
        alternate_centroid();
        update_object_info();
      }
		}
	  else if(done && got_map){
			done = false;
			cv::imwrite(par_workdir + "/structures.png", mapimg);

			update_edtos_side();
			cv::erode(slopeimage_mono,slopeimage_mono,element);
		//	cv::dilate(slopeimage_mono,slopeimage_mono,element);
			cv::threshold(mapimg_mono, building_mono_copy, zval, 50, cv::THRESH_BINARY);

	//		cv::threshold(mapimg_mono, mapimg_mono_copy, 0, zval, cv::THRESH_BINARY);
			//			cv::threshold(slopeimage_mono, slopeimage_mono_copy, zval,50, cv::THRESH_BINARY);
			cv::imwrite(par_workdir + "/structures_mapimage_mono.png", mapimg_mono );
			cv::imwrite(par_workdir + "/structures_building_mono_copy.png", building_mono_copy );
			cv::imwrite(par_workdir + "/structures_slopeimage_tresh.png", slopeimage_mono );

			area_slope  = request_floodfill_slope(get_closest_point_in_slope(pos,50),150);
			area_building = request_floodfill_building(building_centroid.point,150);
      delta_z = target_alt.data - pos.z;
      if(first && pos.z == target_alt.data){
        first = false;
        refresh_buildings_initscan(5,75,pos);
      }
      else if(idle && abs(delta_z) < 1){
        get_next_target_from_candidates();
      }

      building_centroid_pub1.publish(building_centroid1);
      building_centroid_pub2.publish(building_centroid2);
      building_centroid_pub.publish(building_centroid);
			building_entry_point_pub.publish(building_entry_point);
			building_closest_point_pub.publish(building_closest_point);
			building_polygon_pub.publish(building_polygon);

			visited_path_pub.publish(visited_path);
		//	building_path_pub.publish(building_path);
			building_path_visited_pub.publish(building_path_visited);
			cpath_pub.publish(path_candidates);

			state_pub.publish(state_msg);
			targetalt_pub.publish(target_alt);

	//		bpath_pub.publish(building_path);
			cv_bridge::CvImage cv_image;
	    sensor_msgs::Image ros_image;
	    cv_image.image = mapimg;
	    cv_image.encoding = "bgr8";
	    cv_image.toImageMsg(ros_image);
	    pub.publish(ros_image);
	    done = true;
		}
	  rate.sleep();
	  ros::spinOnce();
  }
  return 0;
}
