//path_service:
// example showing how to receive a nav_msgs/Path request
// run with complementary path_client
// this could be useful for
#include <ros/ros.h>
#include <octomap_msgs/conversions.h>
#include <octomap_msgs/Octomap.h>
#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <octomap/OcTreeBase.h>
#include <octomap/octomap_types.h>
#include <tf/transform_datatypes.h>
#include <dynamicEDT3D/dynamicEDTOctomap.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Pose.h>
#include <iostream>
#include <string>
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
tf2_ros::Buffer tfBuffer;

using namespace octomap;
using namespace std;
shared_ptr<DynamicEDTOctomap> edf_ptr;
AbstractOcTree* abs_octree;
shared_ptr<OcTree> octree;
string par_workdir_path;
geometry_msgs::Point bbmin_custom,bbmax_custom,bbmin_octree,bbmax_octree,pos,last_pos;
int xmin,ymin,zmin,xmax,ymax,zmax,range_x,range_y,range_z,zmin_global,zmax_global;
float pos_yaw;
bool par_unknownAsOccupied,get_floor,got_map;
double par_maprad,par_maxobs,par_minobs,last_yaw,par_zjump;
nav_msgs::Path path_candidates,path_visited,path_candidates2d;
geometry_msgs::Vector3 vlp_rpy;
geometry_msgs::PointStamped pnt_midpoint;
geometry_msgs::PoseStamped last_pose;
float delta_hdng = M_PI/2;
float delta_incline = M_PI/24;

std_msgs::Header hdr(){
  std_msgs::Header header;
  header.frame_id = "map";
  header.stamp    = ros::Time::now();
  return header;
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

bool in_poly(geometry_msgs::PolygonStamped polyin, float x, float y){
	ros::Time t0 = ros::Time::now();
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
//	float dt = (ros::Time::now() - t0).toSec();
//	ROS_INFO("sr: %.5f",dt);
  return bool(cross % 2);
}
float get_dst2d(geometry_msgs::Point p1, geometry_msgs::Point p2){
  return(sqrt(pow(p1.x-p2.x,2)+pow(p1.y-p2.y,2)));
}
float get_dst3d(geometry_msgs::Point p1, geometry_msgs::Point p2){
  return(sqrt(pow(p1.x-p2.x,2)+pow(p1.y-p2.y,2)+pow(p1.z-p2.z,2)));
}

bool dst_point_in_path_lim(nav_msgs::Path pathin,geometry_msgs::Point pin,float lim){
  float res,dst;
  if(pathin.poses.size() == 0)
    return res;
  for(int i = 0; i < pathin.poses.size(); i++){
     if(get_dst3d(pathin.poses[i].pose.position,pin) < lim)
        return false;
  }
  return true;
}
bool dst_point_in_path_lim2d(nav_msgs::Path pathin,geometry_msgs::Point pin,float lim){
  float res,dst;
  if(pathin.poses.size() == 0)
    return res;
  for(int i = 0; i < pathin.poses.size(); i++){
     if(get_dst2d(pathin.poses[i].pose.position,pin) < lim)
        return false;
  }
  return true;
}
bool is_within_hdng_incline(geometry_msgs::Point pnt,float maxdelta_hdng, float maxdelta_pitch){
	float hdng_shortest = get_shortest(get_hdng(pnt,pos),vlp_rpy.z);
  if(hdng_shortest < 0)
    hdng_shortest *= -1;
  if(hdng_shortest < maxdelta_hdng){
    float pitch_shortest = get_shortest(atan2(pnt.z-pos.z,get_dst2d(pnt,pos)),vlp_rpy.y);
    if(pitch_shortest < 0)
      pitch_shortest *= -1;
    if(pitch_shortest < maxdelta_pitch)
      return true;
    else
      return false;
  }
  else
    return false;
}
nav_msgs::Path merge_paths(nav_msgs::Path path1,nav_msgs::Path path0){
  for(int i = 0; i < path1.poses.size(); i++){
    path0.poses.push_back(path1.poses[i]);
  }
  return path0;
}
bool update_edto(geometry_msgs::Point midpoint,float collision_radius,float maprad,float z0,float z1,bool unknownAsOccupied){
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

    edf_ptr.reset (new DynamicEDTOctomap(collision_radius,octree.get(),
            boundary_min,
            boundary_max,
            unknownAsOccupied));
    edf_ptr.get()->update();
    xmin    = int(round(boundary_min.x()))+1;  ymin    = int(round(boundary_min.y()))+1; zmin    = int(round(boundary_min.z()));
    xmax    = int(round(boundary_max.x()))-1;  ymax    = int(round(boundary_max.y()))-1; zmax    = int(round(boundary_max.z()));
    range_x = xmax - xmin;                     range_y = ymax - ymin;                    range_z = zmax - zmin;
    int vol = range_x*range_y*range_z;
    if(vol <= 0){
			ROS_INFO("FAILED update_edto FAILED: xmin: %i, ymin: %i, zmin: %i, xmax: %i, ymax: %i, zmax: %i, range_x: %i, range_y: %i, range_z: %i ",xmin,ymin,zmin,xmax,ymax,zmax,range_x,range_y,range_z);
			return false;
		}
  return true;
}

void octomap_callback(const octomap_msgs::Octomap& msg){
  abs_octree=octomap_msgs::fullMsgToMap(msg);
  octree.reset(dynamic_cast<octomap::OcTree*>(abs_octree));
  got_map = true;
}

nav_msgs::Path get_path_candidates(geometry_msgs::Point midpoint){
  geometry_msgs::PoseStamped ps;
  nav_msgs::Path pathout;
  ps.header = hdr();
  pathout.header = hdr();
  octree.get()->getMetricMin(bbmin_octree.x,bbmin_octree.y,bbmin_octree.z);
  octree.get()->getMetricMax(bbmax_octree.x,bbmax_octree.y,bbmax_octree.z);
  for(int z = int(round(bbmin_octree.z)); z < int(round(bbmax_octree.z)); z++){
    update_edto(midpoint,par_maxobs+1,par_maprad,z-0.5,z+0.5,par_unknownAsOccupied);
    for(int y = ymin; y < ymax; y++){
      for(int x = xmin; x < xmax; x++){
        ps.pose.position.x = x;
        ps.pose.position.y = y;
        ps.pose.position.z = z;
        if(is_within_hdng_incline(ps.pose.position,delta_hdng,delta_incline) && dst_point_in_path_lim(pathout,ps.pose.position,3)){
          octomap::point3d p(x,y,z);
          octomap::point3d closestObst;
          float d;
          edf_ptr.get()->getDistanceAndClosestObstacle(p,d,closestObst);
          if(round((par_minobs + par_maxobs)/2) == round(d)){
            ps.pose.orientation = tf::createQuaternionMsgFromYaw(atan2(closestObst.y() - y,closestObst.x() - x));
            pathout.poses.push_back(ps);
          }
        }
      }
    }
  }
  return pathout;
}

nav_msgs::Path get_path_floor(nav_msgs::Path pathin,float collision_radius){
  if(pathin.poses.size() == 0)
    return pathin;
  geometry_msgs::PoseStamped ps;
  nav_msgs::Path pathout,pathout_floor;
  ps.header = hdr();
  pathout.header = hdr();
  for(int i = 0; i < pathin.poses.size(); i++){
    float collision_radius_combined = pos.z-bbmin_octree.z + collision_radius;
    float d;
    update_edto(pathin.poses[i].pose.position,collision_radius_combined,collision_radius,bbmin_octree.z,pathin.poses[i].pose.position.z,par_unknownAsOccupied);
    octomap::point3d p(pathin.poses[i].pose.position.x,pathin.poses[i].pose.position.y,pathin.poses[i].pose.position.z);
    octomap::point3d closestObst;
    edf_ptr.get()->getDistanceAndClosestObstacle(p,d,closestObst);
    ps.pose.position = pathin.poses[i].pose.position;
    if(d > 0 && round(d) < round(collision_radius_combined) && closestObst.z() > ps.pose.position.z-2){
      ps.pose.position.z = closestObst.z() + 2;
      if(dst_point_in_path_lim(pathout,ps.pose.position,3))
        pathout.poses.push_back(ps);
      ps.pose.position.x = closestObst.x();
      ps.pose.position.y = closestObst.y();
      ps.pose.orientation.y = 0.7071;
      ps.pose.orientation.w = 0.7071;
      pathout_floor.poses.push_back(ps);
    }
    else
      pathout.poses.push_back(ps);
  }
  return merge_paths(pathout,pathout_floor);
}
void update_edto_path(nav_msgs::Path pathin,float collision_radius){
  if(pathin.poses.size() == 0)
    return;
  float x_max,y_max,z_max,x_min,y_min,z_min;
  x_max = -1000; y_max = -1000; z_max = -1000;
  x_min = 1000; y_min = 1000; z_min = 1000;
  for(int i = 0; i < pathin.poses.size(); i++){
    if(pathin.poses[i].pose.position.x > x_max)
      x_max = pathin.poses[i].pose.position.x;
    if(pathin.poses[i].pose.position.y > y_max)
      y_max = pathin.poses[i].pose.position.y;
    if(pathin.poses[i].pose.position.z > z_max)
      z_max = pathin.poses[i].pose.position.z;
    if(pathin.poses[i].pose.position.x < x_min)
      x_min = pathin.poses[i].pose.position.x;
    if(pathin.poses[i].pose.position.y < y_min)
      y_min = pathin.poses[i].pose.position.y;
    if(pathin.poses[i].pose.position.z < z_min)
      z_min = pathin.poses[i].pose.position.z;
  }
  float dx = x_max - x_min;
  float dy = y_max - y_min;
  float dz = z_max - z_min;
  geometry_msgs::Point midpoint;
  midpoint.x = (x_min + x_max)/2;
  midpoint.y = (y_min + y_max)/2;
  midpoint.z = (z_min + z_max)/2;
  float mapradtouse = fmax(dx,dy)/2;
  update_edto(midpoint,collision_radius,mapradtouse+collision_radius,z_min-10,z_max+10,false);
  ROS_INFO("EDTO update_edto: xmin: %i, ymin: %i, zmin: %i, xmax: %i, ymax: %i, zmax: %i, range_x: %i, range_y: %i, range_z: %i ",xmin,ymin,zmin,xmax,ymax,zmax,range_x,range_y,range_z);
}
void update_edto_pose(geometry_msgs::Point p1,geometry_msgs::Point p2,float collision_radius){
  float x_max = fmax(p1.x,p2.x);
  float y_max = fmax(p1.y,p2.y);
  float z_max = fmax(p1.z,p2.z) + collision_radius/2;
  float x_min = fmin(p1.x,p2.x);
  float y_min = fmin(p1.y,p2.y);
  float z_min = fmin(p1.z,p2.z) - collision_radius/2;
  float dx = x_max - x_min;
  float dy = y_max - y_min;
  float dz = z_max - z_min;
  geometry_msgs::Point midpoint;
  midpoint.x = (x_min + x_max)/2;
  midpoint.y = (y_min + y_max)/2;
  midpoint.z = (z_min + z_max)/2;
  float mapradtouse = fmax(dx,dy)/2 + collision_radius;
  update_edto(midpoint,collision_radius,mapradtouse,z_min,z_max,false);
}
nav_msgs::Path get_path_safe(nav_msgs::Path pathin,float radius){
  nav_msgs::Path pathout;
  pathout.header = hdr();
  if(pathin.poses.size() == 0)
    return pathout;
//  update_edto_path(pathin,radius);
  for(int i = 0; i < pathin.poses.size(); i++){
    //update_edto(pathin.poses[i].pose.position,collrad+radius,radius,bbmin_octree.z,pathin.poses[i].pose.position.z,par_unknownAsOccupied);
    update_edto_pose(pos,pathin.poses[i].pose.position,radius);
    Eigen::Vector3f pnt1_vec(pos.x,pos.y,pos.z);
    Eigen::Vector3f pnt2_vec(pathin.poses[i].pose.position.x,pathin.poses[i].pose.position.y,pathin.poses[i].pose.position.z);
    Eigen::Vector3f cur_vec = pnt1_vec;
    Eigen::Vector3f stride_vec;
    stride_vec = (pnt2_vec - pnt1_vec).normalized();
    float distance = radius;
    float tot_length = (pnt2_vec - pnt1_vec).norm();
    float cur_ray_len=0;
    float lowest_dist = 100;
    while(cur_ray_len < tot_length){
      cur_vec = cur_vec + stride_vec;
      cur_ray_len = (cur_vec-pnt1_vec).norm();
      point3d stridep(cur_vec.x(),cur_vec.y(),cur_vec.z());
      distance = edf_ptr.get()->getDistance(stridep);
      if(distance < lowest_dist)
        lowest_dist = distance;
    }
    if(lowest_dist > 2.5)
      pathout.poses.push_back(pathin.poses[i]);
  }
  return pathout;
}

geometry_msgs::PointStamped get_point_in_front(std::string frame,float dist_from_base){
  geometry_msgs::PointStamped pnt;
	try
	{
		pnt.header.frame_id		 = frame;
		pnt.point.x 			 	   = dist_from_base;
		pnt                    = tfBuffer.transform(pnt, "map");
	}
	catch (tf2::TransformException &ex)
	{
		ROS_WARN("Failure %s\n", ex.what());
	}
  ROS_INFO("Pos: [x: %.2f y: %.2f z: %.2f] ->front-> [x: %.2f y: %.2f z: %.2f]",pos.x,pos.y,pos.z,pnt.point.x,pnt.point.y,pnt.point.z);
  return pnt;
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
	pos_yaw = tf::getYaw(transformStamped.transform.rotation);
  if(sqrt(pow(transformStamped.transform.translation.x - last_pose.pose.position.x,2)+ pow(transformStamped.transform.translation.y - last_pose.pose.position.y,2)+
    pow(transformStamped.transform.translation.z - last_pose.pose.position.z,2)) >= 1.00){
    last_pose.pose.position    = pos;
    last_pose.pose.orientation = transformStamped.transform.rotation;
    last_pose.header           = transformStamped.header;
    path_visited.poses.push_back(last_pose);
    path_visited.header.stamp  = ros::Time::now();
  }
  if(!get_floor && (abs(last_yaw - pos_yaw) > 1 || get_dst3d(pos,last_pos) > 5)){
    get_floor = true;
  }
}
void checktfvlp(){
  geometry_msgs::TransformStamped transformStamped;
  try{
    transformStamped = tfBuffer.lookupTransform("map","velodyne_aligned",
                             ros::Time(0));
  }
  catch (tf2::TransformException &ex) {
    ROS_WARN("%s",ex.what());
    ros::Duration(1.0).sleep();
  }
	tf2::Matrix3x3 q(tf2::Quaternion(transformStamped.transform.rotation.x, transformStamped.transform.rotation.y, transformStamped.transform.rotation.z, transformStamped.transform.rotation.w));
	q.getRPY(vlp_rpy.x,vlp_rpy.y,vlp_rpy.z);
}
int get_nans(nav_msgs::Path pathin){
  int nancount = 0;
  for(int i = 0; i < pathin.poses.size(); i++){
    if(std::isnan(pathin.poses[i].pose.position.x)
    || std::isnan(pathin.poses[i].pose.position.y)
    || std::isnan(pathin.poses[i].pose.position.z)){
      nancount++;
    }
   }
   return nancount;
}
int main(int argc, char **argv)
{
  ros::init(argc, argv, "tb_pathmini_node");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");
  private_nh.param("workdir_path",par_workdir_path);//*2.0);
  private_nh.param("par_maprad",  par_maprad, 30.0);//*2.0);
  private_nh.param("par_maxobs",  par_maxobs, 20.0);//*2.0);
  private_nh.param("par_zjump",   par_zjump, 3.0);//*2.0);
  private_nh.param("par_minobs",  par_minobs, 2.0);//*2.0);
  nav_msgs::Path path_candidates_sides,path_candidates_full,path_candidates_safe;
  path_candidates_sides.header   = hdr();
  path_candidates_full.header    = hdr();
  path_candidates_safe.header    = hdr();
  path_candidates.header         = hdr();
  path_visited.header            = hdr();

  last_pose.pose.orientation.w = 1;
  path_visited.poses.push_back(last_pose);

  tf2_ros::TransformListener tf2_listener(tfBuffer);

  ros::Publisher pub_candidates = nh.advertise<nav_msgs::Path>("/tb_path/candidates",100);
  ros::Publisher pub_visited   	= nh.advertise<nav_msgs::Path>("/tb_path/visited",10);

  ros::Publisher pub_midpoint         = nh.advertise<geometry_msgs::PointStamped>("/tb_path/midpoint",100);
//  ros::Publisher pub_path             = nh.advertise<nav_msgs::Path>("/tb_path",100);

  ros::Subscriber s1 = nh.subscribe("/octomap_full",1,octomap_callback);
  ROS_INFO("Ready to convert octomaps.");
  ros::Rate rate(2.0);
  ros::Time t0;
  ros::Time last_update = ros::Time::now();
  float collision_radius = 5;
  while(ros::ok()){
    checktf();
    checktfvlp();
    if(got_map){
      if((ros::Time::now()-last_update).toSec() > 5)
        get_floor = true;
      if(get_floor){
        float dt_path_candidates,dt_path_floor,dt_path_edto,dt_path_safe;
        last_update = ros::Time::now();
        t0          = ros::Time::now();

        pnt_midpoint = get_point_in_front("velodyne_aligned",15);
        path_candidates_sides = get_path_candidates(pnt_midpoint.point);
    /*    dt_path_candidates = (ros::Time::now() - t0).toSec();
        t0           = ros::Time::now();
        path_candidates_full = get_path_floor(path_candidates_sides,collision_radius);
        dt_path_floor = (ros::Time::now() - t0).toSec();
        t0             = ros::Time::now();

        update_edto_path(path_candidates_full,collision_radius);
        path_candidates_safe = get_path_safe(path_candidates_full,collision_radius);
        dt_path_safe     = (ros::Time::now() - t0).toSec();
        last_update = ros::Time::now();*/
        ROS_INFO("path_candidates: %i -> floor: %i -> safe: %i",path_candidates_sides.poses.size(),path_candidates_full.poses.size(),path_candidates_safe.poses.size());

        ROS_INFO("TIMER: dt_path_candidates: %.4f, dt_path_floor: %.4f, dt_path_edto: %.4f, dt_path_safe: %.4f",dt_path_candidates,dt_path_floor,dt_path_edto,dt_path_safe);
        int nans_full  = get_nans(path_candidates_sides);
        int nans_sides   = get_nans(path_candidates_full);
        int nans_safe  = get_nans(path_candidates_safe);
        int nans_visited = get_nans(path_visited);
        ROS_INFO("nans_full: %i",nans_full);
        ROS_INFO("nans_sides: %i",nans_sides);
        ROS_INFO("nans_safe: %i",nans_safe);
        ROS_INFO("nans_vstd: %i",nans_visited);

        pub_candidates.publish(path_candidates_sides);
        get_floor = false;
      }
      pub_midpoint.publish(pnt_midpoint);
      pub_visited.publish(path_visited);
    }
	  rate.sleep();
	  ros::spinOnce();
  }
  return 0;
}
