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
int zlvl;
bool par_unknownAsOccupied,get_floor,got_map;
double par_maprad,par_maxobs,par_minobs,last_yaw,par_zjump;
nav_msgs::Path path_zdown,path_candidates,path_floor,path_floor_grid,path_vlp;
std::vector<int> z_lvls;
geometry_msgs::Vector3 vlp_rpy;
geometry_msgs::PointStamped pnt_midpoint;
geometry_msgs::PointStamped closest_obstacle;
geometry_msgs::PoseStamped last_pose;
geometry_msgs::PolygonStamped poly_heading,poly_safe,poly_vstd,poly_vlp;
nav_msgs::Path path_visited,path;
nav_msgs::Path path_mid,path_hi,path_lo;
float delta_hdng = M_PI/2;
float delta_incline = M_PI/24;
cv::Mat mapimg(1000,1000,CV_8UC3,cv::Scalar(0, 0, 0)); //create image, set encoding and size, init pixels to default val
cv::Mat mapimg_mono(1000,1000,CV_8U,cv::Scalar(0)); //create image, set encoding and size, init pixels to default val
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
std_msgs::Header hdr(){
  std_msgs::Header header;
  header.frame_id = "map";
  header.stamp    = ros::Time::now();
  return header;
}
geometry_msgs::PointStamped transformpoint(geometry_msgs::PointStamped pin,std::string frame_out){
  geometry_msgs::PointStamped pout;
  pin.header.stamp = ros::Time();
  try
  {
    pout = tfBuffer.transform(pin, frame_out);
  }
  catch (tf2::TransformException &ex)
  {
        ROS_WARN("Failure %s\n", ex.what());
  }
  return pout;
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


float get_dst2d(geometry_msgs::Point p1, geometry_msgs::Point p2){
  return(sqrt(pow(p1.x-p2.x,2)+pow(p1.y-p2.y,2)));
}
float get_dst3d(geometry_msgs::Point p1, geometry_msgs::Point p2){
  return(sqrt(pow(p1.x-p2.x,2)+pow(p1.y-p2.y,2)+pow(p1.z-p2.z,2)));
}


float getinpath_closestdst(nav_msgs::Path pathin,geometry_msgs::PoseStamped pose_to_check,bool use_3d){
  float lowest_dist = 1000;  float dst;
  if(pathin.poses.size() == 0){
    return lowest_dist;
  }
  for(int i = 0; i < pathin.poses.size(); i++){
    if(use_3d)
      dst = get_dst3d(pathin.poses[i].pose.position,pose_to_check.pose.position);
    else
      dst = get_dst2d(pathin.poses[i].pose.position,pose_to_check.pose.position);
    if(dst < lowest_dist)
      lowest_dist   = dst;
  }
  return lowest_dist;
}

/******** START ******************** CONSTRAIN PATH FUNCTIONS **************** START ******************/
nav_msgs::Path constrain_path_bbpnts(nav_msgs::Path pathin,geometry_msgs::Point bbmin,geometry_msgs::Point bbmax){
  nav_msgs::Path pathout;
  pathout.header = hdr();
  if(pathin.poses.size() == 0)
    return pathout;
  for(int i = 0; i < pathin.poses.size(); i++){
    if(bbmin.x < pathin.poses[i].pose.position.x && pathin.poses[i].pose.position.x < bbmax.x
    && bbmin.y < pathin.poses[i].pose.position.y && pathin.poses[i].pose.position.y < bbmax.y
    && bbmin.z < pathin.poses[i].pose.position.z && pathin.poses[i].pose.position.z < bbmax.z)
      pathout.poses.push_back(pathin.poses[i]);
  }
  return pathout;
}
nav_msgs::Path constrain_path_xyz(nav_msgs::Path pathin, float dx, float dy, float z_lo, float z_hi){
  nav_msgs::Path pathout;
  pathout.header.frame_id="map";
  if(pathin.poses.size() == 0)
    return pathout;
  geometry_msgs::Point bbmin,bbmax;
  bbmin.x = pos.x-dx; bbmin.y = pos.y-dy; bbmin.z = z_lo;
  bbmax.x = pos.x+dx; bbmax.y = pos.y+dy; bbmax.z = z_hi;

  pathout = constrain_path_bbpnts(pathin,bbmin,bbmax);
  return pathout;
}
nav_msgs::Path constrain_path_bbpoly(nav_msgs::Path pathin,geometry_msgs::PolygonStamped poly_bb){
  nav_msgs::Path pathout;
  pathout.header = hdr();
  if(pathin.poses.size() == 0 || poly_bb.polygon.points.size() == 0)
    return pathin;
  for(int i = 0; i < pathin.poses.size(); i++){
    if(in_poly(poly_bb,pathin.poses[i].pose.position.x,pathin.poses[i].pose.position.y)){
      pathout.poses.push_back(pathin.poses[i]);
    }
  }
  return pathout;
}
nav_msgs::Path constrain_path_vstd(nav_msgs::Path pathin,nav_msgs::Path pathin_vstd,float cutoff_dst,bool use_3d){
  nav_msgs::Path pathout;
  pathout.header = hdr();
  if(fmin(pathin.poses.size(),pathin_vstd.poses.size()) == 0){
    return pathin;
  }
  for(int i = 0; i < pathin.poses.size(); i++){
    if(getinpath_closestdst(pathin_vstd,pathin.poses[i],use_3d) > cutoff_dst)
      pathout.poses.push_back(pathin.poses[i]);
  }
  return pathout;
}
nav_msgs::Path constrain_path_close(nav_msgs::Path pathin,nav_msgs::Path pathin_vstd,float cutoff_dst,bool use_3d){
  nav_msgs::Path pathout;
  pathout.header = hdr();
  if(fmin(pathin.poses.size(),pathin_vstd.poses.size()) == 0){
    return pathin;
  }
  for(int i = 0; i < pathin.poses.size(); i++){
    if(getinpath_closestdst(pathin_vstd,pathin.poses[i],use_3d) < cutoff_dst)
      pathout.poses.push_back(pathin.poses[i]);
  }
  return pathout;
}
float dst_visited_area(float x,float y, float z){
  float res,dst;
  geometry_msgs::Point pin;
  pin.x = x; pin.y = y; pin.z = z;
  res = 1000;
  if(path_visited.poses.size() == 0)
    return res;
  for(int i = 0; i < path_visited.poses.size(); i++){
    if(abs(path_visited.poses[i].pose.position.z - pin.z) < 2){
      dst = get_dst3d(path_visited.poses[i].pose.position,pin);
      if(dst < res)
        res = dst;
    }
  }
  return res;
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

void update_path(geometry_msgs::Point midpoint){
  geometry_msgs::PoseStamped ps;
  last_pos = pos;
  last_yaw = pos_yaw;
  ps.header.frame_id ="map";
  path_candidates.poses.resize(0);
  octree.get()->getMetricMin(bbmin_octree.x,bbmin_octree.y,bbmin_octree.z);
  octree.get()->getMetricMax(bbmax_octree.x,bbmax_octree.y,bbmax_octree.z);
  int min_zlvl = fmax(bbmin_octree.z/par_zjump + 3,zlvl-5);
  int max_zlvl = fmin(bbmax_octree.z/par_zjump + 3,zlvl+5);
  if(min_zlvl == max_zlvl)
    return;
  for(int zn = min_zlvl; zn < max_zlvl; zn++){
    int z = z_lvls[zn];
    update_edto(midpoint,par_maxobs+1,par_maprad,z-0.5,z+0.5,par_unknownAsOccupied);
      for(int y = ymin; y < ymax; y++){
        for(int x = xmin; x < xmax; x++){
          ps.pose.position.x = x;
          ps.pose.position.y = y;
          ps.pose.position.z = z;
          if(is_within_hdng_incline(ps.pose.position,delta_hdng,delta_incline) && dst_point_in_path_lim(path_candidates,ps.pose.position,3)){
            octomap::point3d p(x,y,z);
            octomap::point3d closestObst;
            float d;
            edf_ptr.get()->getDistanceAndClosestObstacle(p,d,closestObst);
            //if(round((par_minobs + par_maxobs)/2) == round(d) && in_poly(poly_safe,ps.pose.position.x,ps.pose.position.y)){
            if(round((par_minobs + par_maxobs)/2) == round(d)){
              ps.pose.orientation = tf::createQuaternionMsgFromYaw(atan2(closestObst.y() - y,closestObst.x() - x));
              path_candidates.poses.push_back(ps);
            }
          }
        }
      }
    }
  return;
}

void update_path_floor(geometry_msgs::Point midpoint,int n_grids, float grid_sidelength){
  geometry_msgs::PoseStamped ps;
  ps.header.frame_id ="map";
  path_floor.poses.resize(0);
  path_floor_grid.poses.resize(0);

  octree.get()->getMetricMin(bbmin_octree.x,bbmin_octree.y,bbmin_octree.z);
  octree.get()->getMetricMax(bbmax_octree.x,bbmax_octree.y,bbmax_octree.z);

  int x0 = int(round(midpoint.x)) - grid_sidelength * n_grids / 2 + grid_sidelength / 2;
  int y0 = int(round(midpoint.y)) - grid_sidelength * n_grids / 2 + grid_sidelength / 2;

  ROS_INFO("PathFloor: x0: %i y0: %i grid_Sides/Number %.2f/%i",x0,y0,grid_sidelength,n_grids);

  for(int yn = 0; yn < n_grids; yn++){
    for(int xn = 0; xn < n_grids; xn++){
      geometry_msgs::Point pmid;
      pmid.x = x0 + xn * grid_sidelength;
      pmid.y = y0 + yn * grid_sidelength;
      pmid.z = midpoint.z + par_zjump;
      if(in_poly(poly_safe,pmid.x,pmid.y)){
        //ROS_INFO("PathFloor: xn: %i yn: %i x: %.2f y: %.2f",xn,yn,pmid.x,pmid.y);
        float collrad = pmid.z-bbmin_octree.z;
        update_edto(pmid,collrad,grid_sidelength,bbmin_octree.z,pmid.z,par_unknownAsOccupied);
        float d;
        octomap::point3d p(pmid.x,pmid.y,pmid.z);
        octomap::point3d closestObst;
        edf_ptr.get()->getDistanceAndClosestObstacle(p,d,closestObst);
        //if(round(d) < round(collrad) && ){
        if(round(d) < round(collrad)){
          ps.pose.position.x    = closestObst.x();
          ps.pose.position.y    = closestObst.y();
          ps.pose.position.z    = closestObst.z() + 5;
          ps.pose.orientation.y = 0.7071;
          ps.pose.orientation.w = 0.7071;
          path_floor.poses.push_back(ps);
          ps.pose.position.x    = pmid.x;
          ps.pose.position.y    = pmid.y;
          ps.pose.position.z    = closestObst.z();
          ps.pose.orientation.y = 0.7071;
          ps.pose.orientation.w = 0.7071;
          path_floor_grid.poses.push_back(ps);
        }
      }
    }
  }
  return;
}
geometry_msgs::PolygonStamped test_poly_vlp(float collision_radius,int num_rays,float tot_length){
  geometry_msgs::PoseStamped ps;
  geometry_msgs::PolygonStamped poly;
  ps.header = hdr();
  path_vlp.header = hdr();
  poly.header = hdr();
  path_vlp.poses.resize(0);
  poly.polygon.points.resize(num_rays);
  poly.polygon.points[0].x          = pos.x;
  poly.polygon.points[0].y          = pos.y;
  poly.polygon.points[0].z          = pos.z;
  float rads_pr_i = M_PI / num_rays;
  float x_max,y_max,z_max,x_min,y_min,z_min;
  x_max = -1000; y_max = -1000; z_max = -1000;
  x_min = 1000; y_min = 1000; z_min = 1000;
  std::vector<geometry_msgs::Point> points;
  for(int i = 1; i < num_rays; i++){
    geometry_msgs::PointStamped p,pout;
    p.header.frame_id = "velodyne_aligned";
    float a = -M_PI/2 + rads_pr_i * i;
    p.point.x = tot_length*cos(a);
    p.point.y = tot_length*sin(a);
    p.point.z = 0;
    p.header.stamp = ros::Time();
    pout = transformpoint(p,"map");
    if(pout.point.x > x_max)
      x_max = pout.point.x;
    if(pout.point.y > y_max)
      y_max = pout.point.y;
    if(pout.point.z > z_max)
      z_max = pout.point.z;
    if(pout.point.x < x_min)
      x_min = pout.point.x;
    if(pout.point.y < y_min)
      y_min = pout.point.y;
    if(pout.point.z < z_min)
      z_min = pout.point.z;
    points.push_back(pout.point);
  }
  float dx = x_max - x_min;
  float dy = y_max - y_min;
  float dz = z_max - z_min;
  geometry_msgs::Point midpoint;
  midpoint.x = (x_min + x_max)/2;
  midpoint.y = (y_min + y_max)/2;
  midpoint.z = (z_min + z_max)/2;
  float mapradtouse = fmax(dx,dy)/2;
//  ROS_INFO("POLYVLP: points: %i dx: %.0f dy: %.0f dz: %.0f, midpoint: x: %.0f y: %.0f z: %.0f",points.size(),dx,dy,dz,midpoint.x,midpoint.y,midpoint.z);
    update_edto(midpoint,collision_radius,mapradtouse+collision_radius,z_min-1,z_max+1,false);
  for(int i = 0; i < points.size(); i++){
    Eigen::Vector3f pnt1_vec(pos.x,pos.y,pos.z);
    Eigen::Vector3f pnt2_vec(points[i].x,points[i].y,points[i].z);
    Eigen::Vector3f stride_vec,cur_vec;
    stride_vec = (pnt2_vec - pnt1_vec).normalized() * 1;
    float distance = collision_radius-1;
    float cur_ray_len = 0;
    cur_vec = pnt1_vec;
    while(distance > 2 && cur_ray_len < tot_length){
      cur_vec = cur_vec + stride_vec;
      ps.pose.position.x  = cur_vec.x();
      ps.pose.position.y  = cur_vec.y();
      ps.pose.position.z  = cur_vec.z();
      if(dst_point_in_path_lim(path_vlp,ps.pose.position,3)){

        cur_ray_len = (cur_vec - pnt1_vec).norm();
        octomap::point3d closestObst;
        point3d p(cur_vec.x(),cur_vec.y(),cur_vec.z());
        edf_ptr.get()->getDistanceAndClosestObstacle(p,distance,closestObst);

        float yaw   = atan2(closestObst.y() - cur_vec.y(),closestObst.x() - cur_vec.x());
        float pitch = atan2(cur_vec.z() - closestObst.z(),cur_ray_len);
        ps.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0,pitch,yaw);
        path_vlp.poses.push_back(ps);
    //  ROS_INFO("Distance %.2f,p: x: %.0f y: %.0f z: %.0f",distance,p.x(),p.y(),p.z());
    /*    if(round((par_minobs + par_maxobs)/2) == round(distance)){//3 == round(distance)){
          float yaw   = atan2(closestObst.y() - cur_vec.y(),closestObst.x() - cur_vec.x());
          float pitch = atan2(cur_vec.z() - closestObst.z(),cur_ray_len);
          ps.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0,pitch,yaw);
          path_vlp.poses.push_back(ps);
        }*/
      }
    }
    poly.polygon.points[i].x = cur_vec.x();
    poly.polygon.points[i].y = cur_vec.y();
    poly.polygon.points[i].z = cur_vec.z();
  }
  poly.polygon.points[num_rays-1].x   = pos.x;
  poly.polygon.points[num_rays-1].y   = pos.y;
  poly.polygon.points[num_rays-1].z   = pos.z;
  return poly;
}
/*

void get_zdown(float collision_radius){
  for(int y = ymin; y < ymax; y++){
    for(int x = xmin; x < xmax; x++){
			int z = zmax;
			float dst = collision_radius;
				octomap::point3d p(x,y,z);
				dst = edf_ptr.get()->getDistance(p);
				zn--;
			}
			int r = y2r(y,mapimg.rows,1);
			int c = x2c(x,mapimg.cols,1);
			mapimg.at<cv::Vec3b>(r,c)[2] = z;
  	}
	}
	ROS_INFO("get_zdown complete");
}*/

geometry_msgs::PointStamped get_point_in_front(float dist_from_base){
  geometry_msgs::PointStamped pnt;
	try
	{
		pnt.header.frame_id		 = "base_stabilized";
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
  zlvl  = round(pos.z / par_zjump) + 3;
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
geometry_msgs::PolygonStamped get_poly_surround(float collision_radius,float mapradused,int num_rays){
  geometry_msgs::PolygonStamped polygon;
  polygon.header = hdr();
  update_edto(pos,collision_radius,mapradused,pos.z-1,pos.z+1,false);
  octomap::point3d p(pos.x,pos.y,pos.z);
  octomap::point3d closestObst;
  float d;
  edf_ptr.get()->getDistanceAndClosestObstacle(p,d,closestObst);
  if(d < collision_radius && d > 0){
    closest_obstacle.point.x      = closestObst.x();
    closest_obstacle.point.y      = closestObst.y();
    closest_obstacle.point.z      = closestObst.z();
    closest_obstacle.header.stamp = ros::Time::now();
  }
  float rads_pr_i = 2*M_PI / num_rays;
  polygon.polygon.points.resize(num_rays);
  for(int i  = 0; i < num_rays; i++){
    Eigen::Vector3f pnt1_vec(pos.x,pos.y,pos.z);
    Eigen::Vector3f pnt2_vec(pos.x+mapradused*cos(i*rads_pr_i),mapradused*sin(i*rads_pr_i)+pos.y,pos.z);
    Eigen::Vector3f cur_vec = pnt1_vec;
    float tot_length = (pnt2_vec - pnt1_vec).norm();
    Eigen::Vector3f stride_vec;
    stride_vec = (pnt2_vec - pnt1_vec).normalized() * 1;
    bool visited_clear = true;
    float cur_ray_len=0;
    float distance = collision_radius-1;
    float next_check = collision_radius;
    while(distance > 2 && cur_ray_len < tot_length){
      cur_vec = cur_vec + stride_vec;
      cur_ray_len = (cur_vec-pnt1_vec).norm();
      point3d stridep(cur_vec.x(),cur_vec.y(),cur_vec.z());
      distance = edf_ptr.get()->getDistance(stridep);
    }
    polygon.polygon.points[i].x = cur_vec.x();
    polygon.polygon.points[i].y = cur_vec.y();
    polygon.polygon.points[i].z = cur_vec.z();
  }
  return polygon;
}
geometry_msgs::PolygonStamped get_poly_vstd(float collision_radius,float mapradused,int num_rays){
  geometry_msgs::PolygonStamped polygon;
  polygon.header = hdr();
  float rads_pr_i = 2*M_PI / num_rays;
  polygon.polygon.points.resize(num_rays);
  for(int i  = 0; i < num_rays; i++){
    Eigen::Vector3f pnt1_vec(pos.x,pos.y,pos.z);
    Eigen::Vector3f pnt2_vec(pos.x+mapradused*cos(i*rads_pr_i),mapradused*sin(i*rads_pr_i)+pos.y,pos.z);
    Eigen::Vector3f cur_vec = pnt1_vec;
    float tot_length = (pnt2_vec - pnt1_vec).norm();
    Eigen::Vector3f stride_vec;
    stride_vec = (pnt2_vec - pnt1_vec).normalized() * 1;
    bool visited_clear = true;
    float cur_ray_len=0;
    float distance = 5;
    float next_check = collision_radius;
    while(distance > 2 && cur_ray_len < tot_length){
      cur_vec = cur_vec + stride_vec;
      cur_ray_len = (cur_vec-pnt1_vec).norm();
      point3d stridep(cur_vec.x(),cur_vec.y(),cur_vec.z());
      if(cur_ray_len >= next_check){
       next_check = cur_ray_len + dst_visited_area(cur_vec.x(),cur_vec.y(),cur_vec.z());
       if(dst_visited_area(cur_vec.x(),cur_vec.y(),cur_vec.z()) < 2)
         break;
     }
    }
    polygon.polygon.points[i].x = cur_vec.x();
    polygon.polygon.points[i].y = cur_vec.y();
    polygon.polygon.points[i].z = cur_vec.z();
  }
  return polygon;
}
geometry_msgs::PolygonStamped get_poly_heading(float hdng_delta,int num_points,std::string frame){
  geometry_msgs::PolygonStamped poly;
  poly.header = hdr();
  poly.polygon.points.resize(num_points+1);
  poly.polygon.points[0].x = pos.x;
  poly.polygon.points[0].y = pos.y;
  poly.polygon.points[0].z = pos.z;
  float rads_pr_i = hdng_delta /num_points;
  geometry_msgs::PointStamped p,pout;
  p.header.frame_id = frame;
  p.header.stamp = ros::Time();
  for(int i = 1; i < num_points; i++){
    float a = -hdng_delta/2 + rads_pr_i * i;
    p.point.x = 50*cos(a);
    p.point.y = 50*sin(a);
    p.point.z = pos.z;
    pout = transformpoint(p,"map");
    poly.polygon.points[i].x = pout.point.x;
    poly.polygon.points[i].y = pout.point.y;
    poly.polygon.points[i].z = pos.z;
  }
  poly.polygon.points[num_points].x = pos.x;
  poly.polygon.points[num_points].y = pos.y;
  poly.polygon.points[num_points].z = pos.z;
  return poly;
}


nav_msgs::Path merge_paths(nav_msgs::Path path1,nav_msgs::Path path0){
  for(int i = 0; i < path1.poses.size(); i++){
    path0.poses.push_back(path1.poses[i]);
  }
  return path0;
}

nav_msgs::Path get_paths(nav_msgs::Path pathin_floor,nav_msgs::Path pathin_cand,float dxy,float z0_lo,float z0_hi,float z1_lo,float z1_hi){
  //ROS_INFO("PATHUTIL: paths_in: c: %i f: %i interval#1: [%.0f->%.0f] interval#2: [%.0f->%.0f]",pathin_floor.poses.size(),pathin_cand.poses.size(),z0_lo,z0_hi,z1_lo,z1_hi);
  nav_msgs::Path pathout,pathout_hdng;
  pathout.header      = hdr();
  pathout_hdng.header = hdr();
  pathout = constrain_path_xyz(pathin_cand,dxy,dxy,pos.z+z0_lo,pos.z+z0_hi);
  //ROS_INFO("PATHUTIL(cand): interval#1 [%.0f->%.0f] gives %i poses of %i original",pos.z+z0_lo,pos.z+z0_hi,pathout.poses.size(),pathin_cand.poses.size());
  if(pathout.poses.size() == 0){
    pathout = constrain_path_xyz(pathin_cand,dxy,dxy,pos.z+z1_lo,pos.z+z1_hi);
  //  ROS_INFO("PATHUTIL(cand): interval#2 [%.0f->%.0f] gives %i poses of %i original",pos.z+z1_lo,pos.z+z1_hi,pathout.poses.size(),pathin_cand.poses.size());
  }
  if(pathout.poses.size() == 0){
    pathout = constrain_path_xyz(pathin_floor,dxy,dxy,pos.z+z0_lo,pos.z+z0_hi);
  //  ROS_INFO("PATHUTIL(floor): interval#1 [%.0f->%.0f] gives %i poses of %i original",pos.z+z0_lo,pos.z+z0_hi,pathout.poses.size(),pathin_floor.poses.size());
  }
  if(pathout.poses.size() == 0){
    pathout = constrain_path_xyz(pathin_floor,dxy,dxy,pos.z+z1_lo,pos.z+z1_hi);
  //  ROS_INFO("PATHUTIL(floor): interval#2 [%.0f->%.0f] gives %i poses of %i original",pos.z+z1_lo,pos.z+z1_hi,pathout.poses.size(),pathin_floor.poses.size());
  }
  if(pathout.poses.size() > 0){
    pathout_hdng = constrain_path_bbpoly(pathout,poly_heading);
  //  ROS_INFO("PATHUTIL(hdng): %i poses of %i original",pathout_hdng.poses.size(),pathout.poses.size());
  }
  if(pathout_hdng.poses.size() > 0){
    return pathout_hdng;
  }
  else {
    return pathout;
  }
}
nav_msgs::Path get_path(nav_msgs::Path pathin,float dxy,float z0_lo,float z0_hi,float z1_lo,float z1_hi){
  nav_msgs::Path pathout,pathout_hdng;
  pathout.header      = hdr();
  pathout_hdng.header = hdr();
  pathout = constrain_path_xyz(pathin,dxy,dxy,pos.z+z0_lo,pos.z+z0_hi);
  if(pathout.poses.size() == 0)
    pathout = constrain_path_xyz(pathin,dxy,dxy,pos.z+z1_lo,pos.z+z1_hi);
  if(pathout.poses.size() > 0)
    pathout_hdng = constrain_path_bbpoly(pathout,poly_heading);
  if(pathout_hdng.poses.size() > 0)
    return pathout_hdng;
  else
    return pathout;
}

void update_polygons(){
  poly_heading     = get_poly_heading(M_PI/2,16,"base_stabilized");
  poly_safe        = get_poly_surround(10,50,36);
  poly_vstd        = get_poly_vstd(10,50,36);
}
void get_lo_mid_hi_paths(){
  path_candidates = constrain_path_vstd(path_candidates,path_visited,4,true);

  //path_candidates = constrain_path_vstd(constrain_path_bbpoly(path_candidates,poly_vlp),path_visited,4,true);
  path_floor      = constrain_path_vstd(constrain_path_bbpoly(path_floor_grid,poly_vlp),path_visited,4,true);
  //path_vlp        = constrain_path_vstd(path_vlp,path_visited,4,true);
  //path_candidates = constrain_path_close(path_candidates,path_vlp,4,true);


  path_lo         = get_paths(path_floor,path_candidates,20,-5,-1,-20,-1);
  path_mid        = get_paths(path_floor,path_candidates,20,-3,3,-6,6);
  path_hi         = get_paths(path_floor,path_candidates,20,1,5,1,20);

  ROS_INFO("PATHUTIL: Update paths [lo: %i, mid: %i, hi: %i, vlp: %i]",path_lo.poses.size(),path_mid.poses.size(),path_hi.poses.size(),path_vlp.poses.size());
}
void get_candid_floor_paths(){
  int n_grids           = 10;
  float grid_sidelength = par_maprad*2/n_grids;
  ROS_INFO("Par_maprad = grids*sidelength/2: %.2f = %.2f,grids_sidelength: %.2f, n_grids: %i, ",par_maprad,grid_sidelength*n_grids,grid_sidelength,n_grids);
  pnt_midpoint = get_point_in_front(15);
  update_path(pnt_midpoint.point);
  update_path_floor(pnt_midpoint.point,n_grids,grid_sidelength);
}
void get_zdown(){
  path_zdown.poses.resize(0);
  update_edto(pnt_midpoint.point,1.1,par_maprad,bbmin_octree.z+1,bbmax_octree.z-1,false);
  for(int y = ymin; y < ymax; y++){
    for(int x = xmin; x < xmax; x++){
      for(int z = zmin; x < zmax; x++){
        geometry_msgs::PoseStamped ps;
        ps.pose.position.x  = x;
        ps.pose.position.y  = y;
        ps.pose.position.z  = z;
        if(is_within_hdng_incline(ps.pose.position,delta_hdng,delta_incline)){
    			int r = y2r(y,mapimg.rows,1);
    			int c = x2c(x,mapimg.cols,1);
          octomap::point3d closestObst;
          float z_dist;
          octomap::point3d p(x,y,z);
          edf_ptr.get()->getDistanceAndClosestObstacle(p, z_dist, closestObst);
          if(round(z_dist) == 1 && z_dist < 1.1){

            float hdng_obs = atan2(closestObst.y() - p.y(),closestObst.x() - p.x());
            int cobs = x2c(closestObst.x(),1000,1);
            int robs = y2r(closestObst.y(),1000,1);
            if(mapimg_mono.at<uchar>(robs,cobs) < z)
              mapimg_mono.at<uchar>(robs,cobs) = z;
            if(mapimg.at<cv::Vec3b>(robs,cobs)[1] == 0)
              mapimg.at<cv::Vec3b>(robs,cobs)[1] = z;
            else{
              mapimg.at<cv::Vec3b>(robs,cobs)[0] += 10;
              mapimg.at<cv::Vec3b>(robs,cobs)[2] = z;
            }
            if(closestObst.z() < z)//{
              ps.pose.orientation.y = ps.pose.orientation.w = 0.7071;
            else
              ps.pose.orientation = tf::createQuaternionMsgFromYaw(hdng_obs);
      //if(dst_visited_area(ps.pose.position.x,ps.pose.position.y,ps.pose.position.z) > 5){
      //  vt.push_back(std::make_tuple(path.poses.size(),atan2(y,x),z,sqrt(pow(x-xpos,2)+pow(y-ypos,2))));
            path_zdown.poses.push_back(ps);
          }
        }
    	}
    }
  }
}
int main(int argc, char **argv)
{
  ros::init(argc, argv, "tb_path_node");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");
  private_nh.param("workdir_path",par_workdir_path);//*2.0);
  private_nh.param("par_maprad",  par_maprad, 30.0);//*2.0);
  private_nh.param("par_maxobs",  par_maxobs, 20.0);//*2.0);
  private_nh.param("par_zjump",   par_zjump, 3.0);//*2.0);
  private_nh.param("par_minobs",  par_minobs, 2.0);//*2.0);

  path_candidates.header  = hdr();
  path_floor.header       = hdr();
  last_pose.header        = hdr();
  path_mid.header         = hdr();
  path_hi.header          = hdr();
  path_lo.header          = hdr();
  path_visited.header     = hdr();
  closest_obstacle.header = hdr();
  path_floor_grid.header  = hdr();
  path_zdown.header       = hdr();
  last_pose.pose.orientation.w = 1;
  path_visited.poses.push_back(last_pose);

  for(int i = 0; i < 20; i++){
    z_lvls.push_back(int(round(-par_zjump*3 + par_zjump*i)));
  }

  tf2_ros::TransformListener tf2_listener(tfBuffer);


  ros::Publisher polyvlp_pub 		 	  	= nh.advertise<geometry_msgs::PolygonStamped>("/tb_obs/poly_vlp",100);
  ros::Publisher polysafe_pub 				= nh.advertise<geometry_msgs::PolygonStamped>("/tb_obs/poly_safe",100);
  ros::Publisher polyhdng_pub 				= nh.advertise<geometry_msgs::PolygonStamped>("/tb_obs/poly_hdng",100);
  ros::Publisher polyvstd_pub 				= nh.advertise<geometry_msgs::PolygonStamped>("/tb_obs/poly_vstd",100);
  ros::Publisher pub_closest_obstacle = nh.advertise<geometry_msgs::PointStamped>("/tb_obs/closest",100);

  ros::Publisher pub_orig             = nh.advertise<nav_msgs::Path>("/tb_path/candidates",100);
  ros::Publisher pub_floor            = nh.advertise<nav_msgs::Path>("/tb_path/floor",100);
  ros::Publisher pub_floor_grid       = nh.advertise<nav_msgs::Path>("/tb_path/floor_grid",100);
  ros::Publisher pub_path_visited   	= nh.advertise<nav_msgs::Path>("/tb_path/visited",10);

  ros::Publisher pub_path_lo          = nh.advertise<nav_msgs::Path>("/tb_path/lo",100);
  ros::Publisher pub_path_mid         = nh.advertise<nav_msgs::Path>("/tb_path/mid",100);
  ros::Publisher pub_path_hi          = nh.advertise<nav_msgs::Path>("/tb_path/hi",100);
  ros::Publisher pub_path_vlp         = nh.advertise<nav_msgs::Path>("/tb_path/vlp",100);
  ros::Publisher pub_path_zdown       = nh.advertise<nav_msgs::Path>("/tb_path/zdown",100);

  ros::Publisher pub_midpoint         = nh.advertise<geometry_msgs::PointStamped>("/tb_path/midpoint",100);
//  ros::Publisher pub_path             = nh.advertise<nav_msgs::Path>("/tb_path",100);

  ros::Subscriber s1 = nh.subscribe("/octomap_full",1,octomap_callback);
  ROS_INFO("Ready to convert octomaps.");
  ros::Rate rate(2.0);
  float dt_polygons,dt_paths,dt_lo_mid_hi,dt_vlp,dt_zdown;
  ros::Time t0;
  ros::Time last_update;
  while(ros::ok()){
    checktf();
    checktfvlp();
    if(got_map){
      t0 = ros::Time::now();
      update_polygons();
      dt_polygons  = (ros::Time::now() - t0).toSec();
      t0           = ros::Time::now();
      get_lo_mid_hi_paths();
      dt_lo_mid_hi = (ros::Time::now() - t0).toSec();
      if((ros::Time::now()-last_update).toSec() > 5)
        get_floor = true;
      if(get_floor){
        last_update  = ros::Time::now();

        t0           = ros::Time::now();
        poly_vlp     = test_poly_vlp(par_maxobs,32,30);
        dt_vlp       = (ros::Time::now() - t0).toSec();

        t0           = ros::Time::now();
        get_candid_floor_paths();
        dt_paths     = (ros::Time::now() - t0).toSec();

      //  t0           = ros::Time::now();
      //  get_zdown();
      //  dt_zdown       = (ros::Time::now() - t0).toSec();
      //  cv::imwrite("/home/nuc/path_mapimg.png",mapimg);
      //  cv::imwrite("/home/nuc/path_mapimg_mon.png",mapimg_mono);

        ROS_INFO("TIMER: polygons: %.4f, paths: %.4f, lo_mid_hi: %.4f, dt_vlp: %.4f, dt_zdown: %.4f",dt_polygons,dt_paths,dt_lo_mid_hi,dt_vlp,dt_zdown);
        get_floor = false;
      }


      polyhdng_pub.publish(poly_heading);
      polysafe_pub.publish(poly_safe);
      polyvstd_pub.publish(poly_vstd);
      polyvlp_pub.publish(poly_vlp);

      pub_closest_obstacle.publish(closest_obstacle);
      pub_midpoint.publish(pnt_midpoint);
      pub_path_visited.publish(path_visited);
      pub_floor.publish(path_floor);
      pub_orig.publish(path_candidates);
      pub_floor_grid.publish(path_floor_grid);
      pub_path_lo.publish(path_lo);
      pub_path_mid.publish(path_mid);
      pub_path_hi.publish(path_hi);
      pub_path_vlp.publish(path_vlp);
      pub_path_zdown.publish(path_zdown);
      //pub_path.publish(path_mid);
    }
	  rate.sleep();
	  ros::spinOnce();
  }
  return 0;
}
