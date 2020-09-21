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


using namespace octomap;
using namespace std;

tf2_ros::Buffer tfBuffer;
ros::Publisher targetpoly_pub,target_pub,custpath_pub,pub_centroid,checkpath_pub,path_proximity_pub,path_full_pub,targetalt_pub,pub_tarpose,visited_path_pub,invoke_pub,visited_path_update_pub;
double start_x,start_y,par_maprad;
int mainstate;
std_msgs::UInt8   state_msg;
std_msgs::Float64 target_alt;
nav_msgs::Path visited_path,path_full,path_visited_tar01;
geometry_msgs::PoseStamped last_pose,target;
float centroid_sides = 20;
double par_patience,par_takeoffaltitude;
std::vector<int>blacklist;
int pathindex = 100000;
ros::Time last_feedback;
nav_msgs::Path path_to_check,last_path_to_check;
float global_plan_distance;
float altcmd;
bool got_map,at_floor;
int slopestart,slopestart_targets;
shared_ptr<DynamicEDTOctomap> edf_ptr;
AbstractOcTree* abs_octree;
shared_ptr<OcTree> octree;
geometry_msgs::Point bbmin_custom,bbmax_custom,bbmin_octree,bbmax_octree,pos;
int xmin,ymin,zmin,xmax,ymax,zmax,range_x,range_y,range_z,zmin_global,zmax_global,obsx_last,obsy_last,obsz_last,last_count,centroid_sum_x,centroid_sum_y,centroid_sum_z,centroid_sum_count;
ros::Time last_scan;
std::vector<std::tuple<int,int,int,int,int,int,int>>vt_building;
std::vector<int>rot_abs;
std::vector<int>rot_obs;
float dst_target;
const float rad2deg = 180.0/M_PI;
cv::Mat mapimg(1000,1000,CV_8UC3,cv::Scalar(0, 0, 0)); //create image, set encoding and size, init pixels to default val
std::string par_workdir;
ros::Publisher path_ignore_pub,path_filtered_concatenated_pub;
ros::Publisher pub_tarvis;
geometry_msgs::PointStamped tarvis,closest_obstacle;
nav_msgs::Path path_filtered,building_floors,path_candidates,path_floor_above,path_floor_below,processed_path;
double par_zjump,closest_obstacle_dist;
geometry_msgs::PolygonStamped polygon_min,polygon_max,poly_heading,poly_heading_relative;
std::vector<int> z_lvls;
std_msgs::UInt8 altlvl_msg;
bool centroid_found,building_active;
geometry_msgs::PointStamped building_centroid;
geometry_msgs::PolygonStamped poly_building;
std::vector<geometry_msgs::PoseStamped> targetcmds_sent;
std::vector<std::vector<geometry_msgs::PolygonStamped>> buildings_floorpolygons;
float totarget;
int active_floor;
std::vector<int> building_floors_completed;
std::vector<int> building_floors_known;

std::vector<std::vector<nav_msgs::Path>> buildings_floorpaths;
std::vector<std::vector<nav_msgs::Path>> buildings_floorpaths_shorter;
std::vector<std::tuple<int,int,int,int,int,int,int>>i_type_num_v0_vn_z0_zn;

bool sort_dst_pair(const std::tuple<int,float>& a,
               const std::tuple<int,float>& b)
{
    return (std::get<1>(a) < std::get<1>(b));
}
double constrainAngle(double x){
  if(x > M_PI)
    return (x - M_PI*2);
  else if(x < -M_PI)
    return (x + M_PI*2);
  else
    return x;
}
bool in_poly(geometry_msgs::PolygonStamped polyin, float x, float y){
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
float y2r(float y, float rows,float res){
  return int(rows / 2 - y);
}
float x2c(float x, float cols,float res){
  return int(x + cols/2);
}
int r2y(float r, float rows,float res){
  return float(rows / 2 - r);
}
int c2x(float c, float cols,float res){
  return float(c - cols / 2);
}
bool in_blacklist(int itarget,std::vector<int>blacklist){
  for(int i = 0; i < blacklist.size(); i++){
    if(blacklist[i] == itarget)
      return true;
  }
  return false;
}
float get_dst3d(geometry_msgs::Point p1, geometry_msgs::Point p2){
  return(sqrt(pow(p1.x-p2.x,2)+pow(p1.y-p2.y,2)+pow(p1.z-p2.z,2)));
}
float get_dst2d(geometry_msgs::Point p1, geometry_msgs::Point p2){
  return(sqrt(pow(p1.x-p2.x,2)+pow(p1.y-p2.y,2)));
}
float get_hdng(geometry_msgs::Point p1,geometry_msgs::Point p0){
  float dx = p1.x - p0.x;
  float dy = p1.y - p0.y;
  return atan2(dy,dx);
}

float closest_in_path(nav_msgs::Path pathin,geometry_msgs::Point pin,float dst_min){
  float res,dst;
  res = 1000;
  int res_i;
  if(pathin.poses.size() == 0)
    return res;
  for(int i = 1; i < pathin.poses.size(); i++){
    dst = get_dst2d(pathin.poses[i].pose.position,pin);
    ROS_INFO("CLIENT: Closest: dst %.2f",dst);
    if(dst < res && dst > dst_min){
      res = dst;
      res_i = i;
    }
  }
  return res_i;
}
double get_shortest(double target_hdng,double actual_hdng){
  double a = target_hdng - actual_hdng;
  if(a > M_PI)a -= M_PI*2;
  else if(a < -M_PI)a += M_PI*2;
  return a;
}
cv::Point pnt322cv(geometry_msgs::Point32 pin){
  int c = x2c(pin.x,mapimg.cols,1);
  int r = y2r(pin.y,mapimg.rows,1);
  return cv::Point(c,r);
}
cv::Point pnt2cv(geometry_msgs::Point pin){
  int c = x2c(pin.x,mapimg.cols,1);
  int r = y2r(pin.y,mapimg.rows,1);
  return cv::Point(c,r);
}
void octomap_callback(const octomap_msgs::Octomap& msg){
  abs_octree=octomap_msgs::fullMsgToMap(msg);
  octree.reset(dynamic_cast<octomap::OcTree*>(abs_octree));
  got_map = true;
}

float dst_visited_area(float x,float y, float z){
  float res,dst;
  geometry_msgs::Point pin;
  pin.x = x; pin.y = y; pin.z = z;
  res = 1000;
  if(visited_path.poses.size() == 0)
    return res;
  for(int i = 0; i < visited_path.poses.size(); i++){
    if(abs(visited_path.poses[i].pose.position.z - pin.z) < 2){
      dst = get_dst3d(visited_path.poses[i].pose.position,pin);
      if(dst < res)
        res = dst;
    }
  }
  return res;
}
bool recorded(int x, int y, int z){
  for(int i = 0; i < vt_building.size(); i++){
    if(std::get<0>(vt_building[i])==x
    && std::get<1>(vt_building[i])==y
    && std::get<2>(vt_building[i])==z)
      return true;
  }
  return false;
}

void create_path(float area_sidelength,float area_alt,float centroid_sidelength){
  int num_centroids = int(round(area_sidelength / centroid_sidelength));
  for(int y = 0; y < num_centroids; y++){
    for(int x = 0; x < num_centroids; x++){
      geometry_msgs::PoseStamped pose;
      pose.header.frame_id      = "map";
      pose.header.stamp         = ros::Time::now();
      pose.pose.position.x      = pow(-1,y) * (x * centroid_sidelength - area_sidelength / 2 + centroid_sidelength/2);
      pose.pose.position.y      = -1 * (y * centroid_sidelength - area_sidelength / 2 + centroid_sidelength/2);
      pose.pose.position.z      = area_alt;
      pose.pose.orientation.w   = 1;
      path_full.poses.push_back(pose);
    }
    ROS_INFO("path full %i",path_full.poses.size());
  }
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
  //  ROS_INFO("update_edto[%i points in  sec]: xmin: %i, ymin: %i, zmin: %i, xmax: %i, ymax: %i, zmax: %i, range_x: %i, range_y: %i, range_z: %i ",vol,xmin,ymin,zmin,xmax,ymax,zmax,range_x,range_y,range_z);
  return true;
}

geometry_msgs::PolygonStamped get_surround(float collision_radius,float mapradused,int num_rays){
  update_edto(last_pose.pose.position,collision_radius,mapradused,last_pose.pose.position.z+1,last_pose.pose.position.z-1,false);
  octomap::point3d p(last_pose.pose.position.x,last_pose.pose.position.y,last_pose.pose.position.z);
  octomap::point3d closestObst;
  float d;
  edf_ptr.get()->getDistanceAndClosestObstacle(p,d,closestObst);
  if(d < collision_radius && d > 0){
    closest_obstacle.point.x = closestObst.x();
    closest_obstacle.point.y = closestObst.y();
    closest_obstacle.point.z = closestObst.z();
    closest_obstacle_dist    = d;
  }
  else
    closest_obstacle_dist = 100;
  geometry_msgs::PolygonStamped polygon;
  float rads_pr_i = 2*M_PI / num_rays;
  polygon.polygon.points.resize(num_rays);
  for(int i  = 0; i < num_rays; i++){
    Eigen::Vector3f pnt1_vec(last_pose.pose.position.x,last_pose.pose.position.y,last_pose.pose.position.z);
    Eigen::Vector3f pnt2_vec(last_pose.pose.position.x+mapradused*cos(i*rads_pr_i),mapradused*sin(i*rads_pr_i)+last_pose.pose.position.y,last_pose.pose.position.z);
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

void possible_tasks(){

}
void append_hi_lo(geometry_msgs::Point pos,float yaw,float offsetdst){
  geometry_msgs::Point32 pnt_lo,pnt_hi;
  pnt_lo.x = pos.x - (-offsetdst/2 + offsetdst) * cos(yaw);
  pnt_lo.y = pos.y - (-offsetdst/2 + offsetdst) * sin(yaw);
  pnt_hi.x = pos.x + (-offsetdst/2 + offsetdst) * cos(yaw);
  pnt_hi.y = pos.y + (-offsetdst/2 + offsetdst) * sin(yaw);
  polygon_min.polygon.points.push_back(pnt_lo);
  polygon_max.polygon.points.push_back(pnt_hi);
}

geometry_msgs::Point project_target(float offset){
  float target_yaw = tf::getYaw(target.pose.orientation);
  geometry_msgs::Point t;
  t.z = target.pose.position.z;
  t.x = target.pose.position.x - (-offset/2 + offset) * cos(target_yaw);
  t.y = target.pose.position.y - (-offset/2 + offset) * sin(target_yaw);
  return t;
}

std::vector<int> getinpath_indexes_inrad(nav_msgs::Path pathin,geometry_msgs::Point pos_to_check,float radians){
  std::vector<int> vec_out;
  if(pathin.poses.size() == 0){
    ROS_INFO("PathAdmin: pathin is empty");
    return vec_out;
  }
  for(int i = 0; i < pathin.poses.size(); i++){
    float dist = get_dst2d(pathin.poses[i].pose.position,pos_to_check);
    if(dist <= radians && dist > 0)
      vec_out.push_back(i);
  }
  return vec_out;
}
void draw_poly_heading(float hdng_delta,int num_points){
  float hdng = tf::getYaw(last_pose.pose.orientation);
  float min_hdng = hdng - hdng_delta;
  float max_hdng = hdng + hdng_delta;
  float shortest_maxmin = get_shortest(max_hdng,min_hdng);
  float rads_pr_i = (max_hdng - min_hdng)/num_points;
//  ROS_INFO("Min hdng: %.2f max: %.2f rpri: %.2f shortest_maxmin %.2f",min_hdng,max_hdng,rads_pr_i,shortest_maxmin);
  poly_heading.polygon.points.resize(num_points+1);
  poly_heading.polygon.points[0].x = last_pose.pose.position.x;
  poly_heading.polygon.points[0].y = last_pose.pose.position.y;
  poly_heading.polygon.points[0].z = last_pose.pose.position.z;
  for(int i = 1; i < num_points; i++){
    float a = constrainAngle(min_hdng + rads_pr_i * i);
    poly_heading.polygon.points[i].x = last_pose.pose.position.x + 50*cos(a);
    poly_heading.polygon.points[i].y = last_pose.pose.position.y + 50*sin(a);
    poly_heading.polygon.points[i].z = last_pose.pose.position.z;
  }
  poly_heading.polygon.points[num_points].x = last_pose.pose.position.x;
  poly_heading.polygon.points[num_points].y = last_pose.pose.position.y;
  poly_heading.polygon.points[num_points].z = last_pose.pose.position.z;
}

void targetpose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){

}
int get_zn(float z){
  for(int i = 0; i < z_lvls.size()-1; i++){
    if(z_lvls[i] < z && z_lvls[i+1] > z)
    return i;
  }
  return 0;
}
void update_target(geometry_msgs::PoseStamped ps){
  if(!building_active)
    building_centroid.point = project_target(10);
  float hdng = get_hdng(tarvis.point,ps.pose.position);
  target = ps;
  ROS_INFO("TARGET: hdng[%.2f]: %.2f %.2f %.2f ->  %.2f %.2f %.2f",hdng,tarvis.point.x,tarvis.point.y,tarvis.point.z,ps.pose.position.x,ps.pose.position.y,ps.pose.position.z);
  targetcmds_sent.push_back(target);
  tarvis.header.stamp = ros::Time::now();
  tarvis.point        = target.pose.position;
  target_pub.publish(target);
}
void get_next_task(){
  if(building_active){
    ROS_INFO("TASKMASTER: Inspecting building[%i] building_active %i in slope %i v0 %i vn %i zn %i",
    building_number,building_active,at_floor,slopestart,path_visited.poses.size(),zn);
  }
}
void create_floor(){
  at_floor = false;
  //TODO: FIX ALTITUDE SWITCH && BUILDING SWITCH
  poly_building.polygon.points.resize(0);
  int last_i;
  int vn = visited_path.poses.size();
  int v0 = slopestart;
  int zn = get_zn(visited_path.poses[v0].pose.position.z);

  float last_yaw = tf::getYaw(visited_path.poses[v0].pose.orientation);
  float hdng_lim = M_PI/4;

  for(int i = v0; i < vn; vn++){
    float new_yaw = tf::getYaw(visited_path.poses[i].pose.orientation);
    buildings_floorpaths[building_number][zn].poses.push_back(visited_path.poses[i]);
    if(abs(get_shortest(last_yaw,new_yaw)*100) > abs(hdng_lim*100)
    || get_dst3d(visited_path.poses[i].pose.position,visited_path.poses[last_i].pose.position) > 20){
      last_i = i;
      geometry_msgs::Point32 p;
      p.x = visited_path.poses[i].pose.position.x;
      p.y = visited_path.poses[i].pose.position.y;
      p.z = visited_path.poses[i].pose.position.z;
      poly_building.polygon.points.push_back(p);
      buildings_floorpaths_shorter[building_number][zn].poses.push_back(visited_path.poses[i]);
      ROS_INFO("New Floor Polygon[%i]: [%i points of %i]",i,poly_building.polygon.points.size(),buildings_floorpaths[building_number][zn].poses.size());
    }
  }
  path_floor_below.poses.resize(0);
  path_floor_above.poses.resize(0);
  path_floor_active =
  for(int i = slopestart_targets; i < targetcmds_sent.size(); i++){
    std::vector<int> indexes_in_rad = getinpath_indexes_inrad(path_candidates,targetcmds_sent[i].pose.position,5);
    for(int k = 0; k < indexes_in_rad.size(); k++){
      geometry_msgs::PoseStamped ps;
      ps = path_candidates.poses[indexes_in_rad[k]];
      float dz = ps.pose.position.z - last_pose.pose.position.z;
      if(dz < 0 && dz > -par_zjump*1.5)
        path_floor_below.poses.push_back(ps);  //      buildings_floorpaths[building_number][zn-1].poses.push_back(ps);
      else if(dz > 0 && dz < par_zjump*1.5)
        path_floor_above.poses.push_back(ps);//      buildings_floorpaths[building_number][zn+1].poses.push_back(ps);
    }
  }
  ROS_INFO("New Floor Etage [%i z of %i]: path orig:[%i] below[%i],above[%i]: %i %i",
  buildings_floorpaths[building_number][zn].poses.size(),buildings_floorpaths[building_number][zn-1].poses.size(),
  buildings_floorpaths[building_number][zn+1].poses.size());
  ROS_INFO("New Floor Polygon: [%i points of %i]",poly_building.polygon.points.size(),buildings_floorpaths[building_number][zn].poses.size());
  i_type_num_v0_vn_z0_zn.push_back(std::make_tuple(int(i_type_num_v0_vn_z0_zn.size()),type_building,building_number,slopestart,int(visited_path.poses.size()),zn,zn));
  task_complete = true;
}

void update_centroid(){
  int radians_segmentsize = 72;
  float min_completion  = 0.90;
  std::vector<bool> visited;
  visited.resize(radians_segmentsize);
  centroid_sum_count = visited_path.poses.size() - slopestart;
  building_centroid.header.frame_id ="map";
  building_centroid.header.stamp = ros::Time::now();
  int centroid_sum_x = 0;
  int centroid_sum_y = 0;
  int centroid_sum_z = 0;
  int count = 0;
  int centroid_sum_count = visited_path.poses.size() - slopestart;
  float a_pr_i = rad2deg * (2*M_PI / radians_segmentsize);

  if(centroid_sum_count > 10){
    for(int i = slopestart; i < visited_path.poses.size(); i++){
      centroid_sum_x += int(round(visited_path.poses[i].pose.position.x));
      centroid_sum_y += int(round(visited_path.poses[i].pose.position.y));
      centroid_sum_z += int(round(visited_path.poses[i].pose.position.z));
      int deg = int(round((rad2deg * (M_PI + get_hdng(visited_path.poses[i].pose.position,building_centroid.point)) ) ));
      visited[deg/a_pr_i] = true;
    }
    for(int i = 0; i < radians_segmentsize; i++){
      if(visited[i])
        count++;
    }

    if(abs(count - last_count) >= 5 || count > radians_segmentsize*min_completion){
      building_centroid.point.x = centroid_sum_x / centroid_sum_count;
      building_centroid.point.y = centroid_sum_y / centroid_sum_count;
      building_centroid.point.z = centroid_sum_z / centroid_sum_count;
      last_count = count;
      ROS_INFO("New Building Centroid[%i rads of %i]: %.2f %.2f %.2f",count,radians_segmentsize,building_centroid.point.x,building_centroid.point.y,building_centroid.point.z);
      if(count > radians_segmentsize * min_completion && building_floors.poses.size() == 0){
        create_floor();
      }
    }
  }
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
  if(sqrt(pow(transformStamped.transform.translation.x - last_pose.pose.position.x,2)+ pow(transformStamped.transform.translation.y - last_pose.pose.position.y,2)+
    pow(transformStamped.transform.translation.z - last_pose.pose.position.z,2)) >= 1.00){
    dst_target = get_dst3d(tarvis.point,last_pose.pose.position);
    last_pose.pose.position.x  = transformStamped.transform.translation.x;
    last_pose.pose.position.y  = transformStamped.transform.translation.y;
    last_pose.pose.position.z  = transformStamped.transform.translation.z;
    last_pose.pose.orientation = transformStamped.transform.rotation;
    last_pose.header           = transformStamped.header;
    visited_path.poses.push_back(last_pose);
    visited_path.header.stamp  = ros::Time::now();
    visited_path_update_pub.publish(last_pose);
  }
}

void custorg_cb(const nav_msgs::Path::ConstPtr& msg){
//  shorten_path(*msg,M_PI/4);
}

void candpath_cb(const nav_msgs::Path::ConstPtr& msg){
  path_candidates = *msg;
}

void processedpath_cb(const nav_msgs::Path::ConstPtr& msg){
  processed_path = *msg;
}

void cmdmb_cb(const std_msgs::Bool::ConstPtr& msg){
  /*if(pathindata && exploring){
    exploring = false;
  }*/
}
void update_target_from_processed_path(){
  int closest_i = closest_in_path(processed_path,target.pose.position,3);
  if(closest_i < processed_path.poses.size())
    update_target(processed_path.poses[closest_i]);
  //lse
    //ROS_INFO("Closest_i %i > %i processed_path size:",closest_i,processed_path.poses.size());
}

void check_if_moving(){
  if((ros::Time::now() - visited_path.header.stamp).toSec() > 4){
    if(get_dst3d(target.pose.position,last_pose.pose.position) > 5)
      target_pub.publish(target);
    else
      update_target_from_processed_path();
  }
  else if(get_dst3d(target.pose.position,last_pose.pose.position) < 5)
    update_target_from_processed_path();

}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "tb_fsm_node");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");
  private_nh.param("par_zjump",   par_zjump, 3.0);//*2.0);
  private_nh.param("map_sidelength",par_maprad, 300.0);
  private_nh.param("takeoff_altitude",par_takeoffaltitude, 5.0);
  private_nh.getParam("workdir_path", par_workdir);
  nav_msgs::Path path_template;
  geometry_msgs::PolygonStamped poly_template;

  poly_template.header.frame_id = path_template.header.frame_id = "map";

  std::vector<nav_msgs::Path> path_template_building;
  std::vector<geometry_msgs::PolygonStamped> poly_template_building;

  for(int i = 0; i < 20; i++){
    z_lvls.push_back(int(round(-par_zjump*3 + par_zjump*i)));
    path_template_building.push_back(path_template);
    poly_template_building.push_back(poly_template);
  }

  for(int i = 0; i < 20; i++){
    buildings_floorpolygons.push_back(poly_template_building);
    buildings_floorpaths.push_back(path_template_building);
  }

  buildings_floorpaths_shorter = buildings_floorpaths;
  poly_heading.header.frame_id ="map";

  closest_obstacle.header.frame_id = last_pose.header.frame_id = "map";
  last_pose.pose.orientation.w = 1;
  path_full.header = target.header = visited_path.header = last_pose.header;
  target.pose.orientation.w = 1;
  visited_path.poses.push_back(last_pose);
  create_path(par_maprad,par_takeoffaltitude,centroid_sides);
  path_full.header.stamp = ros::Time::now();

  tf2_ros::TransformListener tf2_listener(tfBuffer);
  visited_path_pub        = nh.advertise<nav_msgs::Path>("/tb_nav/visited_path",10);
  visited_path_update_pub = nh.advertise<geometry_msgs::PoseStamped>("/tb_nav/visited_path_update",100);

  path_full_pub       = nh.advertise<nav_msgs::Path>("/tb_nav/full_path",10);

  //ros::Subscriber s5 = nh.subscribe("/tb_obst/evaluate_fromto_noz",10,&fromto_noz_cb);

  ros::Publisher state_pub    = nh.advertise<std_msgs::UInt8>("/tb_fsm/main_state",10);
  ros::Publisher invoke_pub   = nh.advertise<std_msgs::String>("/tb_invoke",10);
  ros::Publisher pub_altlvl   = nh.advertise<std_msgs::UInt8>("/tb_fsm/altlvl",100);
  ros::Subscriber s12 = nh.subscribe("/tb_path/path_not_visited",100,&processedpath_cb);
  ros::Publisher targetalt_pub       = nh.advertise<std_msgs::Float64>("/tb_cmd/alt_target",100);

  target_pub = nh.advertise<geometry_msgs::PoseStamped>("/tb_cmdmb/target_pose",100);
  targetpoly_pub = nh.advertise<geometry_msgs::PolygonStamped>("/tb_cmdmb/target_poly",100);
  ros::Subscriber s1 = nh.subscribe("/octomap_full",1,octomap_callback);
  ros::Subscriber s3 = nh.subscribe("/tb_path_filtered",100,&candpath_cb);
  ros::Subscriber s2 = nh.subscribe("/tb_cmdmb/target_pose",100,&targetpose_cb);
  ros::Subscriber s44 = nh.subscribe("/tb_cmdmb/success",100,&cmdmb_cb);

  pub_tarvis   = nh.advertise<geometry_msgs::PointStamped>("/tb_cmd/visualize",100);
  custpath_pub  = nh.advertise<nav_msgs::Path>("/tb_path/custom_organize",100);

  ros::Publisher polysafe_pub = nh.advertise<geometry_msgs::PolygonStamped>("/tb_polygon_safe",100);
  ros::Publisher polyhdng_pub = nh.advertise<geometry_msgs::PolygonStamped>("/tb_polygon_hdng",100);
  ros::Publisher polybldng_pub = nh.advertise<geometry_msgs::PolygonStamped>("/tb_polygon_building",100);
  ros::Publisher pub_closest_obstacle = nh.advertise<geometry_msgs::PointStamped>("/tb_obs/closest",100);
  ros::Publisher pub_centroid = nh.advertise<geometry_msgs::PointStamped>("/tb_cmd/building_centroid",100);


  ros::Publisher cmdarm_pub = nh.advertise<std_msgs::Float64>("/tb_cmd/arm_pitch",10);

  geometry_msgs::PolygonStamped polysafe;
  ros::Subscriber s4343 = nh.subscribe("/tb_path/custom_organized",1,&custorg_cb);

  tarvis.header.frame_id = "map";
  std_msgs::Float64 cmdarm_msg;
  ros::Rate rate(5.0);
  ros::Time start = ros::Time::now();
  bool invoke_published;
  bool bag_published,par_live;
  ros::Time last_global_plan,last_info;
  float last_global_plan_distance;
  altlvl_msg.data = 8;
  target.pose.position.z = z_lvls[altlvl_msg.data];

  while(ros::ok()){
    if(got_map){
      if(mainstate == 0 && last_pose.pose.position.z >= 1.0)
          mainstate = 1;
      else if(mainstate == 2 && sqrt(pow(last_pose.pose.position.x,2)+pow(last_pose.pose.position.y,2)) < 3)
          mainstate = 3;
      if(path_full.poses.size() == 0)
        create_path(par_maprad,par_takeoffaltitude,centroid_sides);
      checktf();
      float dt_global_plan= (ros::Time::now() - last_global_plan).toSec();

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
        else if((ros::Time::now() - start).toSec() > 600 && mainstate == 1)
          mainstate = 2;
      }
      if( (ros::Time::now() - last_info).toSec() > 3 && rot_abs.size() > 2){
        last_info = ros::Time::now();
    /*    path_filtered.header.frame_id = "map";
        ROS_INFO("STATE: Locked on building: %i, current points: %i, current_abs_angle: %i obs_angle: %i",at_floor,vt_building.size(),rot_abs[rot_abs.size()-1],rot_obs[rot_obs.size()-1]);
        path_filtered_concatenated_pub.publish(path_filtered);
        polygon_min.header.frame_id = "map";
        polygon_min_pub.publish(polygon_min);
        polygon_max.header.frame_id = "map";
        polygon_max_pub.publish(polygon_max);*/
        if(!building_active && !at_floor){
          octomap::point3d p(last_pose.pose.position.x,last_pose.pose.position.y,last_pose.pose.position.z);
          octomap::point3d closestObst;
          float d;
          update_edto(last_pose.pose.position,30,30,last_pose.pose.position.z+1,last_pose.pose.position.z-1,false);
          edf_ptr.get()->getDistanceAndClosestObstacle(p,d,closestObst);
          if(dst_target < 5 && d < 25 && d > 5){
            at_floor        = true;
            building_active = true;
            slopestart         = visited_path.poses.size();
            slopestart_targets = targetcmds_sent.size();
            building_floors.poses.resize(0);
          }
        }
        else if(at_floor){
          update_centroid();
        }
      }
      check_if_moving();
      custpath_pub.publish(visited_path);

      polysafe = get_surround(10,50,36);
      polysafe.header.frame_id = "map";
      polysafe.header.stamp = ros::Time::now();
      draw_poly_heading(M_PI/2,16);

      if(poly_heading.polygon.points.size() > 0){
        poly_heading.header = polysafe.header;
        polyhdng_pub.publish(poly_heading);
      }
      if(poly_building.polygon.points.size() > 0){
        poly_building.header = polysafe.header;
        polybldng_pub.publish(poly_building);
      }

      std_msgs::Float64 alt_target;
      alt_target.data = z_lvls[altlvl_msg.data];
      pub_closest_obstacle.publish(closest_obstacle);
      polysafe_pub.publish(polysafe);
      path_full_pub.publish(path_full);
      pub_tarvis.publish(tarvis);

      targetalt_pub.publish(alt_target);

      state_pub.publish(state_msg);
      pub_altlvl.publish(altlvl_msg);
      visited_path_pub.publish(visited_path);
      pub_centroid.publish(building_centroid);
    }
    rate.sleep();
    ros::spinOnce();
  }
  return 0;
}
