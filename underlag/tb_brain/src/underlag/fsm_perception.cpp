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
ros::Publisher pub_target_next,pub_path_filtered,pub_path_test,pub_path_visited,pub_idle_time,pub_target_approved,targetpoly_pub,targetalt_pub;

bool bag_published,par_live,got_map,got_hi,got_mid,got_down,everyother;
int zlvl,n_sectors,buildingstate,missionstate,mainstate,xmin,ymin,zmin,xmax,ymax,zmax,range_x,range_y,range_z,zmin_global,zmax_global;
double par_xyjump,par_hdng_penalty_base,par_zfac,par_target_mindst,pos_yaw,par_n_sectors,par_maprad,par_patience,par_takeoffaltitude,par_zjump,closest_obstacle_dist;

std_msgs::UInt8 state_msg,altlvl_msg;
std_msgs::Float64 alt_target,idle_time;

geometry_msgs::Point bbmin_custom,bbmax_custom,bbmin_octree,bbmax_octree,pos;
geometry_msgs::PointStamped target_next_ideal,building_centroid,closest_obstacle,mext_target_ideal;
geometry_msgs::PoseStamped last_pose,target,target_next;
geometry_msgs::PolygonStamped poly_heading,poly_heading_rel,poly_safe;
nav_msgs::Path path_filtered,path_visited,path_full,path;

sensor_msgs::LaserScan scan_cleared,scan_roofs,scan_dangers,scan_buildings;

std::vector<int> z_lvls;
std::vector<int> blacklist;
std::vector<geometry_msgs::PoseStamped> targetcmds_sent;

shared_ptr<DynamicEDTOctomap> edf_ptr;
AbstractOcTree* abs_octree;
shared_ptr<OcTree> octree;



bool sort_dst_pair(const std::tuple<int,float>& a,const std::tuple<int,float>& b){
    return (std::get<1>(a) < std::get<1>(b));
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
double constrainAngle(double x){
  if(x > M_PI)
    return (x - M_PI*2);
  else if(x < -M_PI)
    return (x + M_PI*2);
  else
    return x;
}
float get_dst3d_zfac(geometry_msgs::Point p1, geometry_msgs::Point p2,float zfac){
  return(sqrt(pow(p1.x-p2.x,2)+pow(p1.y-p2.y,2)+pow(zfac*(p1.z-p2.z),2)));
}
float get_dst3d(geometry_msgs::Point p1, geometry_msgs::Point p2){
  return(sqrt(pow(p1.x-p2.x,2)+pow(p1.y-p2.y,2)+pow(p1.z-p2.z,2)));
}
float get_dst2d(geometry_msgs::Point p1, geometry_msgs::Point p2){
  return(sqrt(pow(p1.x-p2.x,2)+pow(p1.y-p2.y,2)));
}
float get_dst2d32(geometry_msgs::Point32 p1, geometry_msgs::Point32 p2){
  return(sqrt(pow(p1.x-p2.x,2)+pow(p1.y-p2.y,2)));
}
float get_hdng(geometry_msgs::Point p1,geometry_msgs::Point p0){
  float dx = p1.x - p0.x;
  float dy = p1.y - p0.y;
  return atan2(dy,dx);
}
int get_zn(float z){
  for(int i = 0; i < z_lvls.size()-1; i++){
    if(z_lvls[i] < z && z_lvls[i+1] > z)
    return i;
  }
  return 0;
}



geometry_msgs::Point get_poly_centroidXY_A(geometry_msgs::PolygonStamped polyin){
    geometry_msgs::Point centroid;
    double signedArea = 0.0;
    double x0 = 0.0; // Current vertex X
    double y0 = 0.0; // Current vertex Y
    double x1 = 0.0; // Next vertex X
    double y1 = 0.0; // Next vertex Y
    double a = 0.0;  // Partial signed area

    // For all vertices except last
    int i=0;

    if(polyin.polygon.points.size() < 2){
      return centroid;
    }
    for (i=0; i<polyin.polygon.points.size()-1; ++i)
    {
        x0 = polyin.polygon.points[i].x;
        y0 = polyin.polygon.points[i].y;
        x1 = polyin.polygon.points[i+1].x;
        y1 = polyin.polygon.points[i+1].y;
        a = x0*y1 - x1*y0;
        signedArea += a;
        centroid.x += (x0 + x1)*a;
        centroid.y += (y0 + y1)*a;
    }

    // Do last vertex separately to avoid performing an expensive
    // modulus operation in each iteration.
    x0 = polyin.polygon.points[i].x;
    y0 = polyin.polygon.points[i].y;
    x1 = polyin.polygon.points[0].x;
    y1 = polyin.polygon.points[0].y;
    a = x0*y1 - x1*y0;
    signedArea += a;
    centroid.x += (x0 + x1)*a;
    centroid.y += (y0 + y1)*a;

    signedArea *= 0.5;
    centroid.x /= (6.0*signedArea);
    centroid.y /= (6.0*signedArea);
    centroid.z = signedArea;

    return centroid;
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

float get_dst2d_zlim(geometry_msgs::Point p1, geometry_msgs::Point p2,float dz_max){
  if(abs(p1.z - p2.z) >= dz_max)
    return 1000;
  else
    return(sqrt(pow(p1.x-p2.x,2)+pow(p1.y-p2.y,2)));
}
std::vector<int> getinpath_indexes_inrad(nav_msgs::Path pathin,int i_to_check,float radians,float dz_max){
  std::vector<int> vec_out;
  if(pathin.poses.size() == 0){
    return vec_out;
  }
  for(int i = 0; i < pathin.poses.size(); i++){
    float dist = get_dst2d_zlim(pathin.poses[i].pose.position,pathin.poses[i_to_check].pose.position,dz_max);
    if(dist <= radians && dist > 0)
      vec_out.push_back(i);
  }
  return vec_out;
}
std::vector<std::vector<int>> getinpath_neighbours(nav_msgs::Path pathin,float radians,float dz_max){
  std::vector<int> neighbours;
  std::vector<std::vector<int>> neighbours_at_index;
  //getinpath_neighbours(pathin,radians);
  if(pathin.poses.size() == 0){
    ROS_INFO("PathAdmin: pathin is empty");
    return neighbours_at_index;
  }
  for(int i = 0; i < pathin.poses.size(); i++){
    neighbours = getinpath_indexes_inrad(pathin,i,radians,dz_max);
    neighbours.push_back(i);
    sort(neighbours.begin(),neighbours.end());
    neighbours_at_index.push_back(neighbours);
  }
  return neighbours_at_index;
}
std::vector<int> get_endpoints(std::vector<std::vector<int>> vec_of_vecs){
  std::vector<int> endpoints;
  for(int i = 0; i < vec_of_vecs.size(); i++){
    if(vec_of_vecs[i].size() == 2){
      endpoints.push_back(i);
    }
  }
  return endpoints;
}
bool in_vec(std::vector<int> vec,int k){
  if(vec.size() == 0)
    return false;
  //	ROS_INFO("in vec: %i",k);
  for(int i = 0; i < vec.size(); i++){
    if(vec[i] == k)
      return true;
  }
  return false;
}
std::vector<int> go_from_endpoint(std::vector<std::vector<int>> vec_of_vecs,int endpoint){
  int next_point = endpoint;
  std::vector<int> vec_out;
  vec_out.push_back(endpoint);

  if(vec_of_vecs[endpoint][0] == endpoint)
    next_point = vec_of_vecs[endpoint][1];
  else
    next_point = vec_of_vecs[endpoint][0];

  while(vec_of_vecs[next_point].size() == 3){
    vec_out.push_back(next_point);
    if(in_vec(vec_out,vec_of_vecs[next_point][0]) && in_vec(vec_out,vec_of_vecs[next_point][1]))
      next_point = vec_of_vecs[next_point][2];
    else if(in_vec(vec_out,vec_of_vecs[next_point][1]) && in_vec(vec_out,vec_of_vecs[next_point][2]))
      next_point = vec_of_vecs[next_point][0];
    else
      next_point = vec_of_vecs[next_point][1];
  //	ROS_INFO("Vec out added next point: %i",next_point);
  }
  if(in_vec(vec_out,vec_of_vecs[next_point][0]))
    vec_out.push_back(vec_of_vecs[next_point][1]);
  else
    vec_out.push_back(vec_of_vecs[next_point][0]);
  return vec_out;
}

nav_msgs::Path get_connected_path(nav_msgs::Path pathin,float radius,int index,float dz_max){
  nav_msgs::Path pathout;
  pathout.header.frame_id = "map";
  if(pathin.poses.size() == 0)
    return pathout;
  std::vector<std::vector<int>> neighbours_at_index;
  neighbours_at_index = getinpath_neighbours(pathin,radius,dz_max);
  std::vector<int> endpoints;
  std::vector<int> open_loop;
  std::vector<std::tuple<int,float>>i_dst;
  endpoints = get_endpoints(neighbours_at_index);
  for(int i = 0; i < endpoints.size(); i++){
    i_dst.push_back(std::make_tuple(endpoints[i],get_dst2d_zlim(pathin.poses[index].pose.position,pathin.poses[endpoints[i]].pose.position,dz_max) ));
  }
  sort(i_dst.begin(),i_dst.end(),sort_dst_pair);
  for(int i = 0; i < i_dst.size(); i++){
    endpoints[i] = std::get<0>(i_dst[i]);
  }
  open_loop  = go_from_endpoint(neighbours_at_index,endpoints[0]);
  for(int k = 0; k < open_loop.size(); k++){
    pathout.poses.push_back(pathin.poses[open_loop[k]]);
  }
  return pathout;
}
geometry_msgs::Point get_p2d_offset_dst(geometry_msgs::Point pin,float dst_margin){
  Eigen::Vector3f pos_vec(pos.x,pos.y,pos.z);
  Eigen::Vector3f pin_vec(pin.x,pin.y,pin.z);
  geometry_msgs::Point pout;
  Eigen::Vector3f stride_vec;
  stride_vec = (pin_vec - pos_vec).normalized();
  float dst  = (pin_vec - pos_vec).norm();
  float new_dst = dst - dst_margin;
  if(new_dst <= 0)
    new_dst = dst;
  Eigen::Vector3f pnt2_actual = pos_vec + stride_vec * new_dst;
  pout.x = pnt2_actual.x();
  pout.y = pnt2_actual.y();
  pout.z = pnt2_actual.z();
  return pout;
}
float get_candidate_penalty(geometry_msgs::Point candidate){
  Eigen::Vector3f candpnt_vec(candidate.x,candidate.y,candidate.z);
  Eigen::Vector3f tarpnt_vec(target.pose.position.x,target.pose.position.y,target.pose.position.z);
  Eigen::Vector3f pnt2_ideal(target_next_ideal.point.x,target_next_ideal.point.y,target_next_ideal.point.z);
  Eigen::Vector3f candidate_stride_vec,pnt2_actual;
  candidate_stride_vec = (candpnt_vec - tarpnt_vec).normalized();
  pnt2_actual = tarpnt_vec + candidate_stride_vec * par_hdng_penalty_base;
  return (pnt2_actual - pnt2_ideal).norm();
}

int get_target(nav_msgs::Path pathin,float dst_min,float zfac){
  if(pathin.poses.size() == 0)
    return 0;
  std::vector<std::tuple<int,float>>dst_tar;
  std::vector<std::tuple<int,float>>dst_cen;
  std::vector<std::tuple<int,float>>dst_tot;
  std::vector<std::tuple<int,float>>penalty;
  for(int i = 1; i < pathin.poses.size(); i++){
    float dst_tar_val = get_dst3d_zfac(pathin.poses[i].pose.position,target.pose.position,zfac);
    float dst_cen_val = get_dst3d_zfac(pathin.poses[i].pose.position,building_centroid.point,zfac);
    float penalty_val = get_candidate_penalty(pathin.poses[i].pose.position);
    float dst_tot_val = dst_tar_val + dst_cen_val + penalty_val;
//    ROS_INFO("Pathin[%i]: dst_target: %.2f, dst_centroid: %.2f, dst_tot: %.2f",i,dst_tar_val,dst_cen_val,dst_tot_val);
    if(dst_tar_val > dst_min){
      dst_tar.push_back(std::make_tuple(i,dst_tar_val));
      dst_cen.push_back(std::make_tuple(i,dst_cen_val));
      dst_tot.push_back(std::make_tuple(i,dst_tot_val));
      penalty.push_back(std::make_tuple(i,penalty_val));
    }
//    else
  //    ROS_INFO("Pathin[%i]: DISCARDED",i);
  }
  sort(dst_tar.begin(),dst_tar.end(),sort_dst_pair);
  sort(dst_cen.begin(),dst_cen.end(),sort_dst_pair);
  sort(dst_tot.begin(),dst_tot.end(),sort_dst_pair);
  sort(penalty.begin(),penalty.end(),sort_dst_pair);
  for(int i = 1; i < dst_tar.size(); i++){
    int tar = std::get<0>(dst_tar[i]);
    int cen = std::get<0>(dst_cen[i]);
    int tot = std::get<0>(dst_tot[i]);
    int pen = std::get<0>(dst_tot[i]);
    float res_tar = std::get<1>(dst_tar[i]);
    float res_cen = std::get<1>(dst_cen[i]);
    float res_tot = std::get<1>(dst_tot[i]);
    float penlty = std::get<1>(dst_tot[i]);
    ROS_INFO("Rank[%i]: Index: %i, %i, %i, Meters: %.2f %.2f %.2f penalty: %.2f",i,tar,cen,tot,res_tar,res_cen,res_tot,penlty);
  }
  int  res_i    = std::get<0>(dst_tot[0]);
  float res_tar = std::get<1>(dst_tar[0]);
  float res_cen = std::get<1>(dst_cen[0]);
  float res_tot = std::get<1>(dst_tot[0]);
  float penlty  = std::get<1>(dst_tot[0]);
  ROS_INFO("Result: Index %i with total of %.2f meters (cen: %.2f tar: %.2f)",res_i,res_tar,res_cen,penlty);
  return res_i;
}
float closest_in_path(nav_msgs::Path pathin,geometry_msgs::Point pin,float dst_min){
  float res,dst;
  res = 1000;
  int res_i;
  if(pathin.poses.size() == 0)
    return res;
  for(int i = 1; i < pathin.poses.size(); i++){
    dst = get_dst2d(pathin.poses[i].pose.position,pin);
    if(dst < res && dst > dst_min){
      res = dst;
      res_i = i;
    }
  }
  return res_i;
}/*
float closest_in_path(nav_msgs::Path pathin,geometry_msgs::Point pin,float dst_min){
  float res,dst;
  res = 1000;
  int res_i,dzabs;
  if(pathin.poses.size() == 0)
    return res;
  for(int i = 1; i < pathin.poses.size(); i++){
    dst = get_dst3d(pathin.poses[i].pose.position,pin);
//int dz = abs(pathin.poses[i].pose.position.z - last_pose.pose.position.z);
//    if(&& dst < res && dst > dst_min || dz < dzabs && dst < 3*res && dst > dst_min || res == 1000 && dst > dst_min){

      res = dst;
      res_i = i;
    }
  }
  return res_i;
}*/
bool in_blacklist(int itarget){
  for(int i = 0; i < blacklist.size(); i++){
    if(blacklist[i] == itarget)
      return true;
  }
  return false;
}

void update_target(geometry_msgs::PoseStamped ps){
  target = ps;
  if(targetcmds_sent.size() > 0){
    Eigen::Vector3f pnt0(targetcmds_sent[targetcmds_sent.size()-1].pose.position.x,targetcmds_sent[targetcmds_sent.size()-1].pose.position.y,targetcmds_sent[targetcmds_sent.size()-1].pose.position.z);
    Eigen::Vector3f pnt1(target.pose.position.x,target.pose.position.y,target.pose.position.z);
    Eigen::Vector3f current_stride_vec,target_ideal;
    current_stride_vec = (pnt1 - pnt0).normalized();
    target_ideal = pnt1 + current_stride_vec * par_hdng_penalty_base;
    target_next_ideal.point.x = target_ideal.x();
    target_next_ideal.point.y = target_ideal.y();
    target_next_ideal.point.z = target_ideal.z();
  }
  targetcmds_sent.push_back(ps);
  pub_target_approved.publish(ps);
}
void update_next_target(geometry_msgs::PoseStamped ps){
  target_next = ps;
//  pub_target_next.publish(ps);
}
nav_msgs::Path path_filter(nav_msgs::Path pathin){
  nav_msgs::Path pathout;
  pathout.header = hdr();
  ROS_INFO("PERCEPTION-PATHTARGETS: %i poses to filter",pathin.poses.size());
  if(pathin.poses.size() == 0)
    return pathout;
  ros::Time t0 = ros::Time::now();
  for(int i = 0; i < pathin.poses.size(); i++){
    if(!in_blacklist(i)){
      if(in_poly(poly_safe,pathin.poses[i].pose.position.x,pathin.poses[i].pose.position.y)){
        if(in_poly(poly_heading_rel,pathin.poses[i].pose.position.x,pathin.poses[i].pose.position.y) ){
          ROS_INFO("PERCEPTION-PATHTARGET[%i] not in blacklist and in poly safe and poly heading rel",i);
          pathout.poses.push_back(pathin.poses[i]);
        }
      }
    }
  }
  float dt = (ros::Time::now() - t0).toSec();
  ROS_INFO("PERCEPTION-PATHTARGETS[%.5f]: %i poses to use",dt,pathout.poses.size());
  return pathout;
}

int aquire_target_from_path(nav_msgs::Path pathin,float min_dst,float zfac){
  int i,i_new,res;
  i     = closest_in_path(pathin,target.pose.position,par_target_mindst);
  i_new = get_target(pathin,min_dst,zfac);
  nav_msgs::Path path2;
  ROS_INFO("BLD: *************RESULT**************");
  ROS_INFO("BLD: closest_i: %i,i_new: %i,i_advanced",i,i_new);
  if(i_new < pathin.poses.size())
    res = i_new;
  else if(i < pathin.poses.size())
    res = i;
  else
    res = -1;
  if(res == -1)
    ROS_INFO("PERCEPTION-PATHTTARGET: ERROR, %i",res);
  else
    ROS_INFO("Blacklisting target: %i",res);

  return res;
}

nav_msgs::Path merge_paths(nav_msgs::Path path_base,nav_msgs::Path path_to_add){
  for(int i = 0; i < path_to_add.poses.size(); i++){
    path_base.poses.push_back(path_to_add.poses[i]);
  }
  return path_base;
}
nav_msgs::Path create_path_xy(float area_sidelength,float z,float centroid_sidelength){
  nav_msgs::Path pathout;
  pathout.header = hdr();
  int num_centroids = int(round(area_sidelength / centroid_sidelength));
  for(int y = 0; y < num_centroids; y++){
    for(int x = 0; x < num_centroids; x++){
      geometry_msgs::PoseStamped pose;
      pose.header.frame_id      = "map";
      pose.header.stamp         = ros::Time::now();
      pose.pose.position.x      = pow(-1,y) * (x * centroid_sidelength - area_sidelength / 2 + centroid_sidelength/2);
      pose.pose.position.y      = -1 * (y * centroid_sidelength - area_sidelength / 2 + centroid_sidelength/2);
      pose.pose.position.z      = z;
      pose.pose.orientation.w   = 1;
      path_full.poses.push_back(pose);
    }
  }
  ROS_INFO("PERCEPTION-PathFull[%i]",path_full.poses.size());
  return pathout;
}
nav_msgs::Path create_path_xyz(float area_sidelength,int z_lvl0, int z_lvl1,float centroid_sidelength){
  nav_msgs::Path pathout;
  pathout.header = hdr();
  for(int i = z_lvl0; i < z_lvl1; i++){
    path = create_path_xy(area_sidelength,z_lvls[i],centroid_sidelength);
    pathout = merge_paths(pathout,path);
  }
  ROS_INFO("PERCEPTION-PathFull[%i]",path_full.poses.size());
  return pathout;
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
      ROS_INFO("PERCEPTION:  FAILED update_edto FAILED: xmin: %i, ymin: %i, zmin: %i, xmax: %i, ymax: %i, zmax: %i, range_x: %i, range_y: %i, range_z: %i ",xmin,ymin,zmin,xmax,ymax,zmax,range_x,range_y,range_z);
      return false;
    }
  //  ROS_INFO("PERCEPTION:  update_edto[%i points in  sec]: xmin: %i, ymin: %i, zmin: %i, xmax: %i, ymax: %i, zmax: %i, range_x: %i, range_y: %i, range_z: %i ",vol,xmin,ymin,zmin,xmax,ymax,zmax,range_x,range_y,range_z);
  return true;
}

geometry_msgs::PolygonStamped get_poly_surround(float collision_radius,float mapradused,int num_rays){
  geometry_msgs::PolygonStamped polygon;
  polygon.header = hdr();
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

geometry_msgs::PolygonStamped get_poly_heading(float hdng_delta,int num_points,std::string frame){
  geometry_msgs::PolygonStamped poly;
  poly.header = hdr();
  poly.polygon.points.resize(num_points+1);
  poly.polygon.points[0].x = last_pose.pose.position.x;
  poly.polygon.points[0].y = last_pose.pose.position.y;
  poly.polygon.points[0].z = last_pose.pose.position.z;
  float rads_pr_i = hdng_delta /num_points;
  geometry_msgs::PointStamped p,pout;
  p.header.frame_id = frame;
  p.header.stamp = ros::Time();
  for(int i = 1; i < num_points; i++){
    float a = -hdng_delta/2 + rads_pr_i * i;
    p.point.x = 50*cos(a);
    p.point.y = 50*sin(a);
    p.point.z = last_pose.pose.position.z;
    pout = transformpoint(p,"map");
    poly.polygon.points[i].x = pout.point.x;
    poly.polygon.points[i].y = pout.point.y;
    poly.polygon.points[i].z = last_pose.pose.position.z;
  }
  poly.polygon.points[num_points].x = last_pose.pose.position.x;
  poly.polygon.points[num_points].y = last_pose.pose.position.y;
  poly.polygon.points[num_points].z = last_pose.pose.position.z;
  return poly;
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
  pos.x  = transformStamped.transform.translation.x;
  pos.y  = transformStamped.transform.translation.y;
  pos.z  = transformStamped.transform.translation.z;
  pos_yaw = tf::getYaw(transformStamped.transform.rotation);
  if(sqrt(pow(transformStamped.transform.translation.x - last_pose.pose.position.x,2)+ pow(transformStamped.transform.translation.y - last_pose.pose.position.y,2)+
    pow(transformStamped.transform.translation.z - last_pose.pose.position.z,2)) >= 1.00){
    last_pose.pose.position = pos;
    last_pose.pose.orientation = transformStamped.transform.rotation;
    last_pose.header           = transformStamped.header;
    path_visited.poses.push_back(last_pose);
    path_visited.header.stamp  = ros::Time::now();
    pub_path_visited.publish(path_visited);
  }

/*  else{
     idle_time.data = (ros::Time::now() - path_visited.header.stamp).toSec();
     if(idle_time.data > 1)
      pub_idle_time.publish(idle_time);
  }*/
}
nav_msgs::Path filter_zlvl(nav_msgs::Path pathin,int zlvl_to_filter){
  nav_msgs::Path pathout;
  pathout.header = hdr();
  for(int i = 0; i < pathin.poses.size();i++){
    if(z_lvls[zlvl_to_filter] == pathin.poses[i].pose.position.z)
      pathout.poses.push_back(pathin.poses[i]);
  }
  return pathout;
}
void targetdelta_cb(const geometry_msgs::Vector3::ConstPtr& msg){
  //if(get_dst3d(pos,target.pose.position) < 3 && path.poses.size() > blacklist.size())
//    update_target(target_next);
}

geometry_msgs::Point32 get_p32(float r, float a,float z){
  geometry_msgs::PointStamped p,pout;
  geometry_msgs::Point32 pp;
  p.point.x = r * cos(a);
  p.point.y = r * sin(a);
  p.point.z = z;
  p.header.frame_id = "base_stabilized";
  p.header.stamp = ros::Time();
  try
  {
    pout = tfBuffer.transform(p, "map");
    pp.x = pout.point.x;
    pp.y = pout.point.y;
    pp.z = pout.point.z;
  }
  catch (tf2::TransformException &ex)
  {
    ROS_WARN("Failure %s\n", ex.what());
  }
  return pp;
}
std::vector<std::vector<geometry_msgs::Point32>> scan_clusters(sensor_msgs::LaserScan scan){
  std::vector<std::vector<geometry_msgs::Point32>> clusters;
  std::vector<geometry_msgs::Point32> cluster;
  std::vector<geometry_msgs::Point32> cluster_mirror;
  float max_cluster_spacing = 5;
  int min_cluster_size      = 0;
  int clusters_minsize = 100;
  int clusters_maxsize = 0;
  float clusters_minrange = 100;
  float clusters_maxrange = 0;
  int totpoints = 0;
  geometry_msgs::Point32 p,last_p;
	for(int i = 0; i < scan.ranges.size(); i++){
    if(scan.ranges[i] > 0){
      float r = scan.ranges[i];
      float a =	scan.angle_increment * i + scan.angle_min;
      if(r < clusters_minrange)
        clusters_minrange =r;
      if(r > clusters_maxrange)
        clusters_maxrange =r;
      p = get_p32(r,a,pos.z);
      ROS_INFO("Ang/Range[%i]: %.3f / %.0f -> x,y: %.0f %.0f",i,a,r,p.x,p.y);
  /*    if(cluster.sized == 0 || (cluster.size() > 0 && get_dst2d32(last_p,p) < max_cluster_spacing)){
        cluster.push_back(p);
        cluster_mirror.push_back(get_p32(r+2,a,pos.z));
      }

    }*/

      if(cluster.size() > 0 && get_dst2d32(last_p,p) > max_cluster_spacing){
        if(cluster.size() > min_cluster_size){
          for(int k = cluster.size(); k > 0; k--){
            cluster.push_back(cluster_mirror[k]);
          }
          clusters.push_back(cluster);
        }
        if(cluster.size() > clusters_maxsize)
          clusters_maxsize = cluster.size();
        if(cluster.size() < clusters_minsize)
          clusters_minsize = cluster.size();
        totpoints += cluster.size();
        cluster.resize(0);
        cluster_mirror.resize(0);
      }
      else{
        last_p = p;
        cluster.push_back(p);
        cluster_mirror.push_back(get_p32(r+2,a,pos.z));
      }
    }
  }
  if(cluster.size() > min_cluster_size){
    for(int k = cluster.size(); k > 0; k--){
      cluster.push_back(cluster_mirror[k]);
    }
    clusters.push_back(cluster);
    if(cluster.size() > clusters_maxsize)
      clusters_maxsize = cluster.size();
    if(cluster.size() < clusters_minsize)
      clusters_minsize = cluster.size();
    totpoints += cluster.size();
  }
  ROS_INFO("PERCEPTION: Scan W/%i ranges returned %i clusters with size %i->%i range %.0f->%.0f(tot %i points)",scan.ranges.size(),clusters.size(),clusters_minsize,clusters_maxsize,clusters_minrange,clusters_maxrange,totpoints);//);
  return clusters;
}
std::vector<geometry_msgs::Point> process_clusters(std::vector<std::vector<geometry_msgs::Point32>> clusters){
  geometry_msgs::Point32 p;
  std::vector<geometry_msgs::Point> centroids_XY_A;
  for(int i = 0; i < clusters.size(); i++){
    if(clusters[i].size() >= 3){
      geometry_msgs::PolygonStamped polyout;
      polyout.header = hdr();
      polyout.polygon.points.resize(clusters[i].size());
      for(int k = 0; k < clusters[i].size(); k++){
        polyout.polygon.points[k] = clusters[i][k];
      }
      polyout.polygon.points.push_back(clusters[i][0]);
      centroids_XY_A.push_back(get_poly_centroidXY_A(polyout));
    }
    else{
      geometry_msgs::Point xy_a;
      xy_a.x = clusters[i][0].x;
      xy_a.y = clusters[i][0].y;
      xy_a.z = 0;
      centroids_XY_A.push_back(xy_a);
    }
  }
  return centroids_XY_A;
}
void floorcompletionpose_cb(const nav_msgs::Path::ConstPtr& msg){

}

void test_transform_cb(const geometry_msgs::Point::ConstPtr& msg){
  geometry_msgs::PointStamped pin,pout,pout2;
  pin.header.frame_id = "map";
  pin.header.stamp    = ros::Time();
  pin.point = *msg;
  pout = transformpoint(pin,"base_stabilized");
  float hdng0 = atan2(pin.point.y,pin.point.x);
  float hdng = atan2(pout.point.y,pout.point.x);
  float hdngd = get_shortest(hdng,hdng0);
  ROS_INFO("PERCEPTION: Transformed point: %.0f %.0f %.0f in map to %.0f %.0f %.0f in base_stabilized (hdng(d: %.3f): %.3f -> %.3f)",pin.point.x,pin.point.y,pin.point.z,pout.point.x,pout.point.y,pout.point.z,hdngd,hdng0,hdng);
  pout2 = transformpoint(pout,"map");
  ROS_INFO("PERCEPTION: Transformed point back to map: %.0f %.0f %.0f",pout2.point.x,pout2.point.y,pout2.point.z);
  if(pout2.point.x == pin.point.x && pout2.point.y == pin.point.y && pout2.point.z == pin.point.z)
    ROS_INFO("PERCEPTION: SUCCESS!!!!!!!!!!!");
  else
    ROS_INFO("PERCEPTION: FAILURE!!!!!!!!!!!");
}
void centroid_cb(const geometry_msgs::PointStamped::ConstPtr& msg){
  building_centroid = *msg;
}
void buildingstate_cb(const std_msgs::UInt8::ConstPtr& msg){
  if(msg->data == 0 && scan_buildings.ranges.size() > 0){
    std::vector<geometry_msgs::Point> building_centroids = process_clusters(scan_clusters(scan_buildings));
    float closest_building_dst = 100;
    int closest_i = 0;
    float closest_building_area = 0;
    geometry_msgs::PoseStamped building_intermediate_target;
    building_intermediate_target.pose.orientation.w = 1;
    for(int i = 0; i < building_centroids.size(); i++){
      float dst = get_dst2d(building_centroids[i],pos);
      if(dst < closest_building_dst){
        closest_i             = i;
        closest_building_area = building_centroids[i].z;
        closest_building_dst  = dst;
        building_intermediate_target.pose.position = get_p2d_offset_dst(building_centroids[i],15);
        ROS_INFO("PERCEPTION: NEW CLOSEST(BUILDING[%i] at %.0f m) ,XY[%.0f,%.0f], AREA: %.0f",i,
        closest_building_dst,closest_building_dst,building_centroids[i].x,building_centroids[i].y,closest_building_area);
      }
    }
    update_target(building_intermediate_target);
  }
  buildingstate = msg->data;
}
void state_cb(const std_msgs::UInt8::ConstPtr& msg){
  mainstate = msg->data;
}
void zlvl_cb(const std_msgs::UInt8::ConstPtr& msg){
  zlvl = msg->data;
  if(msg->data < z_lvls.size())
    target.pose.position.z = z_lvls[msg->data];
}
void scan_cleared_cb(const sensor_msgs::LaserScan::ConstPtr& msg){
  scan_cleared = *msg;
}
void scan_roofs_cb(const sensor_msgs::LaserScan::ConstPtr& msg){
  scan_roofs = *msg;
}
void scan_dangers_cb(const sensor_msgs::LaserScan::ConstPtr& msg){
  scan_dangers = *msg;
}
void scan_buildings_cb(const sensor_msgs::LaserScan::ConstPtr& msg){
  scan_buildings = *msg;
}
void init_stuff(){
  closest_obstacle.header.frame_id = path_full.header.frame_id = path_visited.header.frame_id = last_pose.header.frame_id = "map";
  target.pose.orientation.w = last_pose.pose.orientation.w = 1;
  path_visited.poses.push_back(last_pose);
  for(int i = 0; i < 40; i++){
    z_lvls.push_back(par_zjump*i);
  }
  target_next_ideal.header = hdr();
  path_full = create_path_xyz(150,2,10,par_xyjump*2);
  ROS_INFO("PERCEPTION: STUFF init");
}
void path_cb(const nav_msgs::Path::ConstPtr& msg){
  //if(get_dst2d(pos,target.pose.position) < 3){
    path = filter_zlvl(*msg,zlvl); //get_connected_path(,par_xyjump,i_start,2);
    blacklist.resize(0);
    ROS_INFO("*************PATH TARGETS************");
    for(int i = 0;i<path.poses.size(); i++){
      ROS_INFO("PATTARGETS[%i] found at %0.f,%.0f,%.0f",i,path.poses[i].pose.position.x,path.poses[i].pose.position.y,path.poses[i].pose.position.z);
    }
    int next_i =aquire_target_from_path(path,par_target_mindst,par_zfac);
    if(next_i >= 0 && next_i < path.poses.size())
      update_target(path.poses[next_i]);

      //}
}
  /*

    int i_start = closest_in_path(path,pos,);
    int ii = closest_in_path(path,target.pose.position,par_target_mindst);

    ROS_INFO("PATTARGET[%ii ] found at %0.f,%.0f,%.0f",ii,path.poses[ii].pose.position.x,path.poses[ii].pose.position.y,path.poses[ii].pose.position.z);
    update_target(path.poses[ii]);
  }*/


int main(int argc, char** argv)
{
  ros::init(argc, argv, "tb_fsmperception_node");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");
  private_nh.param("par_zjump",   par_zjump, 3.0);//*2.0);
  private_nh.param("map_sidelength",par_maprad, 300.0);
  private_nh.param("par_target_mindst",   par_target_mindst, 3.0);//*2.0);
  private_nh.param("par_hdng_penalty_base",par_hdng_penalty_base, 3.0);//*2.0);
  private_nh.param("par_zfac",   par_zfac, 3.0);//*2.0);
  private_nh.param("par_xyjump", par_xyjump, 5.0);//*2.0);
  target.pose.orientation.w = 1;
  tf2_ros::TransformListener tf2_listener(tfBuffer);
  init_stuff();

  pub_path_filtered   = nh.advertise<nav_msgs::Path>("/tb_obs/path_filtered",10);
  pub_path_visited    = nh.advertise<nav_msgs::Path>("/tb_nav/path_visited",10);
  //  pub_idle_time       = nh.advertise<std_msgs::Float64>("/tb_nav/idle_time",10);
  pub_target_approved = nh.advertise<geometry_msgs::PoseStamped>("/tb_cmd/approved",100);
  pub_target_next     = nh.advertise<geometry_msgs::PointStamped>("/tb_setp/target_next",100);
//
  ros::Publisher path_full_pub= nh.advertise<nav_msgs::Path>("/tb_nav/full_path",10);
  ros::Publisher polysafe_pub = nh.advertise<geometry_msgs::PolygonStamped>("/tb_obs/poly_safe",100);
  ros::Publisher polyhdng_pub = nh.advertise<geometry_msgs::PolygonStamped>("/tb_obs/poly_hdng",100);
  ros::Publisher polyhdnglast_pub = nh.advertise<geometry_msgs::PolygonStamped>("/tb_obs/poly_hdng_relative",100);
  ros::Publisher pub_closest_obstacle = nh.advertise<geometry_msgs::PointStamped>("/tb_obs/closest",100);
  ros::Publisher pub_centroid_initial = nh.advertise<geometry_msgs::PointStamped>("/tb_bld/centroid_initial",100);
//ros::Publisher pub_target_ideal     = nh.advertise<geometry_msgs::PointStamped>("/tb_setp/target_next_ideal",10);

  ros::Subscriber s1  = nh.subscribe("/octomap_full",1,octomap_callback);
//  ros::Subscriber s3  = nh.subscribe("/test_pointtransform",1,test_transform_cb);
  ros::Subscriber s33  = nh.subscribe("/tb_path",1,&path_cb);

  ros::Subscriber s4  = nh.subscribe("/tb_fsm/building_state",1,buildingstate_cb);
  ros::Subscriber s7  = nh.subscribe("/tb_bld/building_centroid",1,centroid_cb);
  ros::Subscriber s5  = nh.subscribe("/tb_fsm/main_state",100,&state_cb);

  ros::Subscriber s8  = nh.subscribe("/tb_fsm/altlvl",100,&zlvl_cb);
  ros::Subscriber s9  = nh.subscribe("/tb_bld/path_floor",100,&floorcompletionpose_cb);
  ros::Subscriber s10 = nh.subscribe("/tb_setp/target_delta",100,&targetdelta_cb);

  ros::Subscriber s11 = nh.subscribe("/tb_obs/scan_cleared",10, &scan_cleared_cb);
  ros::Subscriber s12 = nh.subscribe("/tb_obs/scan_roofs",10, &scan_roofs_cb);
  ros::Subscriber s13 = nh.subscribe("/tb_obs/scan_dangers",10, &scan_dangers_cb);
  ros::Subscriber s14 = nh.subscribe("/tb_obs/scan_buildings",10, &scan_buildings_cb);

//  ros::Subscriber s10 = nh.subscribe("/tb_path",100,&path_cb);

  //ros::Publisher cmdarm_pub = nh.advertise<std_msgs::Float64>("/tb_cmd/arm_pitch",10);
  std_msgs::Float64 cmdarm_msg;
  nav_msgs::Path filtered_testpath;
  filtered_testpath.header = hdr();

  ros::Rate rate(2.0);
  ros::Time start = ros::Time::now();
  float centroid_sides = 20;
  int count = 0;
  while(ros::ok()){
    if(got_map){
      checktf();
      //create_path(par_maprad,par_takeoffaltitude,centroid_sides);
      poly_heading     = get_poly_heading(M_PI/2,16,"base_stabilized");
      poly_heading_rel = get_poly_heading(M_PI/2,16,"target_last");
      poly_safe        = get_poly_surround(10,50,36);
      polyhdng_pub.publish(poly_heading);
      polyhdnglast_pub.publish(poly_heading_rel);
      polysafe_pub.publish(poly_safe);
//      filtered_testpath = path_testing(path_full,count);
      if(filtered_testpath.poses.size() > 2){
        pub_path_filtered.publish(filtered_testpath);
        int next_target = (filtered_testpath,par_target_mindst,par_zfac);
        update_next_target(filtered_testpath.poses[next_target]);
        pub_target_next.publish(filtered_testpath.poses[next_target]);
         //
         //i_new2 = get_target(pathin,par_target_mindst,zfac);
         //pub_path_filter.publish(path_filtered)
      }
      pub_closest_obstacle.publish(closest_obstacle);
      path_full_pub.publish(path_full);
      pub_target_next.publish(target_next_ideal);
    }
    rate.sleep();
    ros::spinOnce();
  }
  return 0;
}
