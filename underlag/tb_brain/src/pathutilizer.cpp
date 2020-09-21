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
ros::Publisher pub_target_approved;

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
std::vector<int> getinpath_indexes_inrad(nav_msgs::Path pathin,int i_to_check,float radians,float dz_max,int max_neighbours){
  std::vector<int> vec_out;
	std::vector<float> dists;

  if(pathin.poses.size() == 0){
    return vec_out;
  }
  for(int i = 0; i < pathin.poses.size(); i++){
    float dist = get_dst2d_zlim(pathin.poses[i].pose.position,pathin.poses[i_to_check].pose.position,dz_max);
		dists.push_back(dist);
    if(dist <= radians && dist > 0)
			if(vec_out.size() == max_neighbours){
				for(int k = 0;  k < max_neighbours;k++){
					if(dist < dists[k]){
						dists[k] = dist;
						vec_out[k] = i;
					}
				}
			}
      vec_out.push_back(i);
  }
  return vec_out;
}
std::vector<std::vector<int>> getinpath_neighbours(nav_msgs::Path pathin,float radians,float dz_max){
  std::vector<int> neighbours;
  std::vector<std::vector<int>> neighbours_at_index;
	int max_neighbours = 2;
  //getinpath_neighbours(pathin,radians);
  if(pathin.poses.size() == 0){
    ROS_INFO("PathAdmin: pathin is empty");
    return neighbours_at_index;
  }
  for(int i = 0; i < pathin.poses.size(); i++){
    neighbours = getinpath_indexes_inrad(pathin,i,radians,dz_max,max_neighbours);
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
}
bool in_blacklist(int itarget){
  for(int i = 0; i < blacklist.size(); i++){
    if(blacklist[i] == itarget)
      return true;
  }
  return false;
}

void update_target(geometry_msgs::PoseStamped ps){
  target = ps;
  targetcmds_sent.push_back(ps);
  pub_target_approved.publish(ps);
}

int aquire_target_from_path(nav_msgs::Path pathin,float min_dst,float zfac){
  int i,i_new,res;
  i     = closest_in_path(pathin,target.pose.position,par_target_mindst);
  //i_new = get_target(pathin,min_dst,zfac);
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
nav_msgs::Path filter_zlvl(nav_msgs::Path pathin,int zlvl_to_filter){
  nav_msgs::Path pathout;
  pathout.header = hdr();
  for(int i = 0; i < pathin.poses.size();i++){
    if(z_lvls[zlvl_to_filter] == pathin.poses[i].pose.position.z)
      pathout.poses.push_back(pathin.poses[i]);
  }
  return pathout;
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
void path_cb(const nav_msgs::Path::ConstPtr& msg){
  path = *msg;//
		//filter_zlvl(*msg,zlvl); //get_connected_path(,par_xyjump,i_start,2);
  blacklist.resize(0);
  ROS_INFO("*************PATH TARGETS************");
  for(int i = 0;i<path.poses.size(); i++){
    ROS_INFO("PATTARGETS[%i] found at %0.f,%.0f,%.0f",i,path.poses[i].pose.position.x,path.poses[i].pose.position.y,path.poses[i].pose.position.z);
  }
  int next_i =aquire_target_from_path(path,par_target_mindst,par_zfac);
  if(next_i >= 0 && next_i < path.poses.size())
    update_target(path.poses[next_i]);
}
void visited_cb(const nav_msgs::Path::ConstPtr& msg){
  path_visited    = *msg;
}
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
  tf2_ros::TransformListener tf2_listener(tfBuffer);
	target.pose.orientation.w = 1;
  for(int i = 0; i < 40; i++){
    z_lvls.push_back(par_zjump*i);
  }
  pub_target_approved = nh.advertise<geometry_msgs::PoseStamped>("/tb_cmd/approved",100);
//
	ros::Subscriber s9  = nh.subscribe("/tb_nav/path_visited",10, visited_cb);
  ros::Subscriber s33 = nh.subscribe("/tb_path",1,&path_cb);
  ros::Subscriber s5  = nh.subscribe("/tb_fsm/main_state",100,&state_cb);
  ros::Subscriber s8  = nh.subscribe("/tb_fsm/altlvl",100,&zlvl_cb);

  ros::Subscriber s11 = nh.subscribe("/tb_obs/scan_cleared",10, &scan_cleared_cb);
  ros::Subscriber s12 = nh.subscribe("/tb_obs/scan_roofs",10, &scan_roofs_cb);
  ros::Subscriber s13 = nh.subscribe("/tb_obs/scan_dangers",10, &scan_dangers_cb);
  ros::Subscriber s14 = nh.subscribe("/tb_obs/scan_buildings",10, &scan_buildings_cb);

	ros::spin();
  return 0;
}
