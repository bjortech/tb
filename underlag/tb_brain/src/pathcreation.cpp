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
tf2_ros::Buffer tfBuffer;

using namespace octomap;
using namespace std;
shared_ptr<DynamicEDTOctomap> edf_ptr;
AbstractOcTree* abs_octree;
shared_ptr<OcTree> octree;

int buildingstate,zlvl,xmin,ymin,zmin,xmax,ymax,zmax,range_x,range_y,range_z;
bool par_unknownAsOccupied,got_map,done;
double par_xyjump,poly_bb_maprad,par_zrange,par_maprad,par_maxobs,par_minobs,par_zjump,par_min_update_interval;

ros::Time last_update_and_publish_path;
ros::Publisher pub_path_bbpoly,pub_path;
geometry_msgs::Point bbmin_custom,bbmax_custom,bbmin_octree,bbmax_octree,pos,poly_bb_centroid,last_pos;
nav_msgs::Path path_notvstd_full,path_inpoly_full,path_full,path_znlvls_full,path_visited;
geometry_msgs::PolygonStamped poly_bb,polysafe,polyvstd,polyhdng;

std::vector<int> z_lvls;
std::vector<nav_msgs::Path> paths_at_z;
std::vector<nav_msgs::Path> paths_notvstd_at_z;

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
int get_zn(int z){
  for(int i = 0; i < z_lvls.size()-1; i++){
    if(z_lvls[i] <= z && z_lvls[i+1] >= z){
      return i;
    }
  }
  return 0;
}
geometry_msgs::Point32 get_poly_centroid32(geometry_msgs::PolygonStamped polyin){
    geometry_msgs::Point32 centroid;
    double signedArea = 0.0;
    double x0 = 0.0; // Current vertex X
    double y0 = 0.0; // Current vertex Y
    double x1 = 0.0; // Next vertex X
    double y1 = 0.0; // Next vertex Y
    double a = 0.0;  // Partial signed area

    // For all vertices
    for (int i=0; i<polyin.polygon.points.size(); ++i)
    {
        x0 = polyin.polygon.points[i].x;
        y0 = polyin.polygon.points[i].y;
        x1 = polyin.polygon.points[(i+1) % polyin.polygon.points.size()].x;
        y1 = polyin.polygon.points[(i+1) % polyin.polygon.points.size()].y;
        a = x0*y1 - x1*y0;
        signedArea += a;
        centroid.x += (x0 + x1)*a;
        centroid.y += (y0 + y1)*a;
    }

    signedArea *= 0.5;
    centroid.x /= (6.0*signedArea);
    centroid.y /= (6.0*signedArea);

    return centroid;
}

geometry_msgs::Point get_poly_centroid(geometry_msgs::PolygonStamped polyin){
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

    return centroid;
}

float get_dst2d(geometry_msgs::Point p1, geometry_msgs::Point p2){
  return(sqrt(pow(p1.x-p2.x,2)+pow(p1.y-p2.y,2)));
}
float get_dst3d(geometry_msgs::Point p1, geometry_msgs::Point p2){
  return(sqrt(pow(p1.x-p2.x,2)+pow(p1.y-p2.y,2)+pow(p1.z-p2.z,2)));
}
std_msgs::Header hdr(){
  std_msgs::Header header;
  header.frame_id = "map";
  header.stamp    = ros::Time::now();
  return header;
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
float get_poly_limits(geometry_msgs::PolygonStamped polyin,geometry_msgs::Point centroid){
  std::vector<geometry_msgs::Point>out;
  double ltot,dx,dy,dz;
  geometry_msgs::Point lim0,lim1,dtot;
  for(int i = 0; i < (polyin.polygon.points.size()-1); i++){
    geometry_msgs::Point rel_pos;
    rel_pos.x = polyin.polygon.points[i].x - centroid.x;
    rel_pos.y = polyin.polygon.points[i].y - centroid.y;
    rel_pos.z = polyin.polygon.points[i].z - centroid.z;
    if(rel_pos.x < lim0.x)
      lim0.x = rel_pos.x;
    if(rel_pos.y < lim0.y)
      lim0.y = rel_pos.y;
    if(rel_pos.z < lim0.z)
      lim0.z = rel_pos.z;
    if(rel_pos.x > lim1.x)
      lim1.x = rel_pos.x;
    if(rel_pos.y > lim1.y)
      lim1.y = rel_pos.y;
  }
  float maprad_max = fmax(fmax(abs(lim0.x),abs(lim1.x)),fmax(abs(lim0.y),abs(lim1.y)));
  float dst_x,dst_y,dst_z;
  dst_x = lim1.x - lim0.x;
  dst_y = lim1.y - lim0.y;
  dst_z = lim1.z - lim0.z;
  ROS_INFO("PATHUTIL: Poly BoundingBox(min,max):  x: %.2f -> %.2f,y: %.2f -> %.2f,z: %.2f -> %.2f",lim0.x, lim0.y, lim0.z, lim1.x, lim1.y, lim1.z);
  ROS_INFO("PATHUTIL: maprad_max: %.2f range axes:x: %.2f         y: %.2f         z: %.2f",maprad_max,dst_x, dst_y, dst_z);
  out.push_back(lim0);
  out.push_back(lim1);
  return maprad_max;
}
std::vector<int> getinpath_indexes_inrad(nav_msgs::Path pathin,int i_to_check,float radians){
  std::vector<int> vec_out;
  if(pathin.poses.size() == 0){
    ROS_INFO("PathAdmin: pathin is empty");
    return vec_out;
  }
  for(int i = 0; i < pathin.poses.size(); i++){
    float dist = get_dst2d(pathin.poses[i].pose.position,pathin.poses[i_to_check].pose.position);
    if(dist <= radians && dist > 0)
      vec_out.push_back(i);
  }
  return vec_out;
}

bool dst_point_in_path_lim(nav_msgs::Path pathin,geometry_msgs::Point pin,float lim){
  float res,dst;
  if(pathin.poses.size() == 0)
    return true;
  for(int i = 0; i < pathin.poses.size(); i++){
     if(get_dst2d(pathin.poses[i].pose.position,pin) < lim && abs(pathin.poses[i].pose.position.z - pin.z) < par_zjump)
        return false;
  }
  return true;
}

bool update_edto(geometry_msgs::Point midpoint,float collision_radius,float maprad,float z0,float z1,bool unknownAsOccupied){
    octree.get()->getMetricMin(bbmin_octree.x,bbmin_octree.y,bbmin_octree.z);
    octree.get()->getMetricMax(bbmax_octree.x,bbmax_octree.y,bbmax_octree.z);

    z0 = fmin(z0,bbmax_octree.z-1);
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
    //ROS_INFO("update_edto FAILED: xmin: %i, ymin: %i, zmin: %i, xmax: %i, ymax: %i, zmax: %i, range_x: %i, range_y: %i, range_z: %i ",xmin,ymin,zmin,xmax,ymax,zmax,range_x,range_y,range_z);

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
float getinpath_closestdst(nav_msgs::Path pathin,geometry_msgs::PoseStamped pose_to_check,bool use_3d){
  float lowest_dist = 1000;  float dst;
  if(pathin.poses.size() == 0){
    ROS_INFO("PATHUTIL:   PathAdmin: pathin is empty");
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
  ROS_INFO("PATHUTIL: constrain_path_bbpnts: pathin %i poses, x %.0f y %.0f z %.0f -> x %.0f y %.0f z %.0f: Pathout: %i poses",pathin.poses.size(),bbmin.x,bbmin.y,bbmin.z,bbmax.x,bbmax.y,bbmax.z,pathout.poses.size());
  return pathout;
}
nav_msgs::Path constrain_path_zlvl(nav_msgs::Path pathin, int zlvl_min,int zlvl_max){
  nav_msgs::Path pathout;
  pathout.header.frame_id="map";
  if(pathin.poses.size() == 0)
    return pathout;
  geometry_msgs::Point bbmin,bbmax;
  bbmin.x = -1000; bbmin.y = -1000; bbmax.x = 1000; bbmax.y = 1000;
  bbmin.z = z_lvls[zlvl_min]-1;
  bbmax.z = z_lvls[zlvl_max]+1;
  pathout = constrain_path_bbpnts(pathin,bbmin,bbmax);
  ROS_INFO("PATHUTIL: constrain_path_zlvl: pathin %i poses, lvls: %i -> %i pathout: %i poses",pathin.poses.size(),zlvl_min,zlvl_max,pathout.poses.size());
  return pathout;
}
nav_msgs::Path constrain_path_bbpoly(nav_msgs::Path pathin,geometry_msgs::PolygonStamped poly_bb){
  nav_msgs::Path pathout;
  pathout.header = hdr();
  geometry_msgs::Point centroid;
  centroid = get_poly_centroid(poly_bb);
  if(pathin.poses.size() == 0 || poly_bb.polygon.points.size() == 0)
    return pathin;
  for(int i = 0; i < pathin.poses.size(); i++){
    if(in_poly(poly_bb,pathin.poses[i].pose.position.x,pathin.poses[i].pose.position.y)){
      float pos_hdng  = get_hdng(pathin.poses[i].pose.position,centroid);
      float pose_hdng = tf::getYaw(pathin.poses[i].pose.orientation);
      ROS_INFO("PATHUTIL:   Pos hdng: %.2f pose hdng: %.2f",pos_hdng,pose_hdng);
      pathout.poses.push_back(pathin.poses[i]);
      ROS_INFO("PATHUTIL: constrain_path_bbpoly: pathin %i poses, polyin %i points pathout: %i poses",pathin.poses.size(),poly_bb.polygon.points.size(),pathout.poses.size());

    }
  }
  return pathout;
}
float find_max(nav_msgs::Path pathin,std::vector<int> indexes){
  float res;
  int res_i;
  for(int i = 0; i < indexes.size(); i++){
    if(pathin.poses[indexes[i]].pose.position.z > res){
      res = pathin.poses[indexes[i]].pose.position.z;
      res_i = i;
    }
  }
  return res_i;
}
nav_msgs::Path constrain_path_lvls(nav_msgs::Path pathin,int minimum_number_of_levels,bool top_only){
  nav_msgs::Path pathout;
  pathout.header = hdr();
  if(pathin.poses.size() == 0)
    return pathout;
  for(int i = 0; i < pathin.poses.size(); i++){
    std::vector<int> indexes_in_rad;
    indexes_in_rad = getinpath_indexes_inrad(pathin,i,4);
    if(indexes_in_rad.size() >= minimum_number_of_levels){
      int ri = find_max(pathin,indexes_in_rad);
      if(top_only && ri == i){
        pathout.poses.push_back(pathin.poses[i]);
      }
      if(!top_only)
        pathout.poses.push_back(pathin.poses[i]);
    }
  }
  ROS_INFO("PATHUTIL: constrain_path_lvls: pathin %i poses, minlvls: %i pathout: %i poses",pathin.poses.size(),minimum_number_of_levels,pathout.poses.size());
  return pathout;
}
nav_msgs::Path constrain_path_vstd(nav_msgs::Path pathin,nav_msgs::Path pathin_vstd,float cutoff_dst,bool use_3d){
  nav_msgs::Path pathout;
  pathout.header = hdr();
  if(fmin(pathin.poses.size(),pathin_vstd.poses.size()) == 0){
    return pathin;
  }
  ROS_INFO("PATHUTIL:   Pathin size: cand %i vstd %i cutoff %.2f",pathin.poses.size(),pathin_vstd.poses.size(),cutoff_dst);
  for(int i = 0; i < pathin.poses.size(); i++){
    if(getinpath_closestdst(pathin_vstd,pathin.poses[i],use_3d) > cutoff_dst)
      pathout.poses.push_back(pathin.poses[i]);
  }
  ROS_INFO("PATHUTIL:   PathSegmentation VSTD: %i of %i poses not visited",pathout.poses.size(),pathin.poses.size());
  return pathout;
}
nav_msgs::Path remove_visited(nav_msgs::Path pathin){
  return constrain_path_vstd(pathin,path_visited,5,false);
}
/******** END ******************** CONSTRAIN PATH FUNCTIONS   ***************** END ******************/


void update_path(geometry_msgs::Point midpoint,float maprad){
  ros::Time t0 = ros::Time::now();
  nav_msgs::Path path_update;
  path_update.header.frame_id = "map";
  if(got_map){
    octree.get()->getMetricMin(bbmin_octree.x,bbmin_octree.y,bbmin_octree.z);
    octree.get()->getMetricMax(bbmax_octree.x,bbmax_octree.y,bbmax_octree.z);

    int oct_zlvl_min = get_zn(int(bbmin_octree.z));
    int oct_zlvl_max = get_zn(int(bbmax_octree.z));

  //  ROS_INFO("oct_zlvl_min %.2f %.2f %i oct_zlvl_max %i",bbmin_octree.z,bbmax_octree.z,oct_zlvl_min,oct_zlvl_max);
    //int z0 = fmax(,zlvl0);
    //int z1 = fmin(,zlvlend);
//    ROS_INFO("oct_zlvl_min %i oct_zlvl_max %i",oct_zlvl_min,oct_zlvl_max);
//    ROS_INFO("z0 %i z1 %i",z0,z1);
//    if(z0 < z1){
//      ROS_INFO("OK");
//    }
//    else{
//      z0 = zlvl;
  //    z1 = zlvl+1;

  //      ROS_INFO("z0: %i from %.0f vs %i",z0,bbmin_octree.z/par_zjump+1,zlvl0);
  //      ROS_INFO("z1: %i from %.0f vs %i",z1,bbmax_octree.z/par_zjump-1,zlvl0);
  //  }
  last_pos = pos;

  int z0 = fmax(oct_zlvl_min,zlvl-2);
  int z1 = fmax(oct_zlvl_min,zlvl+2);
    for(int zn = z0; zn < z1; zn++){
      int z = z_lvls[zn];
      if(z > bbmin_octree.z && z < bbmax_octree.z){
        update_edto(midpoint,par_maxobs+1,fmax(maprad,30),z_lvls[zn]-0.5,z_lvls[zn]+0.5,par_unknownAsOccupied);
        for(int y = ymin; y < ymax; y++){
          for(int x = xmin; x < xmax; x++){
            geometry_msgs::PoseStamped ps;
            ps.header = hdr();
            ps.pose.position.x = x;
            ps.pose.position.y = y;
						if(in_poly(polysafe,ps.pose.position.x,ps.pose.position.y)){
	            ps.pose.position.z = z_lvls[zn];
	            if(dst_point_in_path_lim(path_full,ps.pose.position,5)){
	              octomap::point3d p(x,y,z_lvls[zn]);
	              octomap::point3d closestObst;
	              float d;
	              edf_ptr.get()->getDistanceAndClosestObstacle(p,d,closestObst);
	              if(round((par_minobs + par_maxobs)/2) == round(d)){
	                ps.pose.orientation = tf::createQuaternionMsgFromYaw(atan2(closestObst.y() - y,closestObst.x() - x));
	                path_full.poses.push_back(ps);
	                paths_at_z[zn].poses.push_back(ps);
	                path_update.poses.push_back(ps);
	              }
	            }
						}
          }
        }
      }
    }
    float dt = (ros::Time::now() - t0).toSec();
    ROS_INFO("PATHUTIL: update_path[%i poses %.5f s]: zlvls: %i -> i%  maprad: %.2f",path_update.poses.size(),dt,z0,z1,maprad);
  }
}

void update_polybb(geometry_msgs::PolygonStamped polyin){
  ros::Time t0 = ros::Time::now();
  nav_msgs::Path pathout;
  poly_bb_centroid = get_poly_centroid(polyin);
  poly_bb_maprad   = get_poly_limits(polyin,poly_bb_centroid);
  poly_bb          = polyin;
  update_path(poly_bb_centroid,poly_bb_maprad);
  pathout = constrain_path_bbpoly(path_full,poly_bb);
  float dt = (ros::Time::now() - t0).toSec();
  ROS_INFO("PATHUTIL: Updating Poly BoundingBox path [%i poses %.5f s] within: %i poses. New poly_bb: %i points, centroid: [x: %.0f y: %.0f z: %.0f] maprad_max: %.0f",pathout.poses.size(),dt,poly_bb.polygon.points.size(),poly_bb_centroid.x,poly_bb_centroid.y,poly_bb_centroid.z,poly_bb_maprad);
  pub_path_bbpoly.publish(pathout);
}
void polybb_cb(const geometry_msgs::PolygonStamped::ConstPtr& msg){
  if(msg->polygon.points.size() != poly_bb.polygon.points.size()){
    update_polybb(*msg);
  }
}
void polysafe_cb(const geometry_msgs::PolygonStamped::ConstPtr& msg){
    polysafe = *msg;
}
void polyhdng_cb(const geometry_msgs::PolygonStamped::ConstPtr& msg){
    polyhdng = *msg;
}
void polyvstd_cb(const geometry_msgs::PolygonStamped::ConstPtr& msg){
    polyvstd = *msg;
}
void zlvl_cb(const std_msgs::UInt8::ConstPtr& msg){
  zlvl = msg->data;
}

void visited_cb(const nav_msgs::Path::ConstPtr& msg){
  path_visited    = *msg;
  pos = path_visited.poses[path_visited.poses.size() - 1].pose.position;
}

nav_msgs::Path get_zlvls(int z0,int z1){
  nav_msgs::Path pathout;
  pathout.header = hdr();
  if(z0 == 0 && z1 >= paths_at_z.size())
    pathout = path_full;
  if(z0 != z1){
    for(int i = z0; i < z1; i++){
      for(int k = 0; k < paths_at_z[i].poses.size(); k++){
        pathout.poses.push_back(paths_at_z[i].poses[k]);
      }
    }
  }
  else
    pathout = paths_at_z[z0];
  ROS_INFO("Path merge_zlvls (%i -> %i): %i poses",z0,z1,pathout.poses.size());
  return pathout;
}

void update_and_publish_path(){
  nav_msgs::Path pathout;
  pathout.header = hdr();
  last_update_and_publish_path = ros::Time::now();
  /*if(buildingstate < 2){
    update_path(pos,35);
    pathout = remove_visited(constrain_path_lvls(get_zlvls(zlvl-2,zlvl+2),1,false));
  }*/
  //if(buildingstate == 2 && get_dst3d(pos,last_pos) > 5){
    update_path(pos,35);
//    pathout = remove_visited(paths_at_z[zlvl]);
//  }
//  else{
    pathout = remove_visited(get_zlvls(zlvl-2,zlvl+2));
//  }
  float dt = (ros::Time::now() - last_update_and_publish_path).toSec();
  ROS_INFO("PATHUTIL: Update & Publish Pathh [%i poses %.5f s]",pathout.poses.size(),dt);
  pub_path.publish(pathout);
}

void check_if_need_to_update(){
  if((ros::Time::now() - last_update_and_publish_path).toSec() > par_min_update_interval)
    update_and_publish_path();
}

void buildingstate_cb(const std_msgs::UInt8::ConstPtr& msg){
  buildingstate = msg->data;
}
void missionstate_cb(const std_msgs::UInt8::ConstPtr& msg){
  if(msg->data == 1)
    check_if_need_to_update();
}
/*
void request_path_cb(const geometry_msgs::Vector3Stamped::ConstPtr& msg){
  ros::Time t0 = ros::Time::now();
  ROS_INFO("Path %s %.0f requested",msg->header.frame_id.c_str(),msg->vector.x,msg->vector.y,msg->vector.z);
  int zn;
  float maprad = msg->vector.x;
  int z0       = msg->vector.y;
  int z1       = msg->vector.z;
  if(z1 == 0 && z0 > 0){
    z1 = zlvl + z0;
    z0 = zlvl - z0;
    ROS_INFO("PATHUTIL: Using z as range, not abs values (z_lvls)");
  }
  nav_msgs::Path pathout;
  pathout.header = hdr();
  nav_msgs::Path path_updates;
  if(msg->header.frame_id == "update")
    pathout = update_path(z0,z1,maprad);
  else if(msg->header.frame_id == "zn_top" || msg->header.frame_id == "zn"){
    bool top_only; int zlvls_min = fmax(1,int(msg->vector.y));
    if(msg->header.frame_id == "zn_top")
      top_only = true;
      pathout = constrain_path_lvls(path_full,zlvls_min,top_only);
  }
  else if(msg->header.frame_id == "tar"){
    if(msg->vector.x == 0)
      maprad = 40;
    path_updates = update_path(maprad,zlvl,zlvl);
    pathout = remove_visited(get_zlvls(zlvl,zlvl));
  }
  else if(msg->header.frame_id == "full" || msg->header.frame_id == "full_notvstd" || msg->header.frame_id == "bbpoly"){
    if(msg->vector.x == 0)
      maprad = 1000;
    if(msg->vector.z == 0)
      z1 = 100;
    if(msg->header.frame_id == "bbpoly")
      pathout = constrain_path_bbpoly(get_zlvls(z0,z1),poly_bb);
    if(msg->header.frame_id == "full_notvstd")
      pathout = remove_visited(get_zlvls(z0,z1));
    if(msg->header.frame_id == "full")
      pathout = get_zlvls(z0,z1);
  }

  float dt = (ros::Time::now() - t0).toSec();
  ROS_INFO("PATHUTIL: Update[%i poses %.5f s]",pathout.poses.size(),dt);
  pub_path.publish(pathout);
}*/

/*
void idletime_cb(const std_msgs::Float64::ConstPtr& msg){
  if(msg->data >= 2 && (ros::Time::now() - path_visited.header.stamp).toSec() >= 2){
    update_path();
  }

}
void bldstate_cb(const std_msgs::UInt8::ConstPtr& msg){
  int min_num_lvls = 1;
  bool use3dnot2d  = false;
  float cutoff_dst = 11.0;
  if(msg->data == 1){
    ROS_INFO("PATHUTILIZER: buildstate: %i, constrain zlvl: %i",msg->data,zlvl);
    pub_bldpath.publish(constrain_path_zlvl(remove_visited(path_znlvls_full),zlvl-2,zlvl+2));
  //  constrain_path_vstd(constrain_path_zlvl(constrain_path_lvls(path_full,min_num_lvls),zlvl,zlvl+1),path_visited,cutoff_dst/2,use3dnot2d));
  }
  else if(msg->data == 2){
    pub_bldpath.publish(path_notvstd_full);
  }
  if(msg->data == 3 && poly_bb.polygon.points.size() > 2){
    ROS_INFO("PATHUTILIZER: buildstate: %i, constrainbbpoly %i long, constrainvstd: path_vstd %i long,cutoff %.1f use3dnot2d: %i",msg->data,poly_bb.polygon.points.size(),path_visited.poses.size(),cutoff_dst/2,use3dnot2d);
    pub_bldpath.publish(remove_visited(path_inpoly_full));
  }
  if(msg->data == 4){
    ROS_INFO("PATHUTILIZER: buildstate: %i, constrain#TODO",msg->data);
  }
}
*/



//pub_bldpath,
//pub_bldpath         = nh.advertise<nav_msgs::Path>("/tb_path/building_path",100);

int main(int argc, char **argv)
{
  ros::init(argc, argv, "tb_fsmpathcreator_node");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");
  private_nh.param("par_zrange",  par_zrange, 2.0);//*2.0);
  private_nh.param("par_maprad",  par_maprad, 30.0);//*2.0);
  private_nh.param("par_maxobs",  par_maxobs, 20.0);//*2.0);
  private_nh.param("par_zjump",   par_zjump, 3.0);//*2.0);
  private_nh.param("par_minobs",  par_minobs, 2.0);//*2.0);
  private_nh.param("par_xyjump", par_xyjump, 5.0);//*2.0);
  private_nh.param("par_min_update_interval",  par_min_update_interval, 4.0);//*2.0);

  nav_msgs::Path path_template;
  path_template.header = hdr();
  path_znlvls_full.header = hdr();
  path_inpoly_full.header = hdr();
  path_notvstd_full.header = hdr();
  path_full.header = hdr();
  for(int i = 0; i < 40; i++){
    z_lvls.push_back(par_zjump*i);
    paths_at_z.push_back(path_template);
    paths_notvstd_at_z.push_back(path_template);
  }
  done = true;
  tf2_ros::TransformListener tf2_listener(tfBuffer);
  ros::Subscriber s0 = nh.subscribe("/octomap_full",1,octomap_callback);
  ros::Subscriber s2 = nh.subscribe("/tb_nav/path_visited",1,&visited_cb);
	ros::Subscriber s1 = nh.subscribe("/tb_obs/polygon",1,polybb_cb);
	ros::Subscriber s3 = nh.subscribe("/tb_obs/poly_safe",1,polysafe_cb);
	ros::Subscriber s4 = nh.subscribe("/tb_obs/poly_hdng",1,polyhdng_cb);
	ros::Subscriber s6 = nh.subscribe("/tb_obs/poly_vstd",1,polyvstd_cb);
  ros::Subscriber s7 = nh.subscribe("/tb_fsm/altlvl",1,&zlvl_cb);
//  ros::Subscriber s7 = nh.subscribe("/tb_path/request_path",100,&request_path_cb);
  ros::Subscriber su = nh.subscribe("/tb_bld/building_state",1,buildingstate_cb);
  ros::Subscriber s89 = nh.subscribe("/tb_fsm/mission_state",1,missionstate_cb);

//  ros::Subscriber s3 = nh.subscribe("/tb_bld/building_state",1,&bldstate_cb);
//  ros::Subscriber s4 = nh.subscribe("/tb_nav/idle_time",100,&idletime_cb);

pub_path            = nh.advertise<nav_msgs::Path>("/tb_path",100);
pub_path_bbpoly     = nh.advertise<nav_msgs::Path>("/tb_path/bbpoly_znlvls",100);
  ros::spin();
  return 0;
}
