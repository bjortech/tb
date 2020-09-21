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

int proximity_range,proximitycount,restcount,zlvl,xmin,ymin,zmin,xmax,ymax,zmax,range_x,range_y,range_z,zmin_global,zmax_global;
bool par_unknownAsOccupied,got_map,done;
double par_zrange,par_maprad,par_maxobs,par_minobs,last_yaw,par_zjump;

ros::Publisher pub_path_notvstd,pub_path_inpoly,pub_path,pub_path_znlvls,pub_path_atzlvl;
geometry_msgs::Point bbmin_custom,bbmax_custom,bbmin_octree,bbmax_octree,pos;
nav_msgs::Path path_notvstd_full,path_inpoly_full,path_full,path_znlvls_full,path_visited;
geometry_msgs::PolygonStamped poly_bb;

std::vector<int> z_lvls;
std::vector<nav_msgs::Path> paths_at_z;
std::vector<nav_msgs::Path> paths_notvstd_at_z;
bool everyother;
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
     if(get_dst3d(pathin.poses[i].pose.position,pin) < lim)
        return false;
  }
  return true;
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
nav_msgs::Path constrain_path_lvls(nav_msgs::Path pathin,int minimum_number_of_levels){
  nav_msgs::Path pathout;
  pathout.header = hdr();
  if(pathin.poses.size() == 0)
    return pathout;
  for(int i = 0; i < pathin.poses.size(); i++){
    std::vector<int> indexes_in_rad;
    indexes_in_rad = getinpath_indexes_inrad(pathin,i,4);
    if(indexes_in_rad.size() >= minimum_number_of_levels)
      pathout.poses.push_back(pathin.poses[i]);
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

void update_path(int zn)
{
  update_edto(pos,par_maxobs+1,par_maprad,z_lvls[zn]-0.5,z_lvls[zn]+0.5,par_unknownAsOccupied);
  for(int y = ymin; y < ymax; y++){
    for(int x = xmin; x < xmax; x++){
      geometry_msgs::PoseStamped ps;
      ps.header = hdr();
      ps.pose.position.x = x;
      ps.pose.position.y = y;
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
        }
      }
    }
  }
}


void update_path_assist(){
  done = false;
  octree.get()->getMetricMin(bbmin_octree.x,bbmin_octree.y,bbmin_octree.z);
  octree.get()->getMetricMax(bbmax_octree.x,bbmax_octree.y,bbmax_octree.z);
  int min_zlvl_total = bbmin_octree.z/par_zjump;
  int max_zlvl_total = bbmax_octree.z/par_zjump;
  int min_zlvl_proximity = fmax(min_zlvl_total,zlvl-par_zrange);
  int max_zlvl_proximity = fmin(max_zlvl_total,zlvl+par_zrange);
  proximitycount++;
  restcount++;
  ros::Time t0 = ros::Time::now();

  if(proximitycount == max_zlvl_proximity)
    proximitycount = min_zlvl_proximity;
  if(restcount == max_zlvl_total)
    restcount = min_zlvl_total;
  int s0 = path_full.poses.size();
  if(everyother){
    everyother = false;
    update_path(proximitycount);
  }
  else{
    everyother = true;
    update_path(restcount);
    paths_notvstd_at_z[restcount] = remove_visited(paths_at_z[restcount]);
  }

  float dt = (ros::Time::now() - t0).toSec();
  ROS_INFO("Time: %.5f dt everyother: %i",dt,everyother);
  done = true;
}
void polybb_cb(const geometry_msgs::PolygonStamped::ConstPtr& msg){
  poly_bb         = *msg;
}
void zlvl_cb(const std_msgs::UInt8::ConstPtr& msg){
  zlvl = msg->data;
}
void visited_cb(const nav_msgs::Path::ConstPtr& msg){
  pos = path_visited.poses[path_visited.poses.size() - 1].pose.position;
  path_visited    = *msg;
}
void request_path_cb(const geometry_msgs::Vector3Stamped::ConstPtr& msg){
  ROS_INFO("Path %s %.0f requested",msg->header.frame_id.c_str(),msg->vector.x);
  int zn;
  if( msg->header.frame_id == "path_full"){
    pub_path.publish(path_full);
  }
  else if( msg->header.frame_id == "path_notvstd_full"){
    //pub_path_notvstd
    pub_path.publish(remove_visited(path_full));
  }
  else if( msg->header.frame_id == "path_znlvls_full"){
    //pub_path_znlvls
    pub_path.publish(constrain_path_lvls(path_full,fmax(msg->vector.x,1)));
  }
  else if( msg->header.frame_id == "path_bbpoly_full"){
    if(poly_bb.polygon.points.size() > 0){
      //pub_path_inpoly
      pub_path.publish(constrain_path_bbpoly(path_full,poly_bb));
    }
  }
  else if(msg->header.frame_id == "path_atzlvl_full"){
    if(msg->vector.x == 0)
      pub_path.publish(paths_at_z[zlvl]);
    else
      pub_path.publish(paths_at_z[msg->vector.x]);
  }
}

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

void state_cb(const std_msgs::UInt8::ConstPtr& msg){
  if(msg->data == 1 && got_map && done)
    update_path_assist();
  else
    ROS_INFO("Update Path Assist still workiing!");
}



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

  nav_msgs::Path path_template;
  path_template.header = hdr();
  path_znlvls_full.header = hdr();
  path_inpoly_full.header = hdr();
  path_notvstd_full.header = hdr();
  path_full.header = hdr();
  proximity_range = 2;
  for(int i = 0; i < 20; i++){
    z_lvls.push_back(par_zjump*i);
    paths_at_z.push_back(path_template);
    paths_notvstd_at_z.push_back(path_template);
  }
  done = true;
  tf2_ros::TransformListener tf2_listener(tfBuffer);
  ros::Subscriber s0 = nh.subscribe("/octomap_full",1,octomap_callback);
  ros::Subscriber s2 = nh.subscribe("/tb_nav/path_visited",1,&visited_cb);
  ros::Subscriber s1 = nh.subscribe("/tb_bld/polygon",1,polybb_cb);
  ros::Subscriber s5 = nh.subscribe("/tb_fsm/altlvl",1,&zlvl_cb);
  ros::Subscriber s6 = nh.subscribe("/tb_fsm/main_state",100,&state_cb);
  ros::Subscriber s7 = nh.subscribe("/tb_path/request_path",100,&request_path_cb);

//  ros::Subscriber s3 = nh.subscribe("/tb_bld/building_state",1,&bldstate_cb);
//  ros::Subscriber s4 = nh.subscribe("/tb_nav/idle_time",100,&idletime_cb);

  pub_path            = nh.advertise<nav_msgs::Path>("/tb_path",100);
//  pub_path_atzlvl     = nh.advertise<nav_msgs::Path>("/tb_path/atzlvl",100);
//  pub_path_znlvls     = nh.advertise<nav_msgs::Path>("/tb_path/znlvls",100);
//  pub_path_notvstd    = nh.advertise<nav_msgs::Path>("/tb_path/notvstd",100);
//  pub_path_inpoly     = nh.advertise<nav_msgs::Path>("/tb_path/inpoly",100);
  ros::spin();
  return 0;
}
