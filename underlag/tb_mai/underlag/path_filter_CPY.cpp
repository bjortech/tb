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

double par_zjump;

ros::Time last_update_and_publish_path;
ros::Publisher pub_path;
geometry_msgs::Point bbmin_custom,bbmax_custom,bbmin_octree,bbmax_octree,pos,poly_bb_centroid,last_pos;
nav_msgs::Path path_visited,path_candidates,path_candidates_floor;
geometry_msgs::PolygonStamped poly_bb,polysafe,polyvstd,polyhdng;
std::vector<int> z_lvls;

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
nav_msgs::Path constrain_path_xyz(nav_msgs::Path pathin, float dx, float dy, float dzmin, float dzmax){
  nav_msgs::Path pathout;
  pathout.header.frame_id="map";
  if(pathin.poses.size() == 0)
    return pathout;
  geometry_msgs::Point bbmin,bbmax;
  bbmin.x = pos.x-dx; bbmin.y = pos.y-dy; bbmin.z = pos.z-dzmin;
  bbmax.x = pos.x+dx; bbmax.y = pos.y+dy; bbmax.z = pos.z+dzmax;
  pathout = constrain_path_bbpnts(pathin,bbmin,bbmax);
  ROS_INFO("PATHUTIL: constrain_path_zlvl: pathin %i poses, lvls: %.2f -> %.2f pathout: %i poses",pathin.poses.size(),dzmin,dzmax,pathout.poses.size());
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
/******** END ******************** CONSTRAIN PATH FUNCTIONS   ***************** END ******************/

void polysafe_cb(const geometry_msgs::PolygonStamped::ConstPtr& msg){
    polysafe = *msg;
}
void polyhdng_cb(const geometry_msgs::PolygonStamped::ConstPtr& msg){
    polyhdng = *msg;
}
void polyvstd_cb(const geometry_msgs::PolygonStamped::ConstPtr& msg){
    polyvstd = *msg;
}
void visited_cb(const nav_msgs::Path::ConstPtr& msg){
  path_visited    = *msg;
  if(path_visited.poses.size() > 1)
    pos = path_visited.poses[path_visited.poses.size() - 1].pose.position;
}
void candidates_cb(const nav_msgs::Path::ConstPtr& msg){
  path_candidates    = *msg;
}
void candidatesfloor_cb(const nav_msgs::Path::ConstPtr& msg){
  path_candidates_floor = *msg;
}

nav_msgs::Path merge_paths(nav_msgs::Path path1,nav_msgs::Path path0){
  for(int i = 0; i < path1.poses.size(); i++){
    path0.poses.push_back(path1.poses[i]);
  }
  return path0;
}

void update_and_publish_path(){
  nav_msgs::Path pathout_cand,pathout_floor,pathout_0,pathout;
  pathout_cand.header  = hdr();
  pathout_floor.header = hdr();
  pathout.header       = hdr();
  last_update_and_publish_path = ros::Time::now();
  pathout_cand = constrain_path_vstd(constrain_path_bbpoly(path_candidates,polysafe),path_visited,4,true);
  pathout_floor   = constrain_path_vstd(constrain_path_bbpoly(path_candidates_floor,polysafe),path_visited,4,true);
  ////////*******************//////////////////////
  pathout_0 = constrain_path_xyz(pathout_cand,20,20,3,3);
  if(pathout_0.poses.size() == 0){
    pathout_0 = constrain_path_xyz(pathout_cand,20,20,20,20);
  }
  if(pathout_0.poses.size() == 0){
    pathout_0 = constrain_path_xyz(pathout_floor,20,20,10,0);
  }
  if(pathout_0.poses.size() == 0){
    pathout_0 = pathout_floor;
  }
  pathout = constrain_path_bbpoly(pathout_0,polyhdng);
  if(pathout.poses.size() == 0){
    pathout = pathout_0;
  }
  float dt = (ros::Time::now() - last_update_and_publish_path).toSec();
  ROS_INFO("PATHUTIL: Update & Publish Pathh [%i poses %.5f s]",pathout.poses.size(),dt);
  pub_path.publish(pathout);
}
void update_and_publish_path2(){
  nav_msgs::Path pathout_cand,pathout_floor,pathout_0,pathout;
  pathout_cand.header  = hdr();
  pathout_floor.header = hdr();
  pathout.header       = hdr();
  last_update_and_publish_path = ros::Time::now();
  pathout_cand  = constrain_path_vstd(constrain_path_bbpoly(path_candidates,polysafe),path_visited,4,true);
  pathout_floor = constrain_path_vstd(constrain_path_bbpoly(path_candidates_floor,polysafe),path_visited,4,true);
  ////////*******************//////////////////////
  pathout_0 = constrain_path_xyz(pathout_cand,20,20,3,3);
  if(pathout_0.poses.size() == 0){
    pathout_0 = constrain_path_xyz(pathout_cand,20,20,20,20);
  }
  if(pathout_0.poses.size() == 0){
    pathout_0 = constrain_path_xyz(pathout_floor,20,20,10,0);
  }
  if(pathout_0.poses.size() == 0){
    pathout_0 = pathout_floor;
  }
  pathout = constrain_path_bbpoly(pathout_0,polyhdng);
  if(pathout.poses.size() == 0){
    pathout = pathout_0;
  }
  float dt = (ros::Time::now() - last_update_and_publish_path).toSec();
  ROS_INFO("PATHUTIL: Update & Publish Pathh [%i poses %.5f s]",pathout.poses.size(),dt);
  pub_path.publish(pathout);
} 
void check_if_need_to_update(){
  if((ros::Time::now() - last_update_and_publish_path).toSec() > 3.0)
    update_and_publish_path();
}
void check_if_need_to_update2(){
  if((ros::Time::now() - last_update_and_publish_path).toSec() > 3.0)
    update_and_publish_path2();
}
void missionstate_cb(const std_msgs::UInt8::ConstPtr& msg){
  if(msg->data == 1)
    check_if_need_to_update();
  else if(msg->data == 2)
    check_if_need_to_update2();
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "tb_pathfilter_node");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");
  private_nh.param("par_zjump",   par_zjump, 3.0);//*2.0);

  for(int i = 0; i < 40; i++){
    z_lvls.push_back(par_zjump*i);
  }
  ros::Subscriber s2 = nh.subscribe("/tb_nav/path_visited",1,&visited_cb);
  ros::Subscriber ss2 = nh.subscribe("/tb_path_filtered",1,&candidates_cb);
  ros::Subscriber ss3 = nh.subscribe("/tb_path_floor",1,&candidatesfloor_cb);
	ros::Subscriber s3 = nh.subscribe("/tb_obs/poly_safe",1,polysafe_cb);
	ros::Subscriber s4 = nh.subscribe("/tb_obs/poly_hdng",1,polyhdng_cb);
	ros::Subscriber s6 = nh.subscribe("/tb_obs/poly_vstd",1,polyvstd_cb);
  ros::Subscriber s89 = nh.subscribe("/tb_fsm/mission_state",1,missionstate_cb);

  pub_path            = nh.advertise<nav_msgs::Path>("/tb_path",100);
  ros::spin();
  return 0;
}
