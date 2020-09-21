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
ros::Publisher pub;
string par_workdir_path;
geometry_msgs::Point bbmin_custom,bbmax_custom,bbmin_octree,bbmax_octree,pos,last_pos;
int xmin,ymin,zmin,xmax,ymax,zmax,range_x,range_y,range_z,zmin_global,zmax_global;
const float rad2deg = 180.0/M_PI;
cv::Mat img(1000,1000,CV_8UC3,cv::Scalar(0, 0, 0)); //create image, set encoding and size, init pixels to default val
cv::Mat mapimg_visited(1000,1000,CV_8U,cv::Scalar(0)); //create image, set encoding and size, init pixels to default val
float pos_yaw;
int zlvl;
double par_visited_rad;
geometry_msgs::PolygonStamped poly_bb;
bool par_unknownAsOccupied;
double par_maprad,par_maxobs,par_minobs,last_yaw,par_zjump;
nav_msgs::Path path_candidates;
std::vector<int> z_lvls;
bool got_map;
std::vector<nav_msgs::Path> paths_at_z;

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

bool sort_dst_pair(const std::tuple<int,float>& a,
               const std::tuple<int,float>& b)
{
    return (std::get<1>(a) < std::get<1>(b));
}

float get_dst2d(geometry_msgs::Point p1, geometry_msgs::Point p2){
  return(sqrt(pow(p1.x-p2.x,2)+pow(p1.y-p2.y,2)));
}
float get_dst3d(geometry_msgs::Point p1, geometry_msgs::Point p2){
  return(sqrt(pow(p1.x-p2.x,2)+pow(p1.y-p2.y,2)+pow(p1.z-p2.z,2)));
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

nav_msgs::Path constrain_path_lvls(nav_msgs::Path pathin,int minlvlvs){
  nav_msgs::Path pathout;
  pathout.header.frame_id = "map";
  pathout.header.stamp    = ros::Time::now();
  for(int i = 0; i < pathin.poses.size(); i++){
    std::vector<int> indexes_in_rad;
    indexes_in_rad = getinpath_indexes_inrad(pathin,pathin.poses[i].pose.position,4);
    if(indexes_in_rad.size() >= minlvlvs)
      pathout.poses.push_back(pathin.poses[i]);
  }
  return pathout;
}
nav_msgs::Path constrain_path_bbpoly(nav_msgs::Path pathin,geometry_msgs::PolygonStamped poly_bb){
  nav_msgs::Path pathout;
  pathout.header.frame_id = "map";
  pathout.header.stamp    = ros::Time::now();
  for(int i = 0; i < pathin.poses.size(); i++){
    if(in_poly(poly_bb,pathin.poses[i].pose.position.x,pathin.poses[i].pose.position.y)){
      float pos_hdng  = get_hdng(pathin.poses[i].pose.position,building_centroid.point);
      float pose_hdng = tf::getYaw(pathin.poses[i].pose.orientation);
      ROS_INFO("Pos hdng: %.2f pose hdng: %.2f",pos_hdng,pose_hdng);
      pathout.poses.push_back(pathin.poses[i]);
    }
  }
  return pathout;
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
nav_msgs::Path constrain_paths_at_z(std::vector<nav_msgs::Path> paths_in,nav_msgs::Path path_in_full){

}
void update_path()
{
  std::vector<float> r0,x0,c0,y0;
  geometry_msgs::PoseStamped ps;
  last_pos = pos;
  last_yaw = pos_yaw;
  ps.header.frame_id ="map";
  nav_msgs::Path path_candidates_to_send;
  path_candidates_to_send.header.frame_id="map";
  path_candidates.header.frame_id ="map";

  octree.get()->getMetricMin(bbmin_octree.x,bbmin_octree.y,bbmin_octree.z);
  octree.get()->getMetricMax(bbmax_octree.x,bbmax_octree.y,bbmax_octree.z);
  int min_zlvl = fmax(bbmin_octree.z/par_zjump + 4,zlvl-2);
  int max_zlvl = fmin(bbmax_octree.z/par_zjump + 4,zlvl+2);
  if(min_zlvl == max_zlvl)
    return;

  for(int zn = min_zlvl; zn < max_zlvl; zn++){
    int z = z_lvls[zn];
    update_edto(pos,par_maxobs+1,par_maprad,z-0.5,z+0.5,par_unknownAsOccupied);
      for(int y = ymin; y < ymax; y++){
        for(int x = xmin; x < xmax; x++){
          ps.pose.position.x = x;
          ps.pose.position.y = y;
          ps.pose.position.z = z;
          if(dst_point_in_path_lim(path_candidates,ps.pose.position,5)){
            octomap::point3d p(x,y,z);
            octomap::point3d closestObst;
            float d;
            edf_ptr.get()->getDistanceAndClosestObstacle(p,d,closestObst);
            if(round((par_minobs + par_maxobs)/2) == round(d)){
              ps.pose.orientation = tf::createQuaternionMsgFromYaw(atan2(closestObst.y() - y,closestObst.x() - x));
              paths_at_z[zn].poses.push_back(ps);
              path_candidates.poses.push_back(ps);
              path_candidates_to_send.poses.push_back(ps);
            }
          }
        }
      }
    }
  if(path_candidates_to_send.poses.size() > 0)
    pub.publish(path_candidates_to_send);
  return;
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
  geometry_msgs::PoseStamped ps;
  ps.header.frame_id ="map";
	pos.x = transformStamped.transform.translation.x;
	pos.y = transformStamped.transform.translation.y;
	pos.z = transformStamped.transform.translation.z;
  if(dst_point_in_path_lim(path_visited,pos,3)){
    ps.pose.position.x  = transformStamped.transform.translation.x;
    ps.pose.position.y  = transformStamped.transform.translation.y;
    ps.pose.position.z  = transformStamped.transform.translation.z;
    ps.pose.orientation = transformStamped.transform.rotation;
    ps.header           = transformStamped.header;
    path_visited.push_back(ps)
  }
  zlvl  = round(pos.z / par_zjump) + 3;
	pos_yaw = tf::getYaw(transformStamped.transform.rotation);
  if(abs(last_yaw - pos_yaw) > 1 || get_dst3d(pos,last_pos) > 5){
    update_path();
  }
}
void polybb_cb(const geometry_msgs::PolygonStamped::ConstPtr& msg){
  poly_bb = *msg;
}

float getinpath_closestdst2d(nav_msgs::Path pathin,geometry_msgs::PoseStamped pose_to_check){
  float lowest_dist = 1000;

  if(pathin.poses.size() == 0){
    ROS_INFO("PathAdmin: pathin is empty");
    return lowest_dist;
  }
  for(int i = 0; i < pathin.poses.size(); i++){
    float dist = get_dst2d(pathin.poses[i].pose.position,pose_to_check.pose.position);
    if(dist < lowest_dist)
      lowest_dist   = dist;
  }
  return lowest_dist;
}

float getinpath_closestdst3d(nav_msgs::Path pathin,geometry_msgs::PoseStamped pose_to_check){
  float lowest_dist = 1000;

  if(pathin.poses.size() == 0){
    ROS_INFO("PathAdmin: pathin is empty");
    return lowest_dist;
  }
  for(int i = 0; i < pathin.poses.size(); i++){
    float dist = get_dst3d(pathin.poses[i].pose.position,pose_to_check.pose.position);
    if(dist < lowest_dist)
      lowest_dist   = dist;
  }
  return lowest_dist;
}

nav_msgs::Path constrain_path_vstd(nav_msgs::Path pathin,nav_msgs::Path pathin_vstd,float cutoff_dst){
  nav_msgs::Path pathout;
	pathout.header.frame_id = "map";
  if(fmin(pathin.poses.size(),pathin_vstd.poses.size()) == 0){
    ROS_INFO("getinpath_not_visited: pathin is empty");
    return pathin;
  }
  ROS_INFO("Pathin size: cand %i vstd %i cutoff %.2f",pathin.poses.size(),pathin_vstd.poses.size(),cutoff_dst);
  for(int i = 0; i < pathin.poses.size(); i++){
    if(getinpath_closestdst3d(pathin_vstd,pathin.poses[i]) > cutoff_dst)
      pathout.poses.push_back(pathin.poses[i]);
  }
  ROS_INFO("PathSegmentation VSTD: %i of %i poses not visited",pathout.poses.size(),pathin.poses.size());
  return pathout;
}
int main(int argc, char **argv)
{
  ros::init(argc, argv, "tb_path_service");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");
  private_nh.param("visited_rad", par_visited_rad, 10.0);//*2.0);
  private_nh.param("workdir_path",par_workdir_path);//*2.0);
  private_nh.param("par_maprad",  par_maprad, 30.0);//*2.0);
  private_nh.param("par_maxobs",  par_maxobs, 20.0);//*2.0);
  private_nh.param("par_zjump",   par_zjump, 3.0);//*2.0);
  private_nh.param("par_minobs",  par_minobs, 2.0);//*2.0);

  for(int i = 0; i < 20; i++){
    z_lvls.push_back(int(round(-par_zjump*3 + par_zjump*i)));
  }
  // ros::Subscriber s2 = nh.subscribe("/tb_nav/visited_path",1,visited_cb);
   ros::Subscriber s12 = nh.subscribe("/tb_polygon_building",1,&polybb_cb);
  // ros::Subscriber s2 = nh.subscribe("/tb_nav/visited_path",1,visited_cb);
pub_centroid  = nh.advertise<geometry_msgs::PointStamped>("/tb_cmd/building_centroid",100);
  tf2_ros::TransformListener tf2_listener(tfBuffer);
  ros::Publisher pub_orig = nh.advertise<nav_msgs::Path>("/tb_path_filtered",100);
  ros::Publisher pub_path_zn   = nh.advertise<nav_msgs::Path>("/tb_path_filtered/zn",100);
  ros::Publisher pub_path_bb   = nh.advertise<nav_msgs::Path>("/tb_path_filtered/bb",100);
  ros::Publisher pub_path_vstd = nh.advertise<nav_msgs::Path>("/tb_path_filtered/vstd",100);
  pub = nh.advertise<nav_msgs::Path>("/tb_path_updates",100);
  ros::Subscriber s1 = nh.subscribe("/octomap_full",1,octomap_callback);
  ROS_INFO("Ready to convert octomaps.");
  ros::Rate rate(1.0);
  nav_msgs::Path path_in_lvls,path_in_bbpoly,path_not_visited;
  while(ros::ok()){
    checktf();
    pub_orig.publish(path_candidates);
    path_in_lvls     = constrain_path_lvls(path_candidates,2);
    path_in_bbpoly   = constrain_path_bbpoly(path_candidates,poly_bb);
    path_not_visited = constrain_path_vstd(path_candidates,path_visited,10);
    pub_path_bb.publish(path_in_bbpoly);
    rate.sleep();
	  ros::spinOnce();
  }
  return 0;
}
