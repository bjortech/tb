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

bool got_map;
int xmin,ymin,zmin,xmax,ymax,zmax,range_x,range_y,range_z,zmin_global,zmax_global;

geometry_msgs::Point bbmin_custom,bbmax_custom,bbmin_octree,bbmax_octree,pos;
geometry_msgs::PointStamped closest_obstacle;
geometry_msgs::PoseStamped last_pose;
geometry_msgs::PolygonStamped poly_heading,poly_safe,poly_vstd;
nav_msgs::Path path_visited,path;

shared_ptr<DynamicEDTOctomap> edf_ptr;
AbstractOcTree* abs_octree;
shared_ptr<OcTree> octree;

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

float get_dst3d(geometry_msgs::Point p1, geometry_msgs::Point p2){
  return(sqrt(pow(p1.x-p2.x,2)+pow(p1.y-p2.y,2)+pow(p1.z-p2.z,2)));
}
float get_dst2d(geometry_msgs::Point p1, geometry_msgs::Point p2){
  return(sqrt(pow(p1.x-p2.x,2)+pow(p1.y-p2.y,2)));
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

geometry_msgs::PolygonStamped get_poly_vstd(float collision_radius,float mapradused,int num_rays){
  geometry_msgs::PolygonStamped polygon;
  polygon.header = hdr();
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

geometry_msgs::PolygonStamped get_poly_surround(float collision_radius,float mapradused,int num_rays){
  geometry_msgs::PolygonStamped polygon;
  polygon.header = hdr();
  update_edto(last_pose.pose.position,collision_radius,mapradused,last_pose.pose.position.z+1,last_pose.pose.position.z-1,false);
  octomap::point3d p(last_pose.pose.position.x,last_pose.pose.position.y,last_pose.pose.position.z);
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
  if(sqrt(pow(transformStamped.transform.translation.x - last_pose.pose.position.x,2)+ pow(transformStamped.transform.translation.y - last_pose.pose.position.y,2)+
    pow(transformStamped.transform.translation.z - last_pose.pose.position.z,2)) >= 1.00){
    last_pose.pose.position    = pos;
    last_pose.pose.orientation = transformStamped.transform.rotation;
    last_pose.header           = transformStamped.header;
    path_visited.poses.push_back(last_pose);
    path_visited.header.stamp  = ros::Time::now();
  }
}
int main(int argc, char** argv)
{
  ros::init(argc, argv, "tb_perception_node");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");
  tf2_ros::TransformListener tf2_listener(tfBuffer);

  last_pose.header        = hdr();
  path_visited.header     = hdr();
  closest_obstacle.header = hdr();
  last_pose.pose.orientation.w = 1;
  path_visited.poses.push_back(last_pose);

  ros::Publisher pub_path_visited   	= nh.advertise<nav_msgs::Path>("/tb_nav/path_visited",10);
	ros::Publisher polysafe_pub 				= nh.advertise<geometry_msgs::PolygonStamped>("/tb_obs/poly_safe",100);
	ros::Publisher polyvstd_pub 				= nh.advertise<geometry_msgs::PolygonStamped>("/tb_obs/poly_vstd",100);
  ros::Publisher polyhdng_pub 				= nh.advertise<geometry_msgs::PolygonStamped>("/tb_obs/poly_hdng",100);
  ros::Publisher pub_closest_obstacle = nh.advertise<geometry_msgs::PointStamped>("/tb_obs/closest",100);

  ros::Subscriber s1  = nh.subscribe("/octomap_full",1,octomap_callback);

  std_msgs::Float64 cmdarm_msg;
  ros::Rate rate(2.0);
  ros::Time start = ros::Time::now();
  float centroid_sides = 20;
  int count = 0;
  while(ros::ok()){
    if(got_map){
      checktf();
      poly_heading     = get_poly_heading(M_PI/2,16,"base_stabilized");
			poly_safe        = get_poly_surround(10,50,36);
			poly_vstd        = get_poly_vstd(10,50,36);
      polyhdng_pub.publish(poly_heading);
      polysafe_pub.publish(poly_safe);
			polyvstd_pub.publish(poly_vstd);
      pub_closest_obstacle.publish(closest_obstacle);
    }
    pub_path_visited.publish(path_visited);
    rate.sleep();
    ros::spinOnce();
  }
  return 0;
}
