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
#include <geometry_msgs/Vector3.h>
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
#include <nav_msgs/Odometry.h>


tf2_ros::Buffer tfBuffer;
bool par_use_sectors;
int n_sectors;
sensor_msgs::LaserScan slos,smis,shis;
sensor_msgs::LaserScan slo,smi,shi,scanout_clear,scanout,scanout_danger,scanout_buildings,scanout_roofs;
double par_n_sectors,par_max_delta_r;
std::vector<float> angles;
std::vector<geometry_msgs::Vector3> ranges;
std::vector<int> streak_start;
std::vector<int> streak_stop;
std::vector<int> streak_closest_i;
bool smi_ready,slo_ready,shi_ready,pub_ready;

sensor_msgs::LaserScan scansectors(sensor_msgs::LaserScan scanin){
  int ranges = scanin.ranges.size();
  //populate the LaserScan message
  ROS_INFO("n_sectors:%i",n_sectors);
  sensor_msgs::LaserScan scan_sectors;
  scan_sectors.header.stamp = scanin.header.stamp;
  scan_sectors.header.frame_id = scanin.header.frame_id;
  scan_sectors.angle_min = scanin.angle_min;
  scan_sectors.angle_max = scanin.angle_max;
  scan_sectors.angle_increment = (scanin.angle_max - scanin.angle_min)/n_sectors;
  scan_sectors.time_increment = 0;
  scan_sectors.range_min = slo.range_min;
  scan_sectors.range_max = slo.range_max;
  scan_sectors.ranges.resize(n_sectors);

  float shortest = 1000.0;
  float angle_shortest;
  int k = 0;
  for(int i = 0; i < ranges; ++i)
  {
    if((scanin.ranges.size()/n_sectors) * k < i){
      k++;
      scan_sectors.ranges[k] = 1000;
    }
    else{
      if(scanin.ranges[i] < scan_sectors.ranges[k]){
        scan_sectors.ranges[k] = scanin.ranges[i];
        if(scanin.ranges[i] < shortest){
          shortest = scanin.ranges[i];
          angle_shortest = scanin.angle_min + scanin.angle_increment * i;
        }
      }
    }
  }
  return scan_sectors;
}
geometry_msgs::Vector3 lo_mi_hi(int i){
  geometry_msgs::Vector3 v;
  if(par_use_sectors){
    if((!std::isinf(slos.ranges[i])) && slos.ranges[i] < slos.range_max && slos.ranges[i] > slos.range_min)
      v.x = slos.ranges[i];
    if((!std::isinf(smis.ranges[i])) && smis.ranges[i] < smis.range_max && smis.ranges[i] > smis.range_min)
      v.y = smis.ranges[i];
    if((!std::isinf(shis.ranges[i])) && shis.ranges[i] < shis.range_max && shis.ranges[i] > shis.range_min)
      v.z = shis.ranges[i];
  }
  else{
    if((!std::isinf(slo.ranges[i])) && slo.ranges[i] < slo.range_max && slo.ranges[i] > slo.range_min)
      v.x = slo.ranges[i];
    if((!std::isinf(smi.ranges[i])) && smi.ranges[i] < smi.range_max && smi.ranges[i] > smi.range_min)
      v.y = smi.ranges[i];
    if((!std::isinf(shi.ranges[i])) && shi.ranges[i] < shi.range_max && shi.ranges[i] > shi.range_min)
      v.z = shi.ranges[i];
  }
  return v;
}

void create_ranges_and_angles(){
  ros::Time t0 = ros::Time::now();

  int ll,lm,lh,lmin,lmax;
  if(par_use_sectors){
    slos = scansectors(slo);
    smis = scansectors(smi);
    shis = scansectors(shi);
     ll = slos.ranges.size();
     lm = smis.ranges.size();
     lh = shis.ranges.size();
     scanout_danger    = smis;
     scanout_buildings = smis;
     scanout_roofs     = smis;
     scanout_clear     = smis;
  }
  else{
    ll = slo.ranges.size();
    lm = smi.ranges.size();
    lh = shi.ranges.size();
    scanout_danger = smi;
    scanout_buildings = smi;
    scanout_roofs = smi;
    scanout_clear = smi;
  }
  scanout_danger.ranges.resize(0);
  scanout_buildings.ranges.resize(0);
  scanout_roofs.ranges.resize(0);
  scanout_clear.ranges.resize(0);
  lmin = fmin(fmin(ll,lm),fmin(lm,lh));
  lmax = fmax(fmax(ll,lm),fmax(lm,lh));
  ranges.resize(lmax);
  scanout_danger.ranges.resize(lmax);
  scanout_buildings.ranges.resize(lmax);
  scanout_roofs.ranges.resize(lmax);
  scanout_clear.ranges.resize(lmax);

  angles.resize(lmax);
  for(int i = 0; i < lmin;i++){
    ranges[i] = lo_mi_hi(i);
  }
  float dt = (ros::Time::now() - t0).toSec();
  ROS_INFO("SCANS: update_ranges & angles[%i ranges %.5f s]]",scanout.ranges.size(),dt);
}
void get_streaks_and_scanout(){
  streak_closest_i.resize(0); streak_stop.resize(0); streak_start.resize(0);
  for(int i = 0; i < ranges.size(); i++){
    if(ranges[i].x + ranges[i].y + ranges[i].z == 0){
      scanout_clear.ranges[i] = scanout_clear.range_max;
    }
    else if(ranges[i].x > 0 && ranges[i].y > 0 && ranges[i].z > 0){
      float rmin = fmin(fmin(ranges[i].x,ranges[i].y),fmin(ranges[i].y,ranges[i].z));
      float rmax = fmax(fmax(ranges[i].x,ranges[i].y),fmax(ranges[i].y,ranges[i].z));
      scanout_danger.ranges[i] = rmin;
      float rd = rmax - rmin;
      if(rd < par_max_delta_r)
        scanout_buildings.ranges[i] = (ranges[i].y + ranges[i].x + ranges[i].z) / 3;
    }
    else if(ranges[i].x > 0 && ranges[i].y == 0 && ranges[i].y == 0){
      scanout_roofs.ranges[i] = ranges[i].x;
    }
    else if(ranges[i].x > 0 && ranges[i].y > 0 && ranges[i].z == 0 && abs(ranges[i].y - ranges[i].x) < par_max_delta_r){
      scanout_roofs.ranges[i] = ranges[i].z;
    }
    else{
      float val = 100;
      if(ranges[i].x > 0)
        val = fmin(val,ranges[i].x);
      if(ranges[i].y > 0)
        val = fmin(val,ranges[i].y);
      if(ranges[i].z > 0)
        val = fmin(val,ranges[i].z);
      scanout_danger.ranges[i] = ranges[i].z;
    }
  }
}
void exequte(){
	slo_ready = false;
	smi_ready = false;
	shi_ready = false;
	if(slo.ranges.size() == smi.ranges.size()
	&& slo.ranges.size() == smi.ranges.size()
	&& smi.ranges.size() > 0){
		create_ranges_and_angles();
		get_streaks_and_scanout();
		pub_ready = true;
	}
}

void scan_dwn_cb(const sensor_msgs::LaserScan::ConstPtr& scan){
  slo = *scan;
	slo_ready = true;
	if(smi_ready && shi_ready && slo_ready)
		exequte();
}
void scan_stabilized_cb(const sensor_msgs::LaserScan::ConstPtr& scan){
  smi = *scan;
	smi_ready = true;
	if(smi_ready && shi_ready && slo_ready)
		exequte();
}
void scan_up_cb(const sensor_msgs::LaserScan::ConstPtr& scan){
  shi = *scan;
	shi_ready = true;
	if(smi_ready && shi_ready && slo_ready)
		exequte();
}
void usesectors_cb(const std_msgs::Bool::ConstPtr& msg){
  par_use_sectors = msg->data;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "tb_perceptionlongrangescanners_node");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");
  private_nh.param("n_sectors",par_n_sectors, 36.0);
  private_nh.param("max_range_diff",par_max_delta_r, 3.0);
  private_nh.param("use_sectors", par_use_sectors, true);//*2.0);

  n_sectors = int(round(par_n_sectors));
  tf2_ros::TransformListener tf2_listener(tfBuffer);

  ros::Subscriber s11 = nh.subscribe("/scan_stabilized_down",10, scan_dwn_cb);
  ros::Subscriber s12 = nh.subscribe("/scan_stabilized_mid",10, scan_stabilized_cb);
  ros::Subscriber s13 = nh.subscribe("/scan_stabilized_up",10,  scan_up_cb);
  ros::Subscriber s14 = nh.subscribe("/tb_obs/use_sectors",10, usesectors_cb);

  ros::Publisher pub_scan_cleared 	= nh.advertise<sensor_msgs::LaserScan>("/tb_obs/scan_cleared",100);
  ros::Publisher pub_scan_roofs  		= nh.advertise<sensor_msgs::LaserScan>("/tb_obs/scan_roofs",100);
  ros::Publisher pub_scan_dangers 	= nh.advertise<sensor_msgs::LaserScan>("/tb_obs/scan_dangers",100);
  ros::Publisher pub_scan_buildings = nh.advertise<sensor_msgs::LaserScan>("/tb_obs/scan_buildings",100);

  ros::Rate rate(2.0);
  ros::Time start = ros::Time::now();
  ros::Time last_update,last_info;
  int buildings_sent = 0;
  while(ros::ok()){
    if(pub_ready){
			pub_ready = false;
      pub_scan_cleared.publish(scanout_clear);
      pub_scan_roofs.publish(scanout_roofs);
      pub_scan_dangers.publish(scanout_danger);
      pub_scan_buildings.publish(scanout_buildings);
    }
    rate.sleep();
    ros::spinOnce();
  }
  return 0;
}
