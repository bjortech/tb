#include "ros/ros.h"
#include <std_msgs/Float64.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Point.h>
#include <iostream>
#include <laser_assembler/AssembleScans2.h>
#include <laser_assembler/AssembleScans.h>
#include <sensor_msgs/LaserScan.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/concatenate.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_listener.h>
#include "tf2_ros/message_filter.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "pcl_ros/transforms.h"
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_ros/buffer.h>
#include <tf2/transform_datatypes.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
double par_assembly_interval;
ros::Time last_assembly;
ros::Publisher pub,scan_repub;
tf2_ros::Buffer tfBuffer;
ros::Time t_r;
laser_assembler::AssembleScans2 srv;
ros::ServiceClient client;
ros::Publisher pub_scan_base_stab,pub_scan_base_alt,pub_scan_base;
bool par_usestabilized;
void assemble_clouds(){
  srv.request.begin = t_r;
  t_r = ros::Time::now();
  srv.request.end   = t_r;
  client.call(srv);
  ROS_INFO("ASSEMBLY: Published Cloud with %u points", (uint32_t)(srv.response.cloud.data.size()));
  pub.publish(srv.response.cloud);
}
void state_cb(const std_msgs::UInt8::ConstPtr& msg){
  if((ros::Time::now() - t_r).toSec() > par_assembly_interval)
    assemble_clouds();
}
void scan_perfect_alt_cb(const sensor_msgs::LaserScan::ConstPtr& scan)
{
  sensor_msgs::LaserScan scan_sectors;
  scan_sectors                 = *scan;
  scan_sectors.header.frame_id = "base_perfect";
	if(!par_usestabilized)
		pub_scan_base.publish(scan_sectors);
	else
  	pub_scan_base_alt.publish(scan_sectors);
}
void scan_stabilized_cb(const sensor_msgs::LaserScan::ConstPtr& scan)
{
  sensor_msgs::LaserScan scan_sectors;
  scan_sectors                 = *scan;
	scan_sectors.header.frame_id = "base_footprint";
	if(par_usestabilized)
		pub_scan_base.publish(scan_sectors);
	else
  	pub_scan_base_stab.publish(scan_sectors);
}
int main(int argc, char **argv)
{
  ros::init(argc, argv,"tb_assembly_node");
  ros::NodeHandle nh;
	ros::NodeHandle private_nh("~");
	private_nh.param("scanbase_stabilized", par_usestabilized, false);//*2.0);
	private_nh.param("assembly_interval", par_assembly_interval, 1.2);//*2.0);

  tf2_ros::TransformListener tf2_listener(tfBuffer);
  t_r =  ros::Time::now();
	ros::Subscriber s0  = nh.subscribe("/scan_perfect_alt", 10,  &scan_perfect_alt_cb);
	ros::Subscriber s1  = nh.subscribe("/scan_stabilized",  10,  &scan_stabilized_cb);
	pub_scan_base_stab  = nh.advertise<sensor_msgs::LaserScan>("/scan_base_stab", 100);
	pub_scan_base_alt	  = nh.advertise<sensor_msgs::LaserScan>("/scan_base_alt", 100);
	pub_scan_base	      = nh.advertise<sensor_msgs::LaserScan>("/scan_base", 100);

  ros::Subscriber s9  = nh.subscribe("/tb_fsm/main_state",    10,  &state_cb);
  pub        = nh.advertise<sensor_msgs::PointCloud2> ("/assembled_cloud2", 1);
  client     = nh.serviceClient<laser_assembler::AssembleScans2>("assemble_scans2");
  ros::spin();

  return 0;
}
