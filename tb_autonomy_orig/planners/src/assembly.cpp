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
float interval = 1.5;
bool active = true;
ros::Time last_assembly;
ros::Publisher scan_repub;
ros::Publisher pub_pc2,pub,pub_pc2f,pub2,pub3;
sensor_msgs::PointCloud2 pc2copy,pc2filtered,white_filtered,white_transformed;
bool got_filtered;
tf2_ros::Buffer tfBuffer;
ros::Time t_r;
laser_assembler::AssembleScans2 srv;
laser_assembler::AssembleScans srv1;
sensor_msgs::PointCloud2 cloud_out,cloud_las;
ros::ServiceClient client;
ros::ServiceClient client1;
bool got_velodyne = false;

void pc2_cb(const sensor_msgs::PointCloud2ConstPtr& msg){
  pc2copy = *msg;
  pc2copy.header.stamp = ros::Time::now();
  pub_pc2.publish(pc2copy);
}

void interval_cb(const std_msgs::Float64::ConstPtr& msg){
  interval = msg->data;
  ROS_INFO("ASSEMBLY: Interval set to %.2f clouds/sec", interval);
}

void pc2filtered_cb(const sensor_msgs::PointCloud2::ConstPtr& msg){
  white_filtered = *msg;
  /*sensor_msgs::PointCloud2Modifier pcd_modifier(white_filtered);
  pcd_modifier.setPointCloud2FieldsByString(1, "xyz");
  geometry_msgs::TransformStamped transformStamped;
  try {
    transformStamped = tfBuffer.lookupTransform("base_stabilized",msg->header.frame_id,
                        ros::Time(0));
    tf2::doTransform(white_filtered, white_transformed, transformStamped);
  }
  catch (tf2::TransformException &ex) {
    ROS_WARN("%s",ex.what());
    ros::Duration(1.0).sleep();
  }

    srv.request.begin = t_r;
    t_r = ros::Time::now();
    srv.request.end   = t_r;
    client.call(srv);
    cloud_las = srv.response.cloud;
    sensor_msgs::PointCloud2Modifier pcd_lasermodifier(cloud_las);
    pcd_lasermodifier.setPointCloud2FieldsByString(1, "xyz");
    if(pcl::concatenatePointCloud(cloud_las,white_filtered,cloud_out)){
      ROS_INFO("Merged cloud to %u points", (uint32_t)(cloud_out.data.size()));
    } */
    sensor_msgs::PointCloud2Modifier pcd_modifier(white_filtered);
    pcd_modifier.setPointCloud2FieldsByString(1, "xyz");
    pub_pc2f.publish(white_filtered);
}/*
sensor_msgs::PointCloud pc2_to_pc1(sensor_msgs::PointCloud2 pc2){
  sensor_msgs::PointCloud pc1;
  pc1.header = pc2.header

  pc1.points.resize(pc2.data.size());
  sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x");
  sensor_msgs::PointCloud2ConstIterator<float> iter_y(*msg, "y");
  sensor_msgs::PointCloud2ConstIterator<float> iter_z(*msg, "z");

  for(int i = 0; iter_x != iter_x.end(); ++i, ++iter_x, ++iter_y, ++iter_z)
  {
    if(!std::isnan(*iter_x) && !std::isnan(*iter_y) && !std::isnan(*iter_z))
    {
      pc1.points[i].x = *iter_x;
      pc1.points[i].y = *iter_y;
      pc1.points[i].z = *iter_z;
    }
  }
  return pc1;
}*/
void pc2_to_pc1_cb(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
  sensor_msgs::PointCloud pc1;
  pc1.header = msg->header;
//	pc1.reserve(msg->data.size() / msg->point_step);

	bool has_x, has_y, has_z, has_rgb, has_i;
	//getFields(cloud_in, has_x, has_y, has_z, has_rgb, has_i);

	//if (!has_x || !has_y || !has_z)
	//{
	//	throw std::runtime_error("cloud_in missing one or more of the xyz fields");
	//}
  pc1.points.resize(msg->data.size());
	sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x");
	sensor_msgs::PointCloud2ConstIterator<float> iter_y(*msg, "y");
	sensor_msgs::PointCloud2ConstIterator<float> iter_z(*msg, "z");

	for(int i = 0; iter_x != iter_x.end(); ++i, ++iter_x, ++iter_y, ++iter_z)
	{
		if(!std::isnan(*iter_x) && !std::isnan(*iter_y) && !std::isnan(*iter_z))
    {
      pc1.points[i].x = *iter_x;
      pc1.points[i].y = *iter_y;
      pc1.points[i].z = *iter_z;
    }
	}
  pub3.publish(pc1);
}

void vlp_cb(const sensor_msgs::PointCloud2::ConstPtr& msg){
  got_velodyne = true;
  //white_filtered = *msg;
//  sensor_msgs::PointCloud2Modifier pcd_modifier(white_filtered);
//  pcd_modifier.setPointCloud2FieldsByString(1, "xyz");
  geometry_msgs::TransformStamped transformStamped;
//  sensor_msgs::PointCloud2Iterator<float> iter_x(white_filtered, "x");
//  sensor_msgs::PointCloud2Iterator<float> iter_y(white_filtered, "y");
//  sensor_msgs::PointCloud2Iterator<float> iter_z(white_filtered, "z");
//  ROS_INFO("iter_x size: %i",iter_x.end());
//  for (int i=0; iter_x != iter_x.end(); ++iter_x) {
//    ROS_INFO("pnt[%i/%i] X: %.0f y: %.0f Z: %.0f",i,iter_x.end(),iter_x[i],iter_y[i],iter_z[i]);
  //  ROS_INFO("pnt[%i/%i] X: %.0f y: %.0f Z: %.0f",i,iter_x.end(),iter_x[i],iter_y[i],iter_z[i]);
    // TODO: do something with the values of x, y, z
//     std::cout << iter_x[0] << ", " << iter_x[1] << ", " << iter_x[2] << '\n';
 //}
  try {
    transformStamped = tfBuffer.lookupTransform("map",msg->header.frame_id,
                        ros::Time(0));
    tf2::doTransform(*msg, white_filtered, transformStamped);
    pub2.publish(white_filtered);
  }
  catch (tf2::TransformException &ex) {
    ROS_WARN("%s",ex.what());
    ros::Duration(1.0).sleep();
  }
  // you pack data in another type (e.g. 3 uchar + 1 uchar for RGB are packed in a float)

}
void assemble_clouds(){
  srv.request.begin = t_r;
  t_r = ros::Time::now();
  srv.request.end   = t_r;
  client.call(srv);
  ROS_INFO("Published Cloud with %u points", (uint32_t)(srv.response.cloud.data.size()));
  pub.publish(srv.response.cloud);
}
void assemble_clouds1(){
  srv1.request.begin = t_r;
  t_r = ros::Time::now();
  srv1.request.end   = t_r;
  client1.call(srv1);
  ROS_INFO("Published Cloud with %u points", (uint32_t)(srv1.response.cloud.points.size()));
  pub3.publish(srv1.response.cloud);
}
void state_cb(const std_msgs::UInt8::ConstPtr& msg){
  if((ros::Time::now() - t_r).toSec() > interval){
    if(!got_velodyne){
      assemble_clouds1();
    }
    else{
      assemble_clouds();
    }
  }
}
void scan_cb(const sensor_msgs::LaserScan::ConstPtr& scan)
{
  sensor_msgs::LaserScan scan_sectors;
  scan_sectors                 = *scan;
  scan_sectors.header.frame_id = "base_perfect";
  scan_repub.publish(scan_sectors);
}
int main(int argc, char **argv)
{
  ros::init(argc, argv,"tb_assembly_node");
  ros::NodeHandle nh;
  tf2_ros::TransformListener tf2_listener(tfBuffer);
  t_r =  ros::Time::now();
//  ros::Subscriber sub = nh.subscribe("/tb_fsm/assembly_interval",10,interval_cb);
  ros::Subscriber s9  = nh.subscribe("/tb_fsm/main_state",    10,  &state_cb);
  ros::Subscriber s1  = nh.subscribe("/scan_base_alt",    10,  &scan_cb);
  ros::Subscriber s2  = nh.subscribe("/velodyne_points",10,    &vlp_cb);
  ros::Subscriber s3  = nh.subscribe("/assembled_cloud",10,         &pc2_to_pc1_cb);
//  ros::Subscriber s11  = nh.subscribe("/pcl_xyzrgb",10,         &pc2_cb);
//  ros::Subscriber s3  = nh.subscribe("/pcl_filtered_range",10, &pc2filtered_cb);

  scan_repub = nh.advertise<sensor_msgs::LaserScan>("/scan_base", 100);
//  pub_pc2    = nh.advertise<sensor_msgs::PointCloud2>("/pc2",10);
//  pub_pc2f   = nh.advertise<sensor_msgs::PointCloud2>("/pc2_filtered",10);
pub        = nh.advertise<sensor_msgs::PointCloud2> ("/assembled_cloud", 1);
pub2        = nh.advertise<sensor_msgs::PointCloud2> ("/tb_obs/cloud_transformed", 1);
pub3        = nh.advertise<sensor_msgs::PointCloud> ("/tb_obs/assembled_pc1", 1);
client     = nh.serviceClient<laser_assembler::AssembleScans2>("assemble_scans2");
client1     = nh.serviceClient<laser_assembler::AssembleScans>("assemble_scans");
  ros::spin();

  return 0;
}
