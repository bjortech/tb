#include "ros/ros.h"
#include <std_msgs/Float64.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Point.h>
#include <iostream>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_listener.h>
#include "tf2_ros/message_filter.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include <tf2_ros/buffer.h>
#include <tf2/transform_datatypes.h>
#include <eigen3/Eigen/Core>

bool got_map;
sensor_msgs::PointCloud2 octomap;
tf2_ros::Buffer tfBuffer;
sensor_msgs::PointCloud pc1;
geometry_msgs::Vector3 vlp_rpy;
geometry_msgs::Point pos;
geometry_msgs::PolygonStamped poly;
float get_dst2d(geometry_msgs::Point32 p2, geometry_msgs::Point p1){
  return(sqrt(pow(p1.x-p2.x,2)+pow(p1.y-p2.y,2)));
}
float get_dst3d(geometry_msgs::Point32 p2, geometry_msgs::Point p1){
  return(sqrt(pow(p1.x-p2.x,2)+pow(p1.y-p2.y,2)+pow(p1.z-p2.z,2)));
}
float get_hdng(geometry_msgs::Point32 p1,geometry_msgs::Point p0){
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
float get_inclination(geometry_msgs::Point32 p2, geometry_msgs::Point p1){
  return atan2(p2.z - p1.z,get_dst2d(p2,p1));
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
//	//ROS_INFO("sr: %.5f",dt);
  return bool(cross % 2);
}
void checktf(){
  geometry_msgs::TransformStamped transformStamped;
  try{
    transformStamped = tfBuffer.lookupTransform("map","velodyne_aligned",
                             ros::Time(0));
  }
  catch (tf2::TransformException &ex) {
    ROS_WARN("%s",ex.what());
    ros::Duration(1.0).sleep();
  }
	pos.x = transformStamped.transform.translation.x;
	pos.y = transformStamped.transform.translation.y;
	pos.z = transformStamped.transform.translation.z;
  tf2::Matrix3x3 q(tf2::Quaternion(transformStamped.transform.rotation.x, transformStamped.transform.rotation.y, transformStamped.transform.rotation.z, transformStamped.transform.rotation.w));
  q.getRPY(vlp_rpy.x,vlp_rpy.y,vlp_rpy.z);
  vlp_rpy.y *= -1;
}
bool is_within_hdng_incline(geometry_msgs::Point32 pnt){
	float dst = get_dst3d(pnt,pos);
	if(dst > 50)
		return false;
	float hdng_shortest = get_shortest(get_hdng(pnt,pos),vlp_rpy.z);
  if(hdng_shortest < 0)
    hdng_shortest *= -1;
  if(hdng_shortest < M_PI/2){
		float pitch_shortest = get_shortest(get_inclination(pnt,pos),vlp_rpy.y);
    if(pitch_shortest < 0)
      pitch_shortest *= -1;
    if(pitch_shortest < M_PI/6)
      return true;
    else
      return false;
  }
  else
    return false;
}
sensor_msgs::PointCloud get_pc_within(){
	sensor_msgs::PointCloud pc_out;
	pc_out.header.frame_id = "map";
	pc_out.header.stamp = ros::Time::now();
	for(int i= 0; i < pc1.points.size(); i++){
		if(is_within_hdng_incline(pc1.points[i]))
			pc_out.points.push_back(pc1.points[i]);
	}
	return pc_out;
}

sensor_msgs::PointCloud2 pc1_to_pc2(sensor_msgs::PointCloud pc1_in){
  sensor_msgs::PointCloud2 pc2_out;
  pc2_out.header = pc1_in.header;
  //pc2_out.data.resize(pc1_in.points.size());
  sensor_msgs::PointCloud2Modifier modifier(pc2_out);
  modifier.setPointCloud2FieldsByString(1,"xyz");
  modifier.resize(pc1_in.points.size());
	sensor_msgs::PointCloud2Iterator<float> iter_x(pc2_out, "x");
	sensor_msgs::PointCloud2Iterator<float> iter_y(pc2_out, "y");
	sensor_msgs::PointCloud2Iterator<float> iter_z(pc2_out, "z");

  for (unsigned int i = 0; i < pc1_in.points.size();
       ++i, ++iter_x, ++iter_y, ++iter_z)
  {
    iter_x[0] = float(pc1_in.points[i].x);
    iter_x[1] = float(pc1_in.points[i].y);
    iter_x[2] = float(pc1_in.points[i].z);
  }
  return pc2_out;
}
sensor_msgs::PointCloud pc2_to_pc1(sensor_msgs::PointCloud2 pc2_in){
  sensor_msgs::PointCloud pc1_out;
  pc1_out.header = pc2_in.header;
  pc1_out.points.resize(pc2_in.data.size());
  sensor_msgs::PointCloud2ConstIterator<float> iter_x(pc2_in, "x");
  sensor_msgs::PointCloud2ConstIterator<float> iter_y(pc2_in, "y");
  sensor_msgs::PointCloud2ConstIterator<float> iter_z(pc2_in, "z");

  for(int i = 0; iter_x != iter_x.end(); ++i, ++iter_x, ++iter_y, ++iter_z)
  {
    if(!std::isnan(*iter_x) && !std::isnan(*iter_y) && !std::isnan(*iter_z))
    {
      pc1_out.points[i].x = *iter_x;
      pc1_out.points[i].y = *iter_y;
      pc1_out.points[i].z = *iter_z;
    }
  }
  return pc1_out;
}
void oct_cb(const sensor_msgs::PointCloud2::ConstPtr& msg){
  if(!got_map){
    pc1.header = msg->header;
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
  	octomap = *msg;
  	got_map = true;
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv,"tb_simulate_oct2sensor_node");
  ros::NodeHandle nh;
	tf2_ros::TransformListener tf2_listener(tfBuffer);

  ros::Subscriber s5  = nh.subscribe("/octomap_point_cloud_centers",  100,&oct_cb);
	ros::Publisher pub2 = nh.advertise<sensor_msgs::PointCloud2> ("/velodyne_points", 1);

  ros::Rate rate(1);
	while(ros::ok()){
		rate.sleep();
		ros::spinOnce();
		if(got_map){
			checktf();
      pub2.publish(pc1_to_pc2(get_pc_within()));
		}
  }
  return 0;
}
