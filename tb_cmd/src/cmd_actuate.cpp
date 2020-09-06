#include <ros/ros.h>
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
#include <eigen3/Eigen/Core>
tf2_ros::Buffer tfBuffer;
std_msgs::Float64 arm_cmd1,arm2_cmd_pan,arm2_cmd_tilt;
float cmd_tilt,cmd_arm2tilt,cmd_arm2pan;
float setp_tilt = -M_PI/14;
float setp_arm2tilt = -M_PI/4;
float setp_arm2pan  = 0.0;
ros::Publisher pub_arm_cmd1,pub_arm_cmd2_pan,pub_arm_cmd2_tilt;
geometry_msgs::Vector3 rpy;
geometry_msgs::Point pos;
geometry_msgs::PointStamped arm1_vp,arm2_vp;
float saturate(float val, float max){
  if((std::isnan(val)) || (std::isinf(val)))
    return 0;
  else if(val < 0 && val*-1 > max)
    return -max;
  else if(val > max)
    return max;
  else
    return val;
}
float get_tilt_setpoint(){
  return cmd_tilt - rpy.y;
}
float get_arm2tilt_setpoint(){
  return cmd_arm2tilt - rpy.y;
}
float get_arm2pan_setpoint(){
  return cmd_arm2pan - rpy.z;
}
float get_dst2d(geometry_msgs::Point p1, geometry_msgs::Point p2){
  return(sqrt(pow(p1.x-p2.x,2)+pow(p1.y-p2.y,2)));
}

float get_dst3d(geometry_msgs::Point p1, geometry_msgs::Point p2){
  return(sqrt(pow(p1.x-p2.x,2)+pow(p1.y-p2.y,2)+pow(p1.z-p2.z,2)));
}
float get_hdng(geometry_msgs::Point p1,geometry_msgs::Point p0){
  float dx = p1.x - p0.x;
  float dy = p1.y - p0.y;
  return atan2(dy,dx);
}
float get_inclination(geometry_msgs::Point p2, geometry_msgs::Point p1){
  return atan2(p2.z - p1.z,get_dst2d(p1,p2));
}
void update_pos(){
  geometry_msgs::TransformStamped transformStamped;
  try{
    transformStamped = tfBuffer.lookupTransform("map","base_link",
                             ros::Time(0));
  }
  catch (tf2::TransformException &ex) {
    ROS_WARN("%s",ex.what());
    ros::Duration(1.0).sleep();
  }
	tf2::Matrix3x3 q(tf2::Quaternion(transformStamped.transform.rotation.x, transformStamped.transform.rotation.y, transformStamped.transform.rotation.z, transformStamped.transform.rotation.w));
	q.getRPY(rpy.x,rpy.y,rpy.z);
	rpy.y *= -1;
	pos.x = transformStamped.transform.translation.x;
  pos.y = transformStamped.transform.translation.y;
  pos.z = transformStamped.transform.translation.z;
}
void update_targets(){
	if((ros::Time::now()-arm1_vp.header.stamp).toSec() < 1.0){
		cmd_tilt = get_inclination(arm1_vp.point,pos);
	}
	else{
		cmd_tilt = setp_tilt;
	}
	if((ros::Time::now()-arm2_vp.header.stamp).toSec() < 1.0){
		cmd_arm2tilt = get_inclination(arm2_vp.point,pos);
		cmd_arm2pan  = get_hdng(arm2_vp.point,pos);
	}
	else{
		cmd_arm2tilt 	= setp_arm2tilt;
		cmd_arm2pan 	= setp_arm2pan;
	}
}
void send_tilt(float armcmd){
  arm2_cmd_tilt.data = armcmd;
  if(arm2_cmd_tilt.data > 0.0)
    arm2_cmd_tilt.data = saturate(arm2_cmd_tilt.data,M_PI/2-M_PI/17);
  if(arm2_cmd_tilt.data < 0.0)
    arm2_cmd_tilt.data = saturate(arm2_cmd_tilt.data,M_PI/2-M_PI/12);
  pub_arm_cmd1.publish(arm2_cmd_tilt);
}
void send_tilt2(float armcmd){
  arm_cmd1.data = armcmd;
  if(arm_cmd1.data > 0.0)
    arm_cmd1.data = saturate(arm_cmd1.data,M_PI/2-M_PI/17);
  if(arm_cmd1.data < 0.0)
    arm_cmd1.data = saturate(arm_cmd1.data,M_PI/2-M_PI/12);
  pub_arm_cmd2_tilt.publish(arm_cmd1);
}
void send_pan(float armcmd){
  arm2_cmd_pan.data = armcmd;
  if(arm2_cmd_pan.data > 0.0)
    arm2_cmd_pan.data = saturate(arm2_cmd_pan.data,-M_PI/3);
  if(arm2_cmd_pan.data < 0.0)
    arm2_cmd_pan.data = saturate(arm2_cmd_pan.data,M_PI/3);
  pub_arm_cmd2_pan.publish(arm2_cmd_pan);
}
void tilt_cb(const std_msgs::Float64::ConstPtr& msg){
  setp_tilt = msg->data;
}
void arm2tilt_cb(const std_msgs::Float64::ConstPtr& msg){
  setp_arm2tilt = msg->data;
}
void arm2pan_cb(const std_msgs::Float64::ConstPtr& msg){
  setp_arm2pan = msg->data;
}
void target_viewpoint_arm1_cb(const geometry_msgs::PointStamped::ConstPtr& msg){
	arm1_vp = *msg;
}
void target_viewpoint_arm2_cb(const geometry_msgs::PointStamped::ConstPtr& msg){
	arm2_vp = *msg;
}
int main(int argc, char **argv){
    ros::init(argc, argv, "tb_cmd_actuation_node");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

		tf2_ros::TransformListener tf2_listener(tfBuffer);
		pub_arm_cmd1		   	= nh.advertise<std_msgs::Float64>("/tilt_controller/command", 100);
		pub_arm_cmd2_pan   	= nh.advertise<std_msgs::Float64>("/pan_controller/command", 100);
		pub_arm_cmd2_tilt 	= nh.advertise<std_msgs::Float64>("/tilt2_controller/command", 100);
		ros::Subscriber s1  = nh.subscribe("/tb_cmd/tilt",1,tilt_cb);
		ros::Subscriber s2  = nh.subscribe("/tb_cmd/arm2_tilt",1,arm2tilt_cb);
		ros::Subscriber s3  = nh.subscribe("/tb_cmd/arm2_pan",1,arm2pan_cb);
		ros::Subscriber s4  = nh.subscribe("/tb_cmd/arm1_viewpoint",1,target_viewpoint_arm1_cb);
		ros::Subscriber s5  = nh.subscribe("/tb_cmd/arm2_viewpoint",1,target_viewpoint_arm2_cb);
		ros::Rate rate(50);

	while(ros::ok()){
		update_pos();
		update_targets();
		send_tilt(get_tilt_setpoint());
		send_tilt2(get_arm2tilt_setpoint());
		send_pan(get_arm2pan_setpoint());
		rate.sleep();
		ros::spinOnce();
	}
	return 0;
}
