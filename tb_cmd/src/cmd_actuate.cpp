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
ros::Publisher pub_arm_cmd1,pub_arm_cmd2_pan,pub_arm_cmd2_tilt;
geometry_msgs::Vector3 rpy;
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
  cmd_tilt = msg->data;
}
void arm2tilt_cb(const std_msgs::Float64::ConstPtr& msg){
  cmd_arm2tilt = msg->data;
}
void arm2pan_cb(const std_msgs::Float64::ConstPtr& msg){
  cmd_arm2pan = msg->data;
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
		ros::Rate rate(50);

	while(ros::ok()){
		update_pos();
		send_tilt(get_tilt_setpoint());
		send_tilt2(get_arm2tilt_setpoint());
		send_pan(get_arm2pan_setpoint());
		rate.sleep();
		ros::spinOnce();
	}
	return 0;
}
