#include <ros/ros.h>
#include <octomap_msgs/conversions.h>
#include <octomap_msgs/Octomap.h>
#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <octomap/OcTreeBase.h>
#include <octomap/octomap_types.h>
#include <dynamicEDT3D/dynamicEDTOctomap.h>
#include <chrono>
#include <std_msgs/UInt8.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf/transform_datatypes.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_listener.h>
#include "tf2_ros/message_filter.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include <nav_msgs/Path.h>
#include <std_msgs/UInt8.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Point32.h>
#include <eigen3/Eigen/Core>
#include <geometry_msgs/Vector3Stamped.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <image_transport/image_transport.h>
using namespace octomap;
using namespace std;
tf2_ros::Buffer tfBuffer;
bool active,new_path,realpath_received;
double par_errorlength,setpoint_yaw,actual_yaw;
geometry_msgs::Point setpoint;
geometry_msgs::PoseStamped pose_setpoint;
const float deg2rad = M_PI/180.0;
const float rad2deg = 180.0/M_PI;
const float earth = 6378137.0;

double get_shortest(double target_hdng,double actual_hdng){
  double a = target_hdng - actual_hdng;
  if(a > M_PI)a -= M_PI*2;
  else if(a < -M_PI)a += M_PI*2;
  return a;
}

double saturate(double val, double max){
    if (abs(val) >= max)
      val = (val>0) ? max : -1 * max;
    else
      val = val;
    if((std::isnan(val)) || (std::isinf(val))){
      return 0;
    }
    else{
      return val;
    }
}
void targetpose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
	setpoint = msg->pose.position;
	pose_setpoint = *msg;
}
geometry_msgs::Vector3 vel_global2local(geometry_msgs::Vector3 global_vel) {
	geometry_msgs::Vector3 local_vel;
  float vel_scalar  = sqrt(global_vel.x * global_vel.x + global_vel.y * global_vel.y);
  float rpy_rel     = atan2(global_vel.y,global_vel.x) - actual_yaw;
  local_vel.x = vel_scalar * cos(rpy_rel);
  local_vel.y = vel_scalar * sin(rpy_rel);
	local_vel.z = global_vel.z;
	return local_vel;
}
int main(int argc,char** argv){
  ros::init(argc,argv,"tb_setp_node");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");
  private_nh.param("setpoint_velocity",  par_errorlength, 5.0);
  tf2_ros::TransformListener tf2_listener(tfBuffer);
  geometry_msgs::TransformStamped tfs;
	ros::Subscriber s0    = nh.subscribe("/tb_cmdmb/target_pose", 10,&targetpose_cb);
	ros::Publisher pub_cmd     = nh.advertise<geometry_msgs::Twist>("/cmd_vel",10);
	ros::Publisher pub_setp    = nh.advertise<geometry_msgs::PoseStamped>("/tb_spnt/target_setpoint",10);

  ros::Rate rate(20.0);
  ros::Time now;
	geometry_msgs::Twist cmd,global;
	geometry_msgs::TransformStamped transformStamped;

  float cmdyaw;
  while(ros::ok()){
		try{
			transformStamped = tfBuffer.lookupTransform("map","base_stabilized",
															 ros::Time(0));
		}
		catch (tf2::TransformException &ex) {
			ROS_WARN("%s",ex.what());
			ros::Duration(1.0).sleep();
		}
		Eigen::Vector3f pnt1_vec(transformStamped.transform.translation.x,transformStamped.transform.translation.y,transformStamped.transform.translation.z);
		Eigen::Vector3f pnt2_vec(setpoint.x,setpoint.y,setpoint.z);
		Eigen::Vector3f stride_vec;
		float error_length = (pnt2_vec - pnt1_vec).norm();
		float cmd_length = saturate(error_length,par_errorlength);

		stride_vec = (pnt2_vec - pnt1_vec).normalized() * cmd_length;
		global.linear.x = stride_vec.x();
		global.linear.y = stride_vec.y();
		global.linear.z = stride_vec.z();
		actual_yaw   = tf::getYaw(transformStamped.transform.rotation);
		setpoint_yaw = atan2(cmd.linear.y,cmd.linear.x);
		cmd.angular.z = saturate(get_shortest(setpoint_yaw,actual_yaw),2);
		cmd.linear = vel_global2local(global.linear);
		pose_setpoint.pose.orientation = tf::createQuaternionMsgFromYaw(atan2(cmd.linear.y,cmd.linear.x));
		pose_setpoint.header.stamp = ros::Time::now();
		pub_setp.publish(pose_setpoint);

		pub_cmd.publish(cmd);
    rate.sleep();
    ros::spinOnce();
  }
  return 0;
}
