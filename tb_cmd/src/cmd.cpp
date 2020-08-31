#include <ros/ros.h>
#include <queue>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_listener.h>
#include "tf2_ros/message_filter.h"
#include "message_filters/subscriber.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include <tf/transform_datatypes.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Range.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Joy.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float32.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/NavSatFix.h>
#include <eigen3/Eigen/Core>
#include <nav_msgs/Path.h>

geometry_msgs::TransformStamped tff,tfh,tfrp,tfm,tf_odom,tf_odom_alt,tfa,tfa2,tfp,tfpos,tfb;
nav_msgs::Odometry odom;
ros::Time time_last_v;
double yaw_odom;
tf2_ros::Buffer tfBuffer;
bool active = true;
nav_msgs::Path path_cmd,path_in;
int path_cmd_i = 0;
geometry_msgs::Point pos;
float poses_pr_sec   = 1.0;
float meters_pr_pose = 0.0;
float meters_pr_sec  = 8.0;
float poses_pr_meter = 0.0;
ros::Time path_start;
bool path_started;
double constrainAngle(double x){
  if(x > M_PI)
    return (x - M_PI*2);
  else if(x < -M_PI)
    return (x + M_PI*2);
  else
    return x;
}
float get_hdng(geometry_msgs::Point p1,geometry_msgs::Point p0){
  float dx = p1.x - p0.x;
  float dy = p1.y - p0.y;
	float h = atan2(dy,dx);
	if(std::isnan(h) || abs(h) > 7)
		h = 0.0;
  return h;
}
void twist_cb(const geometry_msgs::Twist::ConstPtr& msg)
{
  time_last_v                = ros::Time::now();
  odom.twist.twist.angular.z = msg->angular.z;
  odom.twist.twist.linear.x  = msg->linear.x;
  odom.twist.twist.linear.y  = msg->linear.y;
}
void set_z_cb(const std_msgs::Float64::ConstPtr& msg)
{
  tf_odom_alt.transform.translation.z = msg->data;
}
void set_yaw_cb(const std_msgs::Float64::ConstPtr& msg)
{
	yaw_odom = msg->data;
}
void set_xy_cb(const geometry_msgs::PointStamped::ConstPtr& msg)
{
	odom.pose.pose.position.x = msg->point.x;
	odom.pose.pose.position.y = msg->point.y;
}
float get_dst2d(geometry_msgs::Point p1, geometry_msgs::Point p2){
  return(sqrt(pow(p1.x-p2.x,2)+pow(p1.y-p2.y,2)));
}

float get_dst3d(geometry_msgs::Point p1, geometry_msgs::Point p2){
  return(sqrt(pow(p1.x-p2.x,2)+pow(p1.y-p2.y,2)+pow(p1.z-p2.z,2)));
}

void update_pos(){
  geometry_msgs::TransformStamped transformStamped;
  try{
    transformStamped = tfBuffer.lookupTransform("map","base_stabilized",
                             ros::Time(0));
  }
  catch (tf2::TransformException &ex) {
    ROS_WARN("%s",ex.what());
    ros::Duration(1.0).sleep();
  }
  pos.x = transformStamped.transform.translation.x;
  pos.y = transformStamped.transform.translation.y;
  pos.z = transformStamped.transform.translation.z;

}
int getinpath_closestindex2d(nav_msgs::Path pathin,geometry_msgs::Point pnt){
	int lowest_dist_i = 0;
	float lowest_dist = 1000;
	for(int i = 0; i < pathin.poses.size(); i++){
		float dst = get_dst2d(pathin.poses[i].pose.position,pnt);
		if(dst < lowest_dist){
			lowest_dist_i = i;
			lowest_dist = dst;
		}
	}
	return lowest_dist_i;
}
int get_path_cmd_i(){
	if(!path_started){
		int ci = getinpath_closestindex2d(path_cmd,pos);
		float dst_closest = get_dst2d(path_cmd.poses[ci].pose.position,pos);
		ROS_INFO("dst_closest [%i] = %.0f",ci,dst_closest);
		if(dst_closest < 5.0){
			path_started = true;
			path_start = ros::Time::now();
			return 1;
		}
		else
			return 0;
	}
	else{
		float dt_start = (ros::Time::now()-path_start).toSec();
		float dst_behind = dt_start * meters_pr_sec;
		int poses_behind = dst_behind / poses_pr_meter;
		ROS_INFO("Return: %.3f sec %.0f m %i poses",dt_start,dst_behind,poses_behind);
		return poses_behind;
	}
}

/*	if(path_cmd.poses.size() > path_cmd_i+2){
		int closest = getinpath_closestindex2d(path_cmd,odom.pose.pose.position);
		float dst 	= get_dst2d(path_cmd.poses[closest].pose.position,odom.pose.pose.position);
		if(dst > 3.0)
			return closest+4;
		else
			return closest+6;
	}
	return 0;
}*/

void path_cb(const nav_msgs::Path::ConstPtr& msg){
	if(path_cmd.poses.size() == 0 || (path_cmd.poses.size() > 0 && path_cmd_i > path_cmd.poses.size()-10)){
		path_cmd = *msg;
		path_cmd_i = 0;
		path_started = true;
		path_start = ros::Time::now();
		meters_pr_pose = get_dst2d(msg->poses[0].pose.position,msg->poses[1].pose.position);
		poses_pr_meter = 1 / meters_pr_pose;
		poses_pr_sec   = poses_pr_meter * meters_pr_sec;
		ROS_INFO("PATHMASTER[%i]: GOT PATH poses_pr_meter/sec: %.2f/%.2f, meters_pr_pose/sec: %.2f/%.2f",path_cmd.poses.size(),poses_pr_meter,poses_pr_sec,meters_pr_pose,meters_pr_sec);
	}
}
int main(int argc, char **argv){
    ros::init(argc, argv, "tb_cmd_setpoint_node");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");
    ros::Rate rate(20);

    tf2_ros::TransformBroadcaster tf_b;

    odom.header.frame_id         		 = "odom";
    odom.child_frame_id          		 = "base_perfect";
    odom.pose.pose.orientation.w 		 = 1;

    tf_odom.header.frame_id          = "odom";
    tf_odom.child_frame_id           = "base_perfect";
    tf_odom.transform.rotation.w     = 1;

    tf_odom_alt.header.frame_id      = "base_perfect";
    tf_odom_alt.child_frame_id       = "base_perfect_alt";
    tf_odom_alt.transform.rotation.w = 1;

    odom.pose.covariance[0] = (1e-3);
    odom.pose.covariance[7] = (1e-3);
    odom.pose.covariance[14] = (1e-3);
    odom.pose.covariance[21] = (1e-3);
    odom.pose.covariance[28] = (1e-3);
    odom.pose.covariance[35] = (1e-3);
    odom.twist.covariance[0] = 1e-3;
    odom.twist.covariance[7] = 1e-3;
    odom.twist.covariance[14] = 1e-3;
    odom.twist.covariance[21] = 1e-3;
    odom.twist.covariance[35] = 1e-3;

		ros::Subscriber s0      = nh.subscribe("/tb_cmd/path", 100,&path_cb);
    ros::Subscriber s4 			= nh.subscribe("/cmd_vel",     100,&twist_cb);
		ros::Subscriber s1 			= nh.subscribe("/tb_cmd/set_yaw",100,&set_yaw_cb);
		ros::Subscriber s2 			= nh.subscribe("/tb_cmd/set_xy", 100,&set_xy_cb);
		ros::Subscriber s3 			= nh.subscribe("/tb_cmd/set_z",  100,&set_z_cb);
    ros::Publisher odom_pub = nh.advertise<nav_msgs::Odometry>("/odom", 100);
    ros::Time time_last = ros::Time::now();

    tf_odom_alt.transform.translation.z = 10;
    while(ros::ok()){
			update_pos();
			if(path_started){
				float dt_start = (ros::Time::now()-path_start).toSec();
				path_cmd_i = dt_start * poses_pr_meter * meters_pr_sec;
				ROS_INFO("Return: %.3f sec %.0f m %i poses",dt_start,path_cmd_i);
				odom.pose.pose.position.x 					= path_cmd.poses[path_cmd_i].pose.position.x;
				odom.pose.pose.position.y 					= path_cmd.poses[path_cmd_i].pose.position.y;
				tf_odom_alt.transform.translation.z = path_cmd.poses[path_cmd_i].pose.position.z;
				yaw_odom = get_hdng(odom.pose.pose.position,pos);
			}
			else if((ros::Time::now() - time_last_v).toSec() < 0.5){
				float dt = fmin((ros::Time::now() - time_last).toSec(),0.1);
				time_last = ros::Time::now();
				odom.pose.pose.position.x  += ((odom.twist.twist.linear.x*cos(yaw_odom)-odom.twist.twist.linear.y*sin(yaw_odom)) * dt);
				odom.pose.pose.position.y  += ((odom.twist.twist.linear.x*sin(yaw_odom)+odom.twist.twist.linear.y*cos(yaw_odom)) * dt);
				yaw_odom                   += odom.twist.twist.angular.z * dt;
				yaw_odom                    = constrainAngle(yaw_odom);
			}
			else{
				odom.twist.twist.angular.z = odom.twist.twist.linear.x = odom.twist.twist.linear.y  = 0;
			}

			tf_odom.transform.translation.x = odom.pose.pose.position.x;
      tf_odom.transform.translation.y = odom.pose.pose.position.y;
      tf_odom.transform.rotation      = odom.pose.pose.orientation = tf::createQuaternionMsgFromYaw(yaw_odom);

      odom.header.stamp = tf_odom.header.stamp = tf_odom_alt.header.stamp = ros::Time::now();
      tf_b.sendTransform(tf_odom_alt);
      tf_b.sendTransform(tf_odom);
      odom_pub.publish(odom);

			rate.sleep();
			ros::spinOnce();
    }
  return 0;
}
