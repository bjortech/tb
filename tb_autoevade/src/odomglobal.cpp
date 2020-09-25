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
#include <nav_msgs/Path.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float32.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/NavSatFix.h>
#include <eigen3/Eigen/Core>

double par_dt_forward,par_dt_backward;
double get_shortest(double target_yaw,double actual_yaw){
  double a = target_yaw - actual_yaw;
  if(a > M_PI)a -= M_PI*2;
  else if(a < -M_PI)a += M_PI*2;
  return a;
}
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
  return atan2(dy,dx);
}
float get_shortest(float target_heading,float actual_hdng){
  float a = target_heading - actual_hdng;
  if(a > M_PI)a -= M_PI*2;
  else if(a < -M_PI)a += M_PI*2;
  return a;
}

geometry_msgs::Vector3 vel_global2local(geometry_msgs::Vector3 global_vel,float yaw_used) {
	geometry_msgs::Vector3 local_vel;
  float vel_scalar  = sqrt(global_vel.x * global_vel.x + global_vel.y * global_vel.y);
  float rpy_rel     = atan2(global_vel.y,global_vel.x) - yaw_used;
  local_vel.x = vel_scalar * cos(rpy_rel);
  local_vel.y = vel_scalar * sin(rpy_rel);
	local_vel.z = global_vel.z;
	return local_vel;
}
int main(int argc, char **argv){
  ros::init(argc, argv, "tb_robot_odomglobal_node");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");
	double par_dt_forward,par_dt_backward;
	private_nh.param("dt_forward_horizon", par_dt_forward, 1.5);
	private_nh.param("dt_backward_horizon", par_dt_backward, 1.5);
	tf2_ros::Buffer tfBuffer;
	tf2_ros::TransformListener tf2_listener(tfBuffer);
	tf2_ros::TransformBroadcaster tf_b;

	double node_rate = 50;
	geometry_msgs::TransformStamped tf_future;
	nav_msgs::Odometry odom_global;
	geometry_msgs::Vector3 rpy;

	tf_future.header.frame_id        = "map";
	tf_future.child_frame_id         = "base_future";
	tf_future.transform.rotation.w   = 1;

	odom_global.header.frame_id         = "map";
	odom_global.child_frame_id          = "base_stabilized";
	odom_global.pose.pose.orientation.w = 1;

	ros::Publisher pub_global_odom    = nh.advertise<nav_msgs::Odometry>("/tb_autoevade/odom_global", 100);
	ros::Rate rate(node_rate);

	double dt;
  float lastyaw;
  ros::Time time_last = ros::Time::now();
  std::queue<float> vx_que;
  std::queue<float> vy_que;
  std::queue<float> vz_que;
  std::queue<float> vyaw_que;

  int que_size = par_dt_backward * node_rate;
  float vx_sum = 0;
  float vy_sum = 0;
  float vz_sum = 0;
  float vyaw_sum = 0;
  while(ros::ok()){
	  geometry_msgs::TransformStamped tf;
		rate.sleep();
		ros::spinOnce();
	  try{
	    tf = tfBuffer.lookupTransform("map","base_stabilized",
	                             ros::Time(0));
	  }
	  catch (tf2::TransformException &ex) {
	   // ROS_WARN("%s",ex.what());
	    ros::Duration(1.0).sleep();
			continue;
	  }
		float dt 	 = (ros::Time::now() - time_last).toSec();
		time_last  = tf.header.stamp;
    float vyaw = get_shortest(rpy.z,lastyaw)/dt;
    float vz   = (tf.transform.translation.z - odom_global.pose.pose.position.z)/dt;
    float vy   = (tf.transform.translation.y - odom_global.pose.pose.position.y)/dt;
    float vx   = (tf.transform.translation.x - odom_global.pose.pose.position.x)/dt;
		tf2::Matrix3x3 q(tf2::Quaternion(tf.transform.rotation.x, tf.transform.rotation.y, tf.transform.rotation.z, tf.transform.rotation.w));
		q.getRPY(rpy.x,rpy.y,rpy.z);
		//ROS_INFO("ODOMGLOBAL: vxyz %.2f %.2f %.2f, yaw: %.2f vyaw: %.2f dt: %.2f",vx,vy,vz,rpy.z,vyaw,dt);
    if(!std::isnan(vz) && !std::isnan(vy) && !std::isnan(vx) && !std::isnan(vyaw)){
      lastyaw = rpy.z;
      vx_que.push(vx);
      vy_que.push(vy);
      vz_que.push(vz);
      vyaw_que.push(vyaw);

      vx_sum += vx;
      vy_sum += vy;
      vz_sum += vz;
      vyaw_sum += vyaw;

      if(vz_que.size() > que_size){
        vx_sum -= vx_que.front();
        vy_sum -= vy_que.front();
        vz_sum -= vz_que.front();
        vyaw_sum -= vyaw_que.front();
        if(std::isnan(vx_sum) || std::isnan(vy_sum) || std::isnan(vz_sum) || std::isnan(vyaw_sum)){
					ROS_ERROR("RESET: %.2f %.2f %.2f %.2f",vx_sum,vy_sum,vz_sum,vyaw_sum);
          vx_sum = vy_sum = vz_sum = vyaw_sum = 0;
        }
        odom_global.twist.twist.linear.x  = vx_sum / que_size;
        odom_global.twist.twist.linear.y  = vy_sum / que_size;
        odom_global.twist.twist.linear.z  = vz_sum / que_size;
        odom_global.twist.twist.angular.z = vyaw_sum / que_size;

        vx_que.pop();
        vy_que.pop();
        vz_que.pop();
        vyaw_que.pop();
        float hdng = rpy.z;
        float incl = atan2(odom_global.twist.twist.linear.z,sqrt(pow(odom_global.twist.twist.linear.x,2)+pow(odom_global.twist.twist.linear.y,2)));
        //hdng = atan2(odom_global.twist.twist.linear.y,odom_global.twist.twist.linear.x);
        odom_global.pose.pose.position.x = tf.transform.translation.x;
        odom_global.pose.pose.position.y = tf.transform.translation.y;
        odom_global.pose.pose.position.z = tf.transform.translation.z;
		//		ROS_INFO("Pos: %.0f %.0f %.0f - vxyz_ave: (%.2f %.2f %.2f) vxyz: %.2f %.2f %.2f posfuture: %.2f %.2f %.2f",odom_global.pose.pose.position.x,odom_global.pose.pose.position.y,odom_global.pose.pose.position.z,				odom_global.twist.twist.linear.x,odom_global.twist.twist.linear.y,odom_global.twist.twist.linear.z,vx,vy,vz,tf_future.transform.translation.x,tf_future.transform.translation.y,tf_future.transform.translation.z);
				tf_future.transform.translation.x = odom_global.pose.pose.position.x + odom_global.twist.twist.linear.x * par_dt_forward;
				tf_future.transform.translation.y = odom_global.pose.pose.position.y + odom_global.twist.twist.linear.y * par_dt_forward;
				tf_future.transform.translation.z = odom_global.pose.pose.position.z + odom_global.twist.twist.linear.z * par_dt_forward;

        if(std::isnan(tf_future.transform.translation.x) || std::isnan(tf_future.transform.translation.y) || std::isnan(tf_future.transform.translation.z)){
          ROS_ERROR("TF: std_isnan: odomglobal:  %.2f %.2f %.2f %.2f, vsum xyz: %.2f %.2f %.2f %.2f", odom_global.twist.twist.linear.x, odom_global.twist.twist.linear.y,odom_global.twist.twist.linear.z,odom_global.twist.twist.angular.z,vx_sum,vy_sum,vz_sum,vyaw_sum);
        }
        tf_future.transform.rotation = tf::createQuaternionMsgFromYaw(constrainAngle(rpy.z + odom_global.twist.twist.angular.z));
        odom_global.pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0,-incl,hdng);
				tf_future.transform.rotation 			= tf::createQuaternionMsgFromYaw(hdng);
				tf_future.header.stamp = odom_global.header.stamp = ros::Time::now();
        pub_global_odom.publish(odom_global);
				tf_b.sendTransform(tf_future);
      }
    }
  }
  return 0;
}
