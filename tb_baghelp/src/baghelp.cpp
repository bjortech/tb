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

geometry_msgs::TransformStamped tf_odom,tf_odom_alt;
nav_msgs::Odometry odom;
ros::Time time_last_v;
double yaw_odom;
tf2_ros::Buffer tfBuffer;
nav_msgs::Path pathcmd;
int pathcmd_i = 0;
geometry_msgs::Vector3 rpy;

void attitude_quaternion_cb(const geometry_msgs::QuaternionStamped::ConstPtr& msg){
  tf2::Matrix3x3 q(tf2::Quaternion(msg->quaternion.x, msg->quaternion.y, msg->quaternion.z, msg->quaternion.w));
  q.getRPY(rpy.x,rpy.y,rpy.z);
  if(std::isnan(rpy.z) || std::isnan(rpy.y) || std::isnan(rpy.x))
    rpy.z = rpy.x = rpy.y = 0;
}
double constrainAngle(double x){
  if(x > M_PI)
    return (x - M_PI*2);
  else if(x < -M_PI)
    return (x + M_PI*2);
  else
    return x;
}
void set_z_cb(const std_msgs::Float64::ConstPtr& msg)
{
  tf_odom_alt.transform.translation.z = msg->data;
}
void set_yaw_cb(const std_msgs::Float64::ConstPtr& msg)
{
	yaw_odom = msg->data;
}
void position_point_cb(const geometry_msgs::PointStamped::ConstPtr& msg)
{
	odom.pose.pose.position.x = msg->point.x;
	odom.pose.pose.position.y = msg->point.y;
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

		ros::Subscriber s3  = nh.subscribe("/dji_sdk/attitude",             100,&attitude_quaternion_cb);
    ros::Subscriber s5  = nh.subscribe("/dji_sdk/local_position",       100,&position_point_cb);

    ros::Publisher odom_pub = nh.advertise<nav_msgs::Odometry>("/odom", 100);
    ros::Time time_last = ros::Time::now();

    tf_odom_alt.transform.translation.z = 10;

    while(ros::ok()){

			tf_odom.transform.translation.x = odom.pose.pose.position.x;
      tf_odom.transform.translation.y = odom.pose.pose.position.y;
      tf_odom.transform.rotation      = odom.pose.pose.orientation = tf::createQuaternionMsgFromYaw(rpy.y);

      odom.header.stamp = tf_odom.header.stamp = tf_odom_alt.header.stamp = ros::Time::now();
      tf_b.sendTransform(tf_odom_alt);
      tf_b.sendTransform(tf_odom);
      odom_pub.publish(odom);

			rate.sleep();
			ros::spinOnce();
    }
  return 0;
}
