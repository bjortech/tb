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

geometry_msgs::TransformStamped tff,tfh,tfrp,tfm,tfo,tfoa,tfa,tfa2,tfp,tfpos,tfb;
geometry_msgs::Vector3 vlp_rpy;
nav_msgs::Odometry odom;
ros::Time time_last_v;
std_msgs::Float64 pz;
double yaw_odom;
tf2_ros::Buffer tfBuffer;
geometry_msgs::Point pos;
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
float get_hdng(geometry_msgs::Point p1,geometry_msgs::Point p0){
  float dx = p1.x - p0.x;
  float dy = p1.y - p0.y;
  return atan2(dy,dx);
}
float get_dst2d(geometry_msgs::Point p1, geometry_msgs::Point p2){
  return(sqrt(pow(p1.x-p2.x,2)+pow(p1.y-p2.y,2)));
}

float get_dst3d(geometry_msgs::Point p1, geometry_msgs::Point p2){
  return(sqrt(pow(p1.x-p2.x,2)+pow(p1.y-p2.y,2)+pow(p1.z-p2.z,2)));
}
float get_inclination(geometry_msgs::Point p2, geometry_msgs::Point p1){
  return atan2(p2.z - p1.z,get_dst2d(p1,p2));
}
void twist_cb(const geometry_msgs::Twist::ConstPtr& msg)
{
    float dt = ((ros::Time::now() - time_last_v).toSec());
    time_last_v                = ros::Time::now();
    odom.twist.twist.angular.z = msg->angular.z;
    odom.twist.twist.linear.x  = msg->linear.x;
    odom.twist.twist.linear.y  = msg->linear.y;
}
void setodom_cb(const nav_msgs::Odometry::ConstPtr& msg)
{
  odom.twist.twist.angular.z = msg->twist.twist.angular.z;
  odom.twist.twist.linear.x  = msg->twist.twist.linear.x;
  odom.twist.twist.linear.y  = msg->twist.twist.linear.y;
  odom.pose.pose.position.x  = msg->pose.pose.position.x;
  odom.pose.pose.position.y  = msg->pose.pose.position.y;
  odom.pose.pose.orientation = msg->pose.pose.orientation;
}
void perfectalt_cb(const std_msgs::Float64::ConstPtr& msg)
{
  tfoa.transform.translation.z = msg->data;
}

void update_pos_vlp(){
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

int main(int argc, char **argv){
    ros::init(argc, argv, "tb_odom_node");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");
    ros::Rate rate(20);

    tf2_ros::TransformListener tf2_listener(tfBuffer);

    tf2_ros::TransformBroadcaster tf_b;

    odom.header.frame_id         = "odom";
    odom.child_frame_id          = "base_perfect";
    odom.pose.pose.orientation.w = 1;

    tfo.header.frame_id          = "odom";
    tfo.child_frame_id           = "base_perfect";
    tfo.transform.rotation.w     = 1;

    tfm.header.frame_id         = "map";
    tfm.child_frame_id          = "odom";
    tfm.transform.rotation.w    = 1;

    tfoa.header.frame_id          = "base_perfect";
    tfoa.child_frame_id           = "base_perfect_alt";
    tfoa.transform.rotation.w     = 1;

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

    ros::Subscriber s2 = nh.subscribe("/cmd_vel",             100,&twist_cb);
    ros::Subscriber s1 = nh.subscribe("/tb_odom/perfect_alt",  100,&perfectalt_cb);
    ros::Subscriber s0 = nh.subscribe("/tb_odom/reset",  100,&setodom_cb);
    ros::Publisher odom_pub  = nh.advertise<nav_msgs::Odometry>("/odom", 100);
    double dt;
    ros::Time time_last = ros::Time::now();
    float vy,vx;

    tfoa.transform.translation.z = 10;
    while(ros::ok()){
      update_pos_vlp();
      float hdng_pos = get_hdng(odom.pose.pose.position,pos);
      float incl_pos = get_inclination(odom.pose.pose.position,pos);
      float dst_pos  = get_dst2d(odom.pose.pose.position,pos);
      bool notuse = true;
      if(dst_pos > 2.0 && sqrt(pow(get_shortest(hdng_pos,vlp_rpy.z),2)) > M_PI/2){
        vx = 0;
        vy = 0;
      }
      else{
        vx = odom.twist.twist.linear.x;
        vy = odom.twist.twist.linear.y;
      }/*
      if(dst_pos > 10){
        float dst_pos  = get_dst2d(odom.pose.pose.position,pos);
        float vxy = sqrt(pow(odom.twist.twist.linear.x,2)+pow(odom.twist.twist.linear.y,2));
        if(vxy > 1.0){
          float vxn = odom.twist.twist.linear.x / vxy;
          float vyn = odom.twist.twist.linear.y / vxy;
          float maxv = (30 - dst_pos)/4;
          vx = maxv * vxn;
          vy = maxv * vyn;
          ROS_INFO("dst_pos %.2f,vxy %.2f,vxn %.2f,vyn %.2f,maxv %.2f,vx %.2f, vy %.2f",dst_pos,vxy,vxn,vyn,maxv,vx,vy);
        }
      }
      else{
        vx = odom.twist.twist.linear.x;
        vy = odom.twist.twist.linear.y;
      }
*/
      if((ros::Time::now() - time_last_v).toSec() > 0.5) odom.twist.twist.angular.z = odom.twist.twist.linear.x = odom.twist.twist.linear.y  = 0;
      if((ros::Time::now() - time_last).toSec() > 0.03) dt = 0.03;
      else dt= (ros::Time::now() - time_last).toSec();
      time_last = ros::Time::now();
      rate.sleep();
      ros::spinOnce();
      odom.pose.pose.position.x  += ((vx*cos(yaw_odom)-vy*sin(yaw_odom)) * dt);
      odom.pose.pose.position.y  += ((vx*sin(yaw_odom)+vy*cos(yaw_odom)) * dt);
      yaw_odom                   += odom.twist.twist.angular.z * dt;
      yaw_odom                    = constrainAngle(yaw_odom);

      tfo.transform.translation.x = odom.pose.pose.position.x;
      tfo.transform.translation.y = odom.pose.pose.position.y;
      tfo.transform.rotation      = odom.pose.pose.orientation = tf::createQuaternionMsgFromYaw(yaw_odom);

      odom.header.stamp = tfo.header.stamp = tfoa.header.stamp = tfm.header.stamp = ros::Time::now();
      tf_b.sendTransform(tfoa);
      tf_b.sendTransform(tfo);
      tf_b.sendTransform(tfm);
      odom_pub.publish(odom);
    }
  return 0;
}
