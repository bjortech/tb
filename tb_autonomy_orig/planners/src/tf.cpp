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

bool first = true;
double par_zclearing,yaw_odom,target_arm1,target_arm2_tilt,target_arm2_pan,t_int_1,t_int_2,t_int_3;
const float deg2rad = M_PI/180.0; const float rad2deg = 180.0/M_PI; const float earth = 6378137.0;
int mainstate;
bool test_odom = true;

ros::Time time_last_v,time_last_z;
tf2_ros::Buffer tfBuffer;
geometry_msgs::Vector3 rpy,rpyx;
geometry_msgs::Vector3Stamped vlocal_gps,vglobal_gps,armvec_min_max_radprs;
geometry_msgs::TransformStamped tfsdunkarm,tff,tfh,tfrp,tfm,tfo,tfoa,tfa,tfa2,tfp,tfpos,tfb,tfs,tfx,tfv,tf_up,tf_dwn,tfvt;
double vel_xy_max = 5.0;
float zmax_pathpos = 10;
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
float get_dst3d(geometry_msgs::Point p1, geometry_msgs::Point p2){
  return(sqrt(pow(p1.x-p2.x,2)+pow(p1.y-p2.y,2)+pow(p1.z-p2.z,2)));
}
float get_dst2d(geometry_msgs::Point p1, geometry_msgs::Point p2){
  return(sqrt(pow(p1.x-p2.x,2)+pow(p1.y-p2.y,2)));
}
float get_inclination(geometry_msgs::Point p2, geometry_msgs::Point p1){
  return atan2(p2.z - p1.z,get_dst2d(p1,p2));
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
void att_cb(const geometry_msgs::QuaternionStamped::ConstPtr& msg){
  tf2::Matrix3x3 q(tf2::Quaternion(msg->quaternion.x, msg->quaternion.y, msg->quaternion.z, msg->quaternion.w));
  q.getRPY(rpy.x,rpy.y,rpy.z);
  if(std::isnan(rpy.z) || std::isnan(rpy.y) || std::isnan(rpy.x))
    rpy.z = rpy.x = rpy.y = 0;
}
void pos_cb(const geometry_msgs::PointStamped::ConstPtr& msg){
  tff.transform.translation.x = msg->point.x;
  tff.transform.translation.y = msg->point.y;
}
void spose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
  tfs.transform.translation.x = msg->pose.position.x;
  tfs.transform.translation.y = msg->pose.position.y;
  tfs.transform.translation.z = msg->pose.position.z;
  tfs.transform.rotation      = msg->pose.orientation;
}
void height_cb(const std_msgs::Float32::ConstPtr& msg){
  tfh.transform.translation.z = msg->data;
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
geometry_msgs::Quaternion get_quat(geometry_msgs::Point to, geometry_msgs::Point from){
  float incl = atan2(to.z - from.z,get_dst2d(to,from));
  float hdng = get_hdng(to,from);
  ROS_INFO("Incl: %.2f hdng: %.2f",incl,hdng);

  return tf::createQuaternionMsgFromRollPitchYaw(0,incl,hdng);
}
int main(int argc, char **argv){
    ros::init(argc, argv, "tb_tf_node");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    ros::Rate rate(100);
    nav_msgs::Odometry odom_global;
    tf2_ros::TransformBroadcaster tf_b;

    tff.header.frame_id         = "odom";
    tff.child_frame_id          = "base_footprint";
    tff.transform.rotation.w    = 1;

    tfh.header.frame_id         = "base_footprint";
    tfh.child_frame_id          = "base_stabilized";
    tfh.transform.rotation.w    = 1;

    tfrp.header.frame_id        = "base_stabilized";
    tfrp.child_frame_id         = "base_link";
    tfrp.transform.rotation.w   = 1;

    tfpos.header.frame_id         = "odom";
    tfpos.child_frame_id          = "base_position";
    tfpos.transform.rotation.w    = 1;

    tfp.header.frame_id        = "base_stabilized";
    tfp.child_frame_id         = "base_future";
    tfp.transform.rotation.w   = 1;

    odom_global.header.frame_id         = "odom";
    odom_global.child_frame_id          = "global";
    odom_global.pose.pose.orientation.w = 1;

    tfs.header.frame_id          = "map";
    tfs.child_frame_id           = "pose_stereo_slam";
    tfs.transform.rotation.w     = 1;

    tfoa.header.frame_id          = "base_perfect";
    tfoa.child_frame_id           = "base_perfect_alt";
    tfoa.transform.rotation.w     = 1;

    ros::Subscriber s22 = nh.subscribe("/dji_sdk/height_above_takeoff", 100,&height_cb);
    ros::Subscriber s3  = nh.subscribe("/dji_sdk/attitude",             100,&att_cb);
    ros::Subscriber s5  = nh.subscribe("/dji_sdk/local_position",       100,&pos_cb);
    ros::Subscriber ss  = nh.subscribe("/pose",                       100,&spose_cb);
		ros::Publisher pub_global_odom    = nh.advertise<nav_msgs::Odometry>("/odom_global", 100);

		double dt;
    float lastyaw;
    ros::Time time_last = ros::Time::now();
    std::queue<float> vx_que;
    std::queue<float> vy_que;
    std::queue<float> vz_que;
    std::queue<float> vyaw_que;

    int que_size = 100;
    float vx_sum = 0;
    float vy_sum = 0;
    float vz_sum = 0;
    float vyaw_sum = 0;

     geometry_msgs::Vector3 cmd_global,cmd_local;
     geometry_msgs::Point pos_cmd;

     bool ros_inform = false;
    while(ros::ok()){
      rate.sleep();
      ros::spinOnce();

      if((ros::Time::now() - time_last).toSec() > 0.03) dt = 0.03;
      else dt = (ros::Time::now() - time_last).toSec();
      time_last = ros::Time::now();

      tfrp.transform.rotation     	= tf::createQuaternionMsgFromRollPitchYaw(rpy.x,rpy.y,0);
      tff.transform.rotation     	  = tf::createQuaternionMsgFromRollPitchYaw(0,0,rpy.z);
      odom_global.header.stamp = tfs.header.stamp = tfpos.header.stamp = tfp.header.stamp = tfh.header.stamp = tfrp.header.stamp = tff.header.stamp = ros::Time::now();
      tfpos.transform.translation   = tff.transform.translation;
      tfpos.transform.translation.z = tfh.transform.translation.z;
      tf_b.sendTransform(tfrp);
      tf_b.sendTransform(tff);
      tf_b.sendTransform(tfh);
      if(tfs.transform.translation.x + tfs.transform.translation.y + tfs.transform.translation.z > 0)
  			tf_b.sendTransform(tfs);
      tf_b.sendTransform(tfpos);

      float vyaw = get_shortest(rpy.z,lastyaw)/dt;
      float vz   = (tfh.transform.translation.z - odom_global.pose.pose.position.z)/dt;
      float vy   = (tff.transform.translation.y - odom_global.pose.pose.position.y)/dt;
      float vx   = (tff.transform.translation.x - odom_global.pose.pose.position.x)/dt;
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
          if(!std::isnan(vx_sum) && !std::isnan(vy_sum) && !std::isnan(vz_sum) && !std::isnan(vyaw_sum)){
            vx_sum = vy_sum = vz_sum = vyaw_sum = 0;
          }
          odom_global.twist.twist.linear.x = vx_sum / que_size;
          odom_global.twist.twist.linear.y = vy_sum / que_size;
          odom_global.twist.twist.linear.z  = vz_sum / que_size;
          odom_global.twist.twist.angular.z = vyaw_sum / que_size;

          vx_que.pop();
          vy_que.pop();
          vz_que.pop();
          vyaw_que.pop();
          float incl = atan2(odom_global.twist.twist.linear.z,sqrt(pow(odom_global.twist.twist.linear.x,2)+pow(odom_global.twist.twist.linear.y,2)));
          float hdng = atan2(odom_global.twist.twist.linear.y,odom_global.twist.twist.linear.x);
          odom_global.pose.pose.position.x = tff.transform.translation.x;
          odom_global.pose.pose.position.y = tff.transform.translation.y;
          odom_global.pose.pose.position.z = tfh.transform.translation.z;
          tfp.transform.translation.x = odom_global.twist.twist.linear.x * 3.0;
          tfp.transform.translation.y = odom_global.twist.twist.linear.y * 3.0;
          tfp.transform.translation.z = odom_global.twist.twist.linear.z * 3.0;
          if(std::isnan(tfp.transform.translation.x) || std::isnan(tfp.transform.translation.y) || std::isnan(tfp.transform.translation.z)){
            ROS_ERROR("std_isnan: odomglobal:  %.2f %.2f %.2f %.2f, vsum xyz: %.2f %.2f %.2f %.2f", odom_global.twist.twist.linear.x, odom_global.twist.twist.linear.y,
             odom_global.twist.twist.linear.z,odom_global.twist.twist.angular.z,vx_sum,  vy_sum,  vz_sum,  vyaw_sum);
          }
          float yaw_future = constrainAngle(rpy.z + odom_global.twist.twist.angular.z);
          tfp.transform.rotation = tf::createQuaternionMsgFromYaw(yaw_future);
          odom_global.pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0,-incl,hdng);
          pub_global_odom.publish(odom_global);
          tf_b.sendTransform(tfp);
        }
      }
    }
    return 0;
}
