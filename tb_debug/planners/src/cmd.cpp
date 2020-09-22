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

double par_zclearing,yaw_odom,target_arm1;
const float deg2rad = M_PI/180.0; const float rad2deg = 180.0/M_PI; const float earth = 6378137.0;
tf2_ros::Buffer tfBuffer;
geometry_msgs::Vector3 rpy,vlp_rpy,setp_rpy;
sensor_msgs::Joy cmd;
double vel_xy_max = 15.0;
std_msgs::Float64 arm_cmd1;
float target_alt,cmd_tilt;
geometry_msgs::Point pos;
geometry_msgs::PoseStamped pose_setpoint;
std_msgs::Float64 arm1_tilt_msg;
ros::Publisher pub_arm_cmd1;
///////*******/////////
float ptilt_P = 0.6;
float ptilt_D = 0.3;
float ptilt_I = 0.05;
float ptilt_max = M_PI/4;
float itilt_max = M_PI/4;
float dtilt_max = M_PI/4;
float tilt_err,tilt_err_last,p_tilt,tilt_i,tilt_p,tilt_d;
bool tilt_new = false;
ros::Time tilt_time_last;
///////*******/////////

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
float get_zmax(nav_msgs::Path pathin){
  float zmx = 0;
  for(int i = 0; i < pathin.poses.size(); i++){
    if(pathin.poses[i].pose.position.z > zmx){
      zmx = pathin.poses[i].pose.position.z;
    }
  }
  return zmx;
}
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
void send_tilt(float armcmd){
  arm_cmd1.data = armcmd;
  if(arm_cmd1.data > 0.0)
    arm_cmd1.data = saturate(arm_cmd1.data,M_PI/3);
  if(arm_cmd1.data < 0.0)
    arm_cmd1.data = saturate(arm_cmd1.data,M_PI/4);
  //ROS_INFO("armcmd: %.2f arm_cmd: %.2f, M_PI/4: %.2f",armcmd,arm_cmd1.data,M_PI/4);

  pub_arm_cmd1.publish(arm_cmd1);
}
float get_arm_cmd(float target_arm){
  return target_arm - rpy.y;
}
void setpoint_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
  pose_setpoint = *msg;
 tf2::Matrix3x3 q(tf2::Quaternion(msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z, msg->pose.orientation.w));
 q.getRPY(setp_rpy.x,setp_rpy.y,setp_rpy.z);
 setp_rpy.y *= -1;
 cmd_tilt = setp_rpy.y;
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
  tf2::Matrix3x3 q(tf2::Quaternion(transformStamped.transform.rotation.x, transformStamped.transform.rotation.y, transformStamped.transform.rotation.z, transformStamped.transform.rotation.w));
  q.getRPY(vlp_rpy.x,vlp_rpy.y,vlp_rpy.z);
  vlp_rpy.y *= -1;
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

int main(int argc, char **argv){
    ros::init(argc, argv, "tb_cmd_node");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");
    private_nh.param("setpoint_zclearing", par_zclearing, 5.0);//*2.0);

    ros::Rate rate(100);

    tf2_ros::TransformListener tf2_listener(tfBuffer);
    cmd.axes.push_back(0);
    cmd.axes.push_back(0);
    cmd.axes.push_back(0);
    cmd.axes.push_back(0);

    tf2_ros::TransformBroadcaster tf_b;
    nav_msgs::Odometry odom_global;

    ros::Publisher cmd_pub            = nh.advertise<sensor_msgs::Joy>("/tb_exeq/cmd_joy", 100);
    pub_arm_cmd1				= nh.advertise<std_msgs::Float64>("/tilt_controller/command", 100);
    ros::Subscriber ss = nh.subscribe("/tb_setpoint",1,setpoint_cb);

		double dt;
    float lastyaw;
    ros::Time time_last = ros::Time::now();


     float z_err,x_err,t_err,y_err;
     float z_i,x_i,t_i,y_i;
     float z_d,x_d,t_d,y_d;
     float pz_P,px_P,pt_P,py_P;
     float pz_D,px_D,pt_D,py_D;
     float pz_I,px_I,pt_I,py_I;
     pz_P = px_P = pt_P = py_P = 1;
     pz_D = px_D = pt_D = py_D = 0.3;
     pz_I = px_I = pt_I = py_I = 0.01;
     float p_max = 5.0;
     float i_max = 5.0;
     float d_max = 5.0;
     float y_err_last,z_err_last,x_err_last,t_err_last;

     bool ros_inform = false;
     bool use_pid    = false;
     float arm_cmd;
    while(ros::ok()){
      rate.sleep();
      ros::spinOnce();
      ros::Time t = ros::Time::now();

      update_pos();
      update_pos_vlp();
      float dt2 = (ros::Time::now() - t).toSec();
      dt = (ros::Time::now() - time_last).toSec();
      time_last = ros::Time::now();
    //  update_target_pos();
      float target_tilt = cmd_tilt;

     float cmd_z,cmd_x,cmd_y,cmd_t;
     Eigen::Vector3f pnt1_vec(pos.x,pos.y,pos.z);
     Eigen::Vector3f pnt2_vec(pose_setpoint.pose.position.x,pose_setpoint.pose.position.y,target_alt);
     float cmd_length = fmin((pnt2_vec - pnt1_vec).norm(),vel_xy_max);
     Eigen::Vector3f stride_vec = (pnt2_vec - pnt1_vec).normalized()  * cmd_length;
     if(use_pid){

       x_err = stride_vec.x();
       y_err = stride_vec.y();
       z_err = stride_vec.z();
       t_err = target_tilt - vlp_rpy.y; //get_inclination_forward();
       x_err = saturate(x_err,p_max);     y_err = saturate(y_err,p_max);     z_err = saturate(z_err,p_max);  t_err = saturate(t_err,p_max);
       x_i = saturate(x_i,i_max);         y_i = saturate(y_i,i_max);         z_i = saturate(z_i,i_max);      t_i = saturate(t_i,i_max);
       x_d = saturate(x_d,d_max);         y_d = saturate(y_d,d_max);         z_d = saturate(z_d,d_max);      t_d = saturate(t_d,d_max);

        x_i += x_err*dt;  y_i += y_err*dt; z_i += z_err*dt; //t_i += t_err*dt;

        x_d = (x_err - x_err_last) / dt;
        y_d = (y_err - y_err_last) / dt;
        z_d = (z_err - z_err_last) / dt;
        t_d = (t_err - t_err_last) / dt;

        z_err_last = z_err;     x_err_last = x_err;     y_err_last = y_err;  //t_err_last = t_err;
        cmd_x   = (x_err * px_P      + x_d * px_D     + x_i * px_I);
        cmd_y   = (y_err * py_P      + y_d * py_D     + y_i * py_I);
        cmd_z   = (z_err * pz_P      + z_d * pz_D     + z_i * pz_I);
        cmd_t   = (t_err * pt_P      + t_d * pt_D     + t_i * pt_I);

    //    arm_cmd = ideal_mx64_cmd + cmd_t;
        cmd_z += target_alt;
        target_alt = 15;
      }
      else{
        cmd_x         = stride_vec.x();
        cmd_y         = stride_vec.y();
        cmd_z         = pose_setpoint.pose.position.z;
      }
      float yaw_vel        = atan2(cmd_y,cmd_x);
      float yaw_setpoint   = yaw_vel;
      float yaw_error      = get_shortest(yaw_setpoint,rpy.z);
      float yaw_error_abs  = yaw_error;
    //  if(yaw_error < 0)
    //    yaw_error_abs = yaw_error * -1;
    //  if(cmd_length > 1.0 && yaw_error_abs > M_PI/5){
    //    cmd_x = cmd_y = 0;
      //}

      if(pose_setpoint.header.frame_id == "yaw")
        yaw_setpoint = tf::getYaw(pose_setpoint.pose.orientation);
      cmd.axes[0] = cmd_x;
      cmd.axes[1] = cmd_y;
      cmd.axes[2] = cmd_z;
      cmd.axes[3] = yaw_setpoint;
      cmd.header.stamp = ros::Time::now();
      cmd_pub.publish(cmd);
      send_tilt(get_arm_cmd(target_tilt));

      if(ros_inform){
        ROS_INFO("x_err: %.2f (%.2f - %.2f) CMD: %.2f x_i: %.2f x_d: %.2f",x_err,pose_setpoint.pose.position.x,pos.x,cmd_x,x_i,x_d);
        ROS_INFO("y_err: %.2f (%.2f - %.2f) CMD: %.2f y_i: %.2f y_d: %.2f",y_err,pose_setpoint.pose.position.y,pos.y,cmd_y,y_i,y_d);
        ROS_INFO("z_err: %.2f (%.2f - %.2f) CMD: %.2f z_i: %.2f z_d: %.2f",z_err,pose_setpoint.pose.position.z,pos.z,cmd_z,z_i,z_d);
        ROS_INFO("t_err: %.2f (%.2f - %.2f) CMD: %.2f t_i: %.2f t_d: %.2f",t_err,target_tilt,vlp_rpy.y,arm_cmd,t_i,t_d);
        ROS_INFO("Stride: %.2f %.2f %.2f",stride_vec.x(),stride_vec.y(),stride_vec.z());

    //    ROS_INFO("tilt:   %.2f d: %.2f, i %.2f, cmd: %.2f final_cmd: %.2f",t_err,t_d,t_i,cmd_t,arm1_tilt_msg.data);
      }
    }
    return 0;
}
