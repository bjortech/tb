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

sensor_msgs::Joy cmd;
geometry_msgs::TransformStamped tft,tff,tfh,tfrp,tfm,tfo,tfoa,tfa,tfa2,tfp,tfpos,tfb,tfs,tfx,tfv,tf_up,tf_dwn;
geometry_msgs::Vector3Stamped vlocal_gps,vglobal_gps,armvec_min_max_radprs;
geometry_msgs::Vector3 rpy,rpyx;
nav_msgs::Odometry odom;
ros::Time time_last_v;
bool first = true;
int mainstate;
double yaw_odom,target_arm,t_int_1,t_int_2,t_int_3;
tf2_ros::Buffer tfBuffer;

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

void att_cb(const geometry_msgs::QuaternionStamped::ConstPtr& msg){
  tf2::Matrix3x3 q(tf2::Quaternion(msg->quaternion.x, msg->quaternion.y, msg->quaternion.z, msg->quaternion.w));
  q.getRPY(rpy.x,rpy.y,rpy.z);
  first         = false;
}
void imuxsens_cb(const sensor_msgs::Imu::ConstPtr& msg){
    tf2::Matrix3x3 q(tf2::Quaternion(msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w));
    q.getRPY(rpyx.x,rpyx.y,rpyx.z);

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
void abs_cb(const std_msgs::Float64::ConstPtr& msg){
  tfoa.transform.translation.z = msg->data;
}
void height_cb(const std_msgs::Float32::ConstPtr& msg){
  tfh.transform.translation.z = msg->data;
}
void cmd_arm_cb(const std_msgs::Float64::ConstPtr& msg){
  target_arm = msg->data;
}
void twist_cb(const geometry_msgs::Twist::ConstPtr& msg)
{
  if(!first){
    float dt = ((ros::Time::now() - time_last_v).toSec());
    time_last_v                = ros::Time::now();
    odom.twist.twist.angular.z = msg->angular.z;
    odom.twist.twist.linear.x  = msg->linear.x;
    odom.twist.twist.linear.y  = msg->linear.y;
  }
}
void arm_cmd_cb(const geometry_msgs::Vector3::ConstPtr& msg){
  armvec_min_max_radprs.vector = *msg;
  armvec_min_max_radprs.header.stamp = ros::Time::now();
//  ROS_INFO("Arm motion set: min: %.2f -> max: %.2f, radsprsec: %.2f",armvec_min_max_radprs.vector.x,armvec_min_max_radprs.vector.y,armvec_min_max_radprs.vector.z);
  t_int_1    = abs(armvec_min_max_radprs.vector.x - target_arm) / armvec_min_max_radprs.vector.z;
  t_int_2    = abs(armvec_min_max_radprs.vector.y - armvec_min_max_radprs.vector.x) / armvec_min_max_radprs.vector.z;
  t_int_3    = abs(armvec_min_max_radprs.vector.y - target_arm) / armvec_min_max_radprs.vector.z;
  float t_tot = t_int_1 + t_int_2 + t_int_3;
  ROS_INFO("Arm motion set: t_tot: %.2f",t_tot);
}
void motionstate_cb(const std_msgs::UInt8::ConstPtr& msg){
  if(msg->data == 12)
    target_arm = M_PI/4;
  else if(msg->data == 8)
    target_arm = -M_PI/4;
}
void mainstate_cb(const std_msgs::UInt8::ConstPtr& msg){
  mainstate = msg->data;
}
void centroid_cb(const geometry_msgs::PointStamped::ConstPtr& msg){
  tfb.transform.translation.x = msg->point.x;
  tfb.transform.translation.y = msg->point.y;
  tfb.transform.translation.z = msg->point.z;
}
void lasttarget_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
  tft.transform.translation.x = msg->pose.position.x;
  tft.transform.translation.y = msg->pose.position.y;
  tft.transform.translation.z = msg->pose.position.z;
  tft.transform.rotation      = msg->pose.orientation;
}
int main(int argc, char **argv){
    ros::init(argc, argv, "tb_cmdodomtf_node");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");
    ros::Rate rate(100);

    tf2_ros::TransformListener tf2_listener(tfBuffer);
    cmd.axes.push_back(0);
    cmd.axes.push_back(0);
    cmd.axes.push_back(0);
    cmd.axes.push_back(0);

    tf2_ros::TransformBroadcaster tf_b;

    tfm.header.frame_id         = "map";
    tfm.child_frame_id          = "odom";
    tfm.transform.rotation.w    = 1;

    tfa.header.frame_id         = "base_link";
    tfa.child_frame_id          = "mx64";
    tfa.transform.rotation.w    = 1;

    tfpos.header.frame_id         = "odom";
    tfpos.child_frame_id          = "base_position";
    tfpos.transform.rotation.w    = 1;

    tfx.header.frame_id         = "odom";
    tfx.child_frame_id          = "imu_x";
    tfx.transform.rotation.w    = 1;

    tff.header.frame_id         = "odom";
    tff.child_frame_id          = "base_footprint";
    tff.transform.rotation.w    = 1;

    tfh.header.frame_id         = "base_footprint";
    tfh.child_frame_id          = "base_stabilized";
    tfh.transform.rotation.w    = 1;

    tf_up.header.frame_id         = "base_stabilized";
    tf_up.child_frame_id          = "base_stabilized_up";
    tf_up.transform.translation.z = 1;
    tf_up.transform.rotation.w = 1;

    tf_dwn.header.frame_id         = "base_stabilized";
    tf_dwn.child_frame_id          = "base_stabilized_down";
    tf_dwn.transform.translation.z = -1;
    tf_dwn.transform.rotation.w    = 1;

    tfrp.header.frame_id        = "base_stabilized";
    tfrp.child_frame_id         = "base_link";
    tfrp.transform.rotation.w   = 1;

    tfp.header.frame_id        = "base_stabilized";
    tfp.child_frame_id         = "base_future";
    tfp.transform.rotation.w   = 1;

    odom.header.frame_id         = "odom";
    odom.child_frame_id          = "base_perfect";
    odom.pose.pose.orientation.w = 1;

    tfo.header.frame_id          = "odom";
    tfo.child_frame_id           = "base_perfect";
    tfo.transform.rotation.w     = 1;

    tfv.header.frame_id          = "base_position";
    tfv.child_frame_id           = "velodyne_alternative";
    tfv.transform.rotation.w     = 1;

    tfb.header.frame_id          = "map";
    tfb.child_frame_id           = "tfb";
    tfb.transform.rotation.w     = 1;

    tfs.header.frame_id          = "map";
    tfs.child_frame_id           = "pose_stereo_slam";
    tfs.transform.rotation.w     = 1;

    tfoa.header.frame_id          = "base_perfect";
    tfoa.child_frame_id           = "base_perfect_alt";
    tfoa.transform.rotation.w     = 1;

    tft.header.frame_id         = "map";
    tft.child_frame_id          = "target_last";
    tft.transform.rotation.w    = 1;

    vlocal_gps.header.frame_id    = "base_stabilized";
    vglobal_gps.header.frame_id   = "odom";

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

    ros::Subscriber s22= nh.subscribe("/dji_sdk/height_above_takeoff", 100,&height_cb);
    ros::Subscriber s3 = nh.subscribe("/dji_sdk/attitude",             100,&att_cb);
    ros::Subscriber s5 = nh.subscribe("/dji_sdk/local_position",       100,&pos_cb);

    ros::Subscriber s12 = nh.subscribe("/imu/data",           100,&imuxsens_cb);

    ros::Subscriber s15= nh.subscribe("/tb_cmd/arm_motion",   100,&arm_cmd_cb);
    ros::Subscriber s10= nh.subscribe("/tb_cmd/arm_pitch",    100,&cmd_arm_cb);
    ros::Subscriber s9 = nh.subscribe("/tb_cmd/alt_target",   100,&abs_cb);
    ros::Subscriber s18 = nh.subscribe("/tb_fsm/main_state",  100,&mainstate_cb);
    ros::Subscriber sc = nh.subscribe("/tb_bld/building_centroid", 100,&centroid_cb);
    ros::Subscriber sc1 = nh.subscribe("/tb_setp/last_target", 100,&lasttarget_cb);

    ros::Subscriber s8 = nh.subscribe("/cmd_vel",          100,&twist_cb);
    ros::Subscriber ss = nh.subscribe("/pose",             100,&spose_cb);


    ros::Publisher odom_pub  = nh.advertise<nav_msgs::Odometry>("/odom", 100);
    ros::Publisher delta_alt = nh.advertise<std_msgs::Float64>("/tb_fsm/delta_alt", 100);
    ros::Publisher cmd_pub   = nh.advertise<sensor_msgs::Joy>("/tb_exeq/cmd_joy", 100);
    ros::Publisher armcmd_pub= nh.advertise<std_msgs::Float64>("/tilt_controller/command", 100);


    double dt;

    ros::Time time_last = ros::Time::now();
    std_msgs::Float64 delta_z;
    std_msgs::Float64 arm_cmd;
    geometry_msgs::Point offset;

    double syaw;
    while(ros::ok()){
      if((ros::Time::now() - time_last_v).toSec() > 0.5) odom.twist.twist.angular.z = odom.twist.twist.linear.x = odom.twist.twist.linear.y = 0;
      if((ros::Time::now() - time_last).toSec() > 0.03) dt = 0.03;
      else dt= (ros::Time::now() - time_last).toSec();
      time_last = ros::Time::now();
      rate.sleep();
      ros::spinOnce();
      if((abs(offset.x)+abs(offset.y)+abs(offset.z)) < 5){
        odom.pose.pose.position.x  += ((odom.twist.twist.linear.x*cos(yaw_odom)-odom.twist.twist.linear.y*sin(yaw_odom)) * dt);
        odom.pose.pose.position.y  += ((odom.twist.twist.linear.x*sin(yaw_odom)+odom.twist.twist.linear.y*cos(yaw_odom)) * dt);
        yaw_odom                   += odom.twist.twist.angular.z * dt;
        yaw_odom                    = constrainAngle(yaw_odom);
      }

      tfo.transform.translation.x = odom.pose.pose.position.x;
      tfo.transform.translation.y = odom.pose.pose.position.y;
      tfo.transform.rotation      = odom.pose.pose.orientation = tf::createQuaternionMsgFromYaw(yaw_odom);
      if(mainstate == 3 || std::isnan(tfo.transform.translation.x) || std::isnan(tfo.transform.translation.y)){
        tfo.transform.translation.x = 0;
        tfo.transform.translation.y = 0;
        tfo.transform.rotation      = odom.pose.pose.orientation = tf::createQuaternionMsgFromYaw(0);
      }
      offset.x   = tfo.transform.translation.x - tff.transform.translation.x;
      offset.y   = tfo.transform.translation.y - tff.transform.translation.y;
      offset.z   = tfoa.transform.translation.z- tfh.transform.translation.z;

      delta_z.data   = offset.z;
      arm_cmd.data = (rpy.y - target_arm);
// --- == forward
      if((ros::Time::now() - armvec_min_max_radprs.header.stamp).toSec() < (t_int_1+t_int_2+t_int_3)){
        offset.x = offset.y = 0;
        float dt_arm = (ros::Time::now() - armvec_min_max_radprs.header.stamp).toSec();
        if(dt_arm > (t_int_1+t_int_2))
          arm_cmd.data = armvec_min_max_radprs.vector.y + (dt_arm-(t_int_2+t_int_1)) * armvec_min_max_radprs.vector.z;
        else if(dt_arm > t_int_1)
          arm_cmd.data = armvec_min_max_radprs.vector.x - (dt_arm-t_int_1)           * armvec_min_max_radprs.vector.z;
        else
          arm_cmd.data = target_arm                     +  dt_arm                    * armvec_min_max_radprs.vector.z;
      }
      tfpos.transform.translation   = tff.transform.translation;
      tfpos.transform.translation.z = tfh.transform.translation.z;

      if(arm_cmd.data > 0)saturate(arm_cmd.data,M_PI/4);
      if(arm_cmd.data < 0)saturate(arm_cmd.data,M_PI/2);
      cmd.axes[0] = saturate(offset.x,10);
      cmd.axes[1] = saturate(offset.y,10);
      cmd.axes[2] = tfoa.transform.translation.z;
      cmd.axes[3] = yaw_odom;
      cmd.header.frame_id = "cmd_vel";
      tfrp.transform.rotation     = tf::createQuaternionMsgFromRollPitchYaw(rpy.x,rpy.y,0);
      tff.transform.rotation      = tf::createQuaternionMsgFromRollPitchYaw(0,0,rpy.z);
      tfx.transform.translation   = tfpos.transform.translation;
      tfx.transform.rotation      = tf::createQuaternionMsgFromRollPitchYaw(-rpyx.x,-rpyx.y,rpyx.z+M_PI/2);
      tfv.transform.rotation      = tf::createQuaternionMsgFromRollPitchYaw(-rpyx.x,-rpyx.y,rpy.z);
      tf_up.header.stamp = tft.header.stamp = tf_dwn.header.stamp =  tfv.header.stamp = tfx.header.stamp = tfs.header.stamp = tfb.header.stamp = tfa.header.stamp = tfpos.header.stamp = tfp.header.stamp = tfm.header.stamp = tfoa.header.stamp = odom.header.stamp = tfo.header.stamp = cmd.header.stamp = tfh.header.stamp = tfrp.header.stamp = tff.header.stamp = ros::Time::now();
      odom_pub.publish(odom);
      delta_alt.publish(delta_z);
      cmd_pub.publish(cmd);
      armcmd_pub.publish(arm_cmd);
      tf_b.sendTransform(tfoa);
      tf_b.sendTransform(tfo);
      tf_b.sendTransform(tfv);
      tf_b.sendTransform(tfrp);
      tf_b.sendTransform(tff);
      tf_b.sendTransform(tfh);
      tf_b.sendTransform(tfm);
      tf_b.sendTransform(tfp);
      tf_b.sendTransform(tfs);
      tf_b.sendTransform(tfx);
      tf_b.sendTransform(tf_up);
      tf_b.sendTransform(tf_dwn);
      tf_b.sendTransform(tft);

      if(tfb.transform.translation.x + tfb.transform.translation.y + tfb.transform.translation.z > 0)
        tf_b.sendTransform(tfb);
      tf_b.sendTransform(tfpos);
    }
    return 0;
}
