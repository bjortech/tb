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
#include <std_msgs/Float64.h>
#include <std_msgs/Float32.h>
#include <eigen3/Eigen/Core>

tf2_ros::Buffer tfBuffer;
geometry_msgs::Vector3 rpy;
geometry_msgs::TransformStamped tf_xyz,tf_xy_yaw,tf_z,tf_roll_pitch,tf_map;

void attitude_quaternion_cb(const geometry_msgs::QuaternionStamped::ConstPtr& msg){
  tf2::Matrix3x3 q(tf2::Quaternion(msg->quaternion.x, msg->quaternion.y, msg->quaternion.z, msg->quaternion.w));
  q.getRPY(rpy.x,rpy.y,rpy.z);
  if(std::isnan(rpy.z) || std::isnan(rpy.y) || std::isnan(rpy.x))
    rpy.z = rpy.x = rpy.y = 0;
}
void position_point_cb(const geometry_msgs::PointStamped::ConstPtr& msg){
  tf_xy_yaw.transform.translation.x = msg->point.x;
  tf_xy_yaw.transform.translation.y = msg->point.y;
}
void altitude_float_cb(const std_msgs::Float32::ConstPtr& msg){
  tf_z.transform.translation.z = msg->data;
}
int main(int argc, char **argv){
    ros::init(argc, argv, "tb_tf_node");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    ros::Rate rate(100);
    tf2_ros::TransformBroadcaster tf_b;

    tf_map.header.frame_id         = "map";
    tf_map.child_frame_id          = "odom";
    tf_map.transform.rotation.w    = 1;

    tf_xy_yaw.header.frame_id         = "odom";
    tf_xy_yaw.child_frame_id          = "base_footprint";
    tf_xy_yaw.transform.rotation.w    = 1;

    tf_z.header.frame_id         = "base_footprint";
    tf_z.child_frame_id          = "base_stabilized";
    tf_z.transform.rotation.w    = 1;

    tf_roll_pitch.header.frame_id        = "base_stabilized";
    tf_roll_pitch.child_frame_id         = "base_link";
    tf_roll_pitch.transform.rotation.w   = 1;

    tf_xyz.header.frame_id         = "odom";
    tf_xyz.child_frame_id          = "base_position";
    tf_xyz.transform.rotation.w    = 1;

    ros::Subscriber s22 = nh.subscribe("/dji_sdk/height_above_takeoff", 100,&altitude_float_cb);
    ros::Subscriber s3  = nh.subscribe("/dji_sdk/attitude",             100,&attitude_quaternion_cb);
    ros::Subscriber s5  = nh.subscribe("/dji_sdk/local_position",       100,&position_point_cb);

    while(ros::ok()){
      rate.sleep();
      ros::spinOnce();

      tf_roll_pitch.transform.rotation = tf::createQuaternionMsgFromRollPitchYaw(rpy.x,rpy.y,0);
      tf_xy_yaw.transform.rotation     = tf::createQuaternionMsgFromRollPitchYaw(0,0,rpy.z);
      tf_xyz.transform.translation   	 = tf_xy_yaw.transform.translation;
      tf_xyz.transform.translation.z 	 = tf_z.transform.translation.z;

			tf_map.header.stamp = tf_xyz.header.stamp = tf_z.header.stamp = tf_roll_pitch.header.stamp = tf_xy_yaw.header.stamp = ros::Time::now();

  		tf_b.sendTransform(tf_roll_pitch);
      tf_b.sendTransform(tf_xy_yaw);
      tf_b.sendTransform(tf_z);
			tf_b.sendTransform(tf_map);
			tf_b.sendTransform(tf_xyz);
    }
    return 0;
}
