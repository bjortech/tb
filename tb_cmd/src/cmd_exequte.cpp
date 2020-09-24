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
double par_vel_xy_max,par_zclearing;
double zmin;
geometry_msgs::Point get_point_in_map(std::string frame){
  geometry_msgs::TransformStamped transformStamped;
  try{
    transformStamped = tfBuffer.lookupTransform("map",frame,
                             ros::Time(0));
  }
  catch (tf2::TransformException &ex) {
    ROS_WARN("%s",ex.what());
    ros::Duration(1.0).sleep();
  }
	geometry_msgs::Point pnt_out;
  pnt_out.x = transformStamped.transform.translation.x;
  pnt_out.y = transformStamped.transform.translation.y;
  pnt_out.z = transformStamped.transform.translation.z;
	return pnt_out;
}
void set_zmin_cb(const std_msgs::Float64::ConstPtr& msg){
	zmin = msg->data + par_zclearing;
}
int main(int argc, char **argv){
  ros::init(argc, argv, "tb_cmd_exequte_node");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");
	private_nh.param("max_horisontal_velocity", par_vel_xy_max, 8.0);//*2.0);
	private_nh.param("z_min_clearing", par_zclearing, 2.0);//*2.0);

  ros::Rate rate(50);

  tf2_ros::TransformListener tf2_listener(tfBuffer);

  ros::Publisher cmd_pub  = nh.advertise<sensor_msgs::Joy>("/tb_exeq/cmd_joy", 100);
	ros::Subscriber s3 			= nh.subscribe("/tb_autoevade/z_min",  100,&set_zmin_cb);

  while(ros::ok()){
		geometry_msgs::Point position_setpoint = get_point_in_map("base_perfect_alt");
		geometry_msgs::Point position_actual 	 = get_point_in_map("base_stabilized");

		Eigen::Vector3f pnt1_vec(position_actual.x,position_actual.y,position_actual.z);
		Eigen::Vector3f pnt2_vec(position_setpoint.x,position_setpoint.y,position_setpoint.z);

		float cmd_length = (pnt2_vec - pnt1_vec).norm();
		Eigen::Vector3f stride_vec = (pnt2_vec - pnt1_vec).normalized() * fmin(par_vel_xy_max,cmd_length);

		sensor_msgs::Joy cmd;
	  cmd.axes.resize(4);
		cmd.axes[0] = stride_vec.x();
		cmd.axes[1] = stride_vec.y();
		cmd.axes[2] = fmax(position_setpoint.z,zmin);
		cmd.axes[3] = atan2(stride_vec.y(),stride_vec.x());
	  cmd.header.stamp = ros::Time::now();
	  cmd_pub.publish(cmd);
		rate.sleep();
		ros::spinOnce();
  }
  return 0;
}
