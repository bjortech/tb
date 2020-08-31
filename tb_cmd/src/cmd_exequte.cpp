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

double cmd_yaw,par_takeoffaltitude,par_zclearing,yaw_odom;
tf2_ros::Buffer tfBuffer;
sensor_msgs::Joy cmd;
double vel_xy_max = 10.0;
geometry_msgs::Point pos,cmd_pos;
int mainstate = 0;
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
void update_cmd(){
  geometry_msgs::TransformStamped transformStamped;
  try{
    transformStamped = tfBuffer.lookupTransform("map","base_perfect_alt",
                             ros::Time(0));
  }
  catch (tf2::TransformException &ex) {
    ROS_WARN("%s",ex.what());
    ros::Duration(1.0).sleep();
  }
  cmd_pos.x = transformStamped.transform.translation.x;
  cmd_pos.y = transformStamped.transform.translation.y;
  cmd_pos.z = transformStamped.transform.translation.z;
	cmd_yaw = tf::getYaw(transformStamped.transform.rotation);
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


void mainstate_cb(const std_msgs::UInt8::ConstPtr& msg){
  mainstate = msg->data;
}
int main(int argc, char **argv){
    ros::init(argc, argv, "tb_cmd_exequte_node");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    ros::Rate rate(50);

    tf2_ros::TransformListener tf2_listener(tfBuffer);
    cmd.axes.push_back(0);
    cmd.axes.push_back(0);
    cmd.axes.push_back(0);
    cmd.axes.push_back(0);

    tf2_ros::TransformBroadcaster tf_b;

    ros::Publisher cmd_pub  = nh.advertise<sensor_msgs::Joy>("/tb_exeq/cmd_joy", 100);
    ros::Subscriber s01 = nh.subscribe("/tb_fsm/main_state",10,mainstate_cb);

    while(ros::ok()){
      update_cmd();
      update_pos();

       Eigen::Vector3f pnt1_vec(pos.x,pos.y,pos.z);
       Eigen::Vector3f pnt2_vec(cmd_pos.x,cmd_pos.y,cmd_pos.z);
			 float min_vel = 1.0;
			 float cmd_length_orig = (pnt2_vec - pnt1_vec).norm();
			 float cmd_length;
			 if(cmd_length_orig < min_vel)
			 		cmd_length = 0;
				else if(cmd_length_orig > vel_xy_max)
					cmd_length = vel_xy_max;
				else
					cmd_length = cmd_length_orig;
       Eigen::Vector3f stride_vec = (pnt2_vec - pnt1_vec).normalized()  * cmd_length;
      if(mainstate == 0 || mainstate == 3){
        stride_vec.x() = 0;
				stride_vec.y() = 0;
				cmd.axes[2] = 2.0;
				cmd.axes[3] = 0.0;
      }

			cmd.axes[0] = stride_vec.x();
			cmd.axes[1] = stride_vec.y();
			cmd.axes[2] = cmd_pos.z;
			if(cmd_length > min_vel)
				cmd.axes[3] = atan2(stride_vec.y(),stride_vec.x());
			else
				cmd.axes[3] = cmd_yaw;
      cmd.header.stamp = ros::Time::now();
      cmd_pub.publish(cmd);
			rate.sleep();
			ros::spinOnce();
    }
    return 0;
}
