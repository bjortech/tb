#include <ros/ros.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Empty.h>
#include <sensor_msgs/LaserScan.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_listener.h>
#include "tf2_ros/message_filter.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include <tf/transform_datatypes.h>
#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib_msgs/GoalID.h>
#include <std_msgs/UInt8.h>
#include <nav_msgs/GetPlan.h>
ros::Publisher result_pub;
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
std_msgs::Bool success;
std_msgs::Bool failure;
geometry_msgs::PointStamped point;
tf2_ros::Buffer tfBuffer;
ros::ServiceClient path_client;
ros::Publisher pub,pub_plan,pub_cancel_msg;
actionlib_msgs::GoalID cancel_msg;
bool server_active;

void targetpose_cb(const geometry_msgs::PoseStamped::ConstPtr& pose){
  if(pose->header.frame_id != ""){
    MoveBaseClient ac("move_base", true);

    while(!ac.waitForServer(ros::Duration(5.0))){
      ROS_INFO("Waiting for the move_base action server");
    }
    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose.header.frame_id = pose->header.frame_id;
    goal.target_pose.header.stamp = ros::Time::now();
    point.point.x = pose->pose.position.x;
    point.point.y = pose->pose.position.y;

    goal.target_pose.pose.position.x = pose->pose.position.x;
    goal.target_pose.pose.position.y = pose->pose.position.y;
    goal.target_pose.pose.orientation = pose->pose.orientation;
		if(server_active){
			server_active = false;
			pub_cancel_msg.publish(cancel_msg);
		}
		server_active = true;
    ROS_INFO("Sending goal");
    ac.sendGoal(goal);
    ac.waitForResult();

    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
        ROS_INFO("You have arrived to the goal position");
        result_pub.publish(success);
    }
    else{
      ROS_INFO("The base failed for some reason");
      result_pub.publish(failure);
    }
  }
}
void set_xyz_cb(const geometry_msgs::PointStamped::ConstPtr& msg)
{
	if(server_active){
		server_active = false;
		pub_cancel_msg.publish(cancel_msg);
	}
}
int main(int argc, char **argv) {
  ros::init(argc, argv, "tb_mbclient_node");
  ros::NodeHandle nh;
  tf2_ros::TransformListener tf2_listener(tfBuffer);
  success.data = true;
  failure.data = false;
	ros::Subscriber s  = nh.subscribe("/tb_abstractmap/target_candidate", 10,&targetpose_cb);
	ros::Subscriber s5 			= nh.subscribe("/tb_cmd/set_xyz", 100,&set_xyz_cb);
	result_pub  = nh.advertise<std_msgs::Bool>("/tb_cmd/mb_result",10);
	pub_cancel_msg  = nh.advertise<actionlib_msgs::GoalID>("/move_base/cancel",10);
	pub  = nh.advertise<move_base_msgs::MoveBaseActionGoal>("/move_base/goal",10);
  ros::spin();
}
