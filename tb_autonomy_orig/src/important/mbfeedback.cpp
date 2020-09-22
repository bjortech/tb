#include <ros/ros.h>
#include <octomap_msgs/conversions.h>
#include <octomap_msgs/Octomap.h>
#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <octomap/OcTreeBase.h>
#include <octomap/octomap_types.h>
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
#include <tf2/LinearMath/Quaternion.h>
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
#include <nav_msgs/Path.h>
#include <actionlib_msgs/GoalID.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/OccupancyGrid.h>
#include <map_msgs/OccupancyGridUpdate.h>
#include <chrono>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib_msgs/GoalID.h>
#include <nav_msgs/GetPlan.h>


void mbfeedback_cb(const move_base_msgs::MoveBaseActionFeedback::ConstPtr& msg){
		if(state_mb == "target_sent");
			set_mbstate("active");
	//ROS_INFO("mbfeedback received - targetmb approved(?)");
}
void mbres_cb(const move_base_msgs::MoveBaseActionResult::ConstPtr& msg){
	if(msg->status.status == 3){
		set_mbstate("idle");
		state_mapelevation_failed = 0;
	}
	if(msg->status.status == 4){
		set_mbstate("idle");
		state_mapelevation_failed = state_mapelevation;
	}
	update_mbmap(30,400);
	ROS_INFO("MoveBaseRes; %i %s",msg->status.status,state_mb.c_str());
//  ROS_INFO("MoveBaseRes; %i %s",msg->status.status,state_mb.c_str());
	//  PENDING=0  ACTIVE=1  PREEMPTED=2  SUCCEEDED=3  ABORTED=4  REJECTED=5  PREEMPTING=6  RECALLING=7  RECALLED=8*///  ABORTED= 4# The goal was aborted during execution by the action server due
}
int main(int argc, char** argv)
{
  ros::init(argc, argv, "tb_mbclient_node");
  ros::NodeHandle nh;
	ros::NodeHandle private_nh("~");
	ros::Subscriber sb  = nh.subscribe("/move_base/result",100,&mbres_cb);
	ros::Subscriber smbb = nh.subscribe("/move_base/feedback",  100,&mbfeedback_cb);
	ros::spin();

  return 0;
}
