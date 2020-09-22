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
geometry_msgs::PoseStamped ps_start,ps_end;
geometry_msgs::PoseStamped ps_start_offset,ps_end_offset;
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
ros::ServiceClient path_client;
ros::Publisher pub_path,pub_fail;
tf2_ros::Buffer tfBuffer;
geometry_msgs::Point pos,cmd_pos;
bool getting_plan = false;
nav_msgs::Path path_plan;
std_msgs::Empty failmsg;
int mainstate;

nav_msgs::Path get_pathplan(float x, float y){
	nav_msgs::GetPlan srv;
	geometry_msgs::TransformStamped transformStamped;
	try{
		transformStamped = tfBuffer.lookupTransform("map","base_stabilized",
														 ros::Time(0));
	}
	catch (tf2::TransformException &ex) {
		ROS_WARN("%s",ex.what());
		ros::Duration(1.0).sleep();
	}

	srv.request.start.pose.position.x = transformStamped.transform.translation.x;
	srv.request.start.pose.position.y = transformStamped.transform.translation.y;
	srv.request.start.pose.position.z = 0;//transformStamped.transform.translation.z;

	srv.request.goal.pose.position.x = x;
	srv.request.goal.pose.position.y = y;
	srv.request.goal.pose.position.z = 0;//transformStamped.transform.translation.z;
	srv.request.start.pose.orientation.w = 1.0;
	srv.request.goal.pose.orientation.w = 1.0;

	srv.request.start.header.stamp 	  = ros::Time::now();
	srv.request.goal.header.stamp 	  = ros::Time::now();
	srv.request.start.header.frame_id = "map";
	srv.request.goal.header.frame_id 	= "map";

 	srv.request.tolerance = 2.5;
  ROS_INFO("PATHCREATOR: src_request: %.0f %.0f -> %.0f %.0f",srv.request.start.pose.position.x,srv.request.start.pose.position.y,srv.request.goal.pose.position.x,srv.request.goal.pose.position.y);
  ROS_INFO("PATHCREATOR: Make plan: %d", (path_client.call(srv) ? 1 : 0));
  ROS_INFO("PATHCREATOR: Plan size: %d", srv.response.plan.poses.size());
  return srv.response.plan;
}

void targetpose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
	if(mainstate == 1){
		if(!getting_plan){
			path_plan = get_pathplan(msg->pose.position.x,msg->pose.position.y);
			if(path_plan.poses.size() > 0)
				pub_path.publish(path_plan);
			else
				pub_fail.publish(failmsg);
		}
	}
}
void mainstate_cb(const std_msgs::UInt8::ConstPtr& msg){
	mainstate = msg->data;
 if(msg->data == 2 && ((ros::Time::now()-ps_end.header.stamp).toSec() > 10)){
	 path_plan = get_pathplan(0,0);
	 path_plan.header.frame_id = "map";
	 path_plan.header.stamp		  = ros::Time::now();
	 if(path_plan.poses.size() > 0)
		 pub_path.publish(path_plan);
		else
			pub_fail.publish(failmsg);
	}
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "tb_pathplanner_node");
  ros::NodeHandle nh;
	ros::NodeHandle private_nh("~");
	tf2_ros::TransformListener tf2_listener(tfBuffer);

	ros::Subscriber s0  = nh.subscribe("/tb_plan/target_point",1,targetpose_cb);
	ros::Subscriber s1  = nh.subscribe("/tb_fsm/main_state",1,mainstate_cb);
	pub_path 					  = nh.advertise<nav_msgs::Path>("/tb_path/path_to_interpolate",10);
	pub_fail 					  = nh.advertise<std_msgs::Empty>("/tb_path/planning_failed",10);

	path_client       	= nh.serviceClient<nav_msgs::GetPlan>("/move_base/make_plan");
	ros::spin();

  return 0;
}
