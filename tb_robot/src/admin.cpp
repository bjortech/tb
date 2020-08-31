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
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <image_transport/image_transport.h>
#include <dynamicEDT3D/dynamicEDTOctomap.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/OccupancyGrid.h>
#include <map_msgs/OccupancyGridUpdate.h>
#include <chrono>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/UInt8.h>
#include <visualization_msgs/MarkerArray.h>
#include <eigen3/Eigen/Core>
#include <geometry_msgs/Vector3Stamped.h>
#include <cv_bridge/cv_bridge.h>
#include <nav_msgs/Odometry.h>

ros::Publisher cmd_pub,pub_path_visited,invoke_pub;
tf2_ros::Buffer tfBuffer;
ros::Time start,mainnstate_change;

std_msgs::UInt8 mainstate_msg;
nav_msgs::Path path_visited,path_visited_cmd;
geometry_msgs::PoseStamped last_pose,last_pose_cmd;

double par_missionlength;
bool vlp_started,bag_published,par_live,invoke_published;

float get_dst3d(geometry_msgs::Point p1, geometry_msgs::Point p2){
  return(sqrt(pow(p1.x-p2.x,2)+pow(p1.y-p2.y,2)+pow(p1.z-p2.z,2)));
}
float get_shortest(float target_heading,float actual_hdng){
  float a = target_heading - actual_hdng;
  if(a > M_PI)a -= M_PI*2;
  else if(a < -M_PI)a += M_PI*2;
  return a;
}
void set_mainstate(int newstate){
  if(newstate != mainstate_msg.data){
    float dt = (ros::Time::now() - mainnstate_change).toSec();
    ROS_INFO("MAIN: MAINSTATE: %i -> %i (%.3f seconds in state)",mainstate_msg.data,newstate,dt);
    mainnstate_change         = ros::Time::now();
    mainstate_msg.data  = newstate;
  }
}
void checktfcmd(){
  geometry_msgs::TransformStamped transformStamped;
  try{
    transformStamped = tfBuffer.lookupTransform("map","base_perfect_alt",
                             ros::Time(0));
  }
  catch (tf2::TransformException &ex) {
    ROS_WARN("%s",ex.what());
    ros::Duration(1.0).sleep();
  }
  last_pose_cmd.pose.position.x = transformStamped.transform.translation.x;
  last_pose_cmd.pose.position.y = transformStamped.transform.translation.y;
  last_pose_cmd.pose.position.z = transformStamped.transform.translation.z;
  last_pose_cmd.pose.orientation = transformStamped.transform.rotation;
  last_pose_cmd.header           = last_pose.header;
  path_visited_cmd.poses.push_back(last_pose_cmd);
  path_visited_cmd.header        = path_visited.header;
}
void checktf(){
  geometry_msgs::TransformStamped transformStamped;
  try{
    transformStamped = tfBuffer.lookupTransform("map","base_stabilized",
                             ros::Time(0));
  }
  catch (tf2::TransformException &ex) {
    ROS_WARN("%s",ex.what());
    ros::Duration(1.0).sleep();
  }
	//IF DRONE IS 1 METER ABOVE BASELINE ALTITUDE IT HAS TAKEN OFF AND CAN START MISSION
  if(mainstate_msg.data == 0 && transformStamped.transform.translation.z >= 1.0)
    set_mainstate(1);
	//IF DRONE IS RETURNING TO START AFTER ENDED MISSION(STATE==2), AND DELTA DISTANCE IS BELOW 3 METERS - DRONE IS BACK AT START - STATE= 3
	else if(mainstate_msg.data == 2 && sqrt(pow(transformStamped.transform.translation.x,2)+pow(transformStamped.transform.translation.y,2)) < 3)
    set_mainstate(3);


	//THIS IS JUST THE TRACKING FUNCTION THAT CREATES AND PUBLISHES ALL VISITED POINTS IN THE GLOBAL FRAME.
	//THE CONDITION ENSURES THAT A MINIMUM NUMBER OF POSES ARE STORED, CONVENIENT FOR LATER USAGE
  if(sqrt(pow(transformStamped.transform.translation.x - last_pose.pose.position.x,2)+ pow(transformStamped.transform.translation.y - last_pose.pose.position.y,2)+
    pow(transformStamped.transform.translation.z - last_pose.pose.position.z,2)) >= 1.00 || abs(get_shortest(tf::getYaw(transformStamped.transform.rotation),tf::getYaw(last_pose.pose.orientation)) *  180.0/M_PI) > 30){
    last_pose.pose.position.x  = transformStamped.transform.translation.x;
    last_pose.pose.position.y  = transformStamped.transform.translation.y;
    last_pose.pose.position.z  = transformStamped.transform.translation.z;
    last_pose.pose.orientation = transformStamped.transform.rotation;
    last_pose.header           = transformStamped.header;
    path_visited.poses.push_back(last_pose);
    path_visited.header.stamp = ros::Time::now();
		//This function is really identical to the one right above, but it uses the frame of the setpoint rather than the stabilized base frame.
    checktfcmd();
  }
}
void live_stuff(){
	//THIS IS WHERE STRINGS ARE SENT TO THE "tb_invoke_node" - A NODE THAT TAKES A STRING AND WRITES IT AS IF IT WERE IN A TERMINAL
	//I DON'T KNOW WHY, BUT SOME NODES AND LAUNCH-FILES DON'T GET ALONG WITH BOOTSCRIPTS. I HAD TO DO IT LIKE THIS TO BE ABLE TO START ALL NODES ON STARTUP
/*	if((ros::Time::now() - start).toSec() > 5 && !invoke_published){
    std_msgs::String invoke_msg;
    invoke_msg.data = "roslaunch tb_pro m600.launch";
    invoke_published = true;
    invoke_pub.publish(invoke_msg);
  }
	//THIS WAS BECAUSE THE VELODYNE KEPT MESSING UP IT'S IP, NO IDEA WHY.
  else if((ros::Time::now() - start).toSec() > 10 && !vlp_started){
    std_msgs::String invoke_msg;
    invoke_msg.data = "roslaunch tb_pro vlp16.launch";
    vlp_started = true;
    invoke_pub.publish(invoke_msg);*/
  }
	//DUE TO THE NUMBER OF IMAGE TOPICS THE CONVENIENT ROSBAG-RECORD-ALL FUNCTION NEEDS TO BE SPECIFIC....
  if((ros::Time::now() - start).toSec() > 15 && !bag_published){
    std_msgs::String invoke_msg;
    invoke_msg.data = "rosbag record -O /home/nuc/bag.bag";
    bag_published = true;
    invoke_pub.publish(invoke_msg);
  }
	//IF PARAMETER MISSION LENGTH (SECONDS) IS EXCEEDED, SET STATE(2)(GO HOME)
  else if((ros::Time::now() - start).toSec() > par_missionlength && mainstate_msg.data == 1)
    set_mainstate(2);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "tb_cmd_fsm_node");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");
	private_nh.param("missionlength_seconds",  par_missionlength, 180.0);
	private_nh.param("mission_is_live",  			 par_live, true);

  tf2_ros::TransformListener tf2_listener(tfBuffer);
	///********FSM NODE*************////////////
	start = ros::Time::now();
  ros::Rate rate(5.0);
	last_pose.pose.orientation.w = 1;
  last_pose.header.frame_id = path_visited.header.frame_id = "map";

  invoke_pub   												= nh.advertise<std_msgs::String>("/tb_invoke",10);
  ros::Publisher pub_mainstate      	= nh.advertise<std_msgs::UInt8>("/tb_fsm/main_state",10);
  ros::Publisher pub_path_visited     = nh.advertise<nav_msgs::Path>("/tb_world/path_visited",10);
  ros::Publisher pub_path_visited_cmd = nh.advertise<nav_msgs::Path>("/tb_world/path_visited_cmd",10);

  while(ros::ok()){
    if((ros::Time::now() - start).toSec() > 3)
      checktf();
    pub_mainstate.publish(mainstate_msg);
    pub_path_visited.publish(path_visited);
    pub_path_visited_cmd.publish(path_visited_cmd);
		//IF WORKING WITH HARDWARE AND INVOKE NODE IS UP, SEND COMMANDS
    if(par_live)
      live_stuff();
    rate.sleep();
    ros::spinOnce();
  }
  return 0;
}
