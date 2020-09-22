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


using namespace octomap;
using namespace std;
nav_msgs::Odometry  odom;

tf2_ros::Buffer tfBuffer;
ros::Publisher cmd_pub,pub_path_visited,targetpoly_pub,targetcmd_pub,invoke_pub;
double par_takeoffaltitude,par_zjump;
float rad2deg = 180.0/M_PI;
float deg2rad = M_PI/180;
std_msgs::UInt8 mainstate_msg,altlvl_msg,missionstate_msg,buildingstate_msg;
std_msgs::String activity_msg;
std_msgs::Float64 alt_target;
geometry_msgs::PoseStamped last_pose;
bool overriding_altitude,bag_published,par_live,got_map,invoke_published;
ros::Time last_override,start,activity_change,missionstate_change,mainnstate_change,altlvl_change,buildingstate_change;
std::vector<float> z_lvls;
nav_msgs::Path path_visited;
std::string cmd_mode;
geometry_msgs::Point cmd_pos;

float get_dst3d(geometry_msgs::Point p1, geometry_msgs::Point p2){
  return(sqrt(pow(p1.x-p2.x,2)+pow(p1.y-p2.y,2)+pow(p1.z-p2.z,2)));
}
float get_dst2d(geometry_msgs::Point p1, geometry_msgs::Point p2){
  return(sqrt(pow(p1.x-p2.x,2)+pow(p1.y-p2.y,2)));
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
void set_mainstate(int newstate){
  if(newstate != mainstate_msg.data){
    float dt = (ros::Time::now() - mainnstate_change).toSec();
    ROS_INFO("MAIN: MAINSTATE: %i -> %i (%.3f seconds in state)",mainstate_msg.data,newstate,dt);
    mainnstate_change         = ros::Time::now();
    mainstate_msg.data  = newstate;
  }
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
  if(mainstate_msg.data == 0 && transformStamped.transform.translation.z >= 1.0)
    set_mainstate(1);
  else if(mainstate_msg.data == 2 && sqrt(pow(transformStamped.transform.translation.x,2)+pow(transformStamped.transform.translation.y,2)) < 3)
    set_mainstate(3);

  if(sqrt(pow(transformStamped.transform.translation.x - last_pose.pose.position.x,2)+ pow(transformStamped.transform.translation.y - last_pose.pose.position.y,2)+
    pow(transformStamped.transform.translation.z - last_pose.pose.position.z,2)) >= 1.00 || abs(get_shortest(tf::getYaw(transformStamped.transform.rotation),tf::getYaw(last_pose.pose.orientation)) * rad2deg) > 30){
      last_pose.pose.position.x = transformStamped.transform.translation.x;
      last_pose.pose.position.y = transformStamped.transform.translation.y;
      last_pose.pose.position.z = transformStamped.transform.translation.z;
    last_pose.pose.orientation = transformStamped.transform.rotation;
    last_pose.header           = transformStamped.header;
    path_visited.poses.push_back(last_pose);
    path_visited.header.stamp = ros::Time::now();
  }
}

void live_stuff(){
  if((ros::Time::now() - start).toSec() > 5 && !invoke_published){
    std_msgs::String invoke_msg;
    invoke_msg.data = "roslaunch tb_jul m600.launch";
    invoke_published = true;
    invoke_pub.publish(invoke_msg);
  }
  else if((ros::Time::now() - start).toSec() > 10 && !bag_published){
    std_msgs::String invoke_msg;
    invoke_msg.data = "rosbag record -O /home/nuc/bag.bag -a";
    bag_published = true;
    invoke_pub.publish(invoke_msg);
  }
  else if((ros::Time::now() - start).toSec() > 180 && mainstate_msg.data == 1)
    set_mainstate(2);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "tb_fsm_node");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");

  tf2_ros::TransformListener tf2_listener(tfBuffer);

  float centroid_side = 20;
  ros::Rate rate(5.0);
  start = ros::Time::now();
  par_live = true;
  last_pose.header.frame_id = path_visited.header.frame_id = "map";
  last_pose.pose.orientation.w = 1;
  invoke_pub   											= nh.advertise<std_msgs::String>("/tb_invoke",10);
  ros::Publisher pub_mainstate      = nh.advertise<std_msgs::UInt8>("/tb_fsm/main_state",10);
  ros::Publisher pub_path_visited   = nh.advertise<nav_msgs::Path>("/tb_world/path_visited",10);

  while(ros::ok()){
    if((ros::Time::now() - start).toSec() > 3)
      checktf();
    pub_mainstate.publish(mainstate_msg);
    pub_path_visited.publish(path_visited);
    if(par_live)
      live_stuff();
    rate.sleep();
    ros::spinOnce();
  }
  return 0;
}
