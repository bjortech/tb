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

tf2_ros::Buffer tfBuffer;
nav_msgs::Path path_visited,path_visited_cmd;
geometry_msgs::PoseStamped last_pose,last_pose_cmd;
double par_dst3d_resolution,par_degrees_resolution;


float get_dst3d(geometry_msgs::Point p1, geometry_msgs::Point p2){
  return(sqrt(pow(p1.x-p2.x,2)+pow(p1.y-p2.y,2)+pow(p1.z-p2.z,2)));
}
float get_shortest(float target_heading,float actual_hdng){
  float a = target_heading - actual_hdng;
  if(a > M_PI)a -= M_PI*2;
  else if(a < -M_PI)a += M_PI*2;
  return a;
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
	/*
	1) Get latest transform from map to base_stabilized.
	2) Check if movement of base_stabilized* in either xyz(3D position) or degrees(rotation around z-axis (yaw))
	exceeds respective resolution parameters: par_dst3d_resolution,par_degrees_resolution (set in launch file)
		*base_stabilized is the last transform before base_link, it gives 4 Degrees Of Freedom (x,y,z,yaw).
		(transform tree: map->odom->base_footprint->base_stabilized->base_link)
	3) If it is, add the last position to the vector of poses to the chain of geometry_msgs::PoseStamped that creates a path
	*/
	if(sqrt(pow(transformStamped.transform.translation.x - last_pose.pose.position.x,2)+
	  pow(transformStamped.transform.translation.y - last_pose.pose.position.y,2)+
    pow(transformStamped.transform.translation.z - last_pose.pose.position.z,2)) >= par_dst3d_resolution || abs(get_shortest(tf::getYaw(transformStamped.transform.rotation),tf::getYaw(last_pose.pose.orientation)) *  180.0/M_PI) > par_degrees_resolution){
    last_pose.pose.position.x  = transformStamped.transform.translation.x;
    last_pose.pose.position.y  = transformStamped.transform.translation.y;
    last_pose.pose.position.z  = transformStamped.transform.translation.z;
    last_pose.pose.orientation = transformStamped.transform.rotation;
    last_pose.header           = transformStamped.header;
    path_visited.poses.push_back(last_pose);
		path_visited.header = transformStamped.header;
    checktfcmd();
  }
}
int main(int argc, char** argv)
{
  ros::init(argc, argv, "tb_trajectory_writer_node ");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");
	private_nh.param("translation_euclidian_resolution",  par_dst3d_resolution, 180.0);
	private_nh.param("turnangle_degrees_resolution",  	  par_degrees_resolution, 1.0);

  tf2_ros::TransformListener tf2_listener(tfBuffer);
	///********TRAJECTORYWRITING NODE*************////////////
  ros::Rate rate(5.0);
	last_pose.pose.orientation.w = 1;
  last_pose.header.frame_id = path_visited.header.frame_id = "map";

  ros::Publisher pub_path_visited     = nh.advertise<nav_msgs::Path>("/tb_trajectorywriter/visited_poses",10);
  ros::Publisher pub_path_visited_cmd = nh.advertise<nav_msgs::Path>("/tb_trajectorywriter/targeted_poses",10);

  while(ros::ok()){
		checktf();
    pub_path_visited.publish(path_visited);
    pub_path_visited_cmd.publish(path_visited_cmd);

    rate.sleep();
    ros::spinOnce();
  }
  return 0;
}
