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
#include <geometry_msgs/Vector3Stamped.h>
#include <cv_bridge/cv_bridge.h>
#include <nav_msgs/Odometry.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib_msgs/GoalID.h>
#include <nav_msgs/GetPlan.h>

ros::Publisher pub_path;


std_msgs::Header hdr(){
  std_msgs::Header header;
  header.frame_id = "map";
  header.stamp    = ros::Time::now();
  return header;
}

float get_dst2d(geometry_msgs::Point p2, geometry_msgs::Point p1){
  return(sqrt(pow(p1.x-p2.x,2)+pow(p1.y-p2.y,2)));
}
float get_dst3d(geometry_msgs::Point p2, geometry_msgs::Point p1){
  return(sqrt(pow(p1.x-p2.x,2)+pow(p1.y-p2.y,2)+pow(p1.z-p2.z,2)));
}
float get_hdng(geometry_msgs::Point32 p1,geometry_msgs::Point p0){
  float dx = p1.x - p0.x;
  float dy = p1.y - p0.y;
  return atan2(dy,dx);
}
double get_shortest(double target_hdng,double actual_hdng){
  double a = target_hdng - actual_hdng;
  if(a > M_PI)a -= M_PI*2;
  else if(a < -M_PI)a += M_PI*2;
  return a;
}
float get_inclination(geometry_msgs::Point p2, geometry_msgs::Point p1){
  return atan2(p2.z - p1.z,get_dst2d(p2,p1));
}
float get_interval_zmx(nav_msgs::Path pathin,int i0, int in){
  float val_max = 0;
  for(int i = i0; i < in; i++){
    if(val_max < pathin.poses[i].pose.position.z)
      val_max = pathin.poses[i].pose.position.z;
  }
  return val_max;
}

nav_msgs::Path smooth_global_plan(nav_msgs::Path pathin){
  float current_vz = 0;
  nav_msgs::Path pathout = pathin;
  pathout.header = hdr();
  if(pathin.poses.size() < 5)
    return pathout;
  pathout.poses.resize(pathin.poses.size());
  std::queue<float> z_que;
  float z_sum = 0;
  int que_size = 10;
  float d1 = get_dst2d(pathin.poses[0].pose.position,pathin.poses[1].pose.position);
  float d2 = get_dst2d(pathin.poses[1].pose.position,pathin.poses[2].pose.position);
  float d3 = get_dst2d(pathin.poses[2].pose.position,pathin.poses[3].pose.position);
	int num_poses_sample = fmin(pathin.poses.size()-1,5);

  float dst_pr_pose   = (d1+d2+d3)/3.0;
  float dst_intervals = 15.0;
  int num_poses_intervals = dst_intervals / dst_pr_pose;
//  ROS_INFO("PATHCREATOR: Dst_pr_pose: %.2f (%i samples)",dst_pr_pose,num_poses_sample);
  for(int i = 0; i < pathin.poses.size(); i++){
    int i0 = fmax(i-num_poses_intervals/2,1);
    int in = fmin(i+num_poses_intervals/2,pathin.poses.size()-1);
    float zval = get_interval_zmx(pathin,i0,in);
    z_sum += zval;
    z_que.push(zval);
    if(z_que.size() > que_size){
      z_sum -= z_que.front();
      z_que.pop();
    }
    int i_mid = (i - z_que.size()/2);
    int i_set = i_mid;
    pathout.poses[i].pose.position.z = z_sum / z_que.size();
    if(i_mid < 0)
      i_set = i;
    pathout.poses[i_set].pose.position.z = z_sum / z_que.size();
//    ROS_INFO("PATHCREATOR: zmx between i0: %i & in: %i, z: %.0f i_mid: %i i_set: %i",i0,in,zval,i_mid,i_set);
  }
  if(num_poses_intervals < pathout.poses.size()){
    int i_end0 = pathout.poses.size()-num_poses_intervals/2;
    int i_end  = pathout.poses.size();
    float zmax_end = get_interval_zmx(pathout,i_end0,i_end);
    for(int i = fmax(pathout.poses.size()-num_poses_intervals,0); i < pathout.poses.size(); i++){
      pathout.poses[i].pose.position.z = zmax_end;
    }
  }
	pathout.header = hdr();
  return pathout;
}

void path_to_check_cb(const nav_msgs::Path::ConstPtr& msg){
	//DUE TO THE INTERPOLATION RETURNING NAN'S EVERY NOW AND THEN THIS MUST BE FIXED SO THAT RVIZ DON'T CRASH
	nav_msgs::Path pathcmd = smooth_global_plan(*msg);
	pub_path.publish(pathcmd);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "tb_path_zsmoother_node");
	//THIS NODE IS CREATED FOR CLARITY, NORMALLY IT IS INTEGRATED IN A LARGER NODE.
	//THIS DEMONSTRATES HOW EDTO IS USED TO VAIDATE THE MARGINS GIVEN BY THE PATHPLANNER
  ros::NodeHandle nh;
	ros::NodeHandle private_nh("~");
	ros::Subscriber s2  = nh.subscribe("/tb_path/path_to_zsmooth",1,path_to_check_cb);
	pub_path 					  = nh.advertise<nav_msgs::Path>("/tb_path/path_zsmoothed",10);

	ros::spin();

  return 0;
}
