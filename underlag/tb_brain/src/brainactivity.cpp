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
#include <nav_msgs/Odometry.h>
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


void activity_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
  targetcmds_sent.push_back(*msg);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "tb_brainactivity_node");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");
  private_nh.param("par_zjump",   par_zjump, 3.0);
	private_nh.param("par_xyjump", par_xyjump, 5.0);
	for(int i = 0; i < 40; i++){
		z_lvls.push_back(par_zjump*i);
	}
	tf2_ros::TransformListener tf2_listener(tfBuffer);

	ros::Subscriber si11 = nh.subscribe("/tb_obst/rangevertical_ll40",10, laz1d_cb);
	ros::Subscriber si12 = nh.subscribe("/tb_obst/camera_range",10, camrange_cb);
	ros::Subscriber si13 = nh.subscribe("/tb_obst/camera_endpoint",10, cam_obs_cb);
	ros::Subscriber si14 = nh.subscribe("/tb_obst/xy_obs",10, xy_obs_cb);
	ros::Subscriber si4  = nh.subscribe("/tb_obst/z_obs",1,z_obs_cb);
	ros::Subscriber si15 = nh.subscribe("/tb_obst/z_distance",100,&z_distance_cb);
	ros::Subscriber si94 = nh.subscribe("/tb_obst/xy_distance",10,xy_distance_cb);

	pub_altlvl			 	 	= nh.advertise<std_msgs::UInt8>("/tb_cmd/set_altlvl", 100);
	pub_altcmd			 	 	= nh.advertise<std_msgs::UInt8>("/tb_cmd/set_altcmd", 100);
	pub_activity		 	 	= nh.advertise<std_msgs::String>("/tb_cmd/set_activity", 100);
	pub_armstate			 	= nh.advertise<geometry_msgs::Vector3Stamped>("/tb_fsm/set_armstate", 100);

	ros::Subscriber s94 = nh.subscribe("/tb_nav/lowrate_odom",10,lowrateodom_cb);
	ros::Subscriber s9  = nh.subscribe("/tb_nav/path_visited",10,visited_cb);
	ros::Subscriber s2  = nh.subscribe("/tb_path",10, path_cb);
	ros::Subscriber s14 = nh.subscribe("/tb_fsm/mission_state",10,missionstate_cb);
	ros::Subscriber s5  = nh.subscribe("/tb_fsm/main_state",100,&mainstate_cb);

  ros::Rate rate(2.0);
  ros::Time start = ros::Time::now();
  while(ros::ok()){
		print_sitrep();
    rate.sleep();
    ros::spinOnce();
  }
  return 0;
}
