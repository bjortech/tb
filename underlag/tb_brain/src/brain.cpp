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


using namespace octomap;
using namespace std;

tf2_ros::Buffer tfBuffer;
double par_xyjump,par_zjump,par_maprad,xy_dst,z_dst,target_dst;
int zlvl,buildingstate,mainstate,missionstate;
geometry_msgs::PolygonStamped poly_heading,poly_heading_rel,poly_safe;
nav_msgs::Odometry odom;
nav_msgs::Path path_visited,path_full,path;
sensor_msgs::LaserScan scan_cleared,scan_roofs,scan_dangers,scan_buildings;
sensor_msgs::Range camrange,rangevertical_ll40;
std::vector<int> z_lvls;
std::vector<geometry_msgs::PoseStamped> targetcmds_sent;
ros::Publisher pub_altlvl,pub_altcmd,pub_activity,pub_armstate;
float z,xy;
geometry_msgs::PointStamped xy_obs,z_obs,cam_obs;
geometry_msgs::Vector3Stamped armstate;
std_msgs::String current_activity;
geometry_msgs::Vector3 vel_xy_z_az;

float get_dst3d(geometry_msgs::Point p1, geometry_msgs::Point p2){
  return(sqrt(pow(p1.x-p2.x,2)+pow(p1.y-p2.y,2)+pow(p1.z-p2.z,2)));
}
void print_sitrep(){
	float sumvel = vel_xy_z_az.x + vel_xy_z_az.y + vel_xy_z_az.z;
	ROS_INFO("BRAIN-SitRep (closest obstacles): xy-plane state %s with %.2f clearing z-plane state %s with %.2f clearing",xy_obs.header.frame_id.c_str(),xy_dst,z_obs.header.frame_id.c_str(),z_dst);
	ROS_INFO("BRAIN-SitRep (ranges detected):   camera-depthmap-center: [%.2f of %.2f max] 1D-1aser-centerline: [%.2f of %.2f max]",camrange.range,camrange.max_range,rangevertical_ll40.range,rangevertical_ll40.max_range);
	ROS_INFO("BRAIN-SitRep (velocity detected): hori/verti/yawrate[sum: %.2f]: (%.2f / %.2f / %.2f)",sumvel,vel_xy_z_az.x,vel_xy_z_az.y,vel_xy_z_az.z);
	ROS_INFO("BRAIN-SitRep (target pose#%i): %.2f meters",targetcmds_sent.size(),target_dst);

	if(xy_obs.header.frame_id == "map"){

	}

	if(z_obs.header.frame_id  == "map"){

	}

	if(xy_obs.header.frame_id == "out_of_bounds"){

	}

	if(z_obs.header.frame_id  == "out_of_bounds"){

	}

	if(xy_obs.header.frame_id == "cleared"){

	}

	if(z_obs.header.frame_id  == "cleared"){

	}
}
geometry_msgs::PointStamped transformpoint(geometry_msgs::PointStamped pin,std::string frame_out){
  geometry_msgs::PointStamped pout;
  pin.header.stamp = ros::Time();
  try
  {
    pout = tfBuffer.transform(pin, frame_out);
  }
  catch (tf2::TransformException &ex)
  {
        ROS_WARN("Failure %s\n", ex.what());
  }
  return pout;
}

void send_armstate(float vlp16_tilt,float sdunk_tilt,float sdunk_pan,bool override_closest){
	armstate.vector.x = vlp16_tilt;
	armstate.vector.y = sdunk_tilt;
	armstate.vector.z = sdunk_pan;
	if(override_closest)
		armstate.header.frame_id = "override";
	else
		armstate.header.frame_id = "";
	pub_armstate.publish(armstate);
}/*
void assess_proximity(){
	xy_obs.header.frame_id =
	z_obs.header.frame_id  =

}*/
void assess_targets(){
	int path_size = path.poses.size();
}

void lowrateodom_cb(const nav_msgs::Odometry::ConstPtr& msg){
	odom = *msg;
	vel_xy_z_az.x = sqrt(pow(odom.twist.twist.linear.x,2)+pow(odom.twist.twist.linear.y,2));
	vel_xy_z_az.y = odom.twist.twist.linear.z;
	vel_xy_z_az.z = odom.twist.twist.angular.z;
  if(targetcmds_sent.size() > 0)
	 target_dst    = get_dst3d(odom.pose.pose.position,targetcmds_sent[targetcmds_sent.size()-1].pose.position);
}
void altlvl_cb(const std_msgs::UInt8::ConstPtr& msg){

}
void mainstate_cb(const std_msgs::UInt8::ConstPtr& msg){
  mainstate = msg->data;
}
void missionstate_cb(const std_msgs::UInt8::ConstPtr& msg){
 	missionstate = msg->data;
}
void visited_cb(const nav_msgs::Path::ConstPtr& msg){
  path_visited    = *msg;
}
void path_cb(const nav_msgs::Path::ConstPtr& msg){
	path = *msg;
}
void xy_obs_cb(const geometry_msgs::PointStamped::ConstPtr& msg){
	xy_obs = *msg;
}
void z_obs_cb(const geometry_msgs::PointStamped::ConstPtr& msg){
	z_obs = *msg;
}
void z_distance_cb(const std_msgs::Float64::ConstPtr& msg){
	z_dst = msg->data;
}
void xy_distance_cb(const std_msgs::Float64::ConstPtr& msg){
	xy_dst = msg->data;
}
void laz1d_cb(const sensor_msgs::Range::ConstPtr& msg){
	rangevertical_ll40 = *msg;
}
void camrange_cb(const sensor_msgs::Range::ConstPtr& msg){
	camrange = *msg;
}
void cam_obs_cb(const geometry_msgs::PointStamped::ConstPtr& msg){
	cam_obs = *msg;
}
void target_approved_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
  targetcmds_sent.push_back(*msg);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "tb_brain_node");
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
