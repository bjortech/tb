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

std_msgs::UInt8 mainstate_msg,altlvl_msg,missionstate_msg,buildingstate_msg;
std_msgs::String activity_msg;
std_msgs::Float64 alt_target;
geometry_msgs::PoseStamped target,last_pose;
bool overriding_altitude,bag_published,par_live,got_map,invoke_published;
ros::Time last_override,start,activity_change,missionstate_change,mainnstate_change,altlvl_change,buildingstate_change;
std::vector<float> z_lvls;
nav_msgs::Path path_visited;
std::string cmd_mode;
geometry_msgs::Point cmd_pos;
int get_zn(float z){
  for(int i = 0; i < z_lvls.size()-1; i++){
    if(z_lvls[i] <= z && z_lvls[i+1] >= z){
      return i;
    }
  }
  return 0;
}
void set_activity(std::string newactivity){
  if(newactivity != activity_msg.data){
    float dt = (ros::Time::now() - activity_change).toSec();
    ROS_INFO("MAIN: ACTIVITY: %s -> %s (%.3f seconds in state)",activity_msg.data,newactivity,dt);
    activity_change   = ros::Time::now();
    activity_msg.data = newactivity;
  }
}
void set_missionstate(int newstate){
  if(newstate != missionstate_msg.data){
    float dt = (ros::Time::now() - missionstate_change).toSec();
    ROS_INFO("MAIN: MISSIONSTATE: %i -> %i (%.3f seconds in state)",missionstate_msg.data,newstate,dt);
    missionstate_change       = ros::Time::now();
    missionstate_msg.data = newstate;
  }
}
void set_mainstate(int newstate){
  if(newstate != mainstate_msg.data){
    float dt = (ros::Time::now() - mainnstate_change).toSec();
    ROS_INFO("MAIN: MAINSTATE: %i -> %i (%.3f seconds in state)",mainstate_msg.data,newstate,dt);
    mainnstate_change         = ros::Time::now();
    mainstate_msg.data  = newstate;
  }
}
void set_buildingstate(int newstate){
  if(newstate != buildingstate_msg.data){
    float dt = (ros::Time::now() - buildingstate_change).toSec();
    ROS_INFO("MAIN: BUILDINGSTATE: %i -> %i (%.3f seconds in state)",buildingstate_msg.data,newstate,dt);
    buildingstate_change    = ros::Time::now();
    buildingstate_msg.data  = newstate;
  }
}
void set_altlvl(){
  int new_altlvl = get_zn(alt_target.data);
  if(new_altlvl != altlvl_msg.data){
    float dt = (ros::Time::now() - altlvl_change).toSec();
    ROS_INFO("MAIN: ALTLVL %i(%.0fm) -> %i(%.0fm) (%.3f seconds in state)",altlvl_msg.data,z_lvls[altlvl_msg.data],new_altlvl,z_lvls[new_altlvl],dt);
    altlvl_change          = ros::Time::now();
    altlvl_msg.data        = new_altlvl;
  }
}

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

void update_target(geometry_msgs::PoseStamped ps){
  int zn1 = get_zn(ps.pose.position.z);
  int zn0 = get_zn(target.pose.position.z);
  target.pose.orientation = tf::createQuaternionMsgFromYaw(get_hdng(target.pose.position,ps.pose.position));
  ROS_INFO("MAIN-Target: [Hdng: %.2f] x: %.0f, y: %.0f, zn/z: %.0f",get_hdng(target.pose.position,ps.pose.position),ps.pose.position.x,ps.pose.position.y,zn1,ps.pose.position.z);
  ROS_INFO("MAIN-Pos:     [Yaw: %.2f] x: %.0f, y: %.0f, zn/z: %.0f",tf::getYaw(target.pose.orientation),target.pose.position.x,target.pose.position.y,zn0,target.pose.position.z);
  ROS_INFO("MAIN-Dist:         [%.0f] x: %.0f, y: %.0f, zn/z: %.0f",get_dst3d(ps.pose.position,target.pose.position),ps.pose.position.x - target.pose.position.x,ps.pose.position.y - target.pose.position.y,zn1-zn0,ps.pose.position.z - target.pose.position.z);

  target.pose.orientation = ps.pose.orientation;
  target.pose.position.x  = ps.pose.position.x;
  target.pose.position.y  = ps.pose.position.y;
  target.pose.position.z  = ps.pose.position.z;
  alt_target.data         = ps.pose.position.z;
  set_altlvl();
//  targetcmd_pub.publish(target);
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
}
int closest_in_path(nav_msgs::Path pathin){
  float res,dst;
  res = 1000;
  int res_i;
  if(pathin.poses.size() == 0)
    return res;
  for(int i = 1; i < pathin.poses.size(); i++){
    dst = get_dst3d(pathin.poses[i].pose.position,last_pose.pose.position);
    ROS_INFO("CLIENT: Closest: dst %.2f",dst);
    if(dst < res){
      res = dst;
      res_i = i;
    }
  }
  return res_i;
}
void path_cb(const nav_msgs::Path::ConstPtr& msg){
  if(msg->poses.size() >= 1){
    cmd_mode          = "cmd_pos";
    geometry_msgs::PoseStamped ps;
    ps = msg->poses[msg->poses.size()-1];// msg->poses[closest_in_path(*msg)];
    cmd_pos  = ps.pose.position;
    cmd_pub.publish(cmd_pos);
  }
}

void set_altcmd_cb(const std_msgs::Float64::ConstPtr& msg){
  alt_target.data = msg->data;
  set_altlvl();
}
void alttarget_cb(const std_msgs::Float64::ConstPtr& msg){
  alt_target.data = msg->data;
  set_altlvl();
}
void set_activity_cb(const std_msgs::String::ConstPtr& msg){
  set_activity(msg->data);
}
void set_missionstate_cb(const std_msgs::UInt8::ConstPtr& msg){
  set_missionstate(msg->data);
}
void set_altlvl_cb(const std_msgs::UInt8::ConstPtr& msg){
  altlvl_msg.data = msg->data;
  alt_target.data = z_lvls[altlvl_msg.data];
}
void target_approved_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
  update_target(*msg);
}
void cmdpos_cb(const geometry_msgs::Point::ConstPtr& msg)
{
  geometry_msgs::PoseStamped ps;
  ps.pose.position = *msg;
  ps.pose.orientation.w = 1;
  cmd_mode          = "cmd_pos";
  cmd_pos           = *msg;
  update_target(ps);
}
void live_stuff(){
  if((ros::Time::now() - start).toSec() > 5 && !invoke_published){
    std_msgs::String invoke_msg;
    invoke_msg.data = "roslaunch tb_nxtgen m600.launch";
    invoke_published = true;
    invoke_pub.publish(invoke_msg);
  }
  else if((ros::Time::now() - start).toSec() > 10 && !bag_published){
    std_msgs::String invoke_msg;
    invoke_msg.data = "rosbag record -O /home/nuc/bag.bag -a";
    bag_published = true;
    invoke_pub.publish(invoke_msg);
  }
  else if((ros::Time::now() - start).toSec() > 600 && mainstate_msg.data == 1)
    set_mainstate(2);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "tb_fsmmain_node");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");
  private_nh.param("par_zjump",   par_zjump, 3.0);//*2.0);
  private_nh.param("takeoff_altlvl",par_takeoffaltitude, 5.0);

  tf2_ros::TransformListener tf2_listener(tfBuffer);

  float centroid_side = 20;
  ros::Rate rate(5.0);
  start = ros::Time::now();
  altlvl_msg.data = 4;


  for(int i = 0; i < 40; i++){
    z_lvls.push_back(int(round(-par_zjump*3 + par_zjump*i)));
//   z_lvls.push_back(int(round(-par_zjump*3 + par_zjump*i)));
  }

  last_pose.header.frame_id = path_visited.header.frame_id = "map";

  ros::Subscriber s0  = nh.subscribe("/tb_cmdmb/target_pose", 10,&target_approved_cb);
  ros::Subscriber ss3 = nh.subscribe("/tb_cmd/set_altlvl",1,set_altlvl_cb);
  ros::Subscriber ss4 = nh.subscribe("/tb_cmd/alt_target",1,alttarget_cb);
//	ros::Subscriber ss4 = nh.subscribe("/tb_cmd/set_altcmd",1,set_altcmd_cb);
  ros::Subscriber ss6 = nh.subscribe("/tb_cmd/set_activity",1,set_activity_cb);
  ros::Subscriber ss7 = nh.subscribe("/tb_cmd/set_missionstate",1,set_missionstate_cb);
// ros::Subscriber sss = nh.subscribe("/cmd_pos",             100,&cmdpos_cb);
  //ros::Subscriber sp  = nh.subscribe("/tb_path",1,&path_cb);

  //cmd_pub      											= nh.advertise<geometry_msgs::Point>("/cmd_pos",10);
  invoke_pub   											= nh.advertise<std_msgs::String>("/tb_invoke",10);
  ros::Publisher pub_mainstate      = nh.advertise<std_msgs::UInt8>("/tb_fsm/main_state",10);
  ros::Publisher pub_missionstate   = nh.advertise<std_msgs::UInt8>("/tb_fsm/mission_state",10);
	ros::Publisher pub_altlvl         = nh.advertise<std_msgs::UInt8>("/tb_fsm/altlvl",100);
  //ros::Publisher pub_targetalt      = nh.advertise<std_msgs::Float64>("/tb_cmd/alt_target",100);
	ros::Publisher pub_activity       = nh.advertise<std_msgs::String>("/tb_fsm/current_activity",10);

  while(ros::ok()){
    if((ros::Time::now() - start).toSec() > 3)
      checktf();
    else
      alt_target.data = par_takeoffaltitude;
  //  pub_targetalt.publish(alt_target);
    pub_mainstate.publish(mainstate_msg);
    pub_altlvl.publish(altlvl_msg);
		pub_activity.publish(activity_msg);
    pub_missionstate.publish(missionstate_msg);

    if(par_live)
      live_stuff();
    rate.sleep();
    ros::spinOnce();
  }
  return 0;
}
