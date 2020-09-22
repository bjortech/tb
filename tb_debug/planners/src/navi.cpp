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
#include <sensor_msgs/PointCloud.h>
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
#include <tb_msgsrv/Paths.h>

tf2_ros::Buffer tfBuffer;
std::string par_workdir = "/home/nuc/brain/pathfull/heights0.png";
cv::Mat img(1000,1000,CV_8UC3,cv::Scalar(0, 0, 0)); //create image, set encoding and size, init pixels to default val
cv::Mat img_blank(1000,1000,CV_8UC3,cv::Scalar(0, 0, 0)); //create image, set encoding and size, init pixels to default val
cv::Mat img_height(1000,1000,CV_8UC3,cv::Scalar(0, 0, 0)); //create image, set encoding and size, init pixels to default val
cv::Mat img_update(1000,1000,CV_8UC3,cv::Scalar(0, 0, 0)); //create image, set encoding and size, init pixels to default val
cv::Mat img_new(1000,1000,CV_8UC3,cv::Scalar(0, 0, 0)); //create image, set encoding and size, init pixels to default val
cv::Mat img_update_copy(1000,1000,CV_8UC3,cv::Scalar(0, 0, 0)); //create image, set encoding and size, init pixels to default val
std::string state_scanpoint = "target_gp";
std::string state_forwpah   = "filling";
std::string state_motion    = "looking_around";
std::string state_mb = "idle";
int state_looking_around = 0;
float state_octomap_zmax = 0;
float state_octomap_zmin = 0;
float rad2deg = 180.0/M_PI;
float deg2rad = M_PI/180;
///////////********CTRL***********//////////
int count_target_paths = 0;
int mainstate = 0;
bool got_map = false;
bool cansee_proximity = false;
bool par_debug_img;
double par_maprad,par_elevbb,par_zclearing,par_arearad;
///////////********CTRL***********//////////

float state_globalplan_elevz = 0;
std::vector<int> targetindexes_sent;
nav_msgs::Path path_forwsim,path_starsquare,path_global_plan,path_visited,path_grid;
ros::Publisher pub_odomreset,pub_path_forwsim,pub_setpoint,pub_mbtarget,pub_mbelevation,pub_odomelevation;

geometry_msgs::PolygonStamped poly_mb,poly_scanzone,poly_proximity;
ros::Time process_start;
std::string active_process;
using namespace octomap;
using namespace std;
shared_ptr<DynamicEDTOctomap> edf_ptr;
AbstractOcTree* abs_octree;
shared_ptr<OcTree> octree;

geometry_msgs::PointStamped perfalt_pnt,gp_pnt,scantarget_pnt,pos_pnt;
geometry_msgs::Point pos;
geometry_msgs::Vector3 vlp_rpy;
float inc_setpoint,yaw_setpoint;
float par_res = 1.0;
geometry_msgs::PoseStamped ps_forwsim,pose_setpoint,targetmb;
bool yaw_override;
float path_global_plan_zmax = 0;
nav_msgs::Odometry odom_global;
std_msgs::Float64 odom_elevation;
std_msgs::Float64 mb_elevation;
std_msgs::Header hdr(){
  std_msgs::Header header;
  header.frame_id = "map";
  header.stamp    = ros::Time::now();
  return header;
}
nav_msgs::Path create_starsquare(float area_rad){
  nav_msgs::Path path;
  path.poses.resize(9);
  path.header = hdr();
  float z = 0;
  path.poses[0].pose.position.x = 0;
  path.poses[0].pose.position.y = 0;
  path.poses[0].header= hdr();
  path.poses[0].pose.orientation.w = 1;
  path.poses[0].pose.position.z =z;

  path.poses[1].pose.position.x = -area_rad;
  path.poses[1].pose.position.y = area_rad;
  path.poses[1].header= hdr();
  path.poses[1].pose.orientation.w = 1;
  path.poses[1].pose.position.z =z;

  path.poses[2].pose.position.x = area_rad;
  path.poses[2].pose.position.y = 0;
  path.poses[2].header= hdr();
  path.poses[2].pose.orientation.w = 1;
  path.poses[2].pose.position.z =z;

  path.poses[3].pose.position.x = -area_rad;
  path.poses[3].pose.position.y = -area_rad;
  path.poses[3].header= hdr();
  path.poses[3].pose.orientation.w = 1;
  path.poses[3].pose.position.z =z;

  path.poses[4].pose.position.x = area_rad;
  path.poses[4].pose.position.y = area_rad;
  path.poses[4].header= hdr();
  path.poses[4].pose.orientation.w = 1;
  path.poses[4].pose.position.z =z;

  path.poses[5].pose.position.x = -area_rad;
  path.poses[5].pose.position.y = 0;
  path.poses[5].header= hdr();
  path.poses[5].pose.orientation.w = 1;
  path.poses[5].pose.position.z =z;

  path.poses[6].pose.position.x = area_rad;
  path.poses[6].pose.position.y = -area_rad;
  path.poses[6].header= hdr();
  path.poses[6].pose.orientation.w = 1;
  path.poses[6].pose.position.z =z;;


  path.poses[6].pose.position.x = 0;
  path.poses[6].pose.position.y = area_rad;
  path.poses[6].header= hdr();
  path.poses[6].pose.orientation.w = 1;
  path.poses[6].pose.position.z =z;

  path.poses[7].pose.position.x = 0;
  path.poses[7].pose.position.y = -area_rad;
  path.poses[7].header= hdr();
  path.poses[7].pose.orientation.w = 1;
  path.poses[7].pose.position.z =z;;


  path.poses[8].pose.position.x = 0;
  path.poses[8].pose.position.y = 0;
  path.poses[8].header= hdr();
  path.poses[8].pose.orientation.w = 1;
  path.poses[8].pose.position.z =z;
  return path;
}


float y2r(float y){
  return (img_height.rows / 2 - y / par_res);
}
float x2c(float x){
  return (x / par_res + img_height.cols/2);
}
int r2y(float r){
  return int((img_height.rows / 2 - r) * par_res);
}
int c2x(float c){
  return int((c - img_height.cols / 2) * par_res);
}
cv::Point pnt2cv(geometry_msgs::Point pin){
	int c = x2c(pin.x);
	int r = y2r(pin.y);
	return cv::Point(c,r);
}

float get_zmax(nav_msgs::Path pathin){
  float zmx = 0;
  for(int i = 0; i < pathin.poses.size(); i++){
    if(pathin.poses[i].pose.position.z > zmx){
      zmx = pathin.poses[i].pose.position.z;
    }
  }
  return zmx;
}

bool sort_dst_pair(const std::tuple<int,float>& a,
               const std::tuple<int,float>& b)
{
    return (std::get<1>(a) < std::get<1>(b));
}
void start_process(std::string name){
  float dt = (ros::Time::now() - process_start).toSec();
  if(active_process != "")
    ROS_INFO("Process: %s took %.4f sec",active_process.c_str(),dt);
  active_process = name;
  process_start  = ros::Time::now();
}
double saturate(double val, double max){
    if (abs(val) >= max)
      val = (val>0) ? max : -1 * max;
    else
      val = val;
    if((std::isnan(val)) || (std::isinf(val))){
      return 0;
    }
    else{
      return val;
    }
}
double constrainAngle(double x){
  if(x > M_PI)
    return (x - M_PI*2);
  else if(x < -M_PI)
    return (x + M_PI*2);
  else
    return x;
}
float get_hdng(geometry_msgs::Point p1,geometry_msgs::Point p0){
  float dx = p1.x - p0.x;
  float dy = p1.y - p0.y;
  return atan2(dy,dx);
}
float get_dst2d(geometry_msgs::Point p1, geometry_msgs::Point p2){
  return(sqrt(pow(p1.x-p2.x,2)+pow(p1.y-p2.y,2)));
}

float get_dst3d(geometry_msgs::Point p1, geometry_msgs::Point p2){
  return(sqrt(pow(p1.x-p2.x,2)+pow(p1.y-p2.y,2)+pow(p1.z-p2.z,2)));
}
float get_inclination(geometry_msgs::Point p2, geometry_msgs::Point p1){
  return atan2(p2.z - p1.z,get_dst2d(p1,p2));
}
float get_shortest(float target_heading,float actual_hdng){
  float a = target_heading - actual_hdng;
  if(a > M_PI)a -= M_PI*2;
  else if(a < -M_PI)a += M_PI*2;
  return a;
}
bool in_vec(std::vector<int> vec,int k){
	if(vec.size() == 0)
		return false;
	//	//ROS_INFO("in vec: %i",k);
	for(int i = 0; i < vec.size(); i++){
		if(vec[i] == k)
			return true;
	}
	return false;
}
geometry_msgs::Point get_random_pnt(nav_msgs::Path pathin){
  srand (time(NULL));
  int randi = rand() % pathin.poses.size();
  return pathin.poses[randi].pose.position;
}
int get_random_targetindex(nav_msgs::Path pathin,std::vector<int> blacklist){
  nav_msgs::Path pathout;
  pathout.header = hdr();
  for(int i = 0; i < pathin.poses.size(); i++){
    if(!in_vec(blacklist,i)){
      pathout.poses.push_back(pathin.poses[i]);
    }
  }
  srand (time(NULL));
  int randi = rand() % pathout.poses.size();
  return randi;
}
int get_next_targetindex(nav_msgs::Path pathin,std::vector<int> targetindexes_sent,bool use_random){
  if(use_random)
    return get_random_targetindex(pathin,targetindexes_sent);
  else{
    for(int i = 0; i < pathin.poses.size(); i++){
      if(!in_vec(targetindexes_sent,i))
        return i;
    }
    return -1;
  }
}

float get_inclination_target(){
  return get_inclination(scantarget_pnt.point,pos);
}
float get_inclination_error(float inclination_target){
  return get_shortest(get_inclination_target(),vlp_rpy.y);
}

float get_error_pos2d(){
  return get_dst2d(pos,pose_setpoint.pose.position);
}
float get_error_pos3d(){
  return get_dst3d(pos,pose_setpoint.pose.position);
}
float get_error_pos_z(){
  return pose_setpoint.pose.position.z - pos.z;
}
float get_error_incl(){
  return get_shortest(inc_setpoint,vlp_rpy.y);
}
float get_error_yaw(){
  return get_shortest(yaw_setpoint,vlp_rpy.z);
}
bool is_at_setpoint(){
  int err3d    = get_error_pos3d();
  int erryaw   = abs(get_error_yaw()*rad2deg);
  int errinc  = abs(get_error_incl()*rad2deg);
  if(err3d > 2)
    return false;
  if(yaw_override && erryaw > 15)
    return false;
  if(errinc > 10)
    return false;
  return true;
}
void send_mbtarget(geometry_msgs::Point pnt){
  targetmb.header = hdr();
  targetmb.pose.position = pnt;
  targetmb.pose.position.z = 0;
  targetmb.pose.orientation = tf::createQuaternionMsgFromYaw(get_hdng(pnt,pos));
  state_mb = "target_sent";
  pub_mbtarget.publish(targetmb);
}
void send_setpoint(geometry_msgs::Point pnt, float inc, float yaw, bool override_yaw){
  pose_setpoint.header.stamp = ros::Time::now();
  pose_setpoint.pose.position = pnt;
  inc_setpoint = inc;
  yaw_setpoint = yaw;
  yaw_override = override_yaw;
  pose_setpoint.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0,-inc,yaw);
  if(override_yaw)
    pose_setpoint.header.frame_id = "yaw";
  else
    pose_setpoint.header.frame_id = "map";
  pub_setpoint.publish(pose_setpoint);
}
void update_pos_vlp(){
  geometry_msgs::TransformStamped transformStamped;
  try{
    transformStamped = tfBuffer.lookupTransform("map","velodyne_aligned",
                             ros::Time(0));
  }
  catch (tf2::TransformException &ex) {
    ROS_WARN("%s",ex.what());
    ros::Duration(1.0).sleep();
  }
  pos.x = transformStamped.transform.translation.x;
  pos.y = transformStamped.transform.translation.y;
  pos.z = transformStamped.transform.translation.z;
  tf2::Matrix3x3 q(tf2::Quaternion(transformStamped.transform.rotation.x, transformStamped.transform.rotation.y, transformStamped.transform.rotation.z, transformStamped.transform.rotation.w));
  q.getRPY(vlp_rpy.x,vlp_rpy.y,vlp_rpy.z);
  vlp_rpy.y *= -1;
}


void setpoint_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
  pose_setpoint = *msg;
}
void vstd_cb(const nav_msgs::Path::ConstPtr& msg){
  path_visited = *msg;
}
void odom_cb(const nav_msgs::Odometry::ConstPtr& msg){
  if((ros::Time::now() - ps_forwsim.header.stamp).toSec() > 0.2){
    ps_forwsim.pose.position    = msg->pose.pose.position;
    ps_forwsim.pose.orientation = msg->pose.pose.orientation;
    path_forwsim.header         = hdr();
    ps_forwsim.header           = hdr();
    if(state_mb == "active"){
      if(msg->twist.twist.linear.x != 0 || msg->twist.twist.angular.z != 0)
        path_forwsim.poses.push_back(ps_forwsim);
      if(path_forwsim.poses.size() > 1){
        float dn = get_dst2d(path_forwsim.poses[path_forwsim.poses.size()-1].pose.position,path_forwsim.poses[0].pose.position);
        float dt = (path_forwsim.poses[path_forwsim.poses.size()-1].header.stamp - path_forwsim.poses[0].header.stamp).toSec();
        if(dn > 10 || dt > 3.0){
          state_forwpah = "active";
          pub_path_forwsim.publish(path_forwsim);
        }
        else
          state_forwpah = "filling";
      }
    }
  }
}

void mbfeedback_cb(const move_base_msgs::MoveBaseActionFeedback::ConstPtr& msg){
  if(state_mb != "active"){
    state_mb = "active";
  //  nav_msgs::Odometry odomreset;
  //  odomreset = odom_global;
  //  odomreset.pose.pose.orientation = tf::createQuaternionMsgFromYaw(vlp_rpy.z);
  //  pub_odomreset.publish(odom_global);
  }
    //ROS_INFO("mbfeedback received - targetmb approved(?)");
}
void mbres_cb(const move_base_msgs::MoveBaseActionResult::ConstPtr& msg){
  ROS_INFO("MoveBaseRes; %i",msg->status.status);
  state_mb = "idle";
  if(msg->status.status == 3){
  }
  if(msg->status.status == 4){
    ROS_INFO("targetmb complete");
  }
  //  PENDING=0  ACTIVE=1  PREEMPTED=2  SUCCEEDED=3  ABORTED=4  REJECTED=5  PREEMPTING=6  RECALLING=7  RECALLED=8*///  ABORTED= 4# The goal was aborted during execution by the action server due
}
void cmdmbres_cb(const std_msgs::Bool::ConstPtr& msg){
  state_mb = "idle";
  ROS_INFO("MoveBase Result: %i (true/false)",msg->data);
}
nav_msgs::Path cutoff_abs(nav_msgs::Path pathin,std::string type,float val_0,float val_N){
  if(pathin.poses.size() < 3)
    return pathin;
  nav_msgs::Path pathout;
  pathout.header = hdr();
  if(type == "hdng"){
    for(int i = 0; i < pathin.poses.size(); i++){
      float hdng = get_hdng(pathin.poses[i].pose.position,pos);
      if(hdng >= val_0 && hdng <= val_N)
        pathout.poses.push_back(pathin.poses[i]);
    }
  }
  if(type == "hdng_rel"){
    for(int i = 0; i < pathin.poses.size(); i++){
      float hdng_rel = get_shortest(get_hdng(pathin.poses[i].pose.position,pos),vlp_rpy.z);
      if(hdng_rel >= val_0 && hdng_rel <= val_N)
        pathout.poses.push_back(pathin.poses[i]);
    }
  }
  if(type == "hdng_back"){
    for(int i = 0; i < pathin.poses.size(); i++){
      float hdng_rel = get_shortest(get_hdng(pathin.poses[i].pose.position,pos),vlp_rpy.z);
      if(hdng_rel > M_PI/2 || hdng_rel < -M_PI/2)
        pathout.poses.push_back(pathin.poses[i]);
    }
  }
  if(type == "z"){
    for(int i = 0; i < pathin.poses.size(); i++){
      if(val_0 >= pathin.poses[i].pose.position.z && pathin.poses[i].pose.position.z <= val_N){
        pathout.poses.push_back(pathin.poses[i]);
      }
    }
  }
  if(type == "dst"){
    for(int i = 0; i < pathin.poses.size(); i++){
      float dst = get_dst2d(pathin.poses[i].pose.position,pos);
      if(dst >= val_0 && dst <= val_N){
        pathout.poses.push_back(pathin.poses[i]);
      }
    }
  }
  return pathout;
}
std::vector<float> analyze_path(nav_msgs::Path pathin){
  std::vector<float> vals;
  int num_z0 = 0;
  float zmax = 0;
  float closest_zpos = 1000;
  float farthest = 0;
  for(int i = 0; i < pathin.poses.size(); i++){
    float z = pathin.poses[i].pose.position.z;
    float dst = get_dst2d(pathin.poses[i].pose.position,pos);
    if(z >= pos.z && dst < closest_zpos)
      closest_zpos = dst;
    else if(closest_zpos == 1000 && dst > farthest)
      farthest = dst;
    if(z == 0)
      num_z0++;
    if(z > zmax)
      zmax = z;
  }
  vals.push_back(zmax);
  vals.push_back(farthest);
  vals.push_back(float(num_z0));
  vals.push_back(closest_zpos);
  return vals;
}

float get_inclination_forward(nav_msgs::Path pathin){
  float inc_mxmn = 0;
  for(int i = 0; i < pathin.poses.size(); i++){
    if(pathin.poses[i].pose.position.z != 0){
      float inc = get_inclination(pathin.poses[i].pose.position,pos);
        if(inc > inc_mxmn || inc*-1 > inc_mxmn)
          inc_mxmn = inc;
    }
  }
  return inc_mxmn;
}
void gridpath_cb(const nav_msgs::Path::ConstPtr& msg){
  path_grid = *msg;
}
void gpelev_cb(const nav_msgs::Path::ConstPtr& msg){
  path_global_plan = *msg;
}

float get_globalplan_zmax(float dst_cutoff){
  nav_msgs::Path gp_forw = cutoff_abs(path_global_plan,"dst",0,20);
  float zmax_forwsim     = get_zmax(gp_forw);
  return zmax_forwsim;
}
geometry_msgs::Point get_minpos_atpoint(){
  nav_msgs::Path pc = cutoff_abs(path_grid,"dst",0,25);
  geometry_msgs::Point pos_minz;
  pos_minz.z = get_zmax(pc);
  pos_minz.x = pos.x;
  pos_minz.y = pos.y;
  return pos_minz;
}
void forwelev_cb(const nav_msgs::Path::ConstPtr& msg){
  path_forwsim = *msg;
  if(path_forwsim.poses.size() > 0){
    float dn = get_dst2d(path_forwsim.poses[path_forwsim.poses.size()-1].pose.position,path_forwsim.poses[0].pose.position);
    float dt = (path_forwsim.poses[path_forwsim.poses.size()-1].header.stamp - path_forwsim.poses[0].header.stamp).toSec();
    if(dn > 15 || dt > 5.0)
      path_forwsim.poses.erase(path_forwsim.poses.begin());

    geometry_msgs::Point pnt  = path_forwsim.poses[0].pose.position;
    float tyaw                = tf::getYaw(path_forwsim.poses[0].pose.orientation);
    float globalplan_zmax     = get_zmax(path_global_plan);
    geometry_msgs::Point pnt2 = get_minpos_atpoint();
    pnt.z = fmax(pnt2.z,globalplan_zmax);
    //ROS_INFO("Pnt: x %.0f y %.0f yaw: %.0f pnt.z: %.0f globalplan_zmax: %.0f pnt2: %.0f ",pnt.x,pnt.y,tyaw,pnt.z,globalplan_zmax,pnt2.z);
    path_forwsim.poses.erase(path_forwsim.poses.begin());
    send_setpoint(pnt,get_inclination_target(),tyaw,false);
  }
}
geometry_msgs::Point get_ave_pnt(nav_msgs::Path pathin){
  geometry_msgs::Point pnt;
  for(int i = 0; i < pathin.poses.size(); i++){
    pnt.x += pathin.poses[i].pose.position.x;
    pnt.y += pathin.poses[i].pose.position.y;
    pnt.z += pathin.poses[i].pose.position.z;
  }
  pnt.x /= pathin.poses.size();
  pnt.y /= pathin.poses.size();
  pnt.z /= pathin.poses.size();
  return pnt;
}
void set_scantarget_frompath(nav_msgs::Path pathin,bool use_average){
  if(use_average)
    scantarget_pnt.point = get_ave_pnt(pathin);
  else if(pathin.poses.size() > 0){
    scantarget_pnt.point = pathin.poses[0].pose.position;
    for(int i = 0; i < pathin.poses.size(); i++){
      if(pathin.poses[i].pose.position.z > scantarget_pnt.point.z)
        scantarget_pnt.point = pathin.poses[i].pose.position;
    }
  }
  ROS_INFO("Scantarget set: %.0f %.0f %.0f",scantarget_pnt.point.x,scantarget_pnt.point.y,scantarget_pnt.point.z);
}


void look_in_direction(float hdng,float distance0,float distance1){
  nav_msgs::Path pathsegm = cutoff_abs(cutoff_abs(path_grid,"dst",distance0,distance1),"hdng",hdng-M_PI/4,hdng+M_PI/4);
  set_scantarget_frompath(pathsegm,false);
  geometry_msgs::Point pnt = get_minpos_atpoint();
  float zmax = get_zmax(pathsegm)+5;
  ROS_INFO("Look in: %.2f %.0f %.0f pnt: %.0f/%.0f",hdng,distance0,distance1,pnt.z,zmax);

  pnt.z = fmax(pnt.z,zmax);
  send_setpoint(pnt,get_inclination_target(),hdng,true);
}

void look_around(int state){
  if(state < 4){
    float d0 = state * 5;
    look_in_direction(0,d0,d0+10);
  }
  else if(state < 8){
    float d0 = (state-4) * 5;
    look_in_direction(M_PI/2,d0,d0+10);
  }
  else if(state < 12){
    float d0 = (state-8) * 5;
    look_in_direction(M_PI,d0,d0+10);
  }
  else if(state < 16){
    float d0 = (state-12) * 5;
    look_in_direction(-M_PI/2,d0,d0+10);
  }
}

void setpoint_from_gridpath(){

//  nav_msgs::Path pf       = ;
//  std::vector<float> pf_v = analyze_path(pf);

  //nav_msgs::Path pr = cutoff_abs(path_grid,"hdng_rel",-M_PI/2,-M_PI/6);
  //nav_msgs::Path pl = cutoff_abs(path_grid,"hdng_rel",M_PI/6,M_PI/2);
  //nav_msgs::Path pb = cutoff_abs(path_grid,"hdng_back",0,0);
  //nav_msgs::Path pc = cutoff_abs(path_grid,"dst",0,12);
  //nav_msgs::Path pc = cutoff_abs(path_grid,"dst",0,30);

  //nav_msgs::Path vstd = cutoff_abs(path_visited,"dst",0,50);
  //nav_msgs::Path pfv = cutoff_abs(vstd,"hdng_rel",-M_PI/6,M_PI/6);
  //nav_msgs::Path prv = cutoff_abs(vstd,"hdng_rel",-M_PI/2,-M_PI/6);
  //nav_msgs::Path plv = cutoff_abs(vstd,"hdng_rel",M_PI/6,M_PI/2);
  //nav_msgs::Path pbv = cutoff_abs(vstd,"hdng_back",0,0);

  //int vstd_f = pfv.poses.size();
  //int vstd_r = prv.poses.size();
  //int vstd_l = plv.poses.size();
  //int vstd_b = pbv.poses.size();

  //std::vector<float> pf_v = analyze_path(pf);
  //std::vector<float> pr_v = analyze_path(pr);
  //std::vector<float> pl_v = analyze_path(pl);
  //std::vector<float> pb_v = analyze_path(pb);
  //std::vector<float> pc_v = analyze_path(pc);
//  float closest_inclination = get_inclination_forward(pc);
//  if(state_mb == "idle"){

//}
  //ROS_INFO("Zmax0: f: %.0f r: %.0f l: %.0f b: %.0f c: %.0f",pf_v[0],pr_v[0],pl_v[0],pb_v[0],pc_v[0]);
  //ROS_INFO("DstN:  f: %.0f r: %.0f l: %.0f b: %.0f c: %.0f",pf_v[1],pr_v[1],pl_v[1],pb_v[1],pc_v[1]);
  //ROS_INFO("Numz0: f: %.0f r: %.0f l: %.0f b: %.0f c: %.0f",pf_v[2],pr_v[2],pl_v[2],pb_v[2],pc_v[2]);
  //ROS_INFO("DstZ:  f: %.0f r: %.0f l: %.0f b: %.0f c: %.0f",pf_v[3],pr_v[3],pl_v[3],pb_v[3],pc_v[3]);
}

void odomglobal_cb(const nav_msgs::Odometry::ConstPtr& msg){
  //ROS_INFO("odomglobal_cb");
  if((ros::Time::now() - odom_global.header.stamp).toSec() > 0.2){
    odom_global = *msg;
  }
}
void progress_looking_around(){
  if(state_looking_around > 16){
    state_motion = "";
    state_looking_around = 0;
  }
  else{
    state_looking_around++;
    look_around(state_looking_around);
  }
}
void progress_starpath(){
  if(state_mb == "idle"){
    int targetindex = get_next_targetindex(path_starsquare,targetindexes_sent,false);
    targetindexes_sent.push_back(targetindex);
    if(targetindex > 0 && targetindex < path_starsquare.poses.size())
      send_mbtarget(path_starsquare.poses[targetindex].pose.position);
    else
      state_motion = "done";
  }
}
void progress_random_waypoints(){
  send_mbtarget(get_random_pnt(cutoff_abs(path_grid,"hdng_rel",-M_PI/2,-M_PI/5)));
}

void progress_motion(){
  bool at_setpoint    = is_at_setpoint();
  if(at_setpoint){
    ROS_INFO("STATES: [mb: %s gp: %i sp: %s fw-in: %i] st: (%.0f) sm: %s at_setp: %i] ",state_mb.c_str(),path_global_plan.poses.size(),state_forwpah.c_str(),path_forwsim.poses.size(),scantarget_pnt.point.z,state_motion.c_str(),at_setpoint);

    if(state_mb == "idle"){
      if(state_motion == "looking_around")
        progress_looking_around();
      else if(state_motion == "random_waypoints")
        progress_random_waypoints();
      else if(state_motion == "starpath_waypoints")
        progress_starpath();
    }
  }
}
int main(int argc, char** argv)
{
  ros::init(argc, argv, "tb_navi_node");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");
  private_nh.param("mission_arearad", par_arearad, 75.0);

	tf2_ros::TransformListener tf2_listener(tfBuffer);
  path_starsquare = create_starsquare(75);
  targetindexes_sent.push_back(0);
  ros::Subscriber smbb = nh.subscribe("/move_base/feedback",100,&mbfeedback_cb);
  ros::Subscriber sb   = nh.subscribe("/move_base/result",100,&mbres_cb);
  ros::Subscriber sa   = nh.subscribe("/tb_cmdmb/success",1,cmdmbres_cb);

  ros::Subscriber s2   = nh.subscribe("/tb_env/gp_elevated",1,gpelev_cb);
  ros::Subscriber s1   = nh.subscribe("/tb_env/fw_elevated",1,forwelev_cb);
  ros::Subscriber s3   = nh.subscribe("/tb_env/gridpath",1,gridpath_cb);
  ros::Subscriber s9   = nh.subscribe("/tb_setpoint",1,setpoint_cb);

  ros::Subscriber as2 = nh.subscribe("/tb_world/path_visited",10,vstd_cb);
  ros::Subscriber s0  = nh.subscribe("/odom",1,odom_cb);
  ros::Subscriber s55 = nh.subscribe("/odom_global",1,odomglobal_cb);

  pub_odomreset  = nh.advertise<nav_msgs::Odometry>("/tb_odom/reset", 100);
  pub_path_forwsim  = nh.advertise<nav_msgs::Path>("/tb_nav/path_forwsim", 100);
  pub_setpoint      = nh.advertise<geometry_msgs::PoseStamped>("/tb_nav/setpoint",10);
  pub_mbtarget      = nh.advertise<geometry_msgs::PoseStamped>("/tb_cmd/posemb_exeq",10);
  pub_mbelevation   = nh.advertise<std_msgs::Float64>("/tb_env/mapelevation",10);
  pub_odomelevation = nh.advertise<std_msgs::Float64>("/tb_odom/perfect_alt",10);
  ros::Publisher pub_pnt_gp      = nh.advertise<geometry_msgs::PointStamped>("/tb_nav/gp",10);
  ros::Publisher pub_pnt_scantar = nh.advertise<geometry_msgs::PointStamped>("/tb_nav/sp",10);

  ros::Rate rate(20);
  int state = 0;
  while(ros::ok()){
    update_pos_vlp();
    odom_elevation.data = pose_setpoint.pose.position.z;
    mb_elevation.data   = fmax(odom_elevation.data,get_globalplan_zmax(20));
    if(get_zmax(path_grid) > 0){
      progress_motion();
      if(state_motion == ""){
        state_motion = "starpath_waypoints";
      }
    }


    scantarget_pnt.header = hdr();
    pub_mbelevation.publish(mb_elevation);
    pub_odomelevation.publish(odom_elevation);
    pub_pnt_scantar.publish(scantarget_pnt);
  //  update_targetmb();

    rate.sleep();
    ros::spinOnce();
  }
  return 0;
}
//
