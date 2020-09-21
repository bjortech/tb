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

std::string inspection_type = "idle";

ros::Publisher pub_get_next_path,pub_cmd;
geometry_msgs::PoseStamped target,base_pose;
ros::Time activity_change;
std_msgs::Float64 arm1_tilt_msg;
geometry_msgs::Vector3 vlp_rpy;
geometry_msgs::Point pos,pnt_midpoint;
nav_msgs::Odometry odom;
nav_msgs::Path path_vlp,path_obs,path_targets,path_visited,path_cleared_full,path_obstacles_full,path_targets_sent,path_sides,path_down;
int mainstate;
int path_targets_i = 0;
int inspection_count = 0;
double par_vlpmaxtilt,par_vlpmintilt,par_vlptiltinterval,par_takeoffaltitude;
bool tiltlimit_reached,tilting;
cv::Mat img(1000,1000,CV_8UC3,cv::Scalar(0, 0, 0)); //create image, set encoding and size, init pixels to default val
cv::Mat img_blank(1000,1000,CV_8UC3,cv::Scalar(0, 0, 0)); //create image, set encoding and size, init pixels to default val

float y2r(float y, float rows,float res){
  return (rows / 2 - y / res);
}
float x2c(float x, float cols,float res){
  return (x / res + cols/2);
}
int r2y(float r, float rows,float res){
  return int((rows / 2 - r) * res);
}
int c2x(float c, float cols,float res){
  return int((c - cols / 2) * res);
}
cv::Point pnt2cv(geometry_msgs::Point pin){
	int c = x2c(pin.x,img.cols,1);
	int r = y2r(pin.y,img.rows,1);
	return cv::Point(c,r);
}
std_msgs::Header hdr(){
  std_msgs::Header header;
  header.frame_id = "map";
  header.stamp    = ros::Time::now();
  return header;
}

double get_shortest(double target_hdng,double actual_hdng){
  double a = target_hdng - actual_hdng;
  if(a > M_PI)a -= M_PI*2;
  else if(a < -M_PI)a += M_PI*2;
  return a;
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

nav_msgs::Path create_path(geometry_msgs::Point midpoint,float area_sidelength,int num_centroids){
  nav_msgs::Path pathout;
  float centroid_sidelength = area_sidelength / num_centroids;
  for(int y = 0; y < num_centroids; y++){
    for(int x = 0; x < num_centroids; x++){
      geometry_msgs::PoseStamped pose;
      pose.header.frame_id      = "map";
      pose.header.stamp         = ros::Time::now();
      pose.pose.position.x      = midpoint.x + pow(-1,y) * (x * centroid_sidelength - area_sidelength / 2 + centroid_sidelength/2);
      pose.pose.position.y      = midpoint.y + -1 * (y * centroid_sidelength - area_sidelength / 2 + centroid_sidelength/2);
      pose.pose.position.z      = midpoint.z;
      pose.pose.orientation.w   = 1;
      pathout.poses.push_back(pose);
    }
  }
  pathout.header.frame_id = "map";
  pathout.header.stamp = ros::Time::now();
  return pathout;
}


void request_paths(){
	path_down.header.frame_id = "empty";
	path_sides.header.frame_id = "empty";
	path_down.poses.resize(0);
	path_sides.poses.resize(0);
  ROS_INFO("BRAIN: Requesting paths");
	std_msgs::UInt8 get_path;
  pub_get_next_path.publish(get_path);
}

void set_target_pose(geometry_msgs::PoseStamped new_target_pose){
	target 	 = new_target_pose;
	path_targets_sent.poses.push_back(new_target_pose);
	pub_cmd.publish(new_target_pose.pose.position);
}
void set_target_path(nav_msgs::Path pathin){
	path_targets       = pathin;
	path_targets_i     = 0;
	set_target_pose(path_targets.poses[path_targets_i]);
}

void check_path_progress(){
  if(path_targets_i < path_targets.poses.size()){
    float dst_to_target = get_dst3d(path_targets.poses[path_targets_i].pose.position,pos);
    if(dst_to_target < 5){
      ROS_INFO("BRAIN: Dst to target[%i of %i]: %.0f",path_targets_i,path_targets.poses.size(),dst_to_target);
      if(path_targets_i+1 < path_targets.poses.size()){
        path_targets_i++;
        set_target_pose(path_targets.poses[path_targets_i]);
      }
      else
        request_paths();
    }
  }
  else{
		inspection_type == "idle";
		request_paths();
	}
}
void check_pnt_progress(){
	float dst_to_target = get_dst3d(target.pose.position,pos);
	if(dst_to_target < 5)
		inspection_type == "idle";
}



bool dst_point_in_path_lim(nav_msgs::Path pathin,geometry_msgs::Point pin,float lim){
  float res,dst;
  if(pathin.poses.size() == 0)
    return res;
  for(int i = 0; i < pathin.poses.size(); i++){
     if(get_dst3d(pathin.poses[i].pose.position,pin) < lim)
        return false;
  }
  return true;
}
void append_world_poses(nav_msgs::Path pathin,std::string type){
	for(int i = 0; i < pathin.poses.size(); i++){
		if(type == "cleared"){
			if(dst_point_in_path_lim(path_cleared_full,pathin.poses[i].pose.position,5))
				path_cleared_full.poses.push_back(pathin.poses[i]);
		}
		if(type == "obstacles"){
			if(dst_point_in_path_lim(path_obstacles_full,pathin.poses[i].pose.position,5))
				path_obstacles_full.poses.push_back(pathin.poses[i]);
		}
	}
}

void checktf(){
  geometry_msgs::TransformStamped transformStamped;
  try{
    transformStamped = tfBuffer.lookupTransform("map","velodyne_aligned",
                             ros::Time(0));
  }
  catch (tf2::TransformException &ex) {
    ROS_WARN("%s",ex.what());
    ros::Duration(1.0).sleep();
  }
	pos.x   = transformStamped.transform.translation.x;
	pos.y   = transformStamped.transform.translation.y;
	pos.z   = transformStamped.transform.translation.z;
	tf2::Matrix3x3 q(tf2::Quaternion(transformStamped.transform.rotation.x, transformStamped.transform.rotation.y, transformStamped.transform.rotation.z, transformStamped.transform.rotation.w));
	q.getRPY(vlp_rpy.x,vlp_rpy.y,vlp_rpy.z);
	float delta_vlp = get_shortest(arm1_tilt_msg.data,vlp_rpy.y);
	if(delta_vlp < 0.05 && delta_vlp > -0.05)
		tilting = false;
}

void pathdown_cb(const nav_msgs::Path::ConstPtr& msg){
  path_down = *msg;
	if(inspection_type == "down"){
		if(msg->poses.size() > 0){
			ROS_INFO("BEHAV: Continuing inspection with new path (%i poses)",msg->poses.size());
			set_target_path(*msg);
		}
		else{
			ROS_INFO("BEHAV: Path cannot continue, poses is empty, idling");
			inspection_type = "idle";
		}
	}
}
void pathsides_cb(const nav_msgs::Path::ConstPtr& msg){
  path_sides = *msg;
	if(inspection_type == "sides"){
		if(msg->poses.size() > 0){
			ROS_INFO("BEHAV: Continuing inspection with new path (%i poses)",msg->poses.size());
			set_target_path(*msg);
		}
		else{
			ROS_INFO("BEHAV: Path cannot continue, poses is empty, idling");
			inspection_type = "idle";
		}
	}
}
void pathvlp_cb(const nav_msgs::Path::ConstPtr& msg){
  path_vlp = *msg;
  if(msg->poses.size() > 0)
    base_pose = msg->poses[0];
	append_world_poses(*msg,"cleared");
}
void pathvlpobs_cb(const nav_msgs::Path::ConstPtr& msg){
  path_obs = *msg;
	append_world_poses(*msg,"obstacles");
}
void lowrateodom_cb(const nav_msgs::Odometry::ConstPtr& msg){
  odom = *msg;
}

void pntmid_cb(const geometry_msgs::PointStamped::ConstPtr& msg){
	pnt_midpoint = msg->point;
}
void pathvstd_cb(const nav_msgs::Path::ConstPtr& msg){
	path_visited = *msg;
}
void mainstate_cb(const std_msgs::UInt8::ConstPtr& msg){
  mainstate = msg->data;
}
void set_tilt(float radians){
  float new_tilt = radians;
  if(new_tilt < par_vlpmintilt){
    tiltlimit_reached  = true;
    new_tilt = par_vlpmintilt;
  }
  else if(new_tilt > par_vlpmaxtilt){
    tiltlimit_reached  = true;
    new_tilt = par_vlpmaxtilt;
  }
	else
		tiltlimit_reached = false;
  if(arm1_tilt_msg.data - new_tilt > 0.1)
    tilting = true;
  ROS_INFO("BRAIN: ARM - tilt from %.2f -> %.2f",arm1_tilt_msg.data,new_tilt);
  arm1_tilt_msg.data = new_tilt;
}

void increment_tilt(float radians){
  float new_tilt = arm1_tilt_msg.data + radians;
  set_tilt(new_tilt);
}
void set_tilt_degrees(float degrees){
  float deg2rad = M_PI/180.0;
  set_tilt(deg2rad*degrees);
}
void increment_tilt_degrees(float degrees){
  float deg2rad = M_PI/180.0;
  increment_tilt(deg2rad*degrees);
}

void update_inspection(){
	if(inspection_type == "sides"
	|| inspection_type == "down"
	|| inspection_type == "area")
		check_path_progress();
	else if(inspection_type == "pnt")
		check_pnt_progress();
	else
		inspection_type = "idle";
}
void set_inspection(std::string type){
	inspection_count++;
	float dt = (ros::Time::now() - activity_change).toSec();
	ROS_INFO("MAIN: ACTIVITY: %s -> %s (%.3f seconds in state)",inspection_type.c_str(),type.c_str(),dt);
	inspection_type = type;

	if(type == "area")
		set_target_path(create_path(pos,30,4));
	if(type == "sides")
		set_target_path(path_sides);
	if(type == "down")
		set_target_path(path_down);
	if(type == "pnt")
		set_target_pose(target);
}

void progress_inspection(int idle_count){
	if(idle_count == 0)
		request_paths();
	if(idle_count == 1)
		set_tilt_degrees(20);
	if(idle_count == 2)
		set_tilt_degrees(-20);
	if(idle_count == 3)
		set_tilt(0);
	if(idle_count == 4){
    ROS_INFO("Set_Inspection, inspection_count: %i",inspection_count);
		if(inspection_count == 0)
			set_inspection("area");
		if(inspection_count == 1)
			set_inspection("sides");
		if(inspection_count == 2)
			set_inspection("down");
	}
}
cv::Scalar get_color(int base_blue,int base_green, int base_red){
	cv::Scalar color;
	color[0] = base_blue;
	color[1] = base_green;
	color[2] = base_red;
	return color;
}
void draw_path_at_img(nav_msgs::Path pathin,geometry_msgs::Point p0,
	 bool path_line,bool pose_yawline,bool pose_rectangle,bool pose_circle,bool pose_pnt,
  cv::Scalar color, int pose_size){
	geometry_msgs::Point pyaw,prect0,prect1,pnt;
  if(pathin.poses.size() < 1)
    return;
	if(p0.x == 0 && p0.y == 0)
		p0 = pathin.poses[0].pose.position;

	for(int i = 0; i < pathin.poses.size(); i++){
		pnt = pathin.poses[i].pose.position;
		float yaw = tf::getYaw(pathin.poses[i].pose.orientation);
		if(path_line)
			cv::line (img,  pnt2cv(p0), pnt2cv(pnt),color,1,cv::LINE_8,0);
		p0 = pnt;
		if(pose_yawline){
			pyaw.x = pnt.x + pose_size*2 * cos(yaw);
			pyaw.y = pnt.y + pose_size*2 * sin(yaw);
			cv::line (img,  pnt2cv(pnt), pnt2cv(pyaw),color,1,cv::LINE_8,0);
		}
		if(pose_rectangle){
			prect0.x = pnt.x-pose_size;
			prect0.y = pnt.y-pose_size;
			prect1.x = pnt.x+pose_size*2;
			prect1.y = pnt.y+pose_size*2;
			cv::rectangle(img, pnt2cv(prect0),pnt2cv(prect1),color,1,8,0);
		}
		if(pose_circle){
			cv::circle(img,pnt2cv(pnt),pose_size,color,1);
		}
		if(pose_pnt){
			img.at<cv::Vec3b>( y2r(pnt.y,img.rows,1),x2c(pnt.x,img.cols,1) )[0] = color[0];
			img.at<cv::Vec3b>( y2r(pnt.y,img.rows,1),x2c(pnt.x,img.cols,1) )[1] = color[1];
			img.at<cv::Vec3b>( y2r(pnt.y,img.rows,1),x2c(pnt.x,img.cols,1) )[2] = color[2];
		}
	}
}
void draw_options(int count){
  geometry_msgs::Point p0;
  img_blank.copyTo(img);
  cv::Scalar color_target = get_color(25,25,200);
  cv::Scalar color_sides  = get_color(125,0,0);
  cv::Scalar color_down   = get_color(0,125,0);
  cv::Scalar color_vlp    = get_color(50,50,50);
  cv::Scalar color_area   = get_color(0,50,50);
  cv::Scalar color_clear  = get_color(100,50,0);
  cv::Scalar color_obs    = get_color(0,50,100);
  nav_msgs::Path path_area = create_path(pos,30,4);

  int pcnt_cleared_full = path_cleared_full.poses.size();
  int pcnt_obstacles_full = path_obstacles_full.poses.size();
  int pcnt_area = path_area.poses.size();
  int pcnt_vlp = path_vlp.poses.size();
  int pcnt_sides = path_sides.poses.size();
  int pcnt_down = path_down.poses.size();
  int pcnt_targets = path_targets.poses.size();
  ROS_INFO("pcnt_cleared_full %i,pcnt_obstacles_full %i,pcnt_area %i,pcnt_vlp %i,pcnt_sides %i,pcnt_down %i,pcnt_targets %i",
            pcnt_cleared_full,   pcnt_obstacles_full,   pcnt_area,   pcnt_vlp,   pcnt_sides,   pcnt_down,   pcnt_targets);

  draw_path_at_img(path_cleared_full,p0,false,false,false,false,true,color_clear,1);
  draw_path_at_img(path_obstacles_full,p0,false,false,false,false,true,color_obs,1);
  draw_path_at_img(path_area,p0,true,false,false,false,true,color_area,1);
  draw_path_at_img(path_vlp,p0,true,false,false,false,true,color_vlp,1);

  draw_path_at_img(path_sides,pos,true,true,false,true,false,color_sides,2);
  draw_path_at_img(path_down,pos,true,false,true,false,false,color_down,2);
  draw_path_at_img(path_targets,pos,true,true,true,true,false,color_target,2);
  cv::imwrite("/home/nuc/brain/draw_options_"+std::to_string(count)+".png",img);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "tb_behavior_node");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");
	private_nh.param("par_vlpmaxtilt",par_vlpmaxtilt, M_PI/10);
	private_nh.param("par_vlpmintilt",par_vlpmintilt, -M_PI/10);
	private_nh.param("par_vlptiltinterval",par_vlptiltinterval, M_PI/10);
	private_nh.param("takeoff_altlvl",par_takeoffaltitude, 5.0);
	tf2_ros::TransformListener tf2_listener(tfBuffer);
	std_msgs::Float64 target_alt_msg;
	path_cleared_full.header   = hdr();
	path_obstacles_full.header = hdr();
	target.pose.position.z 	   = par_takeoffaltitude;
	target.pose.orientation.w  = 1;
	ros::Subscriber s1 = nh.subscribe("/tb_path/vlp_path",1,pathvlp_cb);
	ros::Subscriber s2 = nh.subscribe("/tb_path/vlp_obstacles",1,pathvlpobs_cb);
	ros::Subscriber s3 = nh.subscribe("/tb_path/out_down",10,pathdown_cb);
	ros::Subscriber s4 = nh.subscribe("/tb_path/out_side",10,pathsides_cb);
	ros::Subscriber s5 = nh.subscribe("/tb_path/visited",10,pathvstd_cb);

	ros::Subscriber s8 = nh.subscribe("/tb_fsm/main_state",100,&mainstate_cb);
	ros::Subscriber s9 = nh.subscribe("/tb_nav/lowrate_odom",10,lowrateodom_cb);

	ros::Publisher pub_path_cleared_full   = nh.advertise<nav_msgs::Path>("/tb_path/cleared_full",100);
	ros::Publisher pub_path_obstacles_full = nh.advertise<nav_msgs::Path>("/tb_path/obstacles_full",100);
  ros::Publisher pub_path = nh.advertise<nav_msgs::Path>("/tb_path/pathout",10);

	ros::Publisher pub_altcmd	 = nh.advertise<std_msgs::Float64>("/tb_cmd/alt_target", 100);
	ros::Publisher pub_tiltvlp = nh.advertise<std_msgs::Float64>("/tb_cmd/arm1_tilt", 10);
	ros::Publisher pub_tilt  	 = nh.advertise<std_msgs::Float64>("/tb_cmd/arm2_tilt", 10);
	ros::Publisher pub_pan	   = nh.advertise<std_msgs::Float64>("/tb_cmd/arm2_pan", 10);

	ros::Publisher pub_target_pose = nh.advertise<geometry_msgs::PoseStamped>("/cmd_pose",10);

	pub_get_next_path	 	= nh.advertise<std_msgs::UInt8>("/tb_path/get", 100);
	pub_cmd      				= nh.advertise<geometry_msgs::Point>("/cmd_pos",10);
	ros::Rate rate(2.0);
	int count = 0;
	int idle_count = 0;
	std::string last_type;
  while(ros::ok()){
		checktf();
    if(idle_count > 4)
      idle_count = 0;
		if(inspection_type == "idle"){
			if(!tilting){
        ROS_INFO("Idle count: %i",idle_count);
				progress_inspection(idle_count);
				idle_count++;
			}
		}
		if(mainstate == 1)
			check_path_progress();
    count++;
    ROS_INFO("inspection_type: %s",inspection_type.c_str());
    draw_options(count);
		target_alt_msg.data = target.pose.position.z;
    pub_target_pose.publish(target);
    pub_tiltvlp.publish(arm1_tilt_msg);
    pub_altcmd.publish(target_alt_msg);
    pub_path.publish(path_targets);

    pub_path_cleared_full.publish(path_cleared_full);
    pub_path_obstacles_full.publish(path_obstacles_full);
		pub_target_pose.publish(target);
    rate.sleep();
    ros::spinOnce();
  }
  return 0;
}
