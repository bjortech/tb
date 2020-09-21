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


using namespace std;

tf2_ros::Buffer tfBuffer;


const float rad2deg = 180.0/M_PI;
double par_zjump;
bool centroid_found,building_active,at_floor,path_requested;
int building_active_floor,building_n_active,building_floor_v0,building_floor_n0_targets,building_v0,building_n0_targets,missionstate,last_completion;
std_msgs::UInt8 bn_sn_msg;
ros::Publisher pub_bnstate,pub_path2proc,pub_poly,pub_centroid,pub_target;
geometry_msgs::PoseStamped last_pose,target;
geometry_msgs::PointStamped building_centroid;
geometry_msgs::PolygonStamped poly_building;
nav_msgs::Path visited_path,path_filtered,building_floors,path_candidates,path_floor_above,path_floor_below,processed_path,znpath,bbpath,vstdpath;
std::vector<int> building_floors_completed;
std::vector<int> building_floors_known;
std::vector<int> z_lvls;
std::vector<int> buildings_roofs;
std::vector<int> buildings_bases;
std::vector<int> buildings_known;
std::vector<bool> buildings_roofs_explored;
std::vector<geometry_msgs::Point> buildings_centroids;
std::vector<geometry_msgs::PoseStamped> targetcmds_sent;
std::vector<std::vector<nav_msgs::Path>> buildings_floorpaths;
std::vector<std::vector<geometry_msgs::PolygonStamped>> buildings_floorpolygons;

geometry_msgs::Point project_target(float offset){
  float target_yaw = tf::getYaw(target.pose.orientation);
  geometry_msgs::Point t;
  t.z = target.pose.position.z;
  t.x = target.pose.position.x - (-offset/2 + offset) * cos(target_yaw);
  t.y = target.pose.position.y - (-offset/2 + offset) * sin(target_yaw);
  return t;
}

void display_building(int bn){
	for(int i = 0; i < buildings_floorpaths[bn].size(); i++){
		ROS_INFO("Buildings_floorpaths[#:%i] size: %i",i,buildings_floorpaths[bn][i].poses.size());
	}
}
bool sort_dst_pair(const std::tuple<int,float>& a,
               const std::tuple<int,float>& b)
{
    return (std::get<1>(a) < std::get<1>(b));
}
double constrainAngle(double x){
  if(x > M_PI)
    return (x - M_PI*2);
  else if(x < -M_PI)
    return (x + M_PI*2);
  else
    return x;
}
bool in_poly(geometry_msgs::PolygonStamped polyin, float x, float y){
  int cross = 0;
	geometry_msgs::Point point;
	point.x = x;
	point.y = y;
  for(int i = 0,j = polyin.polygon.points.size()-1; i < polyin.polygon.points.size(); j=i++){//  for (int i = 0, j = vertices_.size() - 1; i < vertices_.size(); j = i++) {
    if ( ((polyin.polygon.points[i].y > point.y) != (polyin.polygon.points[j].y > point.y))
           && (point.x < (polyin.polygon.points[j].x - polyin.polygon.points[i].x) * (point.y - polyin.polygon.points[i].y) /
            (polyin.polygon.points[j].y - polyin.polygon.points[i].y) + polyin.polygon.points[i].x) )
    {
      cross++;
    }
  }
  return bool(cross % 2);
}

int get_zn(float z){
  for(int i = 0; i < z_lvls.size()-1; i++){
    if(z_lvls[i] < z && z_lvls[i+1] > z)
    return i;
  }
  return 0;
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
float get_hdng32(geometry_msgs::Point32 p1,geometry_msgs::Point32 p0){
  float dx = p1.x - p0.x;
  float dy = p1.y - p0.y;
  return atan2(dy,dx);
}
float closest_in_path(nav_msgs::Path pathin,geometry_msgs::Point pin,float dst_min){
  float res,dst;
  res = 1000;
  int res_i;
  if(pathin.poses.size() == 0)
    return res;
  for(int i = 1; i < pathin.poses.size(); i++){
    dst = get_dst3d(pathin.poses[i].pose.position,pin);
    if(dst < res && dst > dst_min){
      res = dst;
      res_i = i;
    }
  }
  return res_i;
}
double get_shortest(double target_hdng,double actual_hdng){
  double a = target_hdng - actual_hdng;
  if(a > M_PI)a -= M_PI*2;
  else if(a < -M_PI)a += M_PI*2;
  return a;
}

void update_target(geometry_msgs::PoseStamped ps){
//  if(!building_active)
//    building_centroid.point = project_target(10);
  targetcmds_sent.push_back(ps);
  pub_target.publish(ps);
}

std::vector<int> getinpath_indexes_inrad(nav_msgs::Path pathin,geometry_msgs::Point pos_to_check,float radians){
  std::vector<int> vec_out;
  if(pathin.poses.size() == 0){
    ROS_INFO("PathAdmin: pathin is empty");
    return vec_out;
  }
  for(int i = 0; i < pathin.poses.size(); i++){
    float dist = get_dst2d(pathin.poses[i].pose.position,pos_to_check);
    if(dist <= radians && dist > 0)
      vec_out.push_back(i);
  }
  return vec_out;
}
bool in_vec(std::vector<int> vec, int val){
  for(int i = 0; i < vec.size(); i++){
    if(vec[i] == val){
      return true;
    }
  }
  return false;
}


void activate_floor(int n){
  building_active_floor = n;
  path_floor_below.poses.resize(0);
  path_floor_above.poses.resize(0);
  at_floor = true;
}

void activate_building(int n){
  building_n_active = n;
  if(!building_active){
    building_active = true;
    building_floors_known.resize(0);
    building_floors_completed.resize(0);
    building_v0           = visited_path.poses.size();
    building_n0_targets   = targetcmds_sent.size();
    building_n_active     = buildings_known.size();
    buildings_known.push_back(building_n_active);
    ROS_INFO("Building Activated at v0: %i n0 %i",building_v0,building_n0_targets);
    activate_floor(get_zn(target.pose.position.z));
  }
}

void evaluate_floorshift(bool seek_up,int zn){
  std::vector<int> alternatives;
  for(int i = 0; i < building_floors_known.size(); i++){
    int known_candidate = building_floors_known[i];
    bool known_complete = in_vec(building_floors_completed,known_candidate);
    ROS_INFO("known_candidate %i completed %i ",known_candidate,known_complete);
    if(!known_complete)
      alternatives.push_back(known_candidate);
  }
  if(alternatives.size() == 0){
    ROS_INFO("No Alternatives!!");
  }
  else if(alternatives.size() == 1){
    activate_floor(alternatives[0]);
  }
  else if(seek_up && alternatives[1] > alternatives[0]) {
    activate_floor(alternatives[0]);
  }
  else{
    activate_floor(alternatives[1]);
  }
}

void create_floor(int target_bn_0,int target_bn_n,float offset,geometry_msgs::Point32 cen){
int numtarget = target_bn_n -  target_bn_0;
at_floor = false;
poly_building.polygon.points.resize(0);
int zn = get_zn(last_pose.pose.position.z);
building_floors_completed.push_back(zn);
	for(int i = target_bn_0; i < target_bn_n; i++){
		geometry_msgs::Point32 p;
		p.x = targetcmds_sent[i].pose.position.x;
		p.y = targetcmds_sent[i].pose.position.y;
		p.z = targetcmds_sent[i].pose.position.z;
		float hdng = get_hdng32(cen,p);
		float hdng_pose = tf::getYaw(targetcmds_sent[i].pose.orientation);
		float hdng_delta = get_shortest(hdng,hdng_pose);
		if(hdng_delta < M_PI/2)
			ROS_INFO("Hdng_delta OK: %.2f (%.2f -> %.2f)",hdng_delta,hdng,hdng_pose);
		else
			ROS_INFO("Hdng_delta DISC: %.2f (%.2f -> %.2f)",hdng_delta,hdng,hdng_pose);
		poly_building.polygon.points.push_back(p);

		buildings_floorpaths[building_n_active][zn].poses.push_back(targetcmds_sent[i]);
		std::vector<int> indexes_in_rad = getinpath_indexes_inrad(path_candidates,targetcmds_sent[i].pose.position,5.5);
		for(int k = 0; k < indexes_in_rad.size(); k++){
			geometry_msgs::PoseStamped ps;
			ps = path_candidates.poses[indexes_in_rad[k]];
			float dz = ps.pose.position.z - last_pose.pose.position.z;
			if(dz < 0 && dz > -par_zjump*1.5)
				path_floor_below.poses.push_back(ps);  //      buildings_floorpaths[building_n_active][zn-1].poses.push_back(ps);
			else if(dz > 0 && dz < par_zjump*1.5)
				path_floor_above.poses.push_back(ps);//      buildings_floorpaths[building_n_active][zn+1].poses.push_back(ps);
		}
	}

	buildings_centroids[building_n_active] = building_centroid.point;

	if(path_floor_below.poses.size() > numtarget - 20){
		ROS_INFO("Path path_floor_below: %i vs targets_z0: %i ",path_floor_below.poses.size(),numtarget);
		building_floors_known.push_back(zn-1);
	}
	else{
		buildings_bases[building_n_active] = zn+1;
		ROS_INFO("BASE of Building[#%i]: zn: %i z: %.2f",building_n_active,zn+1,z_lvls[zn+1]);
	}
	if(path_floor_above.poses.size() > numtarget - 20){
		ROS_INFO("Path path_floor_above: %i vs targets_z0: %i ",path_floor_above.poses.size(),numtarget);
		building_floors_known.push_back(zn+1);
	}
	else{
		buildings_roofs[building_n_active] = zn+1;
		ROS_INFO("ROOF of Building[#%i]: zn: %i z: %.2f",building_n_active,zn+1,z_lvls[zn+1]);
	}

	evaluate_floorshift(true,zn);
}

geometry_msgs::Point get_centroid(int v0,int vn){
	geometry_msgs::Point p;
	int centroid_sum_x = 0;
	int centroid_sum_y = 0;
	int centroid_sum_z = 0;
	int centroid_sum_count = vn - v0;
	for(int i = v0; i < vn; i++){
		centroid_sum_x += int(round(visited_path.poses[i].pose.position.x));
		centroid_sum_y += int(round(visited_path.poses[i].pose.position.y));
		centroid_sum_z += int(round(visited_path.poses[i].pose.position.z));
	}
	p.x = centroid_sum_x / centroid_sum_count;
	p.y = centroid_sum_y / centroid_sum_count;
	p.z = centroid_sum_z / centroid_sum_count;
	ROS_INFO("Centroid Calculated: %i points -> %.2f %.2f %.2f",centroid_sum_count,p.x,p.y,p.z);
	return p;
}
float get_completion_percent(int v0,int vn,geometry_msgs::Point cen){
	int radians_segmentsize = 72;
	std::vector<bool> visited;
	float a_pr_i = rad2deg * (2*M_PI / radians_segmentsize);
	visited.resize(radians_segmentsize);
	int count = 0;
	for(int i = v0; i < vn; i++){
		int deg = int(round((rad2deg * (M_PI + get_hdng(visited_path.poses[i].pose.position,cen)) ) ));
		visited[deg/a_pr_i] = true;
	}
	for(int i = 0; i < radians_segmentsize; i++){
		if(visited[i])
			count++;
	}
	int percentage_completion = count / radians_segmentsize * 100;
	ROS_INFO("Completion: %i radians visited of %i segmentsize, percentage: %i",count,radians_segmentsize,percentage_completion);
	return percentage_completion;
}

void update_centroid(){
	int completion = get_completion_percent(building_floor_v0,visited_path.poses.size(),building_centroid.point);
	if(completion >= 97){
		ROS_INFO("Floor complete");
		if(buildings_centroids[building_n_active].z == 0){
			geometry_msgs::Point32 p;
			p.x = building_centroid.point.x;
			p.y = building_centroid.point.y;
			p.z = building_centroid.point.z;
			create_floor(building_floor_n0_targets,targetcmds_sent.size(),10,p);
		}
	}
	else if(completion > 93){
		ROS_INFO("Sending start pose as final floor target: %.2f %.2f %.2f",visited_path.poses[building_floor_v0].pose.position.x,visited_path.poses[building_floor_v0].pose.position.y,visited_path.poses[building_floor_v0].pose.position.z);
		update_target(visited_path.poses[building_floor_n0_targets]);
	}
	else if(abs(completion - last_completion) >= 5){
		building_centroid.point = get_centroid(building_floor_v0,visited_path.poses.size());
    last_completion = completion;
    ROS_INFO("New Building Centroid[%i percent]: %.2f %.2f %.2f",last_completion,building_centroid.point.x,building_centroid.point.y,building_centroid.point.z);
	}
}


nav_msgs::Path path_2_path_lvls(nav_msgs::Path pathin){
  nav_msgs::Path pathout;
  pathout.header.frame_id = "map";
  pathout.header.stamp    = ros::Time::now();
  for(int i = 0; i < pathin.poses.size(); i++){
      std::vector<int> indexes_in_rad;
      indexes_in_rad = getinpath_indexes_inrad(pathin,pathin.poses[i].pose.position,4);
      if(indexes_in_rad.size() > 1){
        pathout.poses.push_back(pathin.poses[i]);
      }
    }
  return pathout;
}
nav_msgs::Path path_2_path_bb(nav_msgs::Path pathin,geometry_msgs::PolygonStamped poly_bb){
  nav_msgs::Path pathout;
  pathout.header.frame_id = "map";
  pathout.header.stamp    = ros::Time::now();
  for(int i = 0; i < pathin.poses.size(); i++){
    if(in_poly(poly_bb,pathin.poses[i].pose.position.x,pathin.poses[i].pose.position.y)){
      float pos_hdng  = get_hdng(pathin.poses[i].pose.position,building_centroid.point);
      float pose_hdng = tf::getYaw(pathin.poses[i].pose.orientation);
      ROS_INFO("Pos hdng: %.2f pose hdng: %.2f",pos_hdng,pose_hdng);
      pathout.poses.push_back(pathin.poses[i]);
    }
  }
  return pathout;
}
nav_msgs::Path get_path_2_process(int sn){
	path_requested = false;
  nav_msgs::Path path2process;
  ROS_INFO("Pathin: %i candidates",path_candidates.poses.size());
  if(sn == 0){
    ROS_INFO("Path to process in state %i is based on all pathnums with multiple elevations at same grid",sn);
    path2process = path_2_path_lvls(path_candidates);
    float searchrad = 5.5;
    std::vector<int> indexes_in_rad = getinpath_indexes_inrad(path2process,last_pose.pose.position,searchrad);
    if(indexes_in_rad.size() > 0){
      ROS_INFO("Path2process with multiple elevations at grid within radius of %.2f -> building_activation process begin",searchrad);
      activate_building(building_n_active);
    }
  }
  else if(sn == 1){
    path2process = path_candidates;
  }
  else if(sn == 2){
    ROS_INFO("Path to process in state %i is based on known floorplans and set building boundary",sn);
    path2process = path_2_path_bb(path_candidates,poly_building);
  }
  ROS_INFO("Pathout: %i targets",path2process.poses.size());
  return path2process;
}

int current_building_state(int bn){
  int state_out = -1;
  if(!building_active){
    ROS_INFO("Building not active,state: 0");
    state_out = 0;
  }
  else if(building_centroid.point.x == 0 && building_centroid.point.y == 0){
    ROS_INFO("Building centroid not started: State: 0");
    state_out = 0;
  }
  else if(buildings_centroids[bn].z == 0){
    state_out = 1;
    ROS_INFO("Building centroid not found: State: 1");
  }
  else if(buildings_roofs[bn] == 0){
    state_out = 2;
    ROS_INFO("Building roof not found: State: 2");
  }
  else if(buildings_bases[bn] == 0){
    state_out = 2;
    ROS_INFO("Building base not found: State: 3");
  }
  else if(!buildings_roofs_explored[bn]){
    state_out = 2;
    ROS_INFO("Building base not found: State: 3");
  }
  else{
    ROS_INFO("State unknown ERROR");
  }
  bn_sn_msg.data = state_out;
  return state_out;
}

void bn_cb(const std_msgs::UInt8::ConstPtr& msg){
  if(building_n_active != msg->data){
    ROS_INFO("New building n active: %i -> %i",building_n_active,msg->data);
    building_n_active = msg->data;
  }
}

void candpath_cb(const nav_msgs::Path::ConstPtr& msg){
  path_candidates = *msg;
}

void state_cb(const std_msgs::UInt8::ConstPtr& msg){
  if(msg->data == 1 && missionstate != 1){
    ROS_INFO("missionstate changes: %i -> %i",missionstate,msg->data);
  }
	missionstate = msg->data;
}

void create_building_templates(){
  nav_msgs::Path path_template;
  geometry_msgs::PolygonStamped poly_template;
  poly_template.header.frame_id = path_template.header.frame_id = "map";

  std::vector<nav_msgs::Path> path_template_building;
  std::vector<geometry_msgs::PolygonStamped> poly_template_building;
  for(int i = 0; i < 20; i++){
    z_lvls.push_back(int(round(-par_zjump*3 + par_zjump*i)));
    path_template_building.push_back(path_template);
    poly_template_building.push_back(poly_template);
  }
  geometry_msgs::Point p;
  for(int i = 0; i < 20; i++){
    buildings_floorpolygons.push_back(poly_template_building);
    buildings_floorpaths.push_back(path_template_building);
    buildings_centroids.push_back(p);
    buildings_roofs.push_back(0);
    buildings_bases.push_back(0);
    buildings_roofs_explored.push_back(false);
  }
}



void cmd_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
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
  if(sqrt(pow(transformStamped.transform.translation.x - last_pose.pose.position.x,2)+ pow(transformStamped.transform.translation.y - last_pose.pose.position.y,2)+
    pow(transformStamped.transform.translation.z - last_pose.pose.position.z,2)) >= 1.00){
    last_pose.pose.position.x  = transformStamped.transform.translation.x;
    last_pose.pose.position.y  = transformStamped.transform.translation.y;
    last_pose.pose.position.z  = transformStamped.transform.translation.z;
    last_pose.pose.orientation = transformStamped.transform.rotation;
    last_pose.header           = transformStamped.header;
    visited_path.poses.push_back(last_pose);
    visited_path.header.stamp  = ros::Time::now();
  }
}

void aquire_target_from_path(nav_msgs::Path pathin){
	int closest_i = closest_in_path(pathin,target.pose.position,3);
	if(closest_i < pathin.poses.size()){
		update_target(pathin.poses[closest_i]);
	}
}

void processedpath_cb(const nav_msgs::Path::ConstPtr& msg){
	path_requested = false;
  processed_path = *msg;
}

void dsttotarget_cb(const std_msgs::Float64::ConstPtr& msg){
	float dsttotarget = msg->data;
	if(building_active){
		if(dsttotarget < 7 && !path_requested){
			path_requested = true;
      pub_path2proc.publish(get_path_2_process(current_building_state(building_n_active)));
		}
		else if(dsttotarget < 5){
      aquire_target_from_path(vstdpath);
		}
		else
			update_centroid();
	}
}
void deltatarget_cb(const geometry_msgs::Vector3::ConstPtr& msg){
	geometry_msgs::Vector3 deltatarget;
	deltatarget = *msg;
}
void znpath_cb(const nav_msgs::Path::ConstPtr& msg){
  znpath = *msg;
}
void bbpath_cb(const nav_msgs::Path::ConstPtr& msg){
  bbpath = *msg;
}
void vstdpath_cb(const nav_msgs::Path::ConstPtr& msg){
  vstdpath = *msg;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "tb_fsmbuilding_node");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");
  private_nh.param("par_zjump",   par_zjump, 3.0);//*2.0);
  tf2_ros::TransformListener tf2_listener(tfBuffer);
  ros::Publisher pub_orig = nh.advertise<nav_msgs::Path>("/tb_path_filtered",100);

  create_building_templates();
	ros::Subscriber s3  = nh.subscribe("/tb_path_filtered",1,&candpath_cb);
  ros::Subscriber s4 	= nh.subscribe("/tb_fsm/mission_state",100,&state_cb);
	ros::Subscriber s5 	= nh.subscribe("/tb_fsm/building_n_active",100,&bn_cb);
//	ros::Subscriber s6 	= nh.subscribe("/tb_cmdmb/target_pose",100,&cmd_cb);
	ros::Subscriber s12 = nh.subscribe("/tb_path/path_not_visited",100,&processedpath_cb);
	ros::Subscriber s11 = nh.subscribe("/tb_spnt/target_distance",100,&dsttotarget_cb);
	ros::Subscriber s13 = nh.subscribe("/tb_spnt/target_delta",100,&deltatarget_cb);
  	ros::Subscriber s122 = nh.subscribe("/tb_path_filtered/zn",100,&znpath_cb);
  	ros::Subscriber s112 = nh.subscribe("/tb_path_filtered/bb",100,&bbpath_cb);
  	ros::Subscriber s132 = nh.subscribe("/tb_path_filtered/vstd",100,&vstdpath_cb);

  pub_poly      = nh.advertise<geometry_msgs::PolygonStamped>("/tb_polygon_building",100);
	pub_centroid  = nh.advertise<geometry_msgs::PointStamped>("/tb_cmd/building_centroid",100);
	pub_target 	 	= nh.advertise<geometry_msgs::PoseStamped>("/tb_cmd/building_target",100);
	pub_path2proc = nh.advertise<nav_msgs::Path>("/tb_fsm/path_to_process",100);
  pub_bnstate	  = nh.advertise<std_msgs::UInt8>("/tb_bld/bn_state",100);
  ros::Publisher pub_boolcand  = nh.advertise<std_msgs::Bool>("/tb_path/switch_candidate_path_onoff",100);

	ros::Rate rate(1.0);
  while(ros::ok()){
		checktf();
		if(missionstate == 1){
      if((ros::Time::now() - visited_path.header.stamp).toSec() > 4){
        if(!building_active)
          aquire_target_from_path(znpath);
        else
          aquire_target_from_path(vstdpath);
        }
    	pub_bnstate.publish(bn_sn_msg);
  		if(poly_building.polygon.points.size() > 2){
  			pub_poly.publish(poly_building);
        aquire_target_from_path(bbpath);
      }
    }
    rate.sleep();
    ros::spinOnce();
  }
  return 0;
}
