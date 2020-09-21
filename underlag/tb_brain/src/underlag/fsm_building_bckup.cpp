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


using namespace octomap;
using namespace std;

const float rad2deg = 180.0/M_PI;
double par_zjump;
bool centroid_found,building_active,at_floor,path_requested,everyother;
int mainstate,missionstate,zlvl,building_active_floor,building_n_active,building_floor_v0,building_floor_n0_targets,building_v0,building_n0_targets,last_completion;
ros::Publisher pub_loop_completion,pub_floorchange,pub_poly,pub_centroid,pub_bldstate,pub_path_floor;
geometry_msgs::PoseStamped last_pose,target;
geometry_msgs::PointStamped building_centroid,building_centroid_initial;
geometry_msgs::PolygonStamped poly_building;
nav_msgs::Path building_path,path_visited,path_filtered,path_candidates,path_floor_above,path_floor_below;
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
std_msgs::UInt8 bldstate_msg;
geometry_msgs::Point32 get_poly_centroid(geometry_msgs::PolygonStamped polyin){
    geometry_msgs::Point32 centroid;
    double signedArea = 0.0;
    double x0 = 0.0; // Current vertex X
    double y0 = 0.0; // Current vertex Y
    double x1 = 0.0; // Next vertex X
    double y1 = 0.0; // Next vertex Y
    double a = 0.0;  // Partial signed area

    // For all vertices
    for (int i=0; i<polyin.polygon.points.size(); ++i)
    {
        x0 = polyin.polygon.points[i].x;
        y0 = polyin.polygon.points[i].y;
        x1 = polyin.polygon.points[(i+1) % polyin.polygon.points.size()].x;
        y1 = polyin.polygon.points[(i+1) % polyin.polygon.points.size()].y;
        a = x0*y1 - x1*y0;
        signedArea += a;
        centroid.x += (x0 + x1)*a;
        centroid.y += (y0 + y1)*a;
    }

    signedArea *= 0.5;
    centroid.x /= (6.0*signedArea);
    centroid.y /= (6.0*signedArea);

    return centroid;
}
std_msgs::Header hdr(){
  std_msgs::Header header;
  header.frame_id = "map";
  header.stamp    = ros::Time::now();
  return header;
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
int get_zn(float z){
  for(int i = 0; i < z_lvls.size()-1; i++){
    if(z_lvls[i] < z && z_lvls[i+1] > z)
    return i;
  }
  return 0;
}

std::vector<int> getinpath_indexes_inrad(nav_msgs::Path pathin,geometry_msgs::Point pos_to_check,float radians){
  std::vector<int> vec_out;
  if(pathin.poses.size() == 0){
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
geometry_msgs::Point project_target(float offset){
  float target_yaw = tf::getYaw(target.pose.orientation);
  geometry_msgs::Point t;
  t.z = target.pose.position.z;
  t.x = target.pose.position.x - (-offset/2 + offset) * cos(target_yaw);
  t.y = target.pose.position.y - (-offset/2 + offset) * sin(target_yaw);
  return t;
}

void activate_floor(int n){
  building_active_floor = n;
  path_floor_below.poses.resize(0);
  path_floor_above.poses.resize(0);
  at_floor = true;
  std_msgs::UInt8 new_floor_msg;
  new_floor_msg.data = n;
  pub_floorchange.publish(new_floor_msg);
}

void activate_building(int n){
  building_n_active = n;
  if(!building_active){
    bldstate_msg.data = 2;
    building_active = true;
    building_floors_known.resize(0);
    building_floors_completed.resize(0);
    building_v0           = path_visited.poses.size();
    building_n_active     = buildings_known.size();
    buildings_known.push_back(building_n_active);
    ROS_INFO("BLD: Building Activated at v0: %i n0 %i",building_v0,building_n0_targets);
    activate_floor(get_zn(target.pose.position.z));
  }
}

void evaluate_floorshift(bool seek_up,int zn){
  std::vector<int> alternatives;
  for(int i = 0; i < building_floors_known.size(); i++){
    int known_candidate = building_floors_known[i];
    bool known_complete = in_vec(building_floors_completed,known_candidate);
    ROS_INFO("BLD: known_candidate %i completed %i ",known_candidate,known_complete);
    if(!known_complete)
      alternatives.push_back(known_candidate);
  }
  if(alternatives.size() == 0){
    ROS_INFO("BLD: No Alternatives!!");
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
geometry_msgs::Point32 offset32(geometry_msgs::Point32 pin, float rad,float offset){
  float target_yaw = tf::getYaw(target.pose.orientation);
  geometry_msgs::Point32 t;
  t.z = pin.z;
  t.x = pin.x + offset * cos(rad);
  t.y = pin.y + offset * sin(rad);
  return t;
}
geometry_msgs::PolygonStamped project_poly(geometry_msgs::PolygonStamped polyin, float offset){
	geometry_msgs::Point32 centroid;
	centroid = get_poly_centroid(polyin);
	for(int i = 0; i < polyin.polygon.points.size(); i++){
		float rad = get_hdng32(polyin.polygon.points[i],centroid);
		polyin.polygon.points[i] = offset32(polyin.polygon.points[i],rad,offset);
	}
	return polyin;
}
geometry_msgs::PolygonStamped path2poly(nav_msgs::Path pathin){
  geometry_msgs::PolygonStamped polyout;
  polyout.header = hdr();
  polyout.polygon.points.resize(pathin.poses.size());
  for(int i = 0; i < pathin.poses.size(); i++){
    polyout.polygon.points[i].x = round(path_visited.poses[i].pose.position.x);
    polyout.polygon.points[i].y = round(path_visited.poses[i].pose.position.y);
    polyout.polygon.points[i].z = round(path_visited.poses[i].pose.position.z);
  }
  return polyout;
}

void create_floor(){
  at_floor = false;
  poly_building.polygon.points.resize(0);
  int vn = path_visited.poses.size();
  int v0 = building_floor_v0;
  int zn = get_zn(path_visited.poses[v0].pose.position.z);
	building_floors_completed.push_back(zn);

  for(int i = building_floor_n0_targets; i < targetcmds_sent.size(); i++){
		geometry_msgs::Point32 p;
		p.x = targetcmds_sent[i].pose.position.x;
		p.y = targetcmds_sent[i].pose.position.y;
		p.z = targetcmds_sent[i].pose.position.z;
		ROS_INFO("BLD: P[%i] %.2f %.2f %.2f",p.x,p.y,p.z);
    if(p.z > 0 && p.x + p.y != 0){
		  poly_building.polygon.points.push_back(p);
  		buildings_floorpaths[building_n_active][zn].poses.push_back(targetcmds_sent[i]);
      std::vector<int> indexes_in_rad = getinpath_indexes_inrad(path_candidates,targetcmds_sent[i].pose.position,5.5);
      for(int k = 0; k < indexes_in_rad.size(); k++){
        geometry_msgs::PoseStamped ps;
        ps = path_candidates.poses[indexes_in_rad[k]];
        float dz = ps.pose.position.z - building_centroid.point.z;
        if(dz < 0 && dz > -par_zjump*1.5)
          path_floor_below.poses.push_back(ps);  //      buildings_floorpaths[building_n_active][zn-1].poses.push_back(ps);
        else if(dz > 0 && dz < par_zjump*1.5)
          path_floor_above.poses.push_back(ps);//      buildings_floorpaths[building_n_active][zn+1].poses.push_back(ps);
      }
    }
  }
	poly_building = project_poly(poly_building,10);
  int numtarget = targetcmds_sent.size() -  building_floor_n0_targets;
  if(path_floor_below.poses.size() > numtarget - 20){
		ROS_INFO("BLD: Path path_floor_below: %i vs targets_z0: %i ",path_floor_below.poses.size(),numtarget);
		building_floors_known.push_back(zn-1);
	}
	else{
		buildings_bases[building_n_active] = zn+1;
		ROS_INFO("BLD: BASE of Building[#%i]: zn: %i z: %.2f",building_n_active,zn+1,z_lvls[zn+1]);
	}
	if(path_floor_above.poses.size() > numtarget - 20){
		ROS_INFO("BLD: Path path_floor_above: %i vs targets_z0: %i ",path_floor_above.poses.size(),numtarget);
		building_floors_known.push_back(zn+1);
	}
	else{
		buildings_roofs[building_n_active] = zn+1;
		ROS_INFO("BLD: ROOF of Building[#%i]: zn: %i z: %.2f",building_n_active,zn+1,z_lvls[zn+1]);
	}
  buildings_centroids[building_n_active] = building_centroid.point;

  ROS_INFO("BLD: New Floor Etage path orig:[%i] below[%i],above[%i]",
  buildings_floorpaths[building_n_active][zn].poses.size(),path_floor_below.poses.size(),
  path_floor_above.poses.size());
  evaluate_floorshift(true,zn);
}

geometry_msgs::PointStamped get_centroid(int v0,int vn){
	geometry_msgs::PointStamped p;
	p.header.frame_id = "map";
	p.header.stamp    = ros::Time::now();
	int centroid_sum_x = 0;
	int centroid_sum_y = 0;
	int centroid_sum_z = 0;
	int centroid_sum_count = vn - v0;
	for(int i = v0; i < vn; i++){
		centroid_sum_x += int(round(path_visited.poses[i].pose.position.x));
		centroid_sum_y += int(round(path_visited.poses[i].pose.position.y));
		centroid_sum_z += int(round(path_visited.poses[i].pose.position.z));
	}
	p.point.x = centroid_sum_x / centroid_sum_count;
	p.point.y = centroid_sum_y / centroid_sum_count;
	p.point.z = centroid_sum_z / centroid_sum_count;
	ROS_INFO("BLD: Centroid Calculated: %i points -> %.2f %.2f %.2f",centroid_sum_count,p.point.x,p.point.y,p.point.z);
	return p;
}

float get_completion_percent(int v0,int vn,geometry_msgs::Point cen){
	int radians_segmentsize = 72;
	std::vector<bool> visited;
	float a_pr_i = rad2deg * (2*M_PI / radians_segmentsize);
	visited.resize(radians_segmentsize);
	int count = 0;
	for(int i = v0; i < vn; i++){
		int deg = int(round((rad2deg * (M_PI + get_hdng(path_visited.poses[i].pose.position,cen)) ) ));
		visited[deg/a_pr_i] = true;
	}
	for(int i = 0; i < radians_segmentsize; i++){
		if(visited[i]){
			count++;
    }
	}
	int percentage_completion = count / radians_segmentsize * 100;
	ROS_INFO("BLD: Completion: %i radians visited of %i segmentsize, percentage: %i",count,radians_segmentsize,percentage_completion);
	return percentage_completion;
}
void update_completion(int completion){
  nav_msgs::Path path_floor;
  path_floor.header = hdr();
  float n = path_visited.poses.size() - building_floor_v0;
  float c = completion / 100;
  float pose_pr_prosent = n / c;
  path_floor.poses.resize(pose_pr_prosent);
  for(int i = building_floor_v0; i < path_visited.poses.size(); i++){
    path_floor.poses[i - building_floor_v0] = path_visited.poses[i];
  }
  ROS_INFO("BLD: ZLVL: Completion: %i with %i poses  -> pose_pr_prosent: %.2f b_v0 %i b_vn %i z %.2f",completion,n,pose_pr_prosent,building_n_active,building_active,at_floor,building_floor_v0, path_visited.poses.size(),path_visited.poses[building_floor_v0].pose.position.z);
  pub_path_floor.publish(path_floor);
}
void update_centroid(){
	building_centroid = get_centroid(building_floor_v0,path_visited.poses.size());
	int completion = get_completion_percent(building_floor_v0,path_visited.poses.size(),building_centroid.point);
  int dc = completion - last_completion;
  if(dc > 10){
    update_completion(completion);
  }
	if(completion >= 97){
		ROS_INFO("BLD: Floor complete");
		if(buildings_centroids[building_n_active].z == 0){
			create_floor();
		}
	}
  else{
    int completion_init = get_completion_percent(building_floor_v0,path_visited.poses.size(),building_centroid.point);
    int completion_new = get_completion_percent(building_floor_v0,path_visited.poses.size(),building_centroid.point);
    ROS_INFO("BLD: Initial centroid completion_init[%i]: %.0f %.0f vs new centroid completion_old[%i]: %.0f %.0f",completion_init,building_centroid_initial.point.x,building_centroid_initial.point.y,completion_new,building_centroid.point.x,building_centroid.point.y);
    pub_centroid.publish(building_centroid);
  }
}

int building_defloration(int bn){
  int total_floorpath = 0;
	for(int i = 0; i < buildings_floorpaths[bn].size(); i++){
    total_floorpath += buildings_floorpaths[bn][i].poses.size();
    if(buildings_floorpaths[bn][i].poses.size() > 1)
      ROS_INFO("BLD: building_defloration Buildings_floorpaths[#:%i] size: %i",i,buildings_floorpaths[bn][i].poses.size());
  }
  ROS_INFO("BLD: check_building[#:%i] total size: %i",bn,total_floorpath);
  return total_floorpath;
}

void missionstate_cb(const std_msgs::UInt8::ConstPtr& msg){
  if(msg->data == 1){
    building_n_active = msg->data;
    float dst2d = get_dst2d(building_centroid_initial.point,last_pose.pose.position);
    if(bldstate_msg.data == 0){
      if(building_defloration(msg->data) == 0){
        ROS_INFO("BLD:  Building %i is never approached before. Setting building state to 1, distance2d is %.2f meters",msg->data,dst2d);
        bldstate_msg.data = 1;
        pub_bldstate.publish(bldstate_msg);
      }
    }
    else if(bldstate_msg.data == 1){
      if(dst2d < 15){
        ROS_INFO("BLD: Building %i is now active",msg->data,dst2d);
        activate_building(building_n_active);
      }
    }
    else if(bldstate_msg.data == 2){
      update_centroid();
    }
    else if(bldstate_msg.data == 3){
      ROS_INFO("BLD:  Building %i has %i points in poly bounding box. Setting building state to 3 and Requesting vstd and boundingbox based filtration from path utilizer",msg->data,poly_building.polygon.points.size());
      poly_building.header.frame_id = "map";
      pub_poly.publish(poly_building);
    }
    else
      ROS_INFO("BLD:  Building ERROR bldstate_msg: %i",bldstate_msg.data);
    pub_bldstate.publish(bldstate_msg);
    }
  }
/*  else if(buildings_roofs[msg->data] != 0 && buildings_bases[msg->data] != 0){
    bldstate_msg.data = 4;
    ROS_INFO("BLD:  Building %i has found roof at %i meters and base at %i meters. Setting state to 4 and requesting roof filtration from path utilizer",
    msg->data,buildings_roofs[msg->data],buildings_bases[msg->data]);
  }
  else if(buildings_roofs_explored[msg->data]){
    bldstate_msg.data = 0;
    building_active = false;
    ROS_INFO("BLD:  Building %i is completed. Setting state to 0 and deactivating building");
  }  if(everyother){
      everyother = false;
    }
    else{
      everyother = true;
    }
    */


void vstdpath_cb(const nav_msgs::Path::ConstPtr& msg){
  path_visited = *msg;
  if(path_visited.poses.size() > 3)
    last_pose = path_visited.poses[path_visited.poses.size() - 1];
}
int bld_number_from_centroid(geometry_msgs::Point pin){
  return 0;
}
void centroidinitial_cb(const geometry_msgs::PointStamped::ConstPtr& msg){
  building_n_active = bld_number_from_centroid(msg->point);
  if(buildings_centroids[building_n_active].z > 0){
    bldstate_msg.data == 3;
    building_centroid.point = buildings_centroids[building_n_active];
    ROS_INFO("BLD: Received known centroid: %.0f %.0f %.0f - setting building state to %i",msg->point.x,msg->point.y,msg->point.z,building_n_active);
  }
  else{
    ROS_INFO("BLD: Received intial centroid: %.0f %.0f %.0f - setting building state to %i",msg->point.x,msg->point.y,msg->point.z,building_n_active);
    building_centroid_initial = *msg;
    building_centroid = *msg;
    bldstate_msg.data = 0;
  }
}

void create_building_templates(){
  building_n_active = -1;
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
  ROS_INFO("BLD: Buildings created");
}
int main(int argc, char** argv)
{
  ros::init(argc, argv, "tb_fsmbuilding_node");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");
  private_nh.param("par_zjump",   par_zjump, 3.0);//*2.0);

  create_building_templates();
  ros::Subscriber s2 = nh.subscribe("/tb_nav/path_visited",100,&vstdpath_cb);
  ros::Subscriber s7 = nh.subscribe("/tb_fsm/mission_state",100,&missionstate_cb);
  ros::Subscriber s10 = nh.subscribe("/tb_bld/centroid_initial",100,&centroidinitial_cb);

  pub_poly        = nh.advertise<geometry_msgs::PolygonStamped>("/tb_bld/polygon",100);
  pub_centroid    = nh.advertise<geometry_msgs::PointStamped>("/tb_bld/building_centroid",100);
  pub_bldstate    = nh.advertise<std_msgs::UInt8>("/tb_bld/building_state",100);
  pub_path_floor  = nh.advertise<nav_msgs::Path>("/tb_bld/path_floor",100);
  pub_floorchange = nh.advertise<std_msgs::UInt8>("/tb_cmd/set_altlvl",100);

  ros::spin();
  return 0;
}
