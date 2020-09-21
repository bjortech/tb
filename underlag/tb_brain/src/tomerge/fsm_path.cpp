#include <ros/ros.h>
#include <octomap_msgs/conversions.h>
#include <octomap_msgs/Octomap.h>
#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <octomap/OcTreeBase.h>
#include <octomap/octomap_types.h>
#include <tf/transform_datatypes.h>
#include <dynamicEDT3D/dynamicEDTOctomap.h>
#include <chrono>
#include <std_msgs/UInt8.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_listener.h>
#include "tf2_ros/message_filter.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include <nav_msgs/Path.h>
#include <nav_msgs/OccupancyGrid.h>
#include <map_msgs/OccupancyGridUpdate.h>
#include <std_msgs/UInt8.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Point32.h>
#include <eigen3/Eigen/Core>
#include <geometry_msgs/Vector3Stamped.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <image_transport/image_transport.h>
#include <bits/stdc++.h>
using namespace std;

// Declaring the vectors to store color, distance
// and parent
tf2_ros::Buffer tfBuffer;
int zlvl;
bool candidates_generic;
double par_zjump,pos_yaw;
geometry_msgs::Point pos;
geometry_msgs::PoseStamped last_pose;
ros::Publisher custpath_pub,path_cndidat_pub,path_visited_pub,path_unified_pub,path_not_visited_pub;
nav_msgs::Path building_path,path_candidates,visited_path,pathcand,path_not_visited;

std::vector<int> z_lvls;
std::vector<nav_msgs::Path> paths_cand_at_lvl;
std::vector<nav_msgs::Path> paths_vstd_at_lvl;
std::vector<nav_msgs::Path> paths_candseg_at_lvl;

bool sort_dst_pair(const std::tuple<int,float>& a,
               const std::tuple<int,float>& b)
{
    return (std::get<1>(a) < std::get<1>(b));
}
int get_zn(float z){
  for(int i = 0; i < z_lvls.size()-1; i++){
    if(z_lvls[i] < z && z_lvls[i+1] > z)
    return i;
  }
  return 0;
}
double get_shortest(double target_hdng,double actual_hdng){
  double a = target_hdng - actual_hdng;
  if(a > M_PI)a -= M_PI*2;
  else if(a < -M_PI)a += M_PI*2;
  return a;
}
float get_dst2d(geometry_msgs::Point p1, geometry_msgs::Point p2){
  return(sqrt(pow(p1.x-p2.x,2)+pow(p1.y-p2.y,2)));
}
float get_dst3d(geometry_msgs::Point p1, geometry_msgs::Point p2){
  return(sqrt(pow(p1.x-p2.x,2)+pow(p1.y-p2.y,2)+pow(p1.z-p2.z,2)));
}

std::vector<int> getinpath_indexes_inrad(nav_msgs::Path pathin,int i_to_check,float radians){
  std::vector<int> vec_out;
  if(pathin.poses.size() == 0){
    ROS_INFO("PathAdmin: pathin is empty");
    return vec_out;
  }
  for(int i = 0; i < pathin.poses.size(); i++){
    float dist = get_dst2d(pathin.poses[i].pose.position,pathin.poses[i_to_check].pose.position);
    if(dist <= radians && dist > 0)
      vec_out.push_back(i);
  }
  return vec_out;
}
std::vector<std::vector<int>> getinpath_neighbours(nav_msgs::Path pathin,float radians){
  std::vector<int> neighbours;
	std::vector<std::vector<int>> neighbours_at_index;
	//getinpath_neighbours(pathin,radians);
  if(pathin.poses.size() == 0){
    ROS_INFO("PathAdmin: pathin is empty");
    return neighbours_at_index;
  }

  for(int i = 0; i < pathin.poses.size(); i++){
		neighbours = getinpath_indexes_inrad(pathin,i,radians);
		neighbours.push_back(i);
		sort(neighbours.begin(),neighbours.end());
    neighbours_at_index.push_back(neighbours);
  }
  return neighbours_at_index;
}
std::vector<int> get_endpoints(std::vector<std::vector<int>> vec_of_vecs){
	std::vector<int> endpoints;
	for(int i = 0; i < vec_of_vecs.size(); i++){
		if(vec_of_vecs[i].size() == 2){
			ROS_INFO("Endpoint found: %i",i);
			endpoints.push_back(i);
		}
	}
	return endpoints;
}
bool in_vec(std::vector<int> vec,int k){
	if(vec.size() == 0)
		return false;
	//	ROS_INFO("in vec: %i",k);
	for(int i = 0; i < vec.size(); i++){
		if(vec[i] == k)
			return true;
	}
	return false;
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
std::vector<int> go_from_endpoint(std::vector<std::vector<int>> vec_of_vecs,int endpoint){
	int next_point = endpoint;
	std::vector<int> vec_out;
	vec_out.push_back(endpoint);

	if(vec_of_vecs[endpoint][0] == endpoint)
		next_point = vec_of_vecs[endpoint][1];
	else
		next_point = vec_of_vecs[endpoint][0];

	while(vec_of_vecs[next_point].size() == 3){
		vec_out.push_back(next_point);
		if(in_vec(vec_out,vec_of_vecs[next_point][0]) && in_vec(vec_out,vec_of_vecs[next_point][1]))
			next_point = vec_of_vecs[next_point][2];
		else if(in_vec(vec_out,vec_of_vecs[next_point][1]) && in_vec(vec_out,vec_of_vecs[next_point][2]))
			next_point = vec_of_vecs[next_point][0];
		else
			next_point = vec_of_vecs[next_point][1];
	//	ROS_INFO("Vec out added next point: %i",next_point);
	}
	if(in_vec(vec_out,vec_of_vecs[next_point][0]))
		vec_out.push_back(vec_of_vecs[next_point][1]);
	else
		vec_out.push_back(vec_of_vecs[next_point][0]);
	return vec_out;
}

std::vector<int> go_from_endpoint2(std::vector<std::vector<int>> vec_of_vecs,int endpoint){
	int next_point = endpoint;
	std::vector<int> vec_out;
	while(vec_of_vecs[next_point].size() == 3){
		vec_out.push_back(next_point);
		if(in_vec(vec_out,vec_of_vecs[next_point][0]) && in_vec(vec_out,vec_of_vecs[next_point][1]))
			next_point = vec_of_vecs[next_point][2];
		else if(in_vec(vec_out,vec_of_vecs[next_point][1]) && in_vec(vec_out,vec_of_vecs[next_point][2]))
			next_point = vec_of_vecs[next_point][0];
		else
			next_point = vec_of_vecs[next_point][1];
  }
	return vec_out;
}

std::vector<std::vector<int>> get_segmented_clusters_in_path2d(nav_msgs::Path pathin,float radius){
	std::vector<std::vector<int>> clusters;
	std::vector<std::vector<int>> neighbours_at_index;
	neighbours_at_index = getinpath_neighbours(pathin,radius);
	std::vector<int> endpoints;
	std::vector<int> pathnum_used;
	std::vector<int> open_loop,open_loop2;
	std::vector<int> closed_loop;
  std::vector<std::tuple<int,float>>i_dst;
	endpoints = get_endpoints(neighbours_at_index);
  for(int i = 0; i < endpoints.size(); i++){
    i_dst.push_back(std::make_tuple(endpoints[i],get_dst2d(pos,pathin.poses[endpoints[i]].pose.position) ));
  }
  sort(i_dst.begin(),i_dst.end(),sort_dst_pair);
  for(int i = 0; i < i_dst.size(); i++){
    endpoints[i] = std::get<0>(i_dst[i]);
  }
	for(int i = 0; i < endpoints.size(); i++){
		if(!in_vec(pathnum_used,endpoints[i])){
      open_loop  = go_from_endpoint(neighbours_at_index,endpoints[i]);
      open_loop2 = go_from_endpoint2(neighbours_at_index,endpoints[i]);
      ROS_INFO("i1_size: %i i2_size: %i",open_loop.size(),open_loop2.size());
      for(int ii = 0; ii < fmax(open_loop.size(),open_loop2.size()); ii++){
        if(ii < open_loop.size() && ii < open_loop2.size()){
          ROS_INFO("i1: %i i2: %i",open_loop[ii],open_loop2[ii]);
        }
      }
			clusters.push_back(open_loop);
			for(int k = 0; k < open_loop.size(); k++){
				pathnum_used.push_back(open_loop[k]);
			}
		}
	}
	return clusters;
}
nav_msgs::Path get_path_segmented2d(nav_msgs::Path pathin,float radians){
  nav_msgs::Path pathout;
  std::vector<std::vector<int>> clusters;
  pathout.header.frame_id = "map";
  if(pathin.poses.size() == 0){
    ROS_INFO("PathAdmin: pathin is empty");
    return pathout;
  }
  clusters = get_segmented_clusters_in_path2d(pathin,radians);
	ROS_INFO("clusters: %i",clusters.size());
  for(int i = 0; i < clusters.size(); i++){
		ROS_INFO("clusters[%i]: size: %i",i,clusters[i].size());
    for(int k = 0; k < clusters[i].size(); k++){
      pathout.poses.push_back(pathin.poses[clusters[i][k]]);
    }
  }
  ROS_INFO("PathProc MAIN UNIFY: %i",pathout.poses.size());
  return pathout;
}
float getinpath_closestdst2d(nav_msgs::Path pathin,geometry_msgs::PoseStamped pose_to_check){
  float lowest_dist = 1000;

  if(pathin.poses.size() == 0){
    ROS_INFO("PathAdmin: pathin is empty");
    return lowest_dist;
  }
  for(int i = 0; i < pathin.poses.size(); i++){
    float dist = get_dst2d(pathin.poses[i].pose.position,pose_to_check.pose.position);
    if(dist < lowest_dist)
      lowest_dist   = dist;
  }
  return lowest_dist;
}
nav_msgs::Path getinpath_not_visited2d(nav_msgs::Path pathin,nav_msgs::Path pathin_vstd,float cutoff_dst){
  nav_msgs::Path pathout;
	pathout.header.frame_id = "map";
  if(fmin(pathin.poses.size(),pathin_vstd.poses.size()) == 0){
    ROS_INFO("getinpath_not_visited: pathin is empty");
    return pathin;
  }
  ROS_INFO("Pathin size: cand %i vstd %i cutoff %.2f",pathin.poses.size(),pathin_vstd.poses.size(),cutoff_dst);
  for(int i = 0; i < pathin.poses.size(); i++){
    if(getinpath_closestdst2d(pathin_vstd,pathin.poses[i]) > cutoff_dst)
      pathout.poses.push_back(pathin.poses[i]);
  }
  ROS_INFO("PathSegmentation VSTD: %i of %i poses not visited",pathout.poses.size(),pathin.poses.size());
  return pathout;
}

nav_msgs::Path unify_path(){
  nav_msgs::Path pathout;
  ROS_INFO("Unifying paths");
	pathout.header.frame_id = "map";
	if(paths_candseg_at_lvl.size() == 0){
		ROS_INFO("Unified path EMPTY");
		return pathout;
	}
	ROS_INFO("paths_candseg_at_lvl: %i",paths_candseg_at_lvl.size());
  for(int i = 0; i < paths_candseg_at_lvl.size()-1; i++){
		ROS_INFO("paths_candseg_at_lvl[%i]: size: %i",i,paths_candseg_at_lvl[i].poses.size());
		if(paths_candseg_at_lvl[i].poses.size() > 0){
	    for(int k = 0; k < paths_candseg_at_lvl[i].poses.size()-1; k++){
	      pathout.poses.push_back(paths_candseg_at_lvl[i].poses[k]);
	    }
		}
  }
  ROS_INFO("Unified path size: %i",pathout.poses.size());
  return pathout;
}

void zlvl_cb(const std_msgs::UInt8::ConstPtr& msg){
  zlvl = msg->data;
}

void pathvstd_cb(const nav_msgs::Path::ConstPtr& msg){
  if(msg->poses.size() > visited_path.poses.size()){
    for(int i = visited_path.poses.size(); i < msg->poses.size(); i++){
      visited_path.poses.push_back(msg->poses[i]);
      int zlvl_in  = msg->poses[i].pose.position.z / par_zjump + 3;
      paths_vstd_at_lvl[zlvl_in].poses.push_back(msg->poses[i]);
    }
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
	pos.x = transformStamped.transform.translation.x;
	pos.y = transformStamped.transform.translation.y;
	pos.z = transformStamped.transform.translation.z;
	pos_yaw = tf::getYaw(transformStamped.transform.rotation);
  if(sqrt(pow(transformStamped.transform.translation.x - last_pose.pose.position.x,2)+ pow(transformStamped.transform.translation.y - last_pose.pose.position.y,2)+
    pow(transformStamped.transform.translation.z - last_pose.pose.position.z,2)) >= 1.00){
    last_pose.pose.position.x  = transformStamped.transform.translation.x;
    last_pose.pose.position.y  = transformStamped.transform.translation.y;
    last_pose.pose.position.z  = transformStamped.transform.translation.z;
    last_pose.pose.orientation = transformStamped.transform.rotation;
    last_pose.header           = transformStamped.header;
    zlvl  = last_pose.pose.position.z / par_zjump + 3;
    paths_vstd_at_lvl[zlvl].poses.push_back(last_pose);
  }
}

void add_path_candidates(nav_msgs::Path pathin){
  for(int i = 0; i < pathin.poses.size(); i++){
    if(dst_point_in_path_lim(path_candidates,pathin.poses[i].pose.position,5)){
      path_candidates.poses.push_back(pathin.poses[i]);
      paths_cand_at_lvl[pathin.poses[i].pose.position.z / par_zjump + 3].poses.push_back(pathin.poses[i]);
    }
  }
}

void pathcand_cb(const nav_msgs::Path::ConstPtr& msg){
  add_path_candidates(*msg);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "tb_fsmpathproc_node");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");
  private_nh.param("par_zjump",   par_zjump, 3.0);//*2.0);

  tf2_ros::TransformListener tf2_listener(tfBuffer);
  nav_msgs::Path path_template;
  path_template.header.frame_id = "map";
  for(int i = 0; i < 20; i++){
    z_lvls.push_back(int(round(-par_zjump*3 + par_zjump*i)));
    paths_cand_at_lvl.push_back(path_template);
    paths_vstd_at_lvl.push_back(path_template);
		paths_candseg_at_lvl.push_back(path_template);
  }
	path_cndidat_pub   = nh.advertise<nav_msgs::Path>("/tb_path/path_cndidat",100);
	path_visited_pub   = nh.advertise<nav_msgs::Path>("/tb_path/path_visited",100);
	path_unified_pub   = nh.advertise<nav_msgs::Path>("/tb_path/path_unified",100);
  path_not_visited_pub  = nh.advertise<nav_msgs::Path>("/tb_path/path_not_visited",100);

  ros::Subscriber s43 = nh.subscribe("/tb_fsm/altlvl",1,&zlvl_cb);
  ros::Subscriber s51 = nh.subscribe("/tb_path_filtered",1,&pathcand_cb);

  ros::Rate rate(1);
  bool done = true;
  ros::Time start = ros::Time::now();
  float radius = 6;
  float cutoff = 5;

  while(ros::ok()){
    checktf();
   if(done){
     done = false;
    if(paths_cand_at_lvl[zlvl].poses.size() > 0){
      path_cndidat_pub.publish(paths_cand_at_lvl[zlvl]);
      paths_candseg_at_lvl[zlvl] = get_path_segmented2d(paths_cand_at_lvl[zlvl],radius);
      if(paths_vstd_at_lvl[zlvl].poses.size() > 0){
        path_visited_pub.publish(paths_vstd_at_lvl[zlvl]);
      }
      path_not_visited = getinpath_not_visited2d(paths_cand_at_lvl[zlvl],paths_vstd_at_lvl[zlvl],10);
      path_not_visited_pub.publish(path_not_visited);
      path_unified_pub.publish(unify_path());
    }
    done = true;
	 }
  rate.sleep();
  ros::spinOnce();
  }
  return 0;
}
