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

geometry_msgs::Point pos;
float pos_yaw;
vector<string> colour;
vector<int> d;
vector<int> p;
ros::Publisher custpath_pub,path_cndidat_pub,path_visited_pub,path_unified_pub,path_not_visited_pub;
double par_zjump,par_maprad,par_hiabs,par_loabs,par_lookahead_distance,par_takeoffaltitude;
nav_msgs::Path building_path_visited,building_path,path_candidates,visited_path,path_active;
std::vector<nav_msgs::Path>building_paths;
ros::Publisher pub_tarpose,visited_path_pub,invoke_pub,targetalt_pub;
std::vector<geometry_msgs::Point>buildings;
std::vector<std::vector<geometry_msgs::Point>>buildings_min_max;
int current_building;
cv::Mat mapimg(1000,1000,CV_8UC3,cv::Scalar(0, 0, 0)); //create image, set encoding and size, init pixels to default val
std::string par_workdir;
int zlvl_min,zlvl_max;
std::vector<nav_msgs::Path> paths_cand_at_lvl;
std::vector<nav_msgs::Path> paths_vstd_at_lvl;
std::vector<nav_msgs::Path> paths_candseg_at_lvl;
std::vector<int> z_lvls;
int zlvl = 0;
nav_msgs::Path pathcand,pathvstd;
nav_msgs::Path path_visited,path_not_visited;
tf2_ros::Buffer tfBuffer;

bool sort_dst_pair(const std::tuple<int,float>& a,
               const std::tuple<int,float>& b)
{
    return (std::get<1>(a) < std::get<1>(b));
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
float get_hdng(geometry_msgs::Point p1,geometry_msgs::Point p0){
  float dx = p1.x - p0.x;
  float dy = p1.y - p0.y;
  return atan2(dy,dx);
}
float dst_point_in_path(nav_msgs::Path pathin,geometry_msgs::Point pin){
  float res,dst;
  res = 1000;
  if(pathin.poses.size() == 0)
    return res;
  for(int i = 0; i < pathin.poses.size(); i++){
    dst = get_dst3d(pathin.poses[i].pose.position,pin);
    if(dst < res)
      res = dst;
  }
  return res;
}
bool in_blacklist(int itarget,std::vector<int>blacklist){
  for(int i = 0; i < blacklist.size(); i++){
    if(blacklist[i] == itarget)
      return true;
  }
  return false;
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
	int count_3 = 0;//count_3,
	int count_2 = 0;//count_2,
	int count_1 = 0;//count_1,
	int count_0 = 0;//count_0;
	ROS_INFO("PathProc#3");

  for(int i = 0; i < pathin.poses.size(); i++){
		neighbours = getinpath_indexes_inrad(pathin,i,radians);
		neighbours.push_back(i);
		sort(neighbours.begin(),neighbours.end());

		if(neighbours.size() == 3){
			count_3++;
			ROS_INFO("Neighbours[%i]: %i, %i, %i",i,neighbours[0],neighbours[1],neighbours[2]);
		}
		if(neighbours.size() == 2){
			count_2++;
			ROS_INFO("Neighbours[%i]: %i, %i",i,neighbours[0],neighbours[1]);
		}
		if(neighbours.size() == 1){
			count_1++;
			ROS_INFO("Neighbours[%i]: %i",i,neighbours[0]);
		}
		if(neighbours.size() == 0){
			count_0++;
			ROS_INFO("Neighbours[%i]: -",i);
		}
		neighbours_at_index.push_back(neighbours);
  }
	ROS_INFO("Counts: 3: %i 2: %i 1: %i 0: %i",count_3,count_2,count_1,count_0);
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
std::vector<int> go_from_endpoint(std::vector<std::vector<int>> vec_of_vecs,int endpoint){
	int next_point = endpoint;
	std::vector<int> vec_out;
	vec_out.push_back(endpoint);

	if(vec_of_vecs[endpoint][0] == endpoint)
		next_point = vec_of_vecs[endpoint][1];
	else
		next_point = vec_of_vecs[endpoint][0];
	ROS_INFO("Next_point: %i",next_point);

	while(vec_of_vecs[next_point].size() == 3){
		vec_out.push_back(next_point);
		ROS_INFO("Next_point: %i",next_point);
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
}
std::vector<int> go_from_midpoint(std::vector<std::vector<int>> vec_of_vecs,int endpoint){
	int start_point = endpoint;
	int next_point  = endpoint;
	if(vec_of_vecs[endpoint][0] == endpoint)
		next_point = vec_of_vecs[endpoint][2];
	else
		next_point = vec_of_vecs[endpoint][0];
	std::vector<int> vec_out;
	vec_out.push_back(endpoint);
	while(vec_of_vecs[next_point].size() == 3){
		vec_out.push_back(next_point);
		ROS_INFO("Next_point: %i",next_point);
		if(in_vec(vec_out,vec_of_vecs[next_point][0]) && in_vec(vec_out,vec_of_vecs[next_point][1]))
			next_point = vec_of_vecs[next_point][2];
		else if(in_vec(vec_out,vec_of_vecs[next_point][1]) && in_vec(vec_out,vec_of_vecs[next_point][2]))
			next_point = vec_of_vecs[next_point][0];
	}
	return vec_out;
}

std::vector<std::vector<int>> get_segmented_clusters_in_path2d(nav_msgs::Path pathin,float radius){
	std::vector<std::vector<int>> clusters;
	std::vector<std::vector<int>> neighbours_at_index;
	neighbours_at_index = getinpath_neighbours(pathin,radius);
	std::vector<int> endpoints;
	std::vector<int> pathnum_used;
	std::vector<int> open_loop;
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
			open_loop = go_from_endpoint(neighbours_at_index,endpoints[i]);
			clusters.push_back(open_loop);
			for(int k = 0; k < open_loop.size(); k++){
				pathnum_used.push_back(open_loop[k]);
			}
		}
	}
  /*
	for(int i = 0; i < pathin.poses.size(); i++){
		if(!in_vec(pathnum_used,i)){
			closed_loop = go_from_midpoint(neighbours_at_index,i);
			clusters.push_back(closed_loop);
			for(int k = 0; k < closed_loop.size(); k++){
				pathnum_used.push_back(closed_loop[k]);
			}
		}
	}*/
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

void pathvstdupd_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
  int zlvl_in  = round(msg->pose.position.z / par_zjump) + 3;
  if(zlvl_in < zlvl_min)
    zlvl_min = zlvl_in;
  if(zlvl_in > zlvl_max)
    zlvl_max = zlvl_in;
  paths_vstd_at_lvl[zlvl_in].poses.push_back(*msg);
}

void pathcandupd_cb(const nav_msgs::Path::ConstPtr& msg){
  ROS_INFO("Pathcand callback %i",msg->poses.size());
  if(msg->poses.size() > 0){
    for(int i = 0; i < msg->poses.size(); i++){
      int zlvl_in  = round(msg->poses[i].pose.position.z / par_zjump) + 3;
      paths_cand_at_lvl[zlvl_in].poses.push_back(msg->poses[i]);
    }
  }
}

void split_pathcand(){
	for(int i = 0; i < paths_cand_at_lvl.size(); i++){
		paths_cand_at_lvl[i].poses.resize(0);
	}
	ROS_INFO("Pathscand: %i",pathcand.poses.size());
	for(int i = 0; i < pathcand.poses.size(); i++){
		int zlvl_in  = round(pathcand.poses[i].pose.position.z / par_zjump) + 3;
		paths_cand_at_lvl[zlvl_in].poses.push_back(pathcand.poses[i]);
	}
}

void pathcand_cb(const nav_msgs::Path::ConstPtr& msg){
	//pathcand = *msg;

  ROS_INFO("Pathcand callback %i",msg->poses.size());
  if(msg->poses.size() > pathcand.poses.size()){
    for(int i = pathcand.poses.size(); i < msg->poses.size(); i++){
      pathcand.poses.push_back(msg->poses[i]);
      int zlvl_in  = round(msg->poses[i].pose.position.z / par_zjump) + 3;
      paths_cand_at_lvl[zlvl_in].poses.push_back(msg->poses[i]);
    }
  }
}

void pathvstd_cb(const nav_msgs::Path::ConstPtr& msg){
  ROS_INFO("Pathcand callback %i",msg->poses.size());
  if(msg->poses.size() > pathvstd.poses.size()){
    for(int i = pathvstd.poses.size(); i < msg->poses.size(); i++){
      pathvstd.poses.push_back(msg->poses[i]);
      int zlvl_in  = round(msg->poses[i].pose.position.z / par_zjump) + 3;
      paths_vstd_at_lvl[zlvl_in].poses.push_back(msg->poses[i]);
    }
  }
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

void custorg_cb(const nav_msgs::Path::ConstPtr& msg){
  nav_msgs::Path pathout;
  if(msg->poses.size() > 2){
    std::vector<std::vector<int>> clusters;
    clusters = get_segmented_clusters_in_path2d(*msg,3);
    ROS_INFO("clusters: %i",clusters.size());
    for(int k = 0; k < clusters[0].size(); k++){
      pathout.poses.push_back(msg->poses[clusters[0][k]]);
    }
    //	split_pathcand();
    path_cndidat_pub.publish(paths_cand_at_lvl[zlvl]);
    float radius = 6;
    paths_candseg_at_lvl[zlvl] = get_path_segmented2d(paths_cand_at_lvl[zlvl],radius);
    if(paths_vstd_at_lvl[zlvl].poses.size() > 0){
      path_visited_pub.publish(paths_vstd_at_lvl[zlvl]);
      path_not_visited = getinpath_not_visited2d(paths_cand_at_lvl[zlvl],paths_vstd_at_lvl[zlvl],10);
      path_not_visited_pub.publish(path_not_visited);
    }
    pathout.header.frame_id ="map";
    custpath_pub.publish(pathout);
  }
}
int main(int argc, char** argv)
{
  ros::init(argc, argv, "tb_pathproc_node");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");
  private_nh.getParam("workdir_path", par_workdir);
  private_nh.param("par_zjump",   par_zjump, 3.0);//*2.0);
  private_nh.param("map_sidelength",par_maprad, 300.0);

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
  custpath_pub          = nh.advertise<nav_msgs::Path>("/tb_path/custom_organized",100);
//  ros::Subscriber s4343 = nh.subscribe("/tb_path/custom_organize",1,&custorg_cb);

  ros::Subscriber s41 = nh.subscribe("/tb_nav/visited_path_update",1,&pathvstdupd_cb);
  //ros::Subscriber s42 = nh.subscribe("/tb_path_updates",1,&pathcandupd_cb);
  ros::Subscriber s43 = nh.subscribe("/tb_fsm/altlvl",1,&zlvl_cb);

  ros::Subscriber s51 = nh.subscribe("/tb_path_filtered",1,&pathcand_cb);
  ros::Subscriber s52 = nh.subscribe("/tb_nav/visited_path",1,&pathvstd_cb);

  //ros::Subscriber s43 = nh.subscribe("/tb_cmd/building_centroid",1,&bld_cb);

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
		//	split_pathcand();
      path_cndidat_pub.publish(paths_cand_at_lvl[zlvl]);
      paths_candseg_at_lvl[zlvl] = get_path_segmented2d(paths_cand_at_lvl[zlvl],radius);
      if(paths_vstd_at_lvl[zlvl].poses.size() > 0){
        path_visited_pub.publish(paths_vstd_at_lvl[zlvl]);
        path_not_visited = getinpath_not_visited2d(paths_cand_at_lvl[zlvl],paths_vstd_at_lvl[zlvl],10);
        path_not_visited_pub.publish(path_not_visited);
      }
      path_unified_pub.publish(unify_path());
    }
    done = true;
	 }
  rate.sleep();
  ros::spinOnce();
  }
  return 0;
}
