//path_service:
// example showing how to receive a nav_msgs/Path request
// run with complementary path_client
// this could be useful for
#include <ros/ros.h>
#include <octomap_msgs/conversions.h>
#include <octomap_msgs/Octomap.h>
#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <octomap/OcTreeBase.h>
#include <octomap/octomap_types.h>
#include <tf/transform_datatypes.h>
#include <dynamicEDT3D/dynamicEDTOctomap.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Pose.h>
#include <iostream>
#include <string>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
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
#include <tb_msgsrv/Paths.h>
#include <tb_msgsrv/PathsStamped.h>

cv::Mat img(1000,1000,CV_8UC3,cv::Scalar(0, 0, 0)); //create image, set encoding and size, init pixels to default val

int count_down = 0;
int count_side = 0;
int count = 0;
ros::Publisher pub_paths;

geometry_msgs::PoseStamped base_pose;
float rad2deg = 180.0/M_PI;

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
double get_shortest(double target_hdng,double actual_hdng){
  double a = target_hdng - actual_hdng;
  if(a > M_PI)a -= M_PI*2;
  else if(a < -M_PI)a += M_PI*2;
  return a;
}
cv::Point pnt2cv(geometry_msgs::Point pin){
	int c = x2c(pin.x,img.cols,1);
	int r = y2r(pin.y,img.rows,1);
	return cv::Point(c,r);
}
bool sort_dst_pair(const std::tuple<int,float>& a,
               const std::tuple<int,float>& b)
{
    return (std::get<1>(a) < std::get<1>(b));
}
int get_closest_pose(nav_msgs::Path pathin,geometry_msgs::Point pnt){
	float closest = 1000;
	int best_tot_i = 0;
	for(int i = 0; i < pathin.poses.size(); i++){
		float dst_obs  = get_dst2d(pathin.poses[i].pose.position,pnt);
		if(dst_obs < closest){
				best_tot_i = i;
				closest = dst_obs;
		}
	}
	return best_tot_i;
}

std::vector<std::vector<int>> get_candi_at_vlpi(nav_msgs::Path pathin_vlp,nav_msgs::Path pathin_cands){
	std::vector<std::vector<int>> cand_i_at_vlp_i;
	cand_i_at_vlp_i.resize(pathin_vlp.poses.size());

	for(int i = 0; i < pathin_cands.poses.size(); i++){
    int i_closest = get_closest_pose(pathin_vlp,pathin_cands.poses[i].pose.position);
    float dst_obs = get_dst2d(pathin_vlp.poses[i_closest].pose.position,pathin_cands.poses[i].pose.position);
    if(dst_obs < 5)
  		cand_i_at_vlp_i[i_closest].push_back(i);
	}
	return cand_i_at_vlp_i;
}

nav_msgs::Path get_vlp_with_candidates(nav_msgs::Path pathin_vlp,std::vector<std::vector<int>> cand_i_at_vlp_i){
	nav_msgs::Path pathout;
	pathout.header = pathin_vlp.header;
	for(int i = 0; i < cand_i_at_vlp_i.size(); i++){
		if(cand_i_at_vlp_i[i].size() > 0)
			pathout.poses.push_back(pathin_vlp.poses[i]);
	}
	return pathout;
}
std::vector<std::vector<int>> get_updated_candi_at_vlpi(std::vector<std::vector<int>> cand_i_at_vlp_i){
std::vector<std::vector<int>> cand_i_at_vlp_i_new;
	for(int i = 0; i < cand_i_at_vlp_i.size(); i++){
		if(cand_i_at_vlp_i[i].size() > 0)
		  cand_i_at_vlp_i_new.push_back(cand_i_at_vlp_i[i]);
	}
	return cand_i_at_vlp_i_new;
}
int get_highest_candidate_number(std::vector<std::vector<int>> cand_i_at_vlp_i){
	int highest = 0;
	for(int i = 0; i < cand_i_at_vlp_i.size(); i++){
		if(cand_i_at_vlp_i[i].size() > highest)
			highest = cand_i_at_vlp_i[i].size();
	}
	return highest;
}

nav_msgs::Path sort_path(nav_msgs::Path pathin,std::string sort_by){
  nav_msgs::Path pathout;
  if(pathin.poses.size() <= 1)
    return pathin;
  pathout.header = pathin.header;
  std::vector<std::tuple<int,float>>i_dst;
  float base_yaw = tf::getYaw(base_pose.pose.orientation);
  for(int i = 0; i < pathin.poses.size(); i++){
		if(sort_by == "dst_2d")
      i_dst.push_back(std::make_tuple(i,get_dst3d(base_pose.pose.position,pathin.poses[i].pose.position)));
    else if(sort_by == "dst_3d")
      i_dst.push_back(std::make_tuple(i,get_dst2d(base_pose.pose.position,pathin.poses[i].pose.position)));
		else if(sort_by == "hdng_abs")
			i_dst.push_back(std::make_tuple(i,abs(get_shortest(get_hdng(pathin.poses[i].pose.position,base_pose.pose.position),base_yaw) * rad2deg)));
    else if(sort_by == "hdng")
      i_dst.push_back(std::make_tuple(i,get_shortest(get_hdng(pathin.poses[i].pose.position,base_pose.pose.position),base_yaw)));
		else if(sort_by == "zabs")
			i_dst.push_back(std::make_tuple(i,abs(pathin.poses[i].pose.position.z - base_pose.pose.position.z)));
    else if(sort_by == "yaw")
			i_dst.push_back(std::make_tuple(i,tf::getYaw(pathin.poses[i].pose.orientation)));
  	}
  sort(i_dst.begin(),i_dst.end(),sort_dst_pair);
  for(int i = 0; i < i_dst.size(); i++){
    pathout.poses.push_back(pathin.poses[std::get<0>(i_dst[i])]);
  }
	return pathout;
}

cv::Scalar get_color(int base_blue,int base_green, int base_red){
	cv::Scalar color;
	color[0] = base_blue;
	color[1] = base_green;
	color[2] = base_red;
	return color;
}
cv::Scalar modify_color(cv::Scalar color_in,float value,float value_max){
	if(value > value_max * 0.6)
		color_in[2] += round(100 * value/value_max);
	else
		color_in[0] += round(100 * value/value_max);
	return color_in;
}

std::vector<nav_msgs::Path> sort_paths_in_vector(std::vector<nav_msgs::Path> path_vector,std::string sort_by){
  for(int i = 0; i < path_vector.size(); i++){
    path_vector[i] = sort_path(path_vector[i],sort_by);
  }
  return path_vector;
}

void dot_pnt(geometry_msgs::Point pnt,int r, int g, int b){
  img.at<cv::Vec3b>( y2r(pnt.y,img.rows,1),x2c(pnt.x,img.cols,1) )[2] = r;
  img.at<cv::Vec3b>( y2r(pnt.y,img.rows,1),x2c(pnt.x,img.cols,1) )[1] = g;
  img.at<cv::Vec3b>( y2r(pnt.y,img.rows,1),x2c(pnt.x,img.cols,1) )[0] = b;
}
geometry_msgs::Point max_score(std::vector<float> scores){
  geometry_msgs::Point maxmin;
  maxmin.x = -100;
  maxmin.y = 100;
  for(int i = 0; i < scores.size(); i++){
    if(maxmin.x < scores[i]){
      maxmin.x = scores[i];
    }
    if(maxmin.y > scores[i]){
      maxmin.y = scores[i];
    }
  }
  return maxmin;
}

////////////////////////////***********************//////////////////////////////
                          //START  /*CLUSTERS*/ //START
////////////////////////////***********************//////////////////////////////
std::vector<int> getinpath_indexes_inrad(nav_msgs::Path pathin,int i_to_check,float radius){
  std::vector<int> vec_out;
  if(pathin.poses.size() == 0){
    ROS_INFO("CLUSTERPathAdmin: pathin is empty");
    return vec_out;
  }
  float yaw0 = tf::getYaw(pathin.poses[i_to_check].pose.orientation);
  for(int i = 0; i < pathin.poses.size(); i++){
    float dist = get_dst2d(pathin.poses[i].pose.position,pathin.poses[i_to_check].pose.position);
    float dyaw = get_shortest(tf::getYaw(pathin.poses[i].pose.orientation),yaw0);
    float dz   = 0;//pathin.poses[i].pose.position.z-pathin.poses[i_to_check].pose.position.z;
    if(dyaw < 0)
      dyaw *= -1;

    if(dist <= radius && dist > 0 && dyaw < M_PI/3 && dz == 0)
      vec_out.push_back(i);
  }
  return vec_out;
}

std::vector<std::vector<int>> getinpath_neighbours(nav_msgs::Path pathin,float radius){
	std::vector<std::vector<int>> neighbours_at_index;
  if(pathin.poses.size() == 0){
    ROS_INFO("CLUSTERPathAdmin: pathin is empty");
    return neighbours_at_index;
  }
  for(int i = 0; i < pathin.poses.size(); i++){
		neighbours_at_index.push_back(getinpath_indexes_inrad(pathin,i,radius));
  	ROS_INFO("CLUSTERneighbours_at_index[%i]: %i",i,neighbours_at_index[i].size());
  }
  return neighbours_at_index;
}

bool in_vec(std::vector<int> vec,int k){
	if(vec.size() == 0)
		return false;
	//	ROS_INFO("CLUSTERin vec: %i",k);
	for(int i = 0; i < vec.size(); i++){
		if(vec[i] == k)
			return true;
	}
	return false;
}

////////////////////////////***********************//////////////////////////////
                          //START  /*CLUSTERS*/ //START
////////////////////////////***********************//////////////////////////////
std::vector<int> getinpath_indexes_inrad(nav_msgs::Path pathin,int i_to_check,float radius, float dz_min,float dz_max){
  std::vector<int> vec_out;
  if(pathin.poses.size() == 0){
    //ROS_INFO("PathAdmin: pathin is empty");
    return vec_out;
  }
  float yaw0 = tf::getYaw(pathin.poses[i_to_check].pose.orientation);
  for(int i = 0; i < pathin.poses.size(); i++){
    float dist = get_dst2d(pathin.poses[i].pose.position,pathin.poses[i_to_check].pose.position);
    float dyaw = get_shortest(tf::getYaw(pathin.poses[i].pose.orientation),yaw0);
    float dz   = pathin.poses[i].pose.position.z-pathin.poses[i_to_check].pose.position.z;
    if(dyaw < 0)
      dyaw *= -1;

    if(dist <= radius && dist > 0 && dyaw < M_PI/8 && dz < dz_max && dz > dz_min)
      vec_out.push_back(i);
  }
  return vec_out;
}

std::vector<std::vector<int>> getinpath_neighbours(nav_msgs::Path pathin,float radius,float dz_min,float dz_max){
	std::vector<std::vector<int>> neighbours_at_index;
  if(pathin.poses.size() == 0){
    //ROS_INFO("PathAdmin: pathin is empty");
    return neighbours_at_index;
  }
  for(int i = 0; i < pathin.poses.size(); i++){
		neighbours_at_index.push_back(getinpath_indexes_inrad(pathin,i,radius,dz_min,dz_max));
  //  //ROS_INFO("neighbours_at_index[%i]: %i",i,neighbours_at_index[i].size());
  }
  return neighbours_at_index;
}

std::vector<int> add_neighbours_index(std::vector<std::vector<int>> neighbours_at_indexes,std::vector<int> neighbours_in_cluster,std::vector<int> indexes_to_add){
  std::vector<int> new_neighbours;
  for(int k = 0; k < indexes_to_add.size(); k++){
    if(!in_vec(neighbours_in_cluster,indexes_to_add[k])
    && !in_vec(new_neighbours,       indexes_to_add[k]))
      new_neighbours.push_back(indexes_to_add[k]);
    for(int i = 0; i < neighbours_at_indexes[indexes_to_add[k]].size(); i++){
      if(!in_vec(neighbours_in_cluster,neighbours_at_indexes[indexes_to_add[k]][i])
      && !in_vec(new_neighbours,       neighbours_at_indexes[indexes_to_add[k]][i])){
        new_neighbours.push_back(neighbours_at_indexes[indexes_to_add[k]][i]);
      }
    }
  }
//  if(new_neighbours.size() > 2)
  //  //ROS_INFO("new neighbours: count %i 0: %i N: %i",new_neighbours.size(),new_neighbours[0],new_neighbours[new_neighbours.size()-1]);
  return new_neighbours;
}

std::vector<int> get_neighbour_cluster(nav_msgs::Path pathin,float radius,int start_index,float dz_min,float dz_max){
  std::vector<std::vector<int>> neighbours_at_index;
  neighbours_at_index = getinpath_neighbours(pathin,radius,dz_min,dz_max);
  std::vector<int> neighbours_in_cluster;
  std::vector<int> indexes_to_add;
  indexes_to_add.push_back(start_index);
  while(indexes_to_add.size() > 0){
    for(int i = 0; i < indexes_to_add.size(); i++){
      neighbours_in_cluster.push_back(indexes_to_add[i]);
    }
    indexes_to_add = add_neighbours_index(neighbours_at_index,neighbours_in_cluster,indexes_to_add);
  }
	////ROS_INFO("Cluster size: %i",neighbours_in_cluster.size());
  return neighbours_in_cluster;
}

std::vector<int> update_neighbours_clustered(std::vector<int> neighbours_clustered,std::vector<int> neighbours_in_cluster){
  for(int i = 0; i < neighbours_in_cluster.size(); i++){
    neighbours_clustered.push_back(neighbours_in_cluster[i]);
  }
  return neighbours_clustered;
}
std::vector<int> get_neighbours_not_clustered(std::vector<int> neighbours_clustered,int path_size){
  std::vector<int> not_clustered;
  for(int i = 0; i < path_size; i++){
    if(!in_vec(neighbours_clustered,i))
      not_clustered.push_back(i);
  }
  return not_clustered;
}

std::vector<std::vector<int>> get_neighbour_clusters(nav_msgs::Path pathin,float radius,float dz_min,float dz_max){
  std::vector<int> neighbours_not_clustered;
  std::vector<int> neighbours_in_cluster;
  std::vector<int> neighbours_clustered;
  std::vector<std::vector<int>> neighbour_clusters;
  while(neighbours_clustered.size() < pathin.poses.size()){
    neighbours_not_clustered = get_neighbours_not_clustered(neighbours_clustered,pathin.poses.size());
    neighbours_in_cluster    = get_neighbour_cluster(pathin,radius,neighbours_not_clustered[0],dz_min,dz_max);
    neighbours_clustered     = update_neighbours_clustered(neighbours_clustered,neighbours_in_cluster);
    neighbour_clusters.push_back(neighbours_in_cluster);
  //  //ROS_INFO("Neighbours cluster: %i / %i, neighbours_in_cluster: %i, neighbour_clusters: %i",neighbours_clustered.size(),neighbours_not_clustered.size(),neighbours_in_cluster.size(),neighbour_clusters.size());
  }

  return neighbour_clusters;
}

tb_msgsrv::PathsStamped paths_from_clusters(nav_msgs::Path pathin,std::vector<std::vector<int>> clusters_in){
	ROS_INFO("Clusters_in: %i, pathposes_in: %i",clusters_in.size(),pathin.poses.size());
  tb_msgsrv::PathsStamped path_clusters;
  for(int i = 0; i < clusters_in.size(); i++){
    if(clusters_in[i].size() > 1){
      nav_msgs::Path path_cluster;
      path_cluster.header = pathin.header;
      for(int k = 0; k < clusters_in[i].size(); k++){
        path_cluster.poses.push_back(pathin.poses[clusters_in[i][k]]);
      }
			ROS_INFO("Cluster: %i path_size: %i",i,path_cluster.poses.size());
      path_clusters.paths.push_back(path_cluster);
    }
  }
  return path_clusters;
}
tb_msgsrv::PathsStamped get_path_clusters(nav_msgs::Path pathin,float r,float dz_min,float dz_max){
	return paths_from_clusters(pathin,get_neighbour_clusters(pathin,r,dz_min,dz_max));
}
////////////////////////////***********************//////////////////////////////
                          //END  /*CLUSTERS*/ //END
////////////////////////////***********************//////////////////////////////

tb_msgsrv::PathsStamped remove_small(tb_msgsrv::PathsStamped pathsin,int min_size){
	tb_msgsrv::PathsStamped pathsout;
	for(int i = 0; i < pathsin.paths.size(); i++){
		if(pathsin.paths[i].poses.size() > min_size){
			pathsout.paths.push_back(pathsin.paths[i]);
		}
	}
	return pathsout;
}

void path_to_cluster_cb(const nav_msgs::Path::ConstPtr& msg){
	pub_paths.publish(get_path_clusters(*msg,2.0,-1.1,1.1));
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "tb_clusters_node");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");

	ros::Subscriber as1 = nh.subscribe("/tb_test/path_raw",10,path_to_cluster_cb);
	pub_paths = nh.advertise<tb_msgsrv::PathsStamped>("/tb_test/clusters",10);
	ros::spin();
  return 0;
}
