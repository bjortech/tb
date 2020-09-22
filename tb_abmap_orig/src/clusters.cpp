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


cv::Mat img(1000,1000,CV_8UC3,cv::Scalar(0, 0, 0)); //create image, set encoding and size, init pixels to default val

nav_msgs::Path path_vlp,path_visited,path_down,path_side,path_sides_raw,path_down_raw;
int count_down = 0;
int count_side = 0;
int count = 0;
ros::Publisher pub_path_down,pub_path_side;

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
    ROS_INFO("PathAdmin: pathin is empty");
    return vec_out;
  }
  float yaw0 = tf::getYaw(pathin.poses[i_to_check].pose.orientation);
  for(int i = 0; i < pathin.poses.size(); i++){
    float dist = get_dst3d(pathin.poses[i].pose.position,pathin.poses[i_to_check].pose.position);
    float dyaw = get_shortest(tf::getYaw(pathin.poses[i].pose.orientation),yaw0);
    float dz   = 0;//pathin.poses[i].pose.position.z-pathin.poses[i_to_check].pose.position.z;
    if(dyaw < 0)
      dyaw *= -1;

    if(dist <= radius && dist > 0 && dyaw < M_PI/8 && dz == 0)
      vec_out.push_back(i);
  }
  return vec_out;
}

std::vector<std::vector<int>> getinpath_neighbours(nav_msgs::Path pathin,float radius){
	std::vector<std::vector<int>> neighbours_at_index;
  if(pathin.poses.size() == 0){
    ROS_INFO("PathAdmin: pathin is empty");
    return neighbours_at_index;
  }
  for(int i = 0; i < pathin.poses.size(); i++){
		neighbours_at_index.push_back(getinpath_indexes_inrad(pathin,i,radius));
  //  ROS_INFO("neighbours_at_index[%i]: %i",i,neighbours_at_index[i].size());
  }
  return neighbours_at_index;
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
  //  ROS_INFO("new neighbours: count %i 0: %i N: %i",new_neighbours.size(),new_neighbours[0],new_neighbours[new_neighbours.size()-1]);
  return new_neighbours;
}

std::vector<int> get_neighbour_cluster(nav_msgs::Path pathin,float radius,int start_index){
  std::vector<std::vector<int>> neighbours_at_index;
  neighbours_at_index = getinpath_neighbours(pathin,radius);
  std::vector<int> neighbours_in_cluster;
  std::vector<int> indexes_to_add;
  indexes_to_add.push_back(start_index);
  while(indexes_to_add.size() > 0){
    for(int i = 0; i < indexes_to_add.size(); i++){
      neighbours_in_cluster.push_back(indexes_to_add[i]);
    }
    indexes_to_add = add_neighbours_index(neighbours_at_index,neighbours_in_cluster,indexes_to_add);
  }
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

std::vector<std::vector<int>> get_neighbour_clusters(nav_msgs::Path pathin,float radius){
  std::vector<int> neighbours_not_clustered;
  std::vector<int> neighbours_in_cluster;
  std::vector<int> neighbours_clustered;
  std::vector<std::vector<int>> neighbour_clusters;
  while(neighbours_clustered.size() < pathin.poses.size()){
    neighbours_not_clustered = get_neighbours_not_clustered(neighbours_clustered,pathin.poses.size());
    neighbours_in_cluster    = get_neighbour_cluster(pathin,radius,neighbours_not_clustered[0]);
    neighbours_clustered     = update_neighbours_clustered(neighbours_clustered,neighbours_in_cluster);
    neighbour_clusters.push_back(neighbours_in_cluster);
  //  ROS_INFO("Neighbours cluster: %i / %i, neighbours_in_cluster: %i, neighbour_clusters: %i",neighbours_clustered.size(),neighbours_not_clustered.size(),neighbours_in_cluster.size(),neighbour_clusters.size());
  }
  return neighbour_clusters;
}
////////////////////////////***********************//////////////////////////////
                          //END  /*CLUSTERS*/ //END
////////////////////////////***********************//////////////////////////////


void pathcandsides_cb(const nav_msgs::Path::ConstPtr& msg){
  path_sides_raw = *msg;
	if(msg->poses.size() == 0)
		pub_path_side.publish(*msg);
}
void pathcanddown_cb(const nav_msgs::Path::ConstPtr& msg){
  path_down_raw = *msg;
	if(msg->poses.size() == 0)
		pub_path_down.publish(*msg);
}
void pathvlp_cb(const nav_msgs::Path::ConstPtr& msg){
  if(msg->poses.size() > 0)
    base_pose = msg->poses[0];
	path_vlp = *msg;
}
void pathvstd_cb(const nav_msgs::Path::ConstPtr& msg){
	path_visited = *msg;
}

void draw_clusters(nav_msgs::Path pathin,std::vector<std::vector<int>> clusters,nav_msgs::Path pathin_vlp, bool down){
  cv::Mat img2(1000,1000,CV_8UC3,cv::Scalar(0, 0, 0));
  cv::Scalar color_path,color_pos;
  std::vector<float> sizes;
  for(int i = 0; i < clusters.size(); i++){
    sizes.push_back(float(clusters[i].size()));
  }
  geometry_msgs::Point maxmin;
  maxmin = max_score(sizes);
  color_pos[0] = 100;
  color_pos[1] = 100;
  bool color_red,color_blue;
  bool color_green = true;
  cv::circle(img2,pnt2cv(base_pose.pose.position),2,color_pos,1);
  for(int i = 0; i < clusters.size(); i++){
    float score_rel = sizes[i] - maxmin.y;
    float score_pnt = (255 * score_rel / (maxmin.x - maxmin.y));
    int color_score = int(score_pnt);
    if(clusters[i].size() > 0){
      if(color_red){
        color_red = false;
        color_blue = true;
        color_path[0] = color_score;
        color_path[1] = color_score;
      }
      else if(color_blue){
        color_blue  = false;
        color_green = true;
        color_path[1] = color_score;
        color_path[2] = color_score;
      }
      else if(color_green){
        color_red = true;
        color_green = false;
        color_path[2]  = color_score;
        color_path[0] = color_score;
      }
      for(int k = 0; k < clusters[i].size(); k++){
        cv::circle(img2,pnt2cv(pathin.poses[clusters[i][k]].pose.position),1,color_path,1);
      }
    }
  }
  for(int i =0 ; i < pathin_vlp.poses.size(); i++){
    color_path[2] = 200;
    color_path[0] = 0;
    color_path[1] = 0;
    geometry_msgs::Point pnt;
    pnt = pathin_vlp.poses[i].pose.position;
    geometry_msgs::Point p1,p0;
    float yaw = tf::getYaw(pathin_vlp.poses[i].pose.orientation);
    p1.x = pnt.x + 2 * cos(yaw);
    p1.y = pnt.y + 2 * sin(yaw);
    cv::line (img2, pnt2cv(pnt), pnt2cv(p1), color_path,1,cv::LINE_8,0);
    cv::circle(img2,pnt2cv(pnt),1,color_path,1);
  }
  if(down)
    cv::imwrite("/home/nuc/brain/cluster/"+std::to_string(count)+"clusters_down.png",img2);
  else
    cv::imwrite("/home/nuc/brain/cluster/"+std::to_string(count)+"clusters_sides.png",img2);
}

int get_best_cluster(std::vector<nav_msgs::Path> paths_clusters){
  cv::Mat img2(1000,1000,CV_8UC3,cv::Scalar(0, 0, 0));
  int best_cluster = 0;
  float best_cluster_start = 1000;
  std::vector<float> sizes;
  for(int i = 0; i < paths_clusters.size(); i++){
    sizes.push_back(float(paths_clusters[i].poses.size()));
  }
  geometry_msgs::Point maxmin;
  maxmin = max_score(sizes);

  for(int i = 0; i < paths_clusters.size(); i++){
      int full = paths_clusters[i].poses.size()-1;
      int half = paths_clusters[i].poses.size()/2 - 2;
      if(half > 0){
        float score_rel = sizes[i] - maxmin.y;
        float score_pnt = (255 * score_rel / (maxmin.x - maxmin.y));
        int color_score = int(score_pnt);

        geometry_msgs::Point pnt_start,pnt_half,pnt_end;
        pnt_start = paths_clusters[i].poses[0].pose.position;
        pnt_half = paths_clusters[i].poses[half].pose.position;
        pnt_end = paths_clusters[i].poses[full].pose.position;
        float dst_start = get_dst2d(base_pose.pose.position,pnt_start);
        float dst_1  = get_dst2d(pnt_half,pnt_end);
        float dst_0  = get_dst2d(pnt_start,pnt_half);
        if(dst_start < best_cluster_start){
          best_cluster_start = dst_start;
          best_cluster = i;
        }
        ROS_INFO("BEST CLUSTER: %i,cluster[%i]: dst start: %.0f dst_half->end: %.0f dst_start->half: %.0f",best_cluster,i,dst_start,dst_1,dst_0);
        cv::Scalar color_path;
        if(color_score < 100)
          color_path[0] = color_score;
        else if(color_score < 175)
          color_path[1] = color_score;
        else
          color_path[2] = color_score;
      cv::circle(img2,pnt2cv(pnt_start),1,color_path,1);
      cv::circle(img2,pnt2cv(pnt_half),1,color_path,1);
      cv::circle(img2,pnt2cv(pnt_end),1,color_path,1);
      cv::line (img2, pnt2cv(pnt_start), pnt2cv(pnt_half), color_path,1,cv::LINE_8,0);
      cv::line (img2, pnt2cv(pnt_half), pnt2cv(pnt_end), color_path,1,cv::LINE_8,0);
    }
  }
  cv::imwrite("/home/nuc/brain/cluster/"+std::to_string(count)+"_best.png",img2);
  return best_cluster;
}

std::vector<nav_msgs::Path> paths_from_clusters(nav_msgs::Path pathin,std::vector<std::vector<int>> clusters_in){
//  ROS_INFO("Clusters_in: %i, pathposes_in: %i",clusters_in.size(),pathin.poses.size());
  std::vector<nav_msgs::Path> path_clusters;
  for(int i = 0; i < clusters_in.size(); i++){
    if(clusters_in[i].size() > 1){
      nav_msgs::Path path_cluster;
      path_cluster.header = pathin.header;
      for(int k = 0; k < clusters_in[i].size(); k++){
        path_cluster.poses.push_back(pathin.poses[clusters_in[i][k]]);
      }
  //    ROS_INFO("Cluster: %i path_size: %i",i,path_cluster.poses.size());
      path_clusters.push_back(path_cluster);
    }
  }
  return path_clusters;
}


nav_msgs::Path transform_path_to_vlp_path(nav_msgs::Path pathin_cluster,nav_msgs::Path pathin_vlp){
  nav_msgs::Path pathout;
  pathout.header = pathin_vlp.header;
  std::vector<std::tuple<int,float>>i_dst;
  for(int i = 0; i < pathin_cluster.poses.size(); i++){
    i_dst.push_back(std::make_tuple(i,float(get_closest_pose(pathin_vlp,pathin_cluster.poses[i].pose.position))));
  }
  sort(i_dst.begin(),i_dst.end(),sort_dst_pair);
  std::vector<int> pnts_at_index;
  float sum_yaw = 0;
  float sum_x = 0;
  float sum_y = 0;
  float sum_z = 0;
  float max_z = 0;
  int closest_pose = int(round(std::get<1>(i_dst[0])));
  int counter = 0;
  for(int i = 0; i < i_dst.size(); i++){
    int index            = std::get<0>(i_dst[i]);
    int new_closest_pose = int(round(std::get<1>(i_dst[i])));
    if(closest_pose != new_closest_pose){
      geometry_msgs::PoseStamped ps_out;
      closest_pose = new_closest_pose;
      float z_used = (pathin_vlp.poses[new_closest_pose].pose.position.z + pathin_vlp.poses[closest_pose].pose.position.z)/2;
      ps_out.pose.position.x = sum_x / float(counter);
      ps_out.pose.position.y = sum_y / float(counter);
      float z_ave = sum_z / float(counter);
      float yaw_ave = sum_yaw / float(counter);
      if(pathin_cluster.poses[0].pose.orientation.y == 0.7071){
        ps_out.pose.orientation = pathin_cluster.poses[0].pose.orientation;
        ps_out.pose.position.z  = z_ave;
      }
      else{
        ps_out.pose.position.z = z_used;
        ps_out.pose.orientation = tf::createQuaternionMsgFromYaw(yaw_ave);
      }
      float dst_closest_i = 0;
      if(pathout.poses.size() > 0){
        int closest_i       = get_closest_pose(pathout,ps_out.pose.position);
        dst_closest_i = get_dst2d(pathout.poses[closest_i].pose.position,ps_out.pose.position);
      }
      ROS_INFO("dst_closest_i: %.0f, closest_pose: %i index: %i, counter: %i, xyz: %.0f %.0f %.0f (used: %.0f) yaw: %.2f",dst_closest_i,closest_pose,index,counter,ps_out.pose.position.x,ps_out.pose.position.y,z_ave,ps_out.pose.position.z,yaw_ave);
      sum_yaw = 0;
      sum_x = 0;
      sum_y = 0;
      sum_z = 0;
      max_z = 0;
      counter = 0;

      ps_out.header = pathin_vlp.header;
      pnts_at_index.push_back(counter);
      if(pathout.poses.size() == 0 || dst_closest_i > 2)
        pathout.poses.push_back(ps_out);
    }
    if(max_z < pathin_cluster.poses[index].pose.position.z)
      max_z = pathin_cluster.poses[index].pose.position.z;

    sum_x += pathin_cluster.poses[index].pose.position.x;
    sum_y += pathin_cluster.poses[index].pose.position.y;
    sum_z += pathin_cluster.poses[index].pose.position.z;
    ROS_INFO("orig: xyz: %.0f %.0f %.0f yaw: %.2f",pathin_cluster.poses[index].pose.position.x,pathin_cluster.poses[index].pose.position.y,pathin_cluster.poses[index].pose.position.z,tf::getYaw(pathin_cluster.poses[index].pose.orientation));
    sum_yaw += tf::getYaw(pathin_cluster.poses[index].pose.orientation);
    counter++;
  }
  return pathout;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "tb_abmap_clusters_node");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");

  ros::Subscriber s2   = nh.subscribe("/tb_path/visited",10,pathvstd_cb);
	ros::Subscriber s12  = nh.subscribe("/tb_edto/path_side_full",10,pathcandsides_cb);
  ros::Subscriber s1   = nh.subscribe("/tb_edto/path_down_full",10,pathcanddown_cb);
	ros::Subscriber s11  = nh.subscribe("/tb_path/cleared",10,pathvlp_cb);
	pub_path_side = nh.advertise<nav_msgs::Path>("/tb_clusters/sides",10);
  pub_path_down = nh.advertise<nav_msgs::Path>("/tb_clusters/down",10);

  ros::Rate rate(1.0);
  while(ros::ok()){
    count++;

    if(path_sides_raw.poses.size() > 5){
      std::vector<std::vector<int>> clusters_sides     = get_neighbour_clusters(path_sides_raw,4);
      std::vector<nav_msgs::Path> paths_clusters_sides = paths_from_clusters(path_sides_raw,clusters_sides);
      paths_clusters_sides = sort_paths_in_vector(paths_clusters_sides,"dst_2d");
      nav_msgs::Path largest_cluster_side;
      for(int i = 0; i < paths_clusters_sides.size(); i++){
        if(largest_cluster_side.poses.size() < paths_clusters_sides[i].poses.size())
          largest_cluster_side = paths_clusters_sides[i];
      }
      int best_side = get_best_cluster(paths_clusters_sides);
      ROS_INFO("Largest cluster side: %i",largest_cluster_side.poses.size());
      if(largest_cluster_side.poses.size() > 0){
        largest_cluster_side = transform_path_to_vlp_path(largest_cluster_side,path_vlp);
        draw_clusters(path_sides_raw,clusters_sides,largest_cluster_side,false);
        pub_path_side.publish(largest_cluster_side);
      }
      path_sides_raw.poses.resize(0);
    }
    if(path_down_raw.poses.size() > 5){
      std::vector<std::vector<int>> clusters_down  = get_neighbour_clusters(path_down_raw,2);
      std::vector<nav_msgs::Path> paths_clusters_down = paths_from_clusters(path_down_raw,clusters_down);
      paths_clusters_down = sort_paths_in_vector(paths_clusters_down,"dst_2d");
      int best_down = get_best_cluster(paths_clusters_down);

      nav_msgs::Path largest_cluster_down;
      for(int i = 0; i < paths_clusters_down.size(); i++){
        if(largest_cluster_down.poses.size() < paths_clusters_down[i].poses.size())
          largest_cluster_down = paths_clusters_down[i];
      }
      ROS_INFO("Largest cluster down: %i",largest_cluster_down.poses.size());
      if(largest_cluster_down.poses.size() > 0){
        largest_cluster_down = transform_path_to_vlp_path(largest_cluster_down,path_vlp);
        draw_clusters(path_down_raw,clusters_down,largest_cluster_down,true);
        pub_path_down.publish(largest_cluster_down);
      }
      path_down_raw.poses.resize(0);
    }
	  rate.sleep();
	  ros::spinOnce();
  }
  return 0;
}
