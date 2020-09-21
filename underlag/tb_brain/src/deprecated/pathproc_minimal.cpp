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

ros::Publisher img_pub;
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
double get_shortest(double target_hdng,double actual_hdng){
  double a = target_hdng - actual_hdng;
  if(a > M_PI)a -= M_PI*2;
  else if(a < -M_PI)a += M_PI*2;
  return a;
}
eturn int((c - cols / 2) * res);
}
float get_dst2d(geometry_msgs::Point p1, geometry_msgs::Point p2){
  return(sqrt(pow(p1.x-p2.x,2)+pow(p1.y-p2.y,2)));
}
float get_hdng(geometry_msgs::Point p1,geometry_msgs::Point p0){
  float dx = p1.x - p0.x;
  float dy = p1.y - p0.y;
  return atan2(dy,dx);
}
bool in_blacklist(int itarget,std::vector<int>blacklist){
  for(int i = 0; i < blacklist.size(); i++){
    if(blacklist[i] == itarget)
      return true;
  }
  return false;
}
int getinpath_closestindex2d(nav_msgs::Path pathin,geometry_msgs::PoseStamped pose_to_check){
  if(pathin.poses.size() == 0){
    ROS_INFO("PathAdmin: pathin is empty");
    return -1;
  }
  int lowest_dist_i = 1000;
  float lowest_dist = 1000;
  for(int i = 0; i < pathin.poses.size(); i++){
    float dist = get_dst2d(pathin.poses[i].pose.position,pose_to_check.pose.position);
    if(dist < lowest_dist && dist > 0){
			ROS_INFO("New closest dist: %.2f index: %i",dist,i);
      lowest_dist_i = i;
      lowest_dist   = dist;
    }
  }
  return lowest_dist_i;
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

std::vector<std::vector<int>> get_segmented_clusters_in_path2d(nav_msgs::Path pathin,float radius){
	std::vector<std::vector<int>> clusters;
	std::vector<std::vector<int>> neighbours_at_index;
	neighbours_at_index = getinpath_neighbours(pathin,radius);
	std::vector<bool> indexes_accounted_for;
	indexes_accounted_for.resize(pathin.poses.size());
	while(true){
		std::vector<int> cluster;
		std::vector<int> last_new_neighbours;
		for(int i = 0; i < neighbours_at_index.size(); i++){
			if(last_new_neighbours.size() == 0 && !indexes_accounted_for[i]){
				last_new_neighbours.push_back(i);
				ROS_INFO("Cluster[#%i] complete, next start point: %i",clusters.size(),i);
			}
		}
		if(last_new_neighbours.size() == 0){
			ROS_INFO("CLUSTERING COMPLETE");
			break;
		}
		while(last_new_neighbours.size() > 0){
			ROS_INFO("Last new neighbours %i",last_new_neighbours.size());
			std::vector<int> new_neighbours;
			for(int i = 0; i < last_new_neighbours.size(); i++){
				for(int k = 0; k < neighbours_at_index[last_new_neighbours[i]].size(); k++){
					int neighbour_index = neighbours_at_index[last_new_neighbours[i]][k];
					ROS_INFO("neighbour_index %i",neighbour_index);
					if(!indexes_accounted_for[neighbour_index]){
						indexes_accounted_for[neighbour_index] = true;
						new_neighbours.push_back(neighbour_index);
					}
				}
			}
			for(int i = 0; i < new_neighbours.size(); i++){
				cluster.push_back(new_neighbours[i]);
			}
			last_new_neighbours = new_neighbours;
		}
		clusters.push_back(cluster);
		ROS_INFO("Cluster[#%i] complete: %i poses long",clusters.size(),cluster.size());
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

int main(int argc, char** argv)
{
  ros::init(argc, argv, "tb_pathproc_node");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");
  private_nh.getParam("workdir_path", par_workdir);
  private_nh.param("par_zjump",   par_zjump, 3.0);//*2.0);
  private_nh.param("map_sidelength",par_maprad, 300.0);

	ros::Publisher path_cndidat_pub   = nh.advertise<nav_msgs::Path>("/tb_path/path_cndidat",100);;

  ros::Subscriber s41 = nh.subscribe("/tb_nav/visited_path_update",1,&pathvstdupd_cb);

  //ros::Subscriber s43 = nh.subscribe("/tb_cmd/building_centroid",1,&bld_cb);

  ros::Publisher pub = nh.advertise<sensor_msgs::Image>("/tb_map/heightmap_requested", 1);
  nav_msgs::Path path_visited;
  ros::Rate rate(1);
  bool done = true;
  ros::Time start = ros::Time::now();
  float radius = 6;

  rate.sleep();
  ros::spinOnce();
  }
  return 0;
}
