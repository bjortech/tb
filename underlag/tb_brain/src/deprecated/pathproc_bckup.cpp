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
bool in_path(nav_msgs::Path pathin,geometry_msgs::PoseStamped ps){
  if(pathin.poses.size() == 0)
    return false;
  for(int i = 0; i < pathin.poses.size(); i++){
    if( pathin.poses[i].pose.position.x == ps.pose.position.x
     && pathin.poses[i].pose.position.y == ps.pose.position.y
     && pathin.poses[i].pose.position.z == ps.pose.position.z)
      return true;
  }
  return false;
}
float dst_point_in_path_zfac(nav_msgs::Path pathin,geometry_msgs::Point pin,float zfac){
  float res,dst;
  res = 1000;
  if(pathin.poses.size() == 0)
    return res;
  for(int i = 0; i < pathin.poses.size(); i++){
    dst = sqrt((pathin.poses[i].pose.position.x-pin.x)*(pathin.poses[i].pose.position.x-pin.x)+
               (pathin.poses[i].pose.position.y-pin.y)*(pathin.poses[i].pose.position.y-pin.y)+
               zfac*(pathin.poses[i].pose.position.z-pin.z)*(pathin.poses[i].pose.position.z-pin.z));
    if(dst < res)
      res = dst;
  }
  return res;
}
float dst_point_in_path(nav_msgs::Path pathin,geometry_msgs::Point pin){
  float res,dst;
  res = 1000;
  if(pathin.poses.size() == 0)
    return res;
  for(int i = 0; i < pathin.poses.size(); i++){
    dst = sqrt((pathin.poses[i].pose.position.x-pin.x)*(pathin.poses[i].pose.position.x-pin.x)+
               (pathin.poses[i].pose.position.y-pin.y)*(pathin.poses[i].pose.position.y-pin.y)+
               (pathin.poses[i].pose.position.z-pin.z)*(pathin.poses[i].pose.position.z-pin.z));
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

std::vector<std::vector<int>> get_segmented_clusters_in_path2d(nav_msgs::Path pathin,float radians){
  std::vector<std::vector<int>> clusters;
  std::vector<bool> indexes_accounted_for;
  indexes_accounted_for.resize(pathin.poses.size());
  ROS_INFO("Pathin: %i",pathin.poses.size());
  for(int i = 0; i < pathin.poses.size(); i++){
    if(!indexes_accounted_for[i]){
      std::vector<int> indexes_in_cluster;
      indexes_in_cluster.push_back(0);
      indexes_in_cluster.push_back(1);
      clusters.push_back(indexes_in_cluster);
    while(indexes_in_cluster.size() > 0){
      for(int i = 0; i < pathin.poses.size(); i++){
        for(int k = 0; k < indexes_in_cluster.size(); k++){
          if(!indexes_accounted_for[i] && k != i){
            float dist = get_dst2d(pathin.poses[i].pose.position,pathin.poses[indexes_in_cluster[k]].pose.position);
            ROS_INFO("k: %i i: %i indexes_checked[%i],dist %.2f",k,i,indexes_checked[k],dist);
            if(dist <= radians){
              clusters[clusters.size()-1].push_back(i);
              indexes_accounted_for[i] = true;
          }
        }
      }
    }
    for(int k = 0; k < indexes_in_cluster.size(); k++)
      clusters.push_back(indexes_in_cluster);
    }
    ROS_INFO("indexes_in_cluster: %i",indexes_in_cluster.size());
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
  for(int i = 0; i < clusters.size(); i++){
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
    return pathout;
  }
  for(int i = 0; i < pathin.poses.size(); i++){
    bool safe = true;
    for(int k = 0; i < pathin_vstd.poses.size(); k++){
      if(safe && getinpath_closestdst2d(pathin,pathin_vstd.poses[k]) < cutoff_dst)
        safe = false;
    }
    if(safe)
      pathout.poses.push_back(pathin.poses[i]);
  }
  ROS_INFO("PathSegmentation VSTD: %i of %i poses not visited",pathout.poses.size(),pathin.poses.size());
  return pathout;
}/*
nav_msgs::Path segment_path_z(nav_msgs::Path pathin,float val_min,float val_max){
	if(pathin.poses.size() < 1){
		ROS_INFO("PathSegmentationZ Failed: No Poses In Path!");
		return;
	}
  nav_msgs::Path pathout;
  pathout.header.frame_id = "map";
	for(int i = 0; i < pathin.poses.size(); i++){
		if(pathin.poses[i].pose.position.z > val_min && pathin.poses[i].pose.position.z < val_max)
      pathout.poses.push_back(pathin.poses[i]);
	}
  int percent = (pathout.poses.size() / pathin.poses.size())*100;
  ROS_INFO("PathSegmentationZ: %i of %i poses (%i percent) between %.2f and %.2f",pathout.poses.size(),pathin.poses.size(),percent,val_min,val_max);
  return pathout;
}

nav_msgs::Path get_consecutive_poses(nav_msgs::Path pathin){
  nav_msgs::Path pathout;
	pathout.header.frame_id = "map";
	ROS_INFO("Get Consecutive Poses: pathin is %i poses long pathout is %i",pathin.poses.size(),pathout.poses.size());
	int closest_index = getinpath_closestindex2d(pathin,last_pose); //= getinpath_closestindex3d(pathin,last_pose,5);
	while(pathin.poses.size() > 0){
		pathout.poses.push_back(pathin.poses[closest_index]);
		pathin.poses.erase(pathin.poses.begin()+closest_index);
    closest_index = getinpath_closestindex2d(pathin,pathout.poses[pathout.poses.size()-1]);
	}
	ROS_INFO("Get Consecutive Poses: pathin is %i poses long pathout is %i",pathin.poses.size(),pathout.poses.size());
  return pathout;
}
nav_msgs::Path sort_path(nav_msgs::Path pathin){
	nav_msgs::Path pathout;
	pathout.header.frame_id = "map";
	geometry_msgs::Point centroid;
	int count = 0;
	float sumx,sumy,sumz;
	for(int i = 0; i < pathin.poses.size(); i++){
	//	if(dst_point_in_path_zfac(visited_path,ps.pose.position,3) > 5){
			count++;
			sumx += pathin.poses[i].pose.position.x;
			sumy += pathin.poses[i].pose.position.y;
			sumz += pathin.poses[i].pose.position.z;
//		}
	}
	centroid.x = sumx/count;
	centroid.y = sumy/count;
	centroid.z = sumz/count;
	float hdng0 = get_hdng(pos,centroid);
	ROS_INFO("Sort Path: Centroid %.2f %.2f %.2f",centroid.x,centroid.y,centroid.z);
	std::vector<std::tuple<int,float>>vt;
	for(int i = 0; i < pathin.poses.size(); i++){
		//	vt.push_back(std::make_tuple(i,get_shortest(get_hdng(pathin.poses[i].pose.position,centroid),hdng0)));
			vt.push_back(std::make_tuple(i,get_hdng(pathin.poses[i].pose.position,centroid)));
	}
	(vt.begin(),vt.end(),sort_hdng_pair);
	for(int i = 0; i < pathin.poses.size(); i++){
		pathout.poses.push_back(pathin.poses[std::get<0>(vt[i])]);
	}
	return pathout;
}*/
std::vector<geometry_msgs::Point> get_path_limits(nav_msgs::Path pathin){
  std::vector<geometry_msgs::Point>out;
  double ltot,dx,dy,dz;
  geometry_msgs::Point lim0,lim1,dtot;
  for(int i = 0; i < (pathin.poses.size()-1); i++){
    if(pathin.poses[i+1].pose.position.x < lim0.x)
      lim0.x = pathin.poses[i+1].pose.position.x;
    if(pathin.poses[i+1].pose.position.y < lim0.y)
      lim0.y = pathin.poses[i+1].pose.position.y;
    if(pathin.poses[i+1].pose.position.z < lim0.z)
      lim0.z = pathin.poses[i+1].pose.position.z;
    if(pathin.poses[i+1].pose.position.x > lim1.x)
      lim1.x = pathin.poses[i+1].pose.position.x;
    if(pathin.poses[i+1].pose.position.y > lim1.y)
      lim1.y = pathin.poses[i+1].pose.position.y;
    if(pathin.poses[i+1].pose.position.z > lim1.z)
      lim1.z = pathin.poses[i+1].pose.position.z;
  }
  float dst_x,dst_y,dst_z;
  dst_x = lim1.x - lim0.x;
  dst_y = lim1.y - lim0.y;
  dst_z = lim1.z - lim0.z;
  ROS_INFO("PathWorker: boundingbox(min,max):  x: %.2f -> %.2f,y: %.2f -> %.2f,z: %.2f -> %.2f",lim0.x, lim0.y, lim0.z, lim1.x, lim1.y, lim1.z);
  ROS_INFO("PathWorker: range axes:            x: %.2f         y: %.2f         z: %.2f",dst_x, dst_y, dst_z);
  out.push_back(lim0);
  out.push_back(lim1);
  return out;
}

void pathvstdupd_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
  int zlvl_in  = round(msg->pose.position.z / par_zjump) + 4;
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
      int zlvl_in  = round(msg->poses[i].pose.position.z / par_zjump) + 4;
      paths_cand_at_lvl[zlvl_in].poses.push_back(msg->poses[i]);
    }
  }
}
void pathcand_cb(const nav_msgs::Path::ConstPtr& msg){
  ROS_INFO("Pathcand callback %i",msg->poses.size());
  if(msg->poses.size() > pathcand.poses.size()){
    for(int i = pathcand.poses.size(); i < msg->poses.size(); i++){
      pathcand.poses.push_back(msg->poses[i]);
      int zlvl_in  = round(msg->poses[i].pose.position.z / par_zjump) + 4;
      paths_cand_at_lvl[zlvl_in].poses.push_back(msg->poses[i]);
    }
  }
}
void pathvstd_cb(const nav_msgs::Path::ConstPtr& msg){
  ROS_INFO("Pathcand callback %i",msg->poses.size());
  if(msg->poses.size() > pathvstd.poses.size()){
    for(int i = pathvstd.poses.size(); i < msg->poses.size(); i++){
      pathvstd.poses.push_back(msg->poses[i]);
      int zlvl_in  = round(msg->poses[i].pose.position.z / par_zjump) + 4;
      paths_vstd_at_lvl[zlvl_in].poses.push_back(msg->poses[i]);
    }
  }
}
nav_msgs::Path unify_path(){
  nav_msgs::Path pathout;
  ROS_INFO("Unifying paths");
	pathout.header.frame_id = "map";
  for(int i = 0; i < paths_candseg_at_lvl.size(); i++){
    for(int k = 0; k < paths_candseg_at_lvl[i].poses.size(); k++){
      pathout.poses.push_back(paths_candseg_at_lvl[i].poses[k]);
    }
  }
  ROS_INFO("Unified path size: %i",pathout.poses.size());
  return pathout;
}
void zlvl_cb(const std_msgs::UInt8::ConstPtr& msg){
  zlvl = msg->data;
}
int main(int argc, char** argv)
{
  ros::init(argc, argv, "tb_pathproc_node");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");
  private_nh.getParam("workdir_path", par_workdir);
  private_nh.param("par_zjump",   par_zjump, 3.0);//*2.0);
  private_nh.param("map_sidelength",par_maprad, 300.0);

  for(int i = 0; i < 20; i++){
    z_lvls.push_back(int(round(-par_zjump*3 + par_zjump*i)));
  }

  nav_msgs::Path path_template;
  path_template.header.frame_id = "map";
  for(int i = 0; i < 20; i++){
    z_lvls.push_back(int(round(-par_zjump*3 + par_zjump*i)));
    paths_cand_at_lvl.push_back(path_template);
    paths_vstd_at_lvl.push_back(path_template);
  }
  ros::Publisher pub_altlvl   = nh.advertise<std_msgs::UInt8>("/",100);

	ros::Publisher path_cndidat_pub   = nh.advertise<nav_msgs::Path>("/tb_path/path_cndidat",100);
	ros::Publisher path_visited_pub   = nh.advertise<nav_msgs::Path>("/tb_path/path_visited",100);
	ros::Publisher path_unified_pub   = nh.advertise<nav_msgs::Path>("/tb_path/path_unified",100);
  ros::Publisher path_not_visited_pub   = nh.advertise<nav_msgs::Path>("/tb_path/path_not_visited",100);

  ros::Subscriber s41 = nh.subscribe("/tb_nav/visited_path_update",1,&pathvstdupd_cb);
  ros::Subscriber s42 = nh.subscribe("/tb_path_updates",1,&pathcandupd_cb);
  ros::Subscriber s43 = nh.subscribe("/tb_fsm/altlvl",1,&zlvl_cb);

  ros::Subscriber s51 = nh.subscribe("/tb_path_filtered",1,&pathcandupd_cb);
  ros::Subscriber s52 = nh.subscribe("/tb_nav/visited_path",1,&pathvstd_cb);

  //ros::Subscriber s43 = nh.subscribe("/tb_cmd/building_centroid",1,&bld_cb);

  ros::Publisher pub = nh.advertise<sensor_msgs::Image>("/tb_map/heightmap_requested", 1);
  nav_msgs::Path path_visited;
  ros::Rate rate(1);
  bool done = true;
  ros::Time start = ros::Time::now();
  float radius = 6;
  float cutoff = 5;

  while(ros::ok()){
   if(done){
     done = false;
    if(paths_cand_at_lvl[zlvl].poses.size() > 0){
      path_cndidat_pub.publish(paths_cand_at_lvl[zlvl]);
      paths_candseg_at_lvl[zlvl] = get_path_segmented2d(paths_cand_at_lvl[zlvl],radius);
      /*if(paths_vstd_at_lvl[zlvl].poses.size() > 0){
        path_visited_pub.publish(paths_vstd_at_lvl[zlvl]);
        path_not_visited_pub.publish(getinpath_not_visited2d(paths_cand_at_lvl[zlvl],paths_vstd_at_lvl[zlvl],10));
      }*/
      path_unified_pub.publish(unify_path());
    }
    cv_bridge::CvImage cv_image;
    sensor_msgs::Image ros_image;
    cv_image.image = mapimg;
    cv_image.encoding = "bgr8";
    cv_image.toImageMsg(ros_image);
    pub.publish(ros_image);
    done = true;
	 }
  rate.sleep();
  ros::spinOnce();
  }
  return 0;
}
