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

using namespace octomap;
using namespace std;
shared_ptr<DynamicEDTOctomap> edf_ptr;
AbstractOcTree* abs_octree;
shared_ptr<OcTree> octree;

ros::Publisher pub;
geometry_msgs::Point bbmin_custom,bbmax_custom,bbmin_octree,bbmax_octree,pos;
int zlvl;
bool got_map;
double par_maprad,par_maxobs,par_minobs,last_yaw,par_zjump,par_visited_rad,pos_yaw;
geometry_msgs::PolygonStamped poly_bb;
nav_msgs::Path path_candidates,path_visited,paths_clustered_and_unified;
std::vector<int> z_lvls;
std::vector<nav_msgs::Path> paths_clustered_at_zn;
geometry_msgs::PoseStamped last_pose;
std_msgs::Header hdr(){
  std_msgs::Header header;
  header.frame_id = "map";
  header.stamp    = ros::Time::now();
  return header;
}
bool sort_dst_pair(const std::tuple<int,float>& a,const std::tuple<int,float>& b){
    return (std::get<1>(a) < std::get<1>(b));
}
float get_hdng(geometry_msgs::Point p1,geometry_msgs::Point p0){
  return atan2(p1.y - p0.y, p1.x - p0.x);
}
float get_shortest(float target_hdng,float actual_hdng){
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

/******** START ******************** POLY & PATH CONVENIENCE FUNCTIONS ***************** START ******************/
geometry_msgs::Point get_poly_centroid(geometry_msgs::PolygonStamped polyin){
    geometry_msgs::Point centroid;
    double signedArea = 0.0;
    double x0 = 0.0; // Current vertex X
    double y0 = 0.0; // Current vertex Y
    double x1 = 0.0; // Next vertex X
    double y1 = 0.0; // Next vertex Y
    double a = 0.0;  // Partial signed area

    // For all vertices except last
    int i=0;

    if(polyin.polygon.points.size() < 2){
      return centroid;
    }
    for (i=0; i<polyin.polygon.points.size()-1; ++i)
    {
        x0 = polyin.polygon.points[i].x;
        y0 = polyin.polygon.points[i].y;
        x1 = polyin.polygon.points[i+1].x;
        y1 = polyin.polygon.points[i+1].y;
        a = x0*y1 - x1*y0;
        signedArea += a;
        centroid.x += (x0 + x1)*a;
        centroid.y += (y0 + y1)*a;
    }

    x0 = polyin.polygon.points[i].x;
    y0 = polyin.polygon.points[i].y;
    x1 = polyin.polygon.points[0].x;
    y1 = polyin.polygon.points[0].y;
    a = x0*y1 - x1*y0;
    signedArea += a;
    centroid.x += (x0 + x1)*a;
    centroid.y += (y0 + y1)*a;

    signedArea *= 0.5;
    centroid.x /= (6.0*signedArea);
    centroid.y /= (6.0*signedArea);

    return centroid;
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

float getinpath_closestdst(nav_msgs::Path pathin,geometry_msgs::PoseStamped pose_to_check,bool use_3d){
  float lowest_dist = 1000;  float dst;
  if(pathin.poses.size() == 0){
    ROS_INFO("PATHUTIL:   PathAdmin: pathin is empty");
    return lowest_dist;
  }
  for(int i = 0; i < pathin.poses.size(); i++){
    if(use_3d)
      dst = get_dst3d(pathin.poses[i].pose.position,pose_to_check.pose.position);
    else
      dst = get_dst2d(pathin.poses[i].pose.position,pose_to_check.pose.position);
    if(dst < lowest_dist)
      lowest_dist   = dst;
  }
  return lowest_dist;
}
/******** END ******************** POLY & PATH CONVENIENCE FUNCTIONS   ***************** END ******************/


/******** START ******************** SUPPORT FOR PATH SEGMENTATION  ***************** START ******************/
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
std::vector<int> getinpath_indexes_inrad(nav_msgs::Path pathin,int i_to_check,float radians){
  std::vector<int> vec_out;
  if(pathin.poses.size() == 0){
    ROS_INFO("PATHUTIL:   PathAdmin: pathin is empty");
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
  if(pathin.poses.size() == 0){
    ROS_INFO("PATHUTIL:   PathAdmin: pathin is empty");
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
bool in_vec(std::vector<int> vec,int k){
	if(vec.size() == 0)
		return false;
	//	ROS_INFO("PATHUTIL:   in vec: %i",k);
	for(int i = 0; i < vec.size(); i++){
		if(vec[i] == k)
			return true;
	}
	return false;
}
std::vector<int> get_endpoints(std::vector<std::vector<int>> vec_of_vecs){
	std::vector<int> endpoints;
	for(int i = 0; i < vec_of_vecs.size(); i++){
		if(vec_of_vecs[i].size() == 2){
			endpoints.push_back(i);
		}
	}
	return endpoints;
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
	//	ROS_INFO("PATHUTIL:   Vec out added next point: %i",next_point);
	}
	if(in_vec(vec_out,vec_of_vecs[next_point][0]))
		vec_out.push_back(vec_of_vecs[next_point][1]);
	else
		vec_out.push_back(vec_of_vecs[next_point][0]);
	return vec_out;
}
/******** END ******************** SUPPORT FOR PATH SEGMENTATION  ***************** END ******************/



/******** START ******************** CONSTRAIN PATH FUNCTIONS **************** START ******************/
nav_msgs::Path constrain_path_bbpnts(nav_msgs::Path pathin,geometry_msgs::Point bbmin,geometry_msgs::Point bbmax){
  nav_msgs::Path pathout;
  pathout.header = hdr();
  if(pathin.poses.size() == 0)
    return pathout;
  for(int i = 0; i < pathin.poses.size(); i++){
    if((bbmin.x < pathin.poses[i].pose.position.x) && (pathin.poses[i].pose.position.x < bbmax.x)
    && (bbmin.y < pathin.poses[i].pose.position.y) && (pathin.poses[i].pose.position.y < bbmax.y)
    && (bbmin.z < pathin.poses[i].pose.position.z) && (pathin.poses[i].pose.position.z < bbmax.z))
      pathout.poses.push_back(pathin.poses[i]);
  }
  ROS_INFO("PATHUTIL: constrain_path_bbpnts: pathin %i poses, x %.0f y %.0f z %.0f -> x %.0f y %.0f z %.0f: Pathout: %i poses",pathin.poses.size(),bbmin.x,bbmin.y,bbmin.z,bbmax.x,bbmax.y,bbmax.z,pathout.poses.size());
  return pathout;
}
nav_msgs::Path constrain_path_zlvl(nav_msgs::Path pathin, int zlvl_min,int zlvl_max){
  nav_msgs::Path pathout;
  pathout.header = hdr();
  if(pathin.poses.size() == 0)
    return pathout;
  geometry_msgs::Point bbmin,bbmax;
  bbmin.x = -1000; bbmin.y = -1000; bbmax.x = 1000; bbmax.y = 1000;
  bbmin.z = z_lvls[zlvl_min]-1;
  bbmax.z = z_lvls[zlvl_max]+1;
  pathout = constrain_path_bbpnts(pathin,bbmin,bbmax);
  ROS_INFO("PATHUTIL: constrain_path_zlvl: pathin %i poses, lvls: %i -> %i pathout: %i poses",pathin.poses.size(),zlvl_min,zlvl_max,pathout.poses.size());
  return pathout;
}
nav_msgs::Path constrain_path_lvls(nav_msgs::Path pathin,int minimum_number_of_levels){
  nav_msgs::Path pathout;
  pathout.header = hdr();
  if(pathin.poses.size() == 0)
    return pathout;
  for(int i = 0; i < pathin.poses.size(); i++){
    std::vector<int> indexes_in_rad;
    indexes_in_rad = getinpath_indexes_inrad(pathin,i,4);
    if(indexes_in_rad.size() >= minimum_number_of_levels)
      pathout.poses.push_back(pathin.poses[i]);
  }
  ROS_INFO("PATHUTIL: constrain_path_lvls: pathin %i poses, minlvls: %i pathout: %i poses",pathin.poses.size(),minimum_number_of_levels,pathout.poses.size());
  return pathout;
}

nav_msgs::Path constrain_path_bbpoly(nav_msgs::Path pathin,geometry_msgs::PolygonStamped poly_bb){
  nav_msgs::Path pathout;
  pathout.header = hdr();
  geometry_msgs::Point centroid;
  centroid = get_poly_centroid(poly_bb);
  if(pathin.poses.size() == 0 || poly_bb.polygon.points.size() == 0)
    return pathin;
  for(int i = 0; i < pathin.poses.size(); i++){
    if(in_poly(poly_bb,pathin.poses[i].pose.position.x,pathin.poses[i].pose.position.y)){
      float pos_hdng  = get_hdng(pathin.poses[i].pose.position,centroid);
      float pose_hdng = tf::getYaw(pathin.poses[i].pose.orientation);
      ROS_INFO("PATHUTIL:   Pos hdng: %.2f pose hdng: %.2f",pos_hdng,pose_hdng);
      pathout.poses.push_back(pathin.poses[i]);
      ROS_INFO("PATHUTIL: constrain_path_bbpoly: pathin %i poses, polyin %i points pathout: %i poses",pathin.poses.size(),poly_bb.polygon.points.size(),pathout.poses.size());

    }
  }
  return pathout;
}
nav_msgs::Path constrain_path_vstd(nav_msgs::Path pathin,nav_msgs::Path pathin_vstd,float cutoff_dst,bool use_3d){
  nav_msgs::Path pathout;
  pathout.header = hdr();
  if(fmin(pathin.poses.size(),pathin_vstd.poses.size()) == 0){
    return pathin;
  }
  ROS_INFO("PATHUTIL:   Pathin size: cand %i vstd %i cutoff %.2f",pathin.poses.size(),pathin_vstd.poses.size(),cutoff_dst);
  for(int i = 0; i < pathin.poses.size(); i++){
    if(getinpath_closestdst(pathin_vstd,pathin.poses[i],use_3d) > cutoff_dst)
      pathout.poses.push_back(pathin.poses[i]);
  }
  ROS_INFO("PATHUTIL:   PathSegmentation VSTD: %i of %i poses not visited",pathout.poses.size(),pathin.poses.size());
  return pathout;
}

/******** END ******************** CONSTRAIN PATH FUNCTIONS   ***************** END ******************/

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
    i_dst.push_back(std::make_tuple(endpoints[i],get_dst2d(last_pose.pose.position,pathin.poses[endpoints[i]].pose.position) ));
  }
  sort(i_dst.begin(),i_dst.end(),sort_dst_pair);
  for(int i = 0; i < i_dst.size(); i++){
    endpoints[i] = std::get<0>(i_dst[i]);
  }
	for(int i = 0; i < endpoints.size(); i++){
		if(!in_vec(pathnum_used,endpoints[i])){
      open_loop  = go_from_endpoint(neighbours_at_index,endpoints[i]);
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
  pathout.header = hdr();
  if(pathin.poses.size() == 0){
    ROS_INFO("PATHUTIL:   PathAdmin: pathin is empty");
    return pathout;
  }
  clusters = get_segmented_clusters_in_path2d(pathin,radians);
	ROS_INFO("PATHUTIL:   clusters: %i",clusters.size());
  for(int i = 0; i < clusters.size(); i++){
		ROS_INFO("PATHUTIL:   clusters[%i]: size: %i",i,clusters[i].size());
    for(int k = 0; k < clusters[i].size(); k++){
      pathout.poses.push_back(pathin.poses[clusters[i][k]]);
    }
  }
  ROS_INFO("PATHUTIL:   PathProc get_path_segmented2d: %i",pathout.poses.size());
  return pathout;
}
nav_msgs::Path unify_path(){
  nav_msgs::Path pathout;
	pathout.header = hdr();
  for(int i = 0; i < paths_clustered_at_zn.size()-1; i++){
		ROS_INFO("PATHUTIL:   paths_clustered_at_zn[%i]: size: %i",i,paths_clustered_at_zn[i].poses.size());
		if(paths_clustered_at_zn[i].poses.size() > 0){
	    for(int k = 0; k < paths_clustered_at_zn[i].poses.size()-1; k++){
	      pathout.poses.push_back(paths_clustered_at_zn[i].poses[k]);
	    }
		}
  }
  ROS_INFO("PATHUTIL:   Unified path size: %i",pathout.poses.size());
  return pathout;
}
/******** END ******************** OCTOMAP & EDTO & DISTANCE FIELD   ***************** END ******************/

/*

ros::Time last_activity_end;
std::vector<std::string> tasks;
std::vector<float> tasks_durations;
void //start_with(std::string activity){
  float dt = (ros::Time::now() - last_activity_end).toSec();
//  ROS_INFO("PATHUTIL:   Time_taker: Now timed: %s. It took %.5f seconds",activity.c_str(),dt);
  if(activity == "done"){
    float total_dt;
    for(int i = 0; i < tasks_durations.size();i++){
      total_dt += tasks_durations[i];
    }
    ROS_INFO("PATHUTIL:   Time_taker: Total time: %.5f",total_dt);
    for(int i = 0; i < tasks_durations.size();i++){
      int percent = round((tasks_durations[i] / total_dt)*100)
      ROS_INFO("PATHUTIL:   Time_taker: %s: %i percent (%.3f seconds)",tasks[i].c_str(),percent,tasks_durations[i]);
    }
    tasks.resize(0);
    tasks_durations.resize(0);
  }
  else{
    tasks.push_back(activity);
    tasks_durations.push_back(dt);
  }
}*/

void zlvl_cb(const std_msgs::UInt8::ConstPtr& msg){
  zlvl = msg->data;
}
void polybb_cb(const geometry_msgs::PolygonStamped::ConstPtr& msg){
  poly_bb         = *msg;
}
void visited_cb(const nav_msgs::Path::ConstPtr& msg){
  path_visited    = *msg;
  if(path_visited.poses.size() > 3)
    last_pose = path_visited.poses[path_visited.poses.size() - 1];
}
void pathcand_cb(const nav_msgs::Path::ConstPtr& msg){
  path_candidates = *msg;
}
void bldstate_cb(const std_msgs::UInt8::ConstPtr& msg){
  int min_num_lvls = 1;
  bool use3dnot2d  = false;
  float cutoff_dst = 11.0;
  int zlvl = last_pose.pose.position.z / par_zjump;
  if(msg->data == 1){
    ROS_INFO("PATHUTILIZER: buildstate: %i, constrain zlvl: %i",msg->data,zlvl);
    pub.publish(constrain_path_zlvl(constrain_path_lvls(path_candidates,min_num_lvls),zlvl-1,zlvl+1));
  }
  if(msg->data == 2 && poly_bb.polygon.points.size() > 2){
    ROS_INFO("PATHUTILIZER: buildstate: %i, constrainbbpoly %i long, constrainvstd: path_vstd %i long,cutoff %.1f use3dnot2d: %i",msg->data,poly_bb.polygon.points.size(),path_visited.poses.size(),cutoff_dst/2,use3dnot2d);
    pub.publish(constrain_path_vstd(constrain_path_bbpoly(path_candidates,poly_bb),path_visited,cutoff_dst/2,use3dnot2d));
  }
  if(msg->data == 3){
    ROS_INFO("PATHUTILIZER: buildstate: %i, constrain#TODO",msg->data);
    ROS_INFO("PATHUTIL:  NOT DEFINED FOR STATE 3 TODO TODO TODO");
  }
}
int main(int argc, char **argv)
{
  ros::init(argc, argv, "tb_fsmpathutilized_node");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");
  private_nh.param("par_zjump",   par_zjump, 3.0);//*2.0);

  nav_msgs::Path path_template;
  path_template.header.frame_id = "map";
  for(int i = 0; i < 20; i++){
    z_lvls.push_back(par_zjump*i);
    paths_clustered_at_zn.push_back(path_template);
  }
  //Mottar nødvendigheter for å kunne filtrere rett
  ros::Subscriber s1 = nh.subscribe("/tb_bld/polygon",1,polybb_cb);
  ros::Subscriber s3 = nh.subscribe("/tb_bld/building_state",1,&bldstate_cb);

  ros::Subscriber s2 = nh.subscribe("/tb_nav/path_visited",1,&visited_cb);
  ros::Subscriber s5 = nh.subscribe("/tb_fsm/altlvl",1,&zlvl_cb);

  ros::Subscriber s6 = nh.subscribe("/tb_path",1,&pathcand_cb);
  pub = nh.advertise<nav_msgs::Path>("/tb_path_requested",100);
	ros::spin();
  return 0;
}
