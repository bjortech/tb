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
#include <tb_msgsrv/Paths.h>

tf2_ros::Buffer tfBuffer;

std::string inspection_type = "idle";

ros::Publisher pub_cmdexplore,pub_cmdmb,pub_target_dash,pub_path_best,pub_get_next_path,pub_cmd;
geometry_msgs::PoseStamped pose_down,pose_side,target,base_pose,target_last,target_final,target_dash;
ros::Time activity_change,last_tilt,path_complete_time;
std_msgs::Float64 arm1_tilt_msg;
geometry_msgs::Vector3 vlp_rpy;
geometry_msgs::Point poly_cleared_centroid,pos,pnt_midpoint,pnt_ref;
nav_msgs::Odometry odom;
float dst_target;
nav_msgs::Path path_down_raw,path_side_raw,path_target_full,path_clear_vlp,path_down_best_in_poly,path_side_best_in_poly,path_full,path_world_visible,path_down_best,path_side_best,path_vlp,path_obs,path_side_full,path_down_full,path_targets,path_visited,path_cleared_full,path_obstacles_full,path_targets_sent,path_side,path_down;
int mainstate;
int blankdraw_counter = 0;
int path_targets_i = 0;
int inspection_count = 0;
double par_zclearing,par_vlpmaxtilt,par_vlpmintilt,par_vlptiltinterval,par_takeoffaltitude;
bool path_complete,tiltlimit_reached,tilting,side_empty;
cv::Mat img(1000,1000,CV_8UC3,cv::Scalar(0, 0, 0)); //create image, set encoding and size, init pixels to default val
cv::Mat img_blank(1000,1000,CV_8UC3,cv::Scalar(0, 0, 0)); //create image, set encoding and size, init pixels to default val
cv::Mat img_target(1000,1000,CV_8UC3,cv::Scalar(0, 0, 0)); //create image, set encoding and size, init pixels to default val
float rad2deg = 180.0/M_PI;
int count_target_paths = 0;
int targets_down_in_poly,targets_side_in_poly;
int dash_stage = 0;
float poly_cleared_centroid_area;
std::vector<int> targets_complete;
tb_msgsrv::Paths paths_active_down,down_in_poly;
tb_msgsrv::Paths paths_active_side,side_in_poly;
geometry_msgs::PolygonStamped poly_cleared,target_final_poly;
tb_msgsrv::Paths paths_candidates,paths_full,paths_down,paths_side;
bool path_side_requested,path_down_requested,path_side_received,path_down_received;
float target_path_distance,target_final_hdng,target_hdng,target_distance;
std::vector<float> path_side_scores_old,path_side_scores_new,path_down_scores_old,path_down_scores_new;
std::string centroids[200][200][20];
ros::Time t0_vz;
geometry_msgs::Point obs_p,obs_r,obs_l,obs_m;
std_msgs::Header hdr(){
  std_msgs::Header header;
  header.frame_id = "map";
  header.stamp    = ros::Time::now();
  return header;
}

bool sort_dst_pair(const std::tuple<int,float>& a,
               const std::tuple<int,float>& b)
{
    return (std::get<1>(a) < std::get<1>(b));
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
cv::Point pnt2cv(geometry_msgs::Point pin){
	int c = x2c(pin.x,img.cols,1);
	int r = y2r(pin.y,img.rows,1);
	return cv::Point(c,r);
}
double saturate(double val, double max){
    if (abs(val) >= max)
      val = (val>0) ? max : -1 * max;
    else
      val = val;
    if((std::isnan(val)) || (std::isinf(val))){
      return 0;
    }
    else{
      return val;
    }
}
cv::Scalar get_color(int base_blue,int base_green, int base_red){
	cv::Scalar color;
	color[0] = base_blue;
	color[1] = base_green;
	color[2] = base_red;
	return color;
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
nav_msgs::Path get_targets_remaining(){
  nav_msgs::Path pathout;
  pathout.header = hdr();
  for(int i = path_targets_i; i < path_targets.poses.size(); i++){
    pathout.poses.push_back(path_targets.poses[i]);
  }
  return pathout;
}
geometry_msgs::PolygonStamped createpoly_square(geometry_msgs::Point pin, float sides){
  geometry_msgs::PolygonStamped poly;
  poly.polygon.points.resize(5);
  poly.header.frame_id = "map";
  poly.polygon.points[0].x = round(pin.x + sides/2);
  poly.polygon.points[0].y = round(pin.y + sides/2);
  poly.polygon.points[1].x = round(pin.x - sides/2);
  poly.polygon.points[1].y = round(pin.y + sides/2);
  poly.polygon.points[2].x = round(pin.x - sides/2);
  poly.polygon.points[2].y = round(pin.y - sides/2);
  poly.polygon.points[3].x = round(pin.x + sides/2);
  poly.polygon.points[3].y = round(pin.y - sides/2);
  poly.polygon.points[4]   = poly.polygon.points[0];
//  ROS_INFO("x %.2f y %.2f x %.2f y %.2f x %.2f y %.2f x %.2f y %.2f x %.2f y %.2f",poly.polygon.points[0].x,poly.polygon.points[0].y,poly.polygon.points[1].x,poly.polygon.points[1].y,poly.polygon.points[2].x,poly.polygon.points[2].y,poly.polygon.points[3].x,poly.polygon.points[3].y,poly.polygon.points[4].x,poly.polygon.points[4].y);
 return poly;
}
geometry_msgs::Point get_poly_centroidarea(geometry_msgs::PolygonStamped polyin){
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

    // Do last vertex separately to avoid performing an expensive
    // modulus operation in each iteration.
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
    centroid.z = signedArea;

    return centroid;
}

double get_shortest(double target_heading,double actual_hdng){
  double a = target_heading - actual_hdng;
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

nav_msgs::Path sort_path(nav_msgs::Path pathin,std::string sort_by){
  nav_msgs::Path pathout;
  if(pathin.poses.size() <= 1)
    return pathin;
  pathout.header = pathin.header;
  std::vector<std::tuple<int,float>>i_dst;
  float base_yaw = tf::getYaw(base_pose.pose.orientation);
  for(int i = 0; i < pathin.poses.size(); i++){
		if(sort_by == "dst_2d")
      i_dst.push_back(std::make_tuple(i,get_dst2d(base_pose.pose.position,pathin.poses[i].pose.position)));
    else if(sort_by == "dst_3d_ref")
      i_dst.push_back(std::make_tuple(i,get_dst3d(pnt_ref,pathin.poses[i].pose.position)));
        else if(sort_by == "dst_3d")
          i_dst.push_back(std::make_tuple(i,get_dst3d(base_pose.pose.position,pathin.poses[i].pose.position)));
    		else if(sort_by == "hdng_abs")
    			i_dst.push_back(std::make_tuple(i,abs(get_shortest(get_hdng(pathin.poses[i].pose.position,base_pose.pose.position),base_yaw) * rad2deg)));
        else if(sort_by == "hdng")
          i_dst.push_back(std::make_tuple(i,get_shortest(get_hdng(pathin.poses[i].pose.position,base_pose.pose.position),base_yaw)));
        else if(sort_by == "zabs")
      		i_dst.push_back(std::make_tuple(i,abs(pathin.poses[i].pose.position.z - base_pose.pose.position.z)));
        else if(sort_by == "z")
          i_dst.push_back(std::make_tuple(i,pathin.poses[i].pose.position.z));
    	}
      sort(i_dst.begin(),i_dst.end(),sort_dst_pair);
  for(int i = 0; i < i_dst.size(); i++){
    pathout.poses.push_back(pathin.poses[std::get<0>(i_dst[i])]);
  }
	return pathout;
}
nav_msgs::Path cutoff_abs(nav_msgs::Path pathin,int degrees){
	nav_msgs::Path pathout;
	pathout.header = hdr();
	float base_yaw = tf::getYaw(base_pose.pose.orientation);
	for(int i = 0; i < pathin.poses.size(); i++){
		int abs_degs =abs(get_shortest(get_hdng(pathin.poses[i].pose.position,base_pose.pose.position),base_yaw) * rad2deg);
		if(abs_degs < degrees){
			pathout.poses.push_back(pathin.poses[i]);
		}
	}
	//ROS_INFO("degrees: %i, pose_in: %i path_out: %i",degrees,pathin.poses.size(),pathout.poses.size());
	return pathout;
}
nav_msgs::Path cutoff_percentage(nav_msgs::Path pathin,int percentage){
  nav_msgs::Path pathout;
  pathout.header = hdr();
  int percent_length = int(round(pathin.poses.size() * percentage / 100));
  for(int i = 0; i < fmax(percent_length,fmin(pathin.poses.size(),3)); i++){
    pathout.poses.push_back(pathin.poses[i]);
  }
  //ROS_INFO("Percent: %i, pose_in: %i, percent_len: %i, path_out: %i",percentage,pathin.poses.size(),percent_length,pathout.poses.size());
  return pathout;
}

int get_closest_i(nav_msgs::Path pathin,geometry_msgs::Point pin){
  float res,dst;
  int closest_i = 0;
  float closest_dst = 100;
  if(pathin.poses.size() == 0)
    return closest_i;
  for(int i = 0; i < pathin.poses.size(); i++){
    float dst = get_dst3d(pathin.poses[i].pose.position,pin);
     if(dst < closest_dst){
       closest_dst = dst;
       closest_i = i;
     }
  }
  return closest_i;
}
int get_farthest_i(nav_msgs::Path pathin,geometry_msgs::Point pin){
  float res,dst;
  int closest_i = 0;
  float farthest_dst = 100;
  if(pathin.poses.size() == 0)
    return closest_i;
  for(int i = 0; i < pathin.poses.size(); i++){
    float dst = get_dst2d(pathin.poses[i].pose.position,pin);
     if(dst > farthest_dst){
       farthest_dst = dst;
       closest_i = i;
     }
  }
  return closest_i;
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
nav_msgs::Path offsetpath(nav_msgs::Path pathin,geometry_msgs::Point offsetpoint){
  for(int i = 0; i < pathin.poses.size(); i++){
    pathin.poses[i].pose.position.x += offsetpoint.x;
    pathin.poses[i].pose.position.y += offsetpoint.y;
  }
  return pathin;
}

nav_msgs::Path create_path(){
  nav_msgs::Path pathout;
  pathout.header = hdr();
	float area_sidelength = 50;
	float total_sidelength = 1000;
	int num_grids = total_sidelength / area_sidelength-1;
  geometry_msgs::PoseStamped pose;
  pose.pose.position.z    = 10;
  pose.pose.orientation.w = 1;
  pose.header = hdr();
  float len = 50;
  for(int i = 0; i < 10; i++){
    for(int k = 0; k < i; k++){
			pose.pose.position.x += pow(-1,i) * len;
      //ROS_INFO("Pnt[%i],x: %.0f y: %.0f",pathout.poses.size(),pose.pose.position.x,pose.pose.position.y);
      pathout.poses.push_back(pose);
    }
    for(int l = 0; l < i; l++){
      pose.pose.position.y += pow(-1,i) * len;
    //  ROS_INFO("Pnt[%i],x: %.0f y: %.0f",pathout.poses.size(),pose.pose.position.x,pose.pose.position.y);
      pathout.poses.push_back(pose);
    }
  }
	return pathout;
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
std::vector<int> getinpath_indexes_simple(nav_msgs::Path pathin,geometry_msgs::Point pnt_to_check,float radius){
  std::vector<int> vec_out;
  if(pathin.poses.size() == 0){
    //ROS_INFO("PathAdmin: pathin is empty");
    return vec_out;
  }
  for(int i = 0; i < pathin.poses.size(); i++){
    if(get_dst2d(pathin.poses[i].pose.position,pnt_to_check) <= radius)
      vec_out.push_back(i);
  }
  return vec_out;
}

bool dst_point_in_path_lim2d(nav_msgs::Path pathin,geometry_msgs::Point pin,float lim){
  float res,dst;
  if(pathin.poses.size() == 0)
    return true;
  for(int i = 0; i < pathin.poses.size(); i++){
     if(get_dst2d(pathin.poses[i].pose.position,pin) < lim)
        return false;
  }
  return true;
}
float get_path_density(nav_msgs::Path pathin){
  pathin      = sort_path(pathin,"dst_2d");
  nav_msgs::Path pathin_proj;
  for(int i = 0; i < pathin.poses.size(); i++){
    if(dst_point_in_path_lim2d(pathin_proj,pathin.poses[i].pose.position,2)){
      pathin_proj.poses.push_back(pathin.poses[i]);
    }
  }
  float dst_tot = 0;
  int neighbours_total = 0;
  for(int i = 0; i < pathin_proj.poses.size(); i++){
    std::vector<int> neighbours = getinpath_indexes_simple(pathin,pathin_proj.poses[i].pose.position,2);
    neighbours_total += neighbours.size();
    if(i > 0)
      dst_tot += get_dst2d(pathin_proj.poses[i].pose.position,pathin_proj.poses[i-1].pose.position);
  }
  float pathin_size = pathin.poses.size();
  float pathin_proj_size = pathin_proj.poses.size();
  float neighbours_tot = neighbours_total;
  float neighbours_ave = neighbours_tot / pathin_proj_size;
  float density_poses  = pathin_size / pathin_proj_size;
  float density_dist   = pathin_size / dst_tot;
  ROS_INFO("DENSITY EVALUATION: [3d-2d: %i -> %i], neighbours / meters: %i / %.0f",pathin.poses.size(),pathin_proj.poses.size(),neighbours_total,dst_tot);
  ROS_INFO("DENSITY EVALUATION: neighbours/poses/distance: [%.3f, %.3f, %.3f]",neighbours_ave,density_poses,density_dist);
  float density = (neighbours_ave + density_poses + density_dist) / 3;
  return density;
}
float score_candidates(nav_msgs::Path pathin){
  float yaw0,yawN,yawT,hdng0N,hdngT0,dst0N,dstT0,dhdng0NT0,dyaw0T,dzT0;
  float best_score = -100;
  float score_mod = 0;
  int best_i = 0;
  bool path_is_down,pose_is_down;
  if(pathin.poses.size() < 2)
    return -100;
  if(pathin.poses[0].pose.orientation.y != 0.7071 && target.pose.orientation.y != 0.7071){
    yaw0  = tf::getYaw(pathin.poses[0].pose.orientation);
    yawN  = tf::getYaw(pathin.poses[pathin.poses.size()-1].pose.orientation);
    yawT  = tf::getYaw(target.pose.orientation);
  }
  hdng0N = get_hdng(pathin.poses[pathin.poses.size()-1].pose.position,pathin.poses[0].pose.position);
  hdngT0 = get_hdng(pathin.poses[0].pose.position,target.pose.position);

  dst0N  = get_dst3d(pathin.poses[pathin.poses.size()-1].pose.position,pathin.poses[0].pose.position);
  dstT0  = get_dst3d(pathin.poses[0].pose.position,target.pose.position);

  dzT0   = pathin.poses[0].pose.position.z - target.pose.position.z;

  dhdng0NT0 = get_shortest(hdng0N,hdngT0);
  dyaw0T    = get_shortest(yaw0,yawT);

	if(dyaw0T < 0)
		dyaw0T *= -1;
  if(dhdng0NT0 < 0)
    dhdng0NT0 *= -1;

  float score_down = 0;
  float score_dst0N = dst0N / 10;
  float score_dstTO = dstT0 / -10;
  float score_hdng  = dhdng0NT0 * -1;
  float score_hyaw  = dyaw0T * -5;
  float score_cands = float(pathin.poses.size()) /10.0;
  float score_z     = -abs(dzT0)/10;
  float score_tot   = score_dstTO + score_hdng + score_cands + score_z + score_dst0N + score_down + score_mod + score_hyaw;
  ROS_INFO("SCORE EVALUATION dstT0 %.2f, score_hdng: %.2f, score_cands: %.2f, score_z: %.2f, score_dst0N: %.2f score_down: %.2f score_hyaw: %.2f tot: %.2f", score_dstTO,score_hdng,score_cands,score_z,score_dst0N,score_down,score_hyaw,score_tot);
  return score_tot;
}
float get_tot_len(nav_msgs::Path pathin){
  float len = 0;
  for(int i = 1; i < pathin.poses.size(); i++){
    len += get_dst3d(pathin.poses[i].pose.position,pathin.poses[i-1].pose.position);
  }
  return len;
}

geometry_msgs::Point get_ave_pnt(nav_msgs::Path pathin){
  geometry_msgs::Point pnt;
  for(int i = 0; i < pathin.poses.size(); i++){
    pnt.x += pathin.poses[i].pose.position.x;
    pnt.y += pathin.poses[i].pose.position.y;
    pnt.z += pathin.poses[i].pose.position.z;
  }
  pnt.x /= pathin.poses.size();
  pnt.y /= pathin.poses.size();
  pnt.z /= pathin.poses.size();
  return pnt;
}
float get_zmax(nav_msgs::Path pathin){
  float zmx = 0;
  for(int i = 0; i < pathin.poses.size(); i++){
    if(pathin.poses[i].pose.position.z > zmx){
      zmx = pathin.poses[i].pose.position.z;
    }
  }
  return zmx;
}


nav_msgs::Path get_zmaxposes(nav_msgs::Path pathin){
  nav_msgs::Path pathout;
  float zmx = get_zmax(pathin);
  for(int i = pathin.poses.size()-1; i > 0; i--){
    if(pathin.poses[i].pose.position.z >= zmx - 1)
       pathout.poses.push_back(pathin.poses[i]);
  }
  pathout.header = hdr();
  return pathout;
}

nav_msgs::Path remove_nth_first(nav_msgs::Path pathin,int i0){
  nav_msgs::Path pathout;
  pathout.header = hdr();
  if(i0 < 0)
    return pathout;
  for(int i = i0; i < pathin.poses.size(); i++){
    pathout.poses.push_back(pathin.poses[i]);
  }
  return pathout;
}
void request_paths(){
	std_msgs::UInt8 get_path;
  pub_get_next_path.publish(get_path);
}
bool pose_is_side(geometry_msgs::PoseStamped pose){
  if(pose.pose.orientation.y == 0.7071)
    return false;
  else
    return true;
}
void draw_pose(geometry_msgs::Point pnt,float yaw,float len, cv::Scalar color){
  geometry_msgs::Point pyaw;
  pyaw.x = pnt.x + len * cos(yaw);
  pyaw.y = pnt.y + len * sin(yaw);
  cv::circle(img,pnt2cv(pnt),2,color,1);
  cv::line (img, pnt2cv(pnt), pnt2cv(pyaw),color,1,cv::LINE_8,0);
}
void set_target_pose(geometry_msgs::PoseStamped new_target_pose){
  if(!pose_is_side(new_target_pose)){
    inspection_type = "down";
  }
  else
    inspection_type = "side";
  target_last = target;
	target      = new_target_pose;
  float dst_targetpos = get_dst3d(pos,target_last.pose.position);
  float dst_targetnew = get_dst3d(pos,target.pose.position);
  float dst_targets = get_dst3d(target.pose.position,target_last.pose.position);
  if(inspection_type == "side"){
    geometry_msgs::Point pyaw;
    pyaw.x = target.pose.position.x + 5 * cos(tf::getYaw(target.pose.orientation));
    pyaw.y = target.pose.position.y + 5 * sin(tf::getYaw(target.pose.orientation));
    cv::circle(img_target,pnt2cv(target.pose.position),2,get_color(0,200,0),1);
    float dyaw = get_dst2d(pyaw,target.pose.position);
    if(dyaw < 10)
      cv::line (img_target, pnt2cv(target.pose.position),pnt2cv(pyaw),get_color(0,200,0),1,cv::LINE_8,0);
  }
  else{
    geometry_msgs::Point prect0,prect1;
    prect0.x = target.pose.position.x-2;
    prect0.y = target.pose.position.y-2;
    prect1.x = target.pose.position.x+2*2;
    prect1.y = target.pose.position.y+2*2;
    cv::rectangle(img_target, pnt2cv(prect0),pnt2cv(prect1),get_color(200,0,0),1,8,0);
  }

  ROS_INFO("New target %.0f m away when %.0f meters remain to last target (%.0f meters apart)",dst_targetnew,dst_targetpos,dst_targets);
  if(dst_targets < 30)
    cv::line (img_target, pnt2cv(target_last.pose.position), pnt2cv(target.pose.position),get_color(0,0,200),1,cv::LINE_8,0);
  new_target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(get_hdng(new_target_pose.pose.position,target_last.pose.position));
  new_target_pose.pose.position = target.pose.position;
  new_target_pose.pose.position.z = 0;
  new_target_pose.header = hdr();
  path_targets.poses.push_back(new_target_pose);
  cv::imwrite("/home/nuc/brain/"+std::to_string(count_target_paths)+"targets.png",img_target);

  pub_cmd.publish(new_target_pose.pose.position);
//  pub_cmdmb.publish(new_target_pose);
}

bool dst_point_in_path_lim(nav_msgs::Path pathin,geometry_msgs::Point pin,float lim){
  float res,dst;
  if(pathin.poses.size() == 0)
    return true;
  for(int i = 0; i < pathin.poses.size(); i++){
     if(get_dst3d(pathin.poses[i].pose.position,pin) < lim)
        return false;
  }
  return true;
}

nav_msgs::Path get_next_candidates(tb_msgsrv::Paths pathsin,geometry_msgs::PoseStamped pose_comparison){
  float yaw0,yawN,yawT,hdng0N,hdngT0,dst0N,dstT0,dhdng0NT0,dyaw0T,dzT0;
  float best_score = -100;
  float score_mod = 0;
  int best_i = 0;
  bool path_is_down,pose_is_down;
  if(pose_comparison.pose.orientation.y == 0.7071)
    pose_is_down = true;
  std::vector<float> scores;
  for(int i = 0; i < pathsin.paths.size(); i++){
    if(pathsin.paths[i].poses.size() > 0){
      score_mod = 0;
      if(pathsin.paths[i].poses[0].pose.orientation.y == 0.7071)
        path_is_down = true;
      else
        path_is_down = false;


      yaw0  = tf::getYaw(pathsin.paths[i].poses[0].pose.orientation);
      yawN  = tf::getYaw(pathsin.paths[i].poses[pathsin.paths[i].poses.size()-1].pose.orientation);
      yawT  = tf::getYaw(pose_comparison.pose.orientation);

      hdng0N = get_hdng(pathsin.paths[i].poses[pathsin.paths[i].poses.size()-1].pose.position,pathsin.paths[i].poses[0].pose.position);
      hdngT0 = get_hdng(pathsin.paths[i].poses[0].pose.position,pose_comparison.pose.position);

      dst0N  = get_dst3d(pathsin.paths[i].poses[pathsin.paths[i].poses.size()-1].pose.position,pathsin.paths[i].poses[0].pose.position);
      dstT0  = get_dst3d(pathsin.paths[i].poses[0].pose.position,pose_comparison.pose.position);

      dzT0   = pathsin.paths[i].poses[0].pose.position.z - pose_comparison.pose.position.z;

      dhdng0NT0 = get_shortest(hdng0N,hdngT0);
      dyaw0T    = get_shortest(yaw0,yawT);
			if(dyaw0T < 0)
				dyaw0T *= -1;
	    if(dhdng0NT0 < 0)
        dhdng0NT0 *= -1;
    }
    else
      score_mod = -20;

    float score_down = 0;
    float score_dst0N = dst0N / 10;
    float score_dstTO = dstT0 / -10;
    float score_hdng  = dhdng0NT0 * -1;
    float score_hyaw  = dyaw0T * -5;
    float score_cands = float(pathsin.paths[i].poses.size()) /10.0;
    float score_z     = -abs(dzT0)/10;
    if(path_is_down)
      score_hyaw = 0;
    if(path_is_down == pose_is_down)
      score_down = 5;
    float score_tot   = score_dstTO + score_hdng + score_cands + score_z + score_dst0N + score_down + score_mod + score_hyaw;
  //  ROS_INFO("score_dstTO  %.2f, score_hdng: %.2f, score_cands: %.2f, score_z: %.2f, score_dst0N: %.2f score_down: %.2f score_hyaw: %.2f tot: %.2f", score_dstTO,        score_hdng,      score_cands,       score_z,             score_dst0N,     score_down,   score_hyaw,      score_tot);
    scores.push_back(score_tot);
    if(score_tot > best_score){
      best_score = score_tot;
      best_i     = i;
    }
  }

	nav_msgs::Path pathout;
	pathout.header = hdr();
	if(pathsin.paths.size() <= best_i)
		return pathout;
	else
		return pathsin.paths[best_i];
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
 dst_target = get_dst3d(target.pose.position,pos);

	pos.x   = transformStamped.transform.translation.x;
	pos.y   = transformStamped.transform.translation.y;
	pos.z   = transformStamped.transform.translation.z;
	tf2::Matrix3x3 q(tf2::Quaternion(transformStamped.transform.rotation.x, transformStamped.transform.rotation.y, transformStamped.transform.rotation.z, transformStamped.transform.rotation.w));
	q.getRPY(vlp_rpy.x,vlp_rpy.y,vlp_rpy.z);
	float delta_vlp = get_shortest(arm1_tilt_msg.data,vlp_rpy.y);
	if(delta_vlp < 0.05 && delta_vlp > -0.05)
		tilting = false;
	else
		tilting = true;
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
  if(arm1_tilt_msg.data - new_tilt > 0.1 || arm1_tilt_msg.data - new_tilt < -0.1)
    tilting = true;
  ROS_INFO("BRAIN: ARM - tilt from %.2f -> %.2f",arm1_tilt_msg.data,new_tilt);
  last_tilt = ros::Time::now();
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


nav_msgs::Path constrain_path_bbpoly(nav_msgs::Path pathin,geometry_msgs::PolygonStamped poly_bb){
  nav_msgs::Path pathout;
  pathout.header = hdr();
  if(pathin.poses.size() == 0 || poly_bb.polygon.points.size() == 0)
    return pathin;
  for(int i = 0; i < pathin.poses.size(); i++){
    if(in_poly(poly_bb,pathin.poses[i].pose.position.x,pathin.poses[i].pose.position.y)){
      pathout.poses.push_back(pathin.poses[i]);
    }
  }
  return pathout;
}
tb_msgsrv::Paths get_paths_in_poly(tb_msgsrv::Paths pathsin){
  tb_msgsrv::Paths pathsout;
   for(int i = 0; i < pathsin.paths.size(); i++){
    nav_msgs::Path path_temp = constrain_path_bbpoly(pathsin.paths[i],target_final_poly);
    if(path_temp.poses.size() > 0)
      pathsout.paths.push_back(path_temp);
  }
  return pathsout;
}

void hdngclear_cb(const nav_msgs::Path::ConstPtr& msg){
  path_clear_vlp = *msg;
}
void rawdown_cb(const nav_msgs::Path::ConstPtr& msg){
  //ROS_INFO("CB#2");
	path_down_raw = *msg;
  float cutoff = 5;
  for(int i = 0; i < msg->poses.size(); i++){
   int ci = get_closest_i(path_down_full,msg->poses[i].pose.position);
   if(get_dst2d(path_down_full.poses[ci].pose.position,msg->poses[i].pose.position) > cutoff)
     path_down_full.poses.push_back(msg->poses[i]);
   else if(msg->poses[i].pose.position.z > path_down_full.poses[ci].pose.position.z)
     path_down_full.poses[ci].pose.position.z = msg->poses[i].pose.position.z;
  }
  //ROS_INFO("CB#2");

}
void rawside_cb(const nav_msgs::Path::ConstPtr& msg){
  //ROS_INFO("CB#1");
	path_side_raw = *msg;
  float cutoff = 5;
 for(int i = 0; i < msg->poses.size(); i++){
  if(get_dst3d(path_side_full.poses[get_closest_i(path_side_full,msg->poses[i].pose.position)].pose.position,msg->poses[i].pose.position) > cutoff)
    path_side_full.poses.push_back(msg->poses[i]);
  }
  //ROS_INFO("CB#1");
}
float update_alt_from_vel(geometry_msgs::Point altitude_ref){
  float velxy       = sqrt(pow(odom.twist.twist.linear.x,2)+pow(odom.twist.twist.linear.y,2));
  float dst_2d      = get_dst2d(altitude_ref,pos);
  float dz,inclination,velz,dt,res;
  dz          = (altitude_ref.z + par_zclearing) - pos.z;
  inclination = atan2(dz,dst_2d);

  if(dst_2d > 2){
    velz = velxy * inclination;
  }
  else{
    velz = saturate(velz,3);
  }
  dt          = (ros::Time::now() - t0_vz).toSec();
  res         = pos.z + velz * dt;
  ROS_INFO("Update_alt: vxy: %.2f d2d: %.0f dz: %.2f incl: %.2f vz: %.2f, res: %.0f",velxy,dst_2d,dz,inclination,velz,res);
  return res;
}
float get_altitude_point(){
  geometry_msgs::Point altpnt_frompath = path_down_full.poses[get_closest_i(path_down_full,pos)].pose.position;
  bool quickfix = true;
  if(quickfix){
    return (altpnt_frompath.z + par_zclearing);
  }
  float dst_altpnt = get_dst2d(altpnt_frompath,pos);
  float dst_obspnt = get_dst2d(obs_m,pos);
  float dst_tarpnt = get_dst2d(target.pose.position,pos);

  ROS_INFO("Altitude altpnt_from dst: %.0f xyz: %.0f %.0f %.0f",dst_altpnt,altpnt_frompath.x,altpnt_frompath.y,altpnt_frompath.z);
  ROS_INFO("Altitude point       dst: %.0f xyz: %.0f %.0f %.0f",dst_obspnt,obs_m.x,obs_m.y,obs_m.z);
  ROS_INFO("Altitude target      dst: %.0f xyz: %.0f %.0f %.0f",dst_obspnt,obs_m.x,obs_m.y,obs_m.z);
  float res = update_alt_from_vel(obs_m);
  t0_vz = ros::Time::now();
  return res;
}
void pathvlp_cb(const nav_msgs::Path::ConstPtr& msg){
  path_vlp = *msg;
  if(msg->poses.size() > 0)
    base_pose = msg->poses[0];
}
void pathvlpobs_cb(const nav_msgs::Path::ConstPtr& msg){
  path_obs = *msg;
}
void lowrateodom_cb(const nav_msgs::Odometry::ConstPtr& msg){
  odom = *msg;
}
void poylcleared_cb(const geometry_msgs::PolygonStamped::ConstPtr& msg){
	poly_cleared = *msg;
  poly_cleared_centroid      = get_poly_centroidarea(poly_cleared);
  poly_cleared_centroid_area = poly_cleared_centroid.z;
  poly_cleared_centroid.z    = base_pose.pose.position.z;
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

void eval_tilting_v2(){
  float lowest_dst = 100;
  float lowest_dst_hdng = 0;
  int lowest_dst_i;
  for(int i = 2; i < path_obs.poses.size(); i++){
    float dst  = get_dst3d(path_obs.poses[i].pose.position,path_clear_vlp.poses[0].pose.position);
    float hdng = get_hdng(path_obs.poses[i].pose.position,path_clear_vlp.poses[0].pose.position);
  //  ROS_INFO("dst %.0f hdng: %.2f",dst,hdng);
    if(hdng < 0.05 && hdng > -0.05){
      if(dst < lowest_dst){
        lowest_dst = dst;
        lowest_dst_i = i;
      }
    }
  }
  if(lowest_dst == 100){
    increment_tilt_degrees(5);
  }
  if(lowest_dst < 18){
    increment_tilt_degrees(-5);
  }
  else if(lowest_dst > 22){
    increment_tilt_degrees(5);
  }
}

void draw_poly(geometry_msgs::PolygonStamped polyin,cv::Scalar color){
  geometry_msgs::Point p1,p2;

  for(int i = 1; i < polyin.polygon.points.size(); i++){
    p1.x = polyin.polygon.points[i-1].x;
    p1.y = polyin.polygon.points[i-1].y;
    p2.x = polyin.polygon.points[i].x;
    p2.y = polyin.polygon.points[i].y;
    cv::line (img, pnt2cv(p1), pnt2cv(p2),color,1,cv::LINE_8,0);
  }
  p1.x = polyin.polygon.points[0].x;
  p1.y = polyin.polygon.points[0].y;
  p2.x = polyin.polygon.points[polyin.polygon.points.size()-1].x;
  p2.y = polyin.polygon.points[polyin.polygon.points.size()-1].y;
  cv::line (img, pnt2cv(p1), pnt2cv(p2),color,1,cv::LINE_8,0);
}


geometry_msgs::PolygonStamped get_polyfinal(geometry_msgs::Point startpoint,std::vector<float> vec_r, std::vector<float> vec_a){
  geometry_msgs::PolygonStamped poly;
  poly.header = hdr();
  poly.polygon.points.resize(vec_r.size() * 2);
  for(int i = 0; i < vec_r.size(); i++){
    poly.polygon.points[i].x = startpoint.x + 2 * cos(vec_a[i]);
    poly.polygon.points[i].y = startpoint.y + 2 * sin(vec_a[i]);
    poly.polygon.points[poly.polygon.points.size()-2-i].x = startpoint.x + vec_r[i] * cos(vec_a[i]);
    poly.polygon.points[poly.polygon.points.size()-2-i].y = startpoint.y + vec_r[i] * sin(vec_a[i]);
  }
  poly.polygon.points[poly.polygon.points.size()-1] = poly.polygon.points[0];
  return poly;
}
float vec_to_ave(std::vector<float> vec_in){
  float vec_sum = 0;
  for(int i = 0; i < vec_in.size(); i++){
    vec_sum += vec_in[i];
  }
  return vec_sum / vec_in.size();
}
float vec_to_max(std::vector<float> vec_in){
  float vec_max = -12310;
  for(int i = 0; i < vec_in.size(); i++){
    if(vec_in[i] > vec_max)
     vec_max = vec_in[i];
  }
  return vec_max;
}
float vec_to_min(std::vector<float> vec_in){
  float vec_min = 133330;
  for(int i = 0; i < vec_in.size(); i++){
    if(vec_in[i] < vec_min)
     vec_min = vec_in[i];
  }
  return vec_min;
}
geometry_msgs::Point max_score(std::vector<float> scores){
  geometry_msgs::Point minmax;
  minmax.x = -100;
  minmax.y = 100;
  for(int i = 0; i < scores.size(); i++){
    if(minmax.x < scores[i]){
      minmax.x = scores[i];
    }
    if(minmax.y > scores[i]){
      minmax.y = scores[i];
    }
  }
  return minmax;
}
std::vector<float> eval_one_path(nav_msgs::Path pathin){
  ROS_INFO("************************************");
  std::vector<float> scores;
  bool pose_is_down;
  if(pathin.poses.size() > 0){
    if(pathin.poses[0].pose.orientation.y == 0.7071)
      pose_is_down = true;
  }
  float s1 = get_path_density(pathin);
  float s2 = score_candidates(pathin);
  scores.push_back(s1);
  scores.push_back(s2);
  ROS_INFO("EVAL PATH %i POSES is_down: %i %.2f %.2f",pathin.poses.size(),pose_is_down,s1,s2);
  ROS_INFO("************************************");
  return scores;
}

float rel_score(std::vector<float> scores, float score){
  geometry_msgs::Point minmax = max_score(scores);
  return (score - minmax.y) / (minmax.x - minmax.y);
}

cv::Scalar rel_color(std::vector<float> scores, float score, std::string rgb){
  cv::Scalar c;
  float relscore = rel_score(scores,score);
  if(rgb == "r")
    c[2] = relscore;
  if(rgb == "g")
    c[1] = relscore;
  if(rgb == "b")
    c[0] = relscore;
  return c;
 }

void eval_paths(tb_msgsrv::Paths pathsin){
  std::vector<float> s1,s2;
  for(int i = 0; i < pathsin.paths.size(); i++){
     std::vector<float> out = eval_one_path(pathsin.paths[i]);
     s1.push_back(out[0]);
     s2.push_back(out[1]);
  }
  geometry_msgs::Point offsetpoint,p0;
  for(int i = 0; i < pathsin.paths.size(); i++){
    draw_path_at_img(pathsin.paths[i],p0,false,false,false,true,false,rel_color(s1,s1[i],"r"),1);
    draw_path_at_img(pathsin.paths[i],p0,false,false,true,false,false,rel_color(s2,s2[i],"b"),1);
  }
  cv::imwrite("/home/nuc/brain/"+std::to_string(count_target_paths)+"_eval.png",img);
}

int get_indexes_within_rad2(nav_msgs::Path pathin,float max_dst,geometry_msgs::Point centroid){
  for(int i = 0; i < pathin.poses.size(); i++){
    if(get_dst3d(pathin.poses[i].pose.position,centroid) > max_dst)
      return i;
  }
  return pathin.poses.size();
}
geometry_msgs::Point get_ave_pnt_ni(nav_msgs::Path pathin,int last_i){
  geometry_msgs::Point pnt;
  int n = fmin(last_i,pathin.poses.size());
  for(int i = 0; i < int(n); i++){
    pnt.x += pathin.poses[i].pose.position.x;
    pnt.y += pathin.poses[i].pose.position.y;
    pnt.z += pathin.poses[i].pose.position.z;
  }
  pnt.x /= n;
  pnt.y /= n;
  pnt.z /= n;
  return pnt;
}
float get_zmax_ni(nav_msgs::Path pathin,int last_i){
  float zmx = 0;
  for(int i = 0; i < pathin.poses.size(); i++){
    if(pathin.poses[i].pose.position.z > zmx){
      zmx = pathin.poses[i].pose.position.z;
    }
  }
  return zmx;
}
nav_msgs::Path get_standardized_cluster(nav_msgs::Path pathin){
  nav_msgs::Path pathout;
  pathout.header = hdr();
  pnt_ref       = base_pose.pose.position;
  geometry_msgs::Point ave_pnt;
  int ave_pnt_i,indexes_in_segment;
  while(pathin.poses.size() > 0){
    pathin             = sort_path(pathin,"dst_3d_ref");
    indexes_in_segment = get_indexes_within_rad2(pathin,5,pnt_ref);
    ave_pnt            = get_ave_pnt_ni(pathin,indexes_in_segment);
    float zmx          = get_zmax_ni(pathin,indexes_in_segment);
    ave_pnt_i          = get_closest_i(pathin,ave_pnt);
    pnt_ref            = pathin.poses[ave_pnt_i].pose.position;
    pathout.poses.push_back(pathin.poses[ave_pnt_i]);
    if(indexes_in_segment >= pathin.poses.size())
      break;
    else
      pathin = remove_nth_first(pathin,indexes_in_segment);
  }
  return pathout;
}

tb_msgsrv::Paths organize_clusters(tb_msgsrv::Paths clusters_in){
	tb_msgsrv::Paths clusters_standardized;
	clusters_standardized.paths.resize(clusters_in.paths.size());
	for(int i= 0; i < clusters_in.paths.size(); i++){
		clusters_standardized.paths[i] = get_standardized_cluster(clusters_in.paths[i]);//get_standardized_cluster(clusters_in.paths[i]);
	}
	return clusters_standardized;
}
void superpathdown_cb(const tb_msgsrv::Paths::ConstPtr& msg){
  paths_down = *msg;
  for(int i = 0; i< paths_down.paths.size(); i++){
    ROS_INFO("PATHS_DOWN: %i paths,%i poses",paths_down.paths.size(),paths_down.paths[i].poses.size());
  }
  paths_active_down = organize_clusters(*msg);
  eval_paths(*msg);
}

void superpathside_cb(const tb_msgsrv::Paths::ConstPtr& msg){
  paths_side = *msg;
  for(int i = 0; i< paths_side.paths.size(); i++){
    ROS_INFO("PATHS_SIDE: %i paths,%i poses",paths_side.paths.size(),paths_side.paths[i].poses.size());
  }
  paths_active_side = organize_clusters(*msg);
  eval_paths(*msg);
}
void closest_pos_cb(const geometry_msgs::PointStamped::ConstPtr& msg){
  obs_p = msg->point;
  if(msg->header.frame_id != "map"){
    obs_p.z = -100;
    obs_p.z = pos.z - par_zclearing;
  }
}
void closest_left_cb(const geometry_msgs::PointStamped::ConstPtr& msg){
  obs_l = msg->point;
  if(msg->header.frame_id != "map")
    obs_l.z = -100;
}
void closest_right_cb(const geometry_msgs::PointStamped::ConstPtr& msg){
  obs_r = msg->point;
  if(msg->header.frame_id != "map")
    obs_r.z = -100;
}
void closest_mid_cb(const geometry_msgs::PointStamped::ConstPtr& msg){
  obs_m = msg->point;
  if(msg->header.frame_id == "map")
    obs_m.z = -100;
}
float get_slope(geometry_msgs::Point p1, geometry_msgs::Point p2){
  return (p2.z - p1.z) / get_dst2d(p1,p2);
}
float get_inclination(geometry_msgs::Point p1, geometry_msgs::Point p2){
  return atan2(p2.z - p1.z,get_dst2d(p1,p2));
}



float get_a_from_dz_r(float dz, float r){
  return asin(dz/r);
}
float get_a_from_dxy_r(float dxy, float r){
  return acos(dxy/r);
}
float get_dz_from_dxy_a(float dxy, float r){
  return acos(dxy/r);
}
float get_dz_from_r_a(float a, float r){
  return r * sin(a);
}
float get_dxy_from_r_a(float a, float r){
  return r * cos(a);
}
/*
void calibrate_tilt(){
  float a = M_PI/8;
  float target_alt = target_range * sin(a);
  float target_dxy = target_range * cos(a);
  float a  = asin(target_alt/target_range);
  float target_range = asin(a/target_range);
  float dst_dm  = get_dst2d(pnt_dm,pos);
  float dst_tar = get_dst2d(target.pose.position,pos);
  float dz      = pnt_dm.z - pos.z;
}*/
/*
ROS_INFO("CURRENT_MODE");
if(mode == "above_plane"){
  setpoint_scanrange = 25;
  setpoint_zclearing = 4;
  target_zmin =
}
if(mode == "side_track"){
  setpoint_zclearing = 0;
  setpoint_scanrange = 10;
  exit_condition = "no_sides";
}
*/
bool use_side(nav_msgs::Path path_side, nav_msgs::Path path_down){
  int scnt = path_side.poses.size();
  int dcnt = path_down.poses.size();
  geometry_msgs::Point pnt_s = get_ave_pnt(path_side);
  geometry_msgs::Point pnt_d = get_ave_pnt(path_down);

  float dst_pnt_dr = get_dst3d(pnt_d,pos);
  float dst_pnt_sm = get_dst3d(pnt_s,pos);

  float zmax_d = get_zmax(path_down);
  float zmax_s = get_zmax(path_side);
  if(scnt > dcnt && pnt_s.z > pnt_d.z)
    return true;
//  else if(dst_pnt_sm < dst_pnt_dr && pnt_s.z > pnt_d.z)
  //  return true;
  else
    return false;
}
void rayranges_cb(const sensor_msgs::LaserScan::ConstPtr& msg){
  ROS_INFO("Rayranges CALLBACK");

  int cnt;
  img_blank.copyTo(img);
  geometry_msgs::PolygonStamped polyleft,polymid,polyright,polyfull,polyfull_max;
  geometry_msgs::PolygonStamped poly,polyleft_max,polymid_max,polyright_max;
  geometry_msgs::Point32 p0,pn,pm;
  p0.x = base_pose.pose.position.x;
  p0.y = base_pose.pose.position.y;
  polyleft.polygon.points.push_back(p0);
  polymid.polygon.points.push_back(p0);
  polyright.polygon.points.push_back(p0);
  polyleft_max.polygon.points.push_back(p0);
  polymid_max.polygon.points.push_back(p0);
  polyright_max.polygon.points.push_back(p0);
  float rel_len_sum;
  float r_sum_left = 0;
  float r_sum_mid = 0;
  float r_sum_right = 0;
  float angle_max = msg->angle_max;
  float angle_mid = angle_max / 2;
  poly.polygon.points.resize(msg->ranges.size());
  bool i_inactive;
  int start_i,end_i;
  end_i = 0;
  start_i = 0;
  std::vector<int> vec_i;

  for(int i = 0; i < msg->ranges.size(); i++){
    float a = msg->angle_min + msg->angle_increment * i;
    float r = msg->ranges[i];
    if(start_i == 0 && msg->ranges[i] < 2 && msg->ranges[i+1] > 2){
      start_i = i+1;
      ROS_INFO(" ranges[%i] a: %.3f r: %.0f start_i: %i",i,a,r,start_i);
    }
    else if(end_i == 0 && msg->ranges[i] > 2 && msg->ranges[i+1] < 2){
      end_i = i;
     ROS_INFO(" ranges[%i] a: %.3f r: %.0f end_i: %i",i,a,r,end_i);
    }
    else{
     ROS_INFO(" ranges[%i] a: %.3f r: %.0f i[%i]->i[%i]",i,a,r,start_i,end_i);
    }
  }
  if(end_i < start_i){
    for(int i = start_i; i < msg->ranges.size(); i++){
      vec_i.push_back(i);
    }
    for(int i = 0; i < end_i; i++){
      vec_i.push_back(i);
    }
  }
  else{
    for(int i = start_i; i < end_i; i++){
      vec_i.push_back(i);
    }
  }
  std::vector<float> l_ranges;
  std::vector<float> l_hdngs;
  std::vector<float> m_ranges;
  std::vector<float> m_hdngs;
  std::vector<float> r_ranges;
  std::vector<float> r_hdngs;
  std::vector<float> ranges;
  std::vector<float> hdngs;
  std::vector<float> ranges_tot;
  std::vector<float> hdngs_tot;

  int veci_1 = round((vec_i.size())/3);
  ROS_INFO("veci_1. %i of %i",veci_1,vec_i.size());
  if(veci_1 == 0)
    return;
  for(int i = 0; i < vec_i.size(); i++){
    float a = msg->angle_min + msg->angle_increment * vec_i[i];
    float r = msg->ranges[vec_i[i]];
    ranges.push_back(r);
    hdngs.push_back(a);
    ranges_tot.push_back(r);
    hdngs_tot.push_back(a);
    if(i == veci_1){
      l_ranges = ranges;
      l_hdngs = hdngs;
      ranges.resize(0);
      hdngs.resize(0);
      ranges.push_back(r);
      hdngs.push_back(a);
    }
    if(i == veci_1*2){
      m_ranges = ranges;
      m_hdngs = hdngs;
      ranges.resize(0);
      hdngs.resize(0);
      ranges.push_back(r);
      hdngs.push_back(a);
    }
  }
  r_ranges = ranges;
  r_hdngs  = hdngs;

  float amid = hdngs_tot[hdngs_tot.size()/2];
  geometry_msgs::Point startpoint;
  startpoint.x = base_pose.pose.position.x; // + cos(hdngs_tot[hdngs_tot.size()/2]);
  startpoint.y = base_pose.pose.position.y; // + sin(hdngs_tot[hdngs_tot.size()/2]);
  float da_l = l_hdngs[l_hdngs.size()-1] - l_hdngs[0];
  float da_m = m_hdngs[m_hdngs.size()-1] - m_hdngs[0];
  float da_r = r_hdngs[r_hdngs.size()-1] - r_hdngs[0];
  float da   = hdngs_tot[hdngs.size()-1] - hdngs_tot[0];


  //ROS_INFO("hdngs0N: %i - %.2f - %.2f rads_da: %.3f",hdngs_tot.size(),hdngs_tot[0],hdngs_tot[hdngs_tot.size()-1],da);

  polyleft     = get_polyfinal(startpoint,l_ranges,l_hdngs);
  polymid      = get_polyfinal(startpoint,m_ranges,m_hdngs);
  polyright    = get_polyfinal(startpoint,r_ranges,r_hdngs);
  polyfull_max = get_polyfinal(startpoint,ranges_tot,hdngs_tot);

  geometry_msgs::Point cen_l = get_poly_centroidarea(polyleft);
  geometry_msgs::Point cen_m = get_poly_centroidarea(polymid);
  geometry_msgs::Point cen_r = get_poly_centroidarea(polyright);
  geometry_msgs::Point cen_a = get_poly_centroidarea(polyfull_max);

  float l_ave = vec_to_ave(l_ranges);
  float l_max = vec_to_max(l_ranges);
  float l_min = vec_to_min(l_ranges);
  float m_ave = vec_to_ave(m_ranges);
  float m_max = vec_to_max(m_ranges);
  float m_min = vec_to_min(m_ranges);
  float r_ave = vec_to_ave(r_ranges);
  float r_max = vec_to_max(r_ranges);
  float r_min = vec_to_min(r_ranges);

  float l_area = abs(cen_l.z);
  float m_area = abs(cen_m.z);
  float r_area = abs(cen_r.z);
  float a_area = abs(cen_a.z);

  cen_l.z = base_pose.pose.position.z;
  cen_m.z = base_pose.pose.position.z;
  cen_r.z = base_pose.pose.position.z;

  nav_msgs::Path path_obs_l = constrain_path_bbpoly(path_obs,polyleft);
  nav_msgs::Path path_obs_m = constrain_path_bbpoly(path_obs,polymid);
  nav_msgs::Path path_obs_r = constrain_path_bbpoly(path_obs,polyright);

  nav_msgs::Path path_clear_l = constrain_path_bbpoly(path_vlp,polyleft);
  nav_msgs::Path path_clear_m = constrain_path_bbpoly(path_vlp,polymid);
  nav_msgs::Path path_clear_r = constrain_path_bbpoly(path_vlp,polyright);

	nav_msgs::Path path_side_l = constrain_path_bbpoly(path_side_raw,polyleft);
  nav_msgs::Path path_side_m = constrain_path_bbpoly(path_side_raw,polymid);
  nav_msgs::Path path_side_r = constrain_path_bbpoly(path_side_raw,polyright);

  nav_msgs::Path path_down_l = constrain_path_bbpoly(path_down_raw,polyleft);
  nav_msgs::Path path_down_m = constrain_path_bbpoly(path_down_raw,polymid);
  nav_msgs::Path path_down_r = constrain_path_bbpoly(path_down_raw,polyright);

	geometry_msgs::Point pnt_sm = get_ave_pnt(path_side_m);
	geometry_msgs::Point pnt_sr = get_ave_pnt(path_side_r);
	geometry_msgs::Point pnt_sl = get_ave_pnt(path_side_l);

	geometry_msgs::Point pnt_dm = get_ave_pnt(path_down_m);
	geometry_msgs::Point pnt_dr = get_ave_pnt(path_down_r);
	geometry_msgs::Point pnt_dl = get_ave_pnt(path_down_l);

  float dst_pnt_sr = get_dst3d(pnt_sr,pos);
  float dst_pnt_sl = get_dst3d(pnt_sl,pos);
  float dst_pnt_dr = get_dst3d(pnt_dr,pos);

  float dst_pnt_dl = get_dst3d(pnt_dl,pos);
  float dst_pnt_sm = get_dst3d(pnt_sm,pos);
  float dst_pnt_dm = get_dst3d(pnt_dm,pos);

  float zmax_obs_l = get_zmax(path_obs_l);
  float zmax_obs_m = get_zmax(path_obs_m);
  float zmax_obs_r = get_zmax(path_obs_r);

	float zmax_dl = get_zmax(path_down_l);
  float zmax_dm = get_zmax(path_down_m);
  float zmax_dr = get_zmax(path_down_r);

	float zmax_sl = get_zmax(path_side_m);
  float zmax_sm = get_zmax(path_side_r);
  float zmax_sr = get_zmax(path_side_l);

  ROS_INFO("Left-angle:  %i - %.2f - %.2f rads_da: %.3f",l_hdngs.size(),l_hdngs[0],l_hdngs[l_hdngs.size()-1],da_l);
  ROS_INFO("Mid-angle:   %i - %.2f - %.2f rads_da: %.3f",m_hdngs.size(),m_hdngs[0],m_hdngs[m_hdngs.size()-1],da_m);
  ROS_INFO("Right-angle: %i - %.2f - %.2f rads_da: %.3f",r_hdngs.size(),r_hdngs[0],r_hdngs[r_hdngs.size()-1],da_r);

  ROS_INFO("Left-range:  %.0f->%.0f ave: %.0f area: %.0f (%.0f tot)",l_min,l_max,l_ave,l_area,a_area);
  ROS_INFO("Mid-range:   %.0f->%.0f ave: %.0f area: %.0f (%.0f tot)",m_min,m_max,m_ave,m_area,a_area);
  ROS_INFO("Right-range: %.0f->%.0f ave: %.0f area: %.0f (%.0f tot)",r_min,r_max,r_ave,r_area,a_area);

  ROS_INFO("Left-obs/clr:  [%i/%i] - side/down[%i/%i] zmax_obs: %.0f",path_obs_l.poses.size(),path_clear_l.poses.size(),path_side_l.poses.size(),path_down_l.poses.size(),zmax_obs_l);
  ROS_INFO("Mid-obs/clr:   [%i/%i] - side/down[%i/%i] zmax_obs: %.0f",path_obs_m.poses.size(),path_clear_m.poses.size(),path_side_m.poses.size(),path_down_m.poses.size(),zmax_obs_m);
  ROS_INFO("Right-obs/clr: [%i/%i] - side/down[%i/%i] zmax_obs: %.0f",path_obs_r.poses.size(),path_clear_r.poses.size(),path_side_r.poses.size(),path_down_r.poses.size(),zmax_obs_r);

	int smcnt = path_side_m.poses.size();
	int srcnt = path_side_r.poses.size();
	int slcnt = path_side_l.poses.size();

	int dmcnt = path_down_m.poses.size();
	int drcnt = path_down_r.poses.size();
	int dlcnt = path_down_l.poses.size();

  int cnt_l = (slcnt + dlcnt);
  int cnt_r = (srcnt + drcnt);
  int cnt_m = (smcnt + dmcnt);

  nav_msgs::Path path_s,path_d;

  bool abandon_mid = false;
  bool got_no_targets = false;
  bool is_cramped = false;
  if(cnt_l < 5 && cnt_r < 5 && cnt_m < 5)
    got_no_targets = true;
  else if(cnt_l > (cnt_r + cnt_m)){
    abandon_mid = true;
    path_s = path_side_l;
    path_d = path_down_l;
  }
  else if(cnt_r > (cnt_l + cnt_m)){
    abandon_mid = true;
    path_s = path_side_r;
    path_d = path_down_r;
  }
  else{
    path_s = path_side_m;
    path_d = path_down_m;
    abandon_mid = false;
  }

  ROS_INFO("Poses %i / %i / %i - abandon_mid: %i got_no_targets: %i",cnt_l,cnt_m,cnt_r,abandon_mid,got_no_targets);

  float ls_ave   = (l_ave + m_ave + r_ave)/3;
  float area_sum = (l_area+m_area+r_area);
  float area_rel = float(area_sum)/float(a_area);
  if(fmin(ls_ave,m_ave) < 10){
    ROS_INFO("RANGE_AVE: %.2f < 10 - is_cramped",ls_ave);
    is_cramped = true;
  }
  if(area_rel < 0.2){
    ROS_INFO("AREA_REL: %.2f < 0.2 - is_cramped",area_rel);
    is_cramped = true;
  }

  ROS_INFO("Area [%.0f of total %.0f: -REL: %.2f] is_cramped: %i",area_sum,a_area,area_rel,ls_ave,is_cramped);

  ROS_INFO("MID   MD: [%.0f %.0f %.0f] MS: [%.0f %.0f %.0f] DST MD: [%.0f] MS: [%.0f]",pnt_dm.x,pnt_dm.y,pnt_dm.z,pnt_sm.x,pnt_sm.y,pnt_sm.z,dst_pnt_dm,dst_pnt_sm);
  ROS_INFO("LEFT  MD: [%.0f %.0f %.0f] MS: [%.0f %.0f %.0f] DST MD: [%.0f] MS: [%.0f]",pnt_dl.x,pnt_dl.y,pnt_dl.z,pnt_sl.x,pnt_sl.y,pnt_sl.z,dst_pnt_dl,dst_pnt_sl);
  ROS_INFO("RIGHT MD: [%.0f %.0f %.0f] MS: [%.0f %.0f %.0f] DST MD: [%.0f] MS: [%.0f]",pnt_dr.x,pnt_dr.y,pnt_dr.z,pnt_sr.x,pnt_sr.y,pnt_sr.z,dst_pnt_dr,dst_pnt_sr);

  ROS_INFO("OBS   P0: [%.0f %.0f %.0f]  M: [%.0f %.0f %.0f] R: [%.0f %.0f %.0f] L: [%.0f %.0f %.0f]",obs_p.x,obs_p.y,obs_p.z,obs_m.x,obs_m.y,obs_m.z,obs_r.x,obs_r.y,obs_r.z,obs_l.x,obs_l.y,obs_l.z);

  geometry_msgs::PoseStamped ps;
  if(is_cramped){
    ROS_INFO("ACTION - IS CRAMPED !");
    if(arm1_tilt_msg.data > 0)
      set_tilt(0);
    else if(arm1_tilt_msg.data > -M_PI/6)
      increment_tilt_degrees(-5);
    else
      par_zclearing += 2;
  }
  else if(got_no_targets){
    ROS_INFO("ACTION - NO TARGETS !");
    if(arm1_tilt_msg.data < 0)
      set_tilt(0);
    else if(arm1_tilt_msg.data < M_PI/6)
      increment_tilt_degrees(5);
    else
      par_zclearing -= 2;
  }
  else{
    if(m_ave > 24 || area_rel > 0.85){
      ROS_INFO("IncrementingTilt ls_ave %.0f > 24 || area_rel %.2f > 0.85",ls_ave,area_rel);
      increment_tilt_degrees(5);
    }
    else if(m_ave < 10 || area_rel < 0.4){
      ROS_INFO("DecrementingTilt ls_ave %.0f > 24 || area_rel %.2f > 0.85",ls_ave,area_rel);
      increment_tilt_degrees(-5);
    }
  }

  if(use_side(path_s,path_d)){
    geometry_msgs::Point pnt_s = get_ave_pnt(path_s);
    ps = path_s.poses[get_closest_i(path_s,pos)];
  }
  else{
    ps.pose.position = get_ave_pnt(path_d);
  }
  if(std::isnan(ps.pose.position.x))
    ROS_INFO("NANWARN");
  else
    set_target_pose(ps);

    /*
  //else if(got_clusters){
  //  get_highest_value_cluster();
  //}
  if(abandon_mid){
    ROS_INFO("ACTION - ABANDON MID !");

    if(obs_l.z > 0 && obs_r.z <= 0)
      ps.pose.position = obs_l;
    else if(obs_r.z > 0 && obs_l.z <= 0)
      ps.pose.position = obs_r;
    else if(obs_r.z > 0 && obs_l.z > 0){
      if(cnt_l > cnt_r)
        ps.pose.position = obs_l;
      else
        ps.pose.position = obs_r;
    }
    else{
      if(cnt_l > cnt_r){
        if(dst_pnt_sl > dst_pnt_dl)
          ps.pose.position = pnt_sl;
        else
          ps.pose.position = pnt_dl;
      }
      else{
        if(dst_pnt_sr > dst_pnt_dr)
          ps.pose.position = pnt_sr;
        else
          ps.pose.position = pnt_dr;
      }
    }
    ROS_INFO("abandon_mid: COUNT(d/s): R: (%i/%i) S: (%i/%i)  DST(d/s) R: (%.0f/%.0f) S: (%.0f/%.0f) OBS: R: [%.0f %.0f %.0f] L: [%.0f %.0f %.0f]",dlcnt,slcnt,drcnt,dlcnt,dst_pnt_dl,dst_pnt_sl,dst_pnt_dr,dst_pnt_sr,obs_r.x,obs_r.y,obs_r.z,obs_l.x,obs_l.y,obs_l.z);
    ps.pose.orientation = tf::createQuaternionMsgFromYaw(get_hdng(ps.pose.position,pos));
    set_target_pose(ps);
  }
  else{
    ROS_INFO("ACTION - MID - SIDE VS DOWN !");

    if(dst_pnt_dm > dst_pnt_sm)
      ps.pose.position = pnt_sm;
    else
      ps.pose.position = pnt_dm;
    ROS_INFO("abandon_mid: COUNT(d/s): R: (%i/%i) S: (%i/%i)  DST(d/s) R: (%.0f/%.0f) S: (%.0f/%.0f) OBS: R: [%.0f %.0f %.0f] L: [%.0f %.0f %.0f]",dlcnt,slcnt,drcnt,dlcnt,dst_pnt_dl,dst_pnt_sl,dst_pnt_dr,dst_pnt_sr,obs_r.x,obs_r.y,obs_r.z,obs_l.x,obs_l.y,obs_l.z);
    ps.pose.orientation = tf::createQuaternionMsgFromYaw(get_hdng(ps.pose.position,pos));
    set_target_pose(ps);
  }*/
  bool got_sm,got_sr,got_sl,got_dm,got_dr,got_dl;

	bool draw_stuff = true;
  if(draw_stuff){
	  cv::Scalar cl,cm,cr,c,co,cc,c2,cdown,cside,ctar;
		ctar[0] = 100;
		ctar[2] = 255;
	  cl[0] = 100;
	  cm[1] = 100;
		cr[2] = 100;
		cdown[2] = 100;
		cdown[1] = 100;
		cside[2] = 100;
		cside[1] = 100;
	  c[2] = 100;
	  c[0] = 100;
		c2[2] = 150;
		c2[0] = 150;
	  co[2] = 150;
	  co[0] = 50;
	  cc[1] = 150;
	  cc[0] = 50;

	  draw_path_at_img(path_obs_l,cen_l,false,false,false,true,false,co,2);
	  draw_path_at_img(path_obs_m,cen_m,false,false,false,true,false,co,2);
	  draw_path_at_img(path_obs_r,cen_r,false,false,false,true,false,co,2);

	  draw_path_at_img(path_clear_l,cen_l,false,false,false,false,true,cc,1);
	  draw_path_at_img(path_clear_m,cen_m,false,false,false,false,true,cc,1);
	  draw_path_at_img(path_clear_r,cen_r,false,false,false,false,true,cc,1);

		draw_path_at_img(path_down_best,cen_r,true,false,false,true,false,cc,1);
		draw_path_at_img(path_side_best,cen_r,true,true,true,false,true,cc,1);

		if(got_dm)
	   cv::circle(img,pnt2cv(pnt_dm),2,cm,1);
	 if(got_dr)
	   cv::circle(img,pnt2cv(pnt_dr),2,cr,1);
	 if(got_dl)
	   cv::circle(img,pnt2cv(pnt_dl),2,cl,1);
	 if(got_sm)
	 	cv::circle(img,pnt2cv(pnt_sm),2,cm,1);
	 if(got_sr)
	 	cv::circle(img,pnt2cv(pnt_sr),2,cr,1);
	 if(got_sl)
	 	cv::circle(img,pnt2cv(pnt_sl),2,cl,1);

	  cv::circle(img,pnt2cv(cen_l),2,cl,1);
	  cv::circle(img,pnt2cv(cen_m),2,cm,1);
	  cv::circle(img,pnt2cv(cen_r),2,cr,1);

	  draw_poly(polyfull_max,c);
	  draw_poly(polyleft,cl);
	  draw_poly(polymid,cm);
		draw_poly(polyright,cr);
		draw_poly(poly_cleared,c2);
		draw_pose(target.pose.position,get_hdng(target.pose.position,base_pose.pose.position),3,ctar);

	  cv::imwrite("/home/nuc/brain/"+std::to_string(count_target_paths)+"clear.png",img);
	}
}

void get(){
  ROS_INFO("Finding target");
  //img_blank.copyTo(img);

  for(int i = 0; i < path_down_best.poses.size(); i++){
    cv::Scalar color;
    color[0] = 50;
    color[1] = 50;
    cv::circle(img,pnt2cv(path_down_best.poses[i].pose.position),1,color,1);
  }
  for(int i = 0; i < path_vlp.poses.size(); i++){
    int ci = get_closest_i(path_down_full,path_vlp.poses[i].pose.position);
    int cis = get_closest_i(path_side_full,path_vlp.poses[i].pose.position);
    float dst_ci = get_dst2d(path_down_full.poses[ci].pose.position,path_vlp.poses[i].pose.position);
    float dst_cis = get_dst2d(path_down_full.poses[cis].pose.position,path_vlp.poses[i].pose.position);
    if(dst_ci < 6 && ci < path_down_full.poses.size()){
      cv::Scalar color;
      color[2] = 5*path_down_full.poses[ci].pose.position.z;
      cv::circle(img,pnt2cv(path_down_full.poses[ci].pose.position),2,color,1);
    }
    if(dst_cis < 6 && cis < path_side_full.poses.size()){
      cv::Scalar color;
      color[0] = 100;
      draw_pose(path_side_full.poses[cis].pose.position,tf::getYaw(path_side_full.poses[cis].pose.orientation),2,color);
    }
  }
  cv::Scalar color;
  color[2] = 200;
  color[1] = 200;
  color[0] = 200;
  geometry_msgs::PoseStamped ps;
  ps.pose = path_down_full.poses[get_closest_i(path_down_full,get_ave_pnt(path_down_best))].pose;
  ps.pose.position.z += 4;
  draw_pose(ps.pose.position,get_hdng(ps.pose.position,base_pose.pose.position),3,color);
  color[0] = 0;
  color[1] = 0;
  color[2] = 200;
  draw_poly(poly_cleared,color);
  count_target_paths++;
  cv::imwrite("/home/nuc/brain/"+std::to_string(count_target_paths)+"_polys.png",img);
  ROS_INFO("target found %.0f %.0f %.0f",ps.pose.position.x,ps.pose.position.y,ps.pose.position.z);
  set_target_pose(ps);
}
int main(int argc, char** argv)
{
  ros::init(argc, argv, "tb_behavior_node");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");
  private_nh.param("par_zclearing",par_zclearing, 5.0);
  private_nh.param("par_vlpmaxtilt",par_vlpmaxtilt, M_PI/5);
	private_nh.param("par_vlpmintilt",par_vlpmintilt, -M_PI/5);
	private_nh.param("par_vlptiltinterval",par_vlptiltinterval, M_PI/10);
	private_nh.param("takeoff_altlvl",par_takeoffaltitude, 5.0);
	tf2_ros::TransformListener tf2_listener(tfBuffer);
	std_msgs::Float64 target_alt_msg;
  path_world_visible.header  = hdr();
	path_cleared_full.header   = hdr();
	path_obstacles_full.header = hdr();
	path_side_full.header 		 = hdr();
  path_down_full.header      = hdr();
  path_targets.header      = hdr();
	target.pose.position.z 	   = par_takeoffaltitude;
	target.pose.orientation.w  = 1;
  path_targets_i = 0;
  path_side_full.poses.push_back(target);
  path_down_full.poses.push_back(target);
  path_targets.poses.push_back(target);
  path_targets.poses[0].pose.position.z = par_takeoffaltitude / 2;
  path_targets.poses.push_back(target);
  ros::Publisher pos_closest_pos   = nh.advertise<geometry_msgs::PointStamped>("//",100);
  ros::Publisher pos_closest_left  = nh.advertise<geometry_msgs::PointStamped>("//",100);
  ros::Publisher pos_closest_right = nh.advertise<geometry_msgs::PointStamped>("//",100);
  ros::Publisher pos_closest_mid   = nh.advertise<geometry_msgs::PointStamped>("//",100);
  ros::Subscriber s0 = nh.subscribe("/tb_path/cleared_poly",1,poylcleared_cb);
  ros::Subscriber s1 = nh.subscribe("/tb_path/cleared",1,pathvlp_cb);
  ros::Subscriber s2 = nh.subscribe("/tb_path/obstacles",1,pathvlpobs_cb);
  ros::Subscriber s3 = nh.subscribe("/tb_path/superpath_side",10,superpathside_cb);
  ros::Subscriber s4 = nh.subscribe("/tb_path/superpath_down",10,superpathdown_cb);
  ros::Subscriber as3 = nh.subscribe("/tb_path/raw_down",10,rawdown_cb);
  ros::Subscriber as5 = nh.subscribe("/tb_path/raw_side",10,rawside_cb);
  ros::Subscriber as4 = nh.subscribe("/tb_path/ranges_cleared",10,rayranges_cb);
  ros::Subscriber asb4 = nh.subscribe("/tb_obs/closest_pos",10,closest_pos_cb);
  ros::Subscriber asbb4 = nh.subscribe("/tb_obs/closest_left",10,closest_left_cb);
  ros::Subscriber assaa4 = nh.subscribe("/tb_obs/closest_right",10,closest_right_cb);
  ros::Subscriber assa4 = nh.subscribe("/tb_obs/closest_mid",10,closest_mid_cb);

  ros::Subscriber s5 = nh.subscribe("/tb_path/visited",10,pathvstd_cb);

  ros::Subscriber s8 = nh.subscribe("/tb_fsm/main_state",100,&mainstate_cb);
  ros::Subscriber s9 = nh.subscribe("/tb_nav/lowrate_odom",10,lowrateodom_cb);
  ros::Subscriber s19 = nh.subscribe("/tb_path/hdng_clear",10,hdngclear_cb);

  ros::Publisher pub_path                = nh.advertise<nav_msgs::Path>("/tb_behav/targets",10);

  ros::Publisher pub_path_down_best      = nh.advertise<nav_msgs::Path>("/tb_behav/down_best",10);
  ros::Publisher pub_path_side_best      = nh.advertise<nav_msgs::Path>("/tb_behav/side_best",10);
  ros::Publisher pub_path_down_full      = nh.advertise<nav_msgs::Path>("/tb_behav/down_full",10);
  ros::Publisher pub_path_side_full      = nh.advertise<nav_msgs::Path>("/tb_behav/side_full",10);

  ros::Publisher pub_path_target_full    = nh.advertise<nav_msgs::Path>("/tb_behav/full",10);

	ros::Publisher pub_altcmd	 = nh.advertise<std_msgs::Float64>("/tb_cmd/alt_target", 100);
	ros::Publisher pub_tiltvlp = nh.advertise<std_msgs::Float64>("/tb_cmd/arm1_tilt", 10);
	ros::Publisher pub_tilt  	 = nh.advertise<std_msgs::Float64>("/tb_cmd/arm2_tilt", 10);
	ros::Publisher pub_pan	   = nh.advertise<std_msgs::Float64>("/tb_cmd/arm2_pan", 10);
  ros::Publisher pub_target_pose = nh.advertise<geometry_msgs::PoseStamped>("/cmd_pose",10);

	pub_get_next_path	 	= nh.advertise<std_msgs::UInt8>("/tb_path/request", 100);
  pub_cmd      				= nh.advertise<geometry_msgs::Point>("/cmd_pos",10);
  pub_cmdmb   				= nh.advertise<geometry_msgs::PoseStamped>("/tb_cmd/posemb",10);
  pub_cmdexplore			= nh.advertise<geometry_msgs::PolygonStamped>("/tb_cmd/poly_explore",10);
	ros::Rate rate(2.0);
	int count = 0;
	int idle_count = 0;
	std::string last_type;
	bool evaluation_started,evaluation_complete;
	int evaluation_stage = 0;
  int dashnumber = 0;
	std::vector<int> tilts_to_evaluate;
  path_target_full = create_path();
  path_target_full.header = hdr();
  ros::Time start = ros::Time::now();

  while(ros::ok()){
    float dst_target = get_dst3d(target.pose.position,pos);
    ROS_INFO("Inspection type: %s target_dst: %.2f ",inspection_type.c_str(),dst_target);
    checktf();
    request_paths();
    count_target_paths++;

    pub_path_target_full.publish(path_target_full);
    if((ros::Time::now() - start).toSec() < 5)
      target_alt_msg.data = par_takeoffaltitude;
    else
      target_alt_msg.data = get_altitude_point();
    pub_target_pose.publish(target);
  //  pub_tiltvlp.publish(arm1_tilt_msg);
  //  pub_altcmd.publish(target_alt_msg);
    pub_path.publish(path_targets);

    path_down_best.header = hdr();
    path_side_best.header = hdr();

    pub_path_down_best.publish(path_down_best);
    pub_path_side_best.publish(path_side_best);
    pub_path_down_full.publish(path_down_full);
    pub_path_side_full.publish(path_side_full);
		pub_target_pose.publish(target);
    rate.sleep();
    ros::spinOnce();
  }
  return 0;
}
