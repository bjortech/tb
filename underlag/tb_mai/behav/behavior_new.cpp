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

ros::Publisher pub_get_next_path,pub_cmd;
geometry_msgs::PoseStamped target,base_pose,target_last;
ros::Time activity_change,last_tilt,path_complete_time;
std_msgs::Float64 arm1_tilt_msg;
geometry_msgs::Vector3 vlp_rpy;
geometry_msgs::Point pos,pnt_midpoint,pnt_ref;
nav_msgs::Odometry odom;
nav_msgs::Path path_world_visible,path_down_best,path_vlp,path_obs,path_side_full,path_down_full,path_targets,path_visited,path_cleared_full,path_obstacles_full,path_targets_sent,path_side,path_down;
int mainstate;
int blankdraw_counter = 0;
int path_targets_i = 0;
int inspection_count = 0;
double par_vlpmaxtilt,par_vlpmintilt,par_vlptiltinterval,par_takeoffaltitude;
bool path_complete,tiltlimit_reached,tilting,side_empty;
cv::Mat img(1000,1000,CV_8UC3,cv::Scalar(0, 0, 0)); //create image, set encoding and size, init pixels to default val
cv::Mat img_blank(1000,1000,CV_8UC3,cv::Scalar(0, 0, 0)); //create image, set encoding and size, init pixels to default val
float rad2deg = 180.0/M_PI;
int count_target_paths = 0;
std::vector<int> targets_complete;
tb_msgsrv::Paths paths_active_down;
tb_msgsrv::Paths paths_active_side;
geometry_msgs::PolygonStamped poly_cleared;
tb_msgsrv::Paths paths_candidates;
bool path_side_requested,path_down_requested,path_side_received,path_down_received;

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

geometry_msgs::Point32 get_poly_centroidarea(geometry_msgs::PolygonStamped polyin){
    geometry_msgs::Point32 centroid;
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
std_msgs::Header hdr(){
  std_msgs::Header header;
  header.frame_id = "map";
  header.stamp    = ros::Time::now();
  return header;
}

double get_shortest(double target_hdng,double actual_hdng){
  double a = target_hdng - actual_hdng;
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

tb_msgsrv::Paths sort_paths_in_vector(tb_msgsrv::Paths path_vector,std::string sort_by){
  for(int i = 0; i < path_vector.paths.size(); i++){
    path_vector.paths[i] = sort_path(path_vector.paths[i],sort_by);
  }
  return path_vector;
}
tb_msgsrv::Paths paths_segmented_hdng(nav_msgs::Path pathin){
  tb_msgsrv::Paths path_vector;
  nav_msgs::Path current_pathsegment;
  current_pathsegment.header = pathin.header;
  nav_msgs::Path path = sort_path(pathin,"hdng");
  float current_heading = get_hdng(path.poses[0].pose.position,base_pose.pose.position);
  for(int i = 1; i < path.poses.size(); i++){
    float candidate_heading = get_hdng(path.poses[i].pose.position,base_pose.pose.position);
    float delta_hdng        = get_shortest(candidate_heading,current_heading);
    int delta_hdng_degs     = abs(delta_hdng * rad2deg);
    if(delta_hdng_degs > 5){
      current_heading = candidate_heading;
      ROS_INFO("pathindex cutoff after %i indexes",current_pathsegment.poses.size());
      path_vector.paths.push_back(current_pathsegment);
      current_pathsegment.poses.resize(0);
    }
    current_pathsegment.poses.push_back(path.poses[i]);
  }
  return path_vector;
}
tb_msgsrv::Paths paths_segmented_z(nav_msgs::Path pathin){
  tb_msgsrv::Paths path_vector;
  nav_msgs::Path current_pathsegment;
  current_pathsegment.header = pathin.header;
  nav_msgs::Path path = sort_path(pathin,"z");
  float  current_z = path.poses[0].pose.position.z;
  for(int i = 0; i < path.poses.size(); i++){
    float candidate_z = path.poses[i].pose.position.z;
    ROS_INFO("candidate_z: %.2f",candidate_z);
    if(abs(current_z - candidate_z) > 0){
      current_z = candidate_z;
      ROS_INFO("pathindex cutoff after %i indexes",current_pathsegment.poses.size());
      path_vector.paths.push_back(current_pathsegment);
      current_pathsegment.poses.resize(0);
    }
    current_pathsegment.poses.push_back(path.poses[i]);
  }
  return path_vector;
}

nav_msgs::Path create_path(geometry_msgs::Point midpoint,float area_sidelength,int num_centroids){
  nav_msgs::Path pathout;
  float centroid_sidelength = area_sidelength / num_centroids;
  for(int y = 0; y < num_centroids; y++){
    for(int x = 0; x < num_centroids; x++){
      geometry_msgs::PoseStamped pose;
      pose.header.frame_id      = "map";
      pose.header.stamp         = ros::Time::now();
      pose.pose.position.x      = midpoint.x + pow(-1,y) * (x * centroid_sidelength - area_sidelength / 2 + centroid_sidelength/2);
      pose.pose.position.y      = midpoint.y + -1 * (y * centroid_sidelength - area_sidelength / 2 + centroid_sidelength/2);
      pose.pose.position.z      = midpoint.z;
      pose.pose.orientation.w   = 1;
      pathout.poses.push_back(pose);
    }
  }
  pathout.header.frame_id = "map";
  pathout.header.stamp = ros::Time::now();
  return pathout;
}
geometry_msgs::Point get_pose_pair(geometry_msgs::PoseStamped posein){
	geometry_msgs::Point point_out;
	float yaw   			= tf::getYaw(posein.pose.orientation);
	point_out.x = posein.pose.position.x + 5 * cos(yaw);
	point_out.y = posein.pose.position.y + 5 * sin(yaw);
	point_out.z = posein.pose.position.z;
	return point_out;
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

int get_best_neighbour(nav_msgs::Path pathin, geometry_msgs::PoseStamped pose_in,std::vector<int> vec_i){
  float best_score = -100;
  float best_dst   = 0;
  float best_hdng  = 0;
  int best_i = -1;
  float current_heading = tf::getYaw(pose_in.pose.orientation);
  for(int i = 0; i < pathin.poses.size(); i++){
    if(!in_vec(vec_i,i)){
      float hdng = get_shortest(get_hdng(pathin.poses[i].pose.position,pose_in.pose.position),current_heading);
      float dst  = get_dst2d(pathin.poses[i].pose.position,pose_in.pose.position);
      if(hdng < 0)
        hdng *= -1;
      if(hdng < M_PI/4 && dst > 0 && dst < 5){
        //float score_dst  = dst / -10;
        float score_hdng = hdng * -1;
    //    float score_tot = score_dst + score_hdng;
        ROS_INFO("pathin[%i]score_hdng: %.2f",i,score_hdng);
        if(score_hdng > best_score){
          best_i     = i;
          best_score = score_hdng;
          best_dst  = dst;
          best_hdng = hdng;
        }
      }
    }
  }
  return best_i;
}

void request_paths(){
/*	path_side.header.frame_id = "empty";
	path_down.poses.resize(0);
	path_side.poses.resize(0);
  ROS_INFO("BRAIN: Requesting paths");*/
	path_side_received = false;
	path_down_received = false;
	path_side_requested = true;
	path_down_requested = true;
	std_msgs::UInt8 get_path;
  pub_get_next_path.publish(get_path);
}

void set_target_pose(geometry_msgs::PoseStamped new_target_pose){
  target_last = target;
	target 	 = new_target_pose;
	path_targets_sent.poses.push_back(new_target_pose);
	pub_cmd.publish(new_target_pose.pose.position);
}
nav_msgs::Path get_targets_remaining(){
  nav_msgs::Path pathout;
  pathout.header = hdr();
  for(int i = path_targets_i; i < path_targets.poses.size(); i++){
    pathout.poses.push_back(path_targets.poses[i]);
  }
  return pathout;
}

float get_dst_remaining(){
  nav_msgs::Path path_targets_remaining = get_targets_remaining();
  if(path_targets_remaining.poses.size() == 0)
    return 0;
  float dst_i0 = get_dst3d(path_targets_remaining.poses[0].pose.position,pos);
  float dst_sum = 0;
  for(int i = 1; i < path_targets_remaining.poses.size(); i++){
    float dst = get_dst3d(path_targets_remaining.poses[i].pose.position,path_targets_remaining.poses[i-1].pose.position);
    dst_sum += dst;
  }
  ROS_INFO("Dst i0: %.0f dst path: %.0f",dst_i0,dst_sum);
  return (dst_sum+dst_i0);
}
nav_msgs::Path merge_paths(nav_msgs::Path path1,nav_msgs::Path path0){
  if(path1.poses.size() > 0 && path0.poses.size() == 0)
    return path1;
  if(path1.poses.size() == 0 && path0.poses.size() > 0)
    return path0;
  if(path1.poses.size() == 0 && path0.poses.size() == 0)
    return path0;
  for(int i = 0; i < path1.poses.size(); i++){
    path0.poses.push_back(path1.poses[i]);
  }
  return path0;
}

void set_target_path(nav_msgs::Path pathin){
  if(pathin.poses[0].pose.orientation.y == 0.7071)
    inspection_type = "down";
  else
    inspection_type = "side";
  if(inspection_type == "down"){
    for(int i = 0; i < pathin.poses.size(); i++){
      pathin.poses[i].pose.position.z += 4;
    }
  }
  float dst_remaining = get_dst_remaining();
	path_complete = false;
  ROS_INFO("pathin: %i ",pathin.poses.size());
	path_targets       = pathin;
	path_targets_i     = 0;
	set_target_pose(path_targets.poses[path_targets_i]);
//    }
//  }
}
int get_next_target(std::vector<int> vec_blacklist,nav_msgs::Path pathin,geometry_msgs::Point current_pos,float current_yaw){
  int best_i = -1;
  float lowest_dist = 1000;
  for(int i = 0; i < pathin.poses.size(); i++){
    if(!in_vec(vec_blacklist,i)){
      float dyaw = get_shortest(get_hdng(pathin.poses[i].pose.position,current_pos),current_yaw);
      if(dyaw < 0)
        dyaw *= -1;
      if(dyaw < M_PI/2){
        float dst = get_dst3d(current_pos,pathin.poses[i].pose.position);
        if(dst < lowest_dist){
          lowest_dist = dst;
          best_i = i;
        }
      }
    }
  }
  return best_i;
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
    ROS_INFO("score_dstTO  %.2f, score_hdng: %.2f, score_cands: %.2f, score_z: %.2f, score_dst0N: %.2f score_down: %.2f score_hyaw: %.2f tot: %.2f",
              score_dstTO,        score_hdng,      score_cands,       score_z,             score_dst0N,     score_down,   score_hyaw,      score_tot);
    scores.push_back(score_tot);
    if(score_tot > best_score){
      best_score = score_tot;
      best_i     = i;
    }
  }
  img_blank.copyTo(img);
  geometry_msgs::Point maxmin;
  maxmin = max_score(scores);
  cv::Scalar color;
  color[1] = 200;
  draw_path_at_img(path_targets,base_pose.pose.position,false,false,false,true,false,color,1);
  color[1] = 0;
  for(int i = 0; i < pathsin.paths.size(); i++){
    float score_rel = scores[i] - maxmin.y;
    float score_pnt = (255 * score_rel / (maxmin.x - maxmin.y));
    int color_score = int(score_pnt);
    color[2] = color_score;
    draw_path_at_img(pathsin.paths[i],pose_comparison.pose.position,true,true,false,true,false,color,1);
  }
  count_target_paths++;

  cv::imwrite("/home/nuc/brain/pathclusters/"+std::to_string(count_target_paths)+"_side_comparison.png",img);
	nav_msgs::Path pathout;
	pathout.header = hdr();
	if(pathsin.paths.size() <= best_i)
		return pathout;
	else
		return pathsin.paths[best_i];
}


tb_msgsrv::Paths get_candidate_paths(bool get_down, bool get_side){
  ROS_INFO("**********************************");
  ROS_INFO("BEHAVIOR-path setting!");
  tb_msgsrv::Paths candidate_paths;
	if(get_down){
	  for(int i = 0; i < paths_active_down.paths.size(); i++){
	    ROS_INFO("PATHS_ACTIVE-down[%i] - %i poses",i,paths_active_down.paths[i].poses.size());
	    candidate_paths.paths.push_back(paths_active_down.paths[i]);
	  }
	}
	if(get_side){
	  for(int i = 0; i < paths_active_side.paths.size(); i++){
	    ROS_INFO("PATHS_ACTIVE-side[%i] - %i poses",i,paths_active_side.paths[i].poses.size());
	    candidate_paths.paths.push_back(paths_active_side.paths[i]);
	  }
	}
	return candidate_paths;
}
nav_msgs::Path path_from_paths(tb_msgsrv::Paths pathsin){
	nav_msgs::Path pathout;
	pathout.header = hdr();
	for(int i = 0; i < pathsin.paths.size(); i++){
		for(int k = 0; k < pathsin.paths[i].poses.size(); k++){
			pathout.poses.push_back(pathsin.paths[i].poses[k]);
		}
	}
	return pathout;
}
geometry_msgs::PoseStamped get_last_target(){
  geometry_msgs::PoseStamped targetpose = target;
  nav_msgs::Path path_targets_remaining = get_targets_remaining();
  if(path_targets_remaining.poses.size() > 0){
    if(path_targets.poses.size() > path_targets_i){
      targetpose = path_targets_remaining.poses[path_targets_remaining.poses.size()-1];
    }
	}
  if(targetpose.pose.orientation.y == 0.7071)
    ROS_INFO("PATHS_ACTIVE-target: %i poses, DOWN",path_targets_remaining.poses.size());
  else
    ROS_INFO("PATHS_ACTIVE-target: %i poses, SIDE",path_targets_remaining.poses.size());
	return targetpose;
}

void check_path_progress(){
	if(!path_complete){
	  if(path_targets_i < path_targets.poses.size()){
	    float dst_to_target = get_dst3d(path_targets.poses[path_targets_i].pose.position,pos);
	    ROS_INFO("BRAIN: Dst to target[%i of %i]: %.0f",path_targets_i,path_targets.poses.size(),dst_to_target);
	    if(dst_to_target < 5){
	      if(path_targets_i+1 < path_targets.poses.size()){
	        path_targets_i++;
	        set_target_pose(path_targets.poses[path_targets_i]);
	      }
	      else{
					path_complete_time = ros::Time::now();
					path_complete = true;
				}
      }
    }
		else{
			path_complete_time = ros::Time::now();
			path_complete = true;
		}
  }
	else
		ROS_INFO("Path is complete");
}

void append_world_poses(nav_msgs::Path pathin,std::string type){
	for(int i = 0; i < pathin.poses.size(); i++){
		if(type == "cleared"){
			if(dst_point_in_path_lim(path_cleared_full,pathin.poses[i].pose.position,5))
				path_cleared_full.poses.push_back(pathin.poses[i]);
		}
		if(type == "obstacles"){
			if(dst_point_in_path_lim(path_obstacles_full,pathin.poses[i].pose.position,5))
				path_obstacles_full.poses.push_back(pathin.poses[i]);
		}
		if(type == "side"){
			if(dst_point_in_path_lim(path_side_full,pathin.poses[i].pose.position,3))
				path_side_full.poses.push_back(pathin.poses[i]);
		}
		if(type == "down"){
			if(dst_point_in_path_lim(path_down_full,pathin.poses[i].pose.position,3))
				path_down_full.poses.push_back(pathin.poses[i]);
		}
	}
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
	pos.x   = transformStamped.transform.translation.x;
	pos.y   = transformStamped.transform.translation.y;
	pos.z   = transformStamped.transform.translation.z;
	tf2::Matrix3x3 q(tf2::Quaternion(transformStamped.transform.rotation.x, transformStamped.transform.rotation.y, transformStamped.transform.rotation.z, transformStamped.transform.rotation.w));
	q.getRPY(vlp_rpy.x,vlp_rpy.y,vlp_rpy.z);
	float delta_vlp = get_shortest(arm1_tilt_msg.data,vlp_rpy.y);
	if(delta_vlp < 0.05 && delta_vlp > -0.05)
		tilting = false;
}
void pathvlp_cb(const nav_msgs::Path::ConstPtr& msg){
  path_vlp = *msg;
  if(msg->poses.size() > 0)
    base_pose = msg->poses[0];
	append_world_poses(*msg,"cleared");
}
void pathvlpobs_cb(const nav_msgs::Path::ConstPtr& msg){
  path_obs = *msg;
	append_world_poses(*msg,"obstacles");
}
void lowrateodom_cb(const nav_msgs::Odometry::ConstPtr& msg){
  odom = *msg;
}
void poylcleared_cb(const geometry_msgs::PolygonStamped::ConstPtr& msg){
	poly_cleared = *msg;
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
void eval_tilting(){
  int obs = path_obs.poses.size();
  if(inspection_type == "side"){
    if(side_empty && !tiltlimit_reached)
      increment_tilt_degrees(-10);
    else if(side_empty && tiltlimit_reached)
      set_tilt_degrees(20);
    else
      set_tilt(0);
  }
  else if(inspection_type == "down"){
    set_tilt_degrees(10);
  }
  else if(obs > 0){
    path_obs       = sort_path(path_obs,"hdng_abs");
    float hdng_obs = get_hdng(path_obs.poses[0].pose.position,base_pose.pose.position);
    float dst_obs  = get_dst3d(path_obs.poses[0].pose.position,base_pose.pose.position);
    ROS_INFO("hdng path_obs sorted by hdng, pose 0: hdng_obs: %.2f dst_obs: %.2f",hdng_obs,dst_obs);
    if((ros::Time::now() - last_tilt).toSec() > 1.0){
      if(dst_obs > 20)
        increment_tilt_degrees(10);
      else
        increment_tilt_degrees(-10);
    }
  }
  else{
    increment_tilt_degrees(10);
  }
}

int get_closest_i(nav_msgs::Path pathin,geometry_msgs::Point pin){
  float res,dst;
  int closest_i = 0;
  float closest_dst = 100;
  if(pathin.poses.size() == 0)
    return closest_i;
  for(int i = 0; i < pathin.poses.size(); i++){
    float dst = get_dst2d(pathin.poses[i].pose.position,pin);
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
int get_indexes_within_rad(nav_msgs::Path pathin,float max_dst,geometry_msgs::Point centroid){
  for(int i = 0; i < pathin.poses.size(); i++){
    if(get_dst3d(pathin.poses[i].pose.position,centroid) > max_dst)
      return i;
  }
  return pathin.poses.size();
}
float get_tot_len(nav_msgs::Path pathin){
  float len = 0;
  for(int i = 1; i < pathin.poses.size(); i++){
    len += get_dst3d(pathin.poses[i].pose.position,pathin.poses[i-1].pose.position);
  }
  return len;
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
nav_msgs::Path get_standardized_cluster_simple(nav_msgs::Path pathin){
  nav_msgs::Path pathout;
  pathout.header = hdr();
  pnt_ref       = base_pose.pose.position;
  geometry_msgs::Point ave_pnt;
  int ave_pnt_i,indexes_in_segment;
  while(pathin.poses.size() > 0){
    pathin             = sort_path(pathin,"dst_3d_ref");
    indexes_in_segment = get_indexes_within_rad(pathin,5,pnt_ref);
    ave_pnt            = get_ave_pnt_ni(pathin,indexes_in_segment);
    float zmx          = get_zmax_ni(pathin,indexes_in_segment);
//    if(pathin.poses[ave_pnt_i].pose.orientation.y = 0.7071)
  //    pathin.poses[ave_pnt_i].pose.position.z = zmx + 3;
    ave_pnt_i          = get_closest_i(pathin,ave_pnt);
    pnt_ref            = pathin.poses[ave_pnt_i].pose.position;
    pathout.poses.push_back(pathin.poses[ave_pnt_i]);
    //ROS_INFO("pathin %i -> pathout %i: indexes_in_segment: %i ave_pnt_i: %i",pathin.poses.size(),pathout.poses.size(),indexes_in_segment,ave_pnt_i);
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
		clusters_standardized.paths[i] = get_standardized_cluster_simple(clusters_in.paths[i]);
	}
	return clusters_standardized;
}

cv::Scalar get_shifting_color(int count,int color_intensity){
  cv::Scalar color;
  if(count == 0)
    color[0] = color_intensity;
  else if(count == 1)
    color[1] = color_intensity;
  else
    color[2] = color_intensity;
  return color;
}

void draw_clusterpaths_shiftingcolor(tb_msgsrv::Paths clusters_in,bool path_line,bool pose_yawline,bool pose_rectangle,bool pose_circle,bool pose_pnt,int pose_size,int color_intensity,bool pos_is_p0){
  geometry_msgs::Point p0;
  if(pos_is_p0)
    p0 = base_pose.pose.position;
  int count = 0;
  for(int i = 0; i < clusters_in.paths.size(); i++){
    count++;
    if(count > 2)
      count = 0;
    draw_path_at_img(clusters_in.paths[i],p0,path_line,pose_yawline,pose_rectangle,pose_circle,pose_pnt,get_shifting_color(count,color_intensity),1);
  }
}

void draw_clusters(tb_msgsrv::Paths clusters_in,std::string draw_type,bool draw_from_blank,std::string cluster_type){
  if(draw_from_blank){
    blankdraw_counter++;
    img_blank.copyTo(img);
  }
  if(draw_type == "minimal")
    draw_clusterpaths_shiftingcolor(clusters_in,false,false,false,false,true,0,40,false);
  else if(draw_type == "lines")
    draw_clusterpaths_shiftingcolor(clusters_in,true,false,false,false,false,0,100,false);
  else if(draw_type == "lines_p0")
    draw_clusterpaths_shiftingcolor(clusters_in,true,false,false,false,false,0,100,true);
  else if(draw_type == "poses")
   draw_clusterpaths_shiftingcolor(clusters_in,false,true,false,true,false,0,100,false);
  else if(draw_type == "lines_poses")
    draw_clusterpaths_shiftingcolor(clusters_in,true,true,false,true,false,3,100,false);
  else
    ROS_INFO("BEHAVIOR: ERROR: draw_type doesnt exist (%s)",draw_type.c_str());
  if(draw_from_blank){
    cv::imwrite("/home/nuc/brain/blankdrawings/"+std::to_string(blankdraw_counter)+"_"+cluster_type+"_"+draw_type+"_blank.png",img);
  }
}
void test(tb_msgsrv::Paths clusters_in,std::string cluster_type){
  ros::Time t_start = ros::Time::now();
  draw_clusters(clusters_in,"minimal",true,cluster_type);
  draw_clusters(clusters_in,"lines",true,cluster_type);
  draw_clusters(clusters_in,"lines_p0",true,cluster_type);
  draw_clusters(clusters_in,"poses",true,cluster_type);
  draw_clusters(clusters_in,"lines_poses",true,cluster_type);
  float dt = (t_start -ros::Time::now()).toSec();
  ROS_INFO("Drawn 5 clusters in %.4f seconds",dt);
}
void append_world_paths(tb_msgsrv::Paths pathsin){
  for(int i = 0; i < pathsin.paths.size(); i++){
    for(int k = 0; k < pathsin.paths[i].poses.size(); k++){
      path_world_visible.poses.push_back(pathsin.paths[i].poses[k]);
    }
  }
}
tb_msgsrv::Paths get_new_world(tb_msgsrv::Paths pathsin,float dst_cutoff,bool use_3d){
  tb_msgsrv::Paths pathsout;
  for(int i = 0; i < pathsin.paths.size(); i++){
    nav_msgs::Path pathout;
    pathout.header = hdr();
    for(int k = 0; k < pathsin.paths[i].poses.size(); k++){
      if(use_3d){
        if(dst_point_in_path_lim(path_visited,pathsin.paths[i].poses[k].pose.position,dst_cutoff))
          pathout.poses.push_back(pathsin.paths[i].poses[k]);
      }
      else{
        if(dst_point_in_path_lim2d(path_visited,pathsin.paths[i].poses[k].pose.position,dst_cutoff))
          pathout.poses.push_back(pathsin.paths[i].poses[k]);
      }
    }
    if(pathout.poses.size() > 0)
      pathsout.paths.push_back(pathout);
  }
  return pathsout;
}


void superpathdown_cb(const tb_msgsrv::Paths::ConstPtr& msg){
	if(path_down_requested)
		path_down_received = true;
	if(msg->paths.size() > 1){
    paths_active_down = organize_clusters(*msg);
    paths_active_down = get_new_world(paths_active_down,5,false);
    append_world_paths(paths_active_down);
  }
}
void superpathside_cb(const tb_msgsrv::Paths::ConstPtr& msg){
	if(path_side_requested)
		path_side_received = true;
	if(msg->paths.size() > 1){
    paths_active_side = organize_clusters(*msg);
    paths_active_side = get_new_world(paths_active_side,5,true);
    append_world_paths(paths_active_side);
  }
}

void pathsdown_cb(const tb_msgsrv::Paths::ConstPtr& msg){

}
void work_objects(int count){
  cv::Scalar color_obs      = get_color(0,0,200);
  cv::Scalar color_clear    = get_color(0,200,0);
  cv::Scalar color_centroid = get_color(200,0,0);
  geometry_msgs::Point32 centroid;
  geometry_msgs::Point poly_cleared_cen,path_vlp_ave_pnt,path_obs_ave_pnt;
  float centroid_area;
  if(path_obs.poses.size() > 0 && path_vlp.poses.size() > 0 && poly_cleared.polygon.points.size() > 2){
    centroid = get_poly_centroidarea(poly_cleared);
    centroid_area = centroid.z;
    poly_cleared_cen.x = centroid.x;
    poly_cleared_cen.y = centroid.y;
    poly_cleared_cen.z = base_pose.pose.position.z;

    path_vlp_ave_pnt = get_ave_pnt_ni(path_vlp,path_vlp.poses.size());
    path_obs_ave_pnt = get_ave_pnt_ni(path_obs,path_obs.poses.size());
    float dst_poly   = get_dst2d(base_pose.pose.position,poly_cleared_cen);
    float obs_hdng0  = get_hdng(path_obs.poses[0].pose.position,base_pose.pose.position);
    float obs_hdngN  = get_hdng(path_obs.poses[path_obs.poses.size()-1].pose.position,base_pose.pose.position);
    float obs_dhdng  = get_shortest(obs_hdngN,obs_hdng0);
    if(obs_dhdng < 0)
      obs_dhdng *= -1;
    img_blank.copyTo(img);
    cv::circle(img,pnt2cv(poly_cleared_cen),2,color_centroid,1);
    cv::circle(img,pnt2cv(path_vlp_ave_pnt),2,color_clear,1);
    cv::circle(img,pnt2cv(path_obs_ave_pnt),2,color_obs,1);
    cv::circle(img,pnt2cv(base_pose.pose.position),2,get_color(100,100,0),1);

    draw_path_at_img(path_vlp,base_pose.pose.position,false,false,false,false,true,color_clear,1);
    draw_path_at_img(path_obs,base_pose.pose.position,false,false,false,false,true,color_obs,1);

		for(int i = 0; i < paths_active_side.paths.size(); i++){
			draw_path_at_img(paths_active_side.paths[i],base_pose.pose.position,true,true,false,true,false,get_color(150,100,50),1);
		}
		for(int i = 0; i < paths_active_down.paths.size(); i++){
			draw_path_at_img(paths_active_down.paths[i],base_pose.pose.position,false,false,true,false,false,get_color(100,150,50),1);
		}

    ROS_INFO("BEHAV-Surrounds: path_clear: %i obs: %i, poly_clear area: %.0f, dst_poly: %.0f, obs_hdng0-N: %.2f (%.2f-%.2f)",
                                path_vlp.poses.size(),path_obs.poses.size(),centroid_area,dst_poly,obs_dhdng,obs_hdng0,obs_hdngN);
    cv::imwrite("/home/nuc/brain/objects"+std::to_string(count)+".png",img);
  }
}

int evaluate_tilt(int tilt_degrees,int stage,int count){
	float dt_path_complete = (ros::Time::now() - path_complete_time).toSec();
	if(stage == 0){
		ROS_INFO("********************************************************");
		ROS_INFO("Evaluating tilt at %i degrees",tilt_degrees);
		stage = 1;
		set_tilt_degrees(tilt_degrees);
	}
	else if(stage == 1){
		if(!tilting){
			ROS_INFO("********************************************************");
			ROS_INFO("Tilt complete, requesting paths");
			stage = 2;
			request_paths();
		}
	}
	else if(stage == 2){
		if(path_side_received && path_down_received){
			ROS_INFO("********************************************************");
			ROS_INFO("Paths received - working surroundings");
			stage = 3;
			work_objects(count);
		}
	}
	else if(stage == 3){
		stage = 4;
		nav_msgs::Path path_active_down	= path_from_paths(paths_active_down);
		nav_msgs::Path path_active_side	= path_from_paths(paths_active_side);

		tb_msgsrv::Paths 			candidate_paths = get_candidate_paths(true,true);
		geometry_msgs::PoseStamped targetpose = get_last_target();
		nav_msgs::Path best_path 							= get_next_candidates(candidate_paths,targetpose);
		paths_candidates.paths.push_back(best_path);
		ROS_INFO("********************************************************");
		ROS_INFO("Best path: %i poses",best_path.poses.size());
	}
	else{
		stage = 0;
	}
	return stage;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "tb_behavior_node");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");
	private_nh.param("par_vlpmaxtilt",par_vlpmaxtilt, M_PI/10);
	private_nh.param("par_vlpmintilt",par_vlpmintilt, -M_PI/10);
	private_nh.param("par_vlptiltinterval",par_vlptiltinterval, M_PI/10);
	private_nh.param("takeoff_altlvl",par_takeoffaltitude, 5.0);
	tf2_ros::TransformListener tf2_listener(tfBuffer);
	std_msgs::Float64 target_alt_msg;
  path_world_visible.header  = hdr();
	path_cleared_full.header   = hdr();
	path_obstacles_full.header = hdr();
	path_side_full.header 		 = hdr();
	path_down_full.header      = hdr();
	target.pose.position.z 	   = par_takeoffaltitude;
	target.pose.orientation.w  = 1;
  ros::Subscriber s0 = nh.subscribe("/tb_path/cleared_poly",1,poylcleared_cb);
  ros::Subscriber s1 = nh.subscribe("/tb_path/cleared",1,pathvlp_cb);
	ros::Subscriber s2 = nh.subscribe("/tb_path/obstacles",1,pathvlpobs_cb);
 	ros::Subscriber s3 = nh.subscribe("/tb_path/superpath_side",10,superpathside_cb);
	ros::Subscriber s4 = nh.subscribe("/tb_path/superpath_down",10,superpathdown_cb);
	ros::Subscriber s5 = nh.subscribe("/tb_path/visited",10,pathvstd_cb);

  ros::Subscriber s8 = nh.subscribe("/tb_fsm/main_state",100,&mainstate_cb);
  ros::Subscriber s9 = nh.subscribe("/tb_nav/lowrate_odom",10,lowrateodom_cb);
  ros::Subscriber s12 = nh.subscribe("/tb_paths/down",10,pathsdown_cb);

	ros::Publisher pub_path_cleared_full   = nh.advertise<nav_msgs::Path>("/tb_behav/cleared_full",100);
	ros::Publisher pub_path_obstacles_full = nh.advertise<nav_msgs::Path>("/tb_behav/obstacles_full",100);
  ros::Publisher pub_path      = nh.advertise<nav_msgs::Path>("/tb_behav/targets",10);
  ros::Publisher pub_path_best = nh.advertise<nav_msgs::Path>("/tb_behav/down_best",10);
  ros::Publisher pub_path_down = nh.advertise<nav_msgs::Path>("/tb_behav/down_ordered",10);
  ros::Publisher pub_path_side = nh.advertise<nav_msgs::Path>("/tb_behav/side_ordered",10);
  ros::Publisher pub_path_world = nh.advertise<nav_msgs::Path>("/tb_path/world",10);

	ros::Publisher pub_altcmd	 = nh.advertise<std_msgs::Float64>("/tb_cmd/alt_target", 100);
	ros::Publisher pub_tiltvlp = nh.advertise<std_msgs::Float64>("/tb_cmd/arm1_tilt", 10);
	ros::Publisher pub_tilt  	 = nh.advertise<std_msgs::Float64>("/tb_cmd/arm2_tilt", 10);
	ros::Publisher pub_pan	   = nh.advertise<std_msgs::Float64>("/tb_cmd/arm2_pan", 10);

	ros::Publisher pub_target_pose = nh.advertise<geometry_msgs::PoseStamped>("/cmd_pose",10);

	pub_get_next_path	 	= nh.advertise<std_msgs::UInt8>("/tb_path/request", 100);
	pub_cmd      				= nh.advertise<geometry_msgs::Point>("/cmd_pos",10);
	ros::Rate rate(2.0);
	int count = 0;
	int idle_count = 0;
	std::string last_type;
	bool evaluation_started,evaluation_complete;
	int evaluation_stage = 0;
	std::vector<int> tilts_to_evaluate;
  while(ros::ok()){
		checktf();
		if(mainstate == 1 && !path_complete){
			ROS_INFO("behav-check path progress ");
			check_path_progress();
		}
		else if(path_complete && !evaluation_started){
			ROS_INFO("STARTING EVALUATION!!");
			tilts_to_evaluate.resize(0);
			paths_candidates.paths.resize(0);
			tilts_to_evaluate.push_back(-15);
			tilts_to_evaluate.push_back(0);
			tilts_to_evaluate.push_back(15);
			evaluation_started = true;
		}
		else if(evaluation_started && !evaluation_complete){
			count++;
			evaluation_stage = evaluate_tilt(tilts_to_evaluate[paths_candidates.paths.size()],evaluation_stage,count);
			if(evaluation_stage == 4 && paths_candidates.paths.size() == tilts_to_evaluate.size())
				evaluation_complete = true;
		}
		else if(evaluation_started && evaluation_complete){
			ROS_INFO("Evaluation complete!");
			draw_clusters(paths_candidates,"lines_poses",true,"best");
			evaluation_complete = false;
			evaluation_started = false;
			nav_msgs::Path patht = get_next_candidates(paths_candidates,target);
			ROS_INFO("Got best target path: %i poses",patht.poses.size());
			if(patht.poses.size() > 0)
				set_target_path(patht);
		}
  	target_alt_msg.data = target.pose.position.z;
    pub_target_pose.publish(target);
    pub_tiltvlp.publish(arm1_tilt_msg);
    pub_altcmd.publish(target_alt_msg);
    pub_path.publish(path_targets);
    pub_path_best.publish(path_down_best);
    pub_path_cleared_full.publish(path_cleared_full);
    pub_path_obstacles_full.publish(path_obstacles_full);
    pub_path_world.publish(path_world_visible);
		pub_target_pose.publish(target);
    rate.sleep();
    ros::spinOnce();
  }
  return 0;
}
