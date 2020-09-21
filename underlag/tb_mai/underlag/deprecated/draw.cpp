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


cv::Mat mapimg_copy(300,300,CV_8UC3,cv::Scalar(0, 0, 0)); //create image, set encoding and size, init pixels to default val
cv::Mat img_side(300,300,CV_8UC3,cv::Scalar(0, 0, 0)); //create image, set encoding and size, init pixels to default val
cv::Mat img_down(300,300,CV_8UC3,cv::Scalar(0, 0, 0)); //create image, set encoding and size, init pixels to default val
cv::Mat img(300,300,CV_8UC3,cv::Scalar(0, 0, 0)); //create image, set encoding and size, init pixels to default val

nav_msgs::Path path_vlp,path_visited,path_down,path_side;
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
	int c = x2c(pin.x,img_down.cols,1);
	int r = y2r(pin.y,img_down.rows,1);
	return cv::Point(c,r);
}
bool sort_dst_pair(const std::tuple<int,float>& a,
               const std::tuple<int,float>& b)
{
    return (std::get<1>(a) < std::get<1>(b));
}
int get_closest_pose(nav_msgs::Path pathin,geometry_msgs::Point pnt,float min_dst){
	float closest = 1000;
	int best_tot_i = -1;
	for(int i = 0; i < pathin.poses.size(); i++){
		float dst_obs  = get_dst2d(pathin.poses[i].pose.position,pnt);
		if(dst_obs < closest && dst_obs < min_dst){
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
		cand_i_at_vlp_i[get_closest_pose(pathin_vlp,pathin_cands.poses[i].pose.position,20)].push_back(i);
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

std::vector<nav_msgs::Path> paths_segmented_hdng(nav_msgs::Path pathin){
  std::vector<nav_msgs::Path> path_vector;
  nav_msgs::Path current_pathsegment;
  current_pathsegment.header = pathin.header;
  nav_msgs::Path path = sort_path(pathin,"hdng");

  float current_heading = get_hdng(path.poses[0].pose.position,base_pose.pose.position);
  for(int i = 1; i < path.poses.size(); i++){
    float candidate_heading = get_hdng(path.poses[i].pose.position,base_pose.pose.position);
    float delta_hdng        = get_shortest(candidate_heading,current_heading);
    int delta_hdng_degs     = abs(delta_hdng * rad2deg);
    if(delta_hdng_degs > 15){
      current_heading = candidate_heading;
      ROS_INFO("pathindex cutoff after %i indexes",current_pathsegment.poses.size());
      path_vector.push_back(current_pathsegment);
      current_pathsegment.poses.resize(0);
    }
    current_pathsegment.poses.push_back(path.poses[i]);
  }
  return path_vector;
}

int get_corresponding_index(nav_msgs::Path path_full,geometry_msgs::PoseStamped pose_to_find){
  for(int i = 0; i < path_full.poses.size(); i++){
    if(  path_full.poses[i].pose.position.x == pose_to_find.pose.position.x
      && path_full.poses[i].pose.position.y == pose_to_find.pose.position.y
      && path_full.poses[i].pose.position.z == pose_to_find.pose.position.z)
        return i;
  }
  ROS_INFO("CORRESPONDING INDEX NOT FOUND");
  return -1;
}

void dot_pnt(geometry_msgs::Point pnt,int r, int g, int b){
  img_down.at<cv::Vec3b>( y2r(pnt.y,img_down.rows,1),x2c(pnt.x,img_down.cols,1) )[2] = r;
  img_down.at<cv::Vec3b>( y2r(pnt.y,img_down.rows,1),x2c(pnt.x,img_down.cols,1) )[1] = g;
  img_down.at<cv::Vec3b>( y2r(pnt.y,img_down.rows,1),x2c(pnt.x,img_down.cols,1) )[0] = b;
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
void draw_scores(std::vector<nav_msgs::Path> paths,std::vector<nav_msgs::Path> cands,std::vector<float> scores){

  geometry_msgs::Point maxmin;
  maxmin = max_score(scores);
  float score_range = maxmin.x - maxmin.y;
  cv::Scalar color_path,color_cands;
  color_path[0]  = 255;
  if(scores.size() == 0)
  cv::circle(img,pnt2cv(base_pose.pose.position),1,color_path,1);
  color_path[0] = 0;
  ROS_INFO("Score min: %.2f max: %.2f dst: %.2f",maxmin.y,maxmin.x,score_range);
  bool color_red,color_blue;
  bool color_green = true;
  for(int i = 0; i < scores.size(); i++){
    cv::Scalar color_path,color_cands;
    float score_rel = scores[i] - maxmin.y;
    float score_pnt = (255 * score_rel / score_range);
    int color_score = int(score_pnt);

    if(color_red){
      color_red = false;
      color_blue = true;
      color_path[0]  = color_score;
      color_cands[1] = color_score;
    }
    else if(color_blue){
      color_blue = false;
      color_green = true;
      color_path[1]  = color_score;
      color_cands[2] = color_score;
    }
    else if(color_green){
      color_red = true;
      color_green = false;
      color_path[2]  = color_score;
      color_cands[0] = color_score;
    }
    nav_msgs::Path path = paths[i];
    nav_msgs::Path cand = cands[i];



    ROS_INFO("Scores: %i score: %.2f/%i, paths: %i cands: %i",i,score_rel,color_score,path.poses.size(),cand.poses.size());
    for(int k = 0; k < cand.poses.size(); k++){
      img.at<cv::Vec3b>( y2r(cand.poses[k].pose.position.y,img.rows,1),x2c(cand.poses[k].pose.position.x,img.cols,1) )[0] = color_cands[0];
      img.at<cv::Vec3b>( y2r(cand.poses[k].pose.position.y,img.rows,1),x2c(cand.poses[k].pose.position.x,img.cols,1) )[1] = color_cands[1];
      img.at<cv::Vec3b>( y2r(cand.poses[k].pose.position.y,img.rows,1),x2c(cand.poses[k].pose.position.x,img.cols,1) )[2] = color_cands[2];
  //    img.at<cv::Vec3b>( y2r(cand.poses[k].pose.position.y,img.rows,1),x2c(cand.poses[k].pose.position.x,img.cols,1) )[1] = color_score;
    }
    for(int l = 0; l < path.poses.size(); l++){
    //  if(l > 0)
      //  cv::line (img, pnt2cv(path.poses[l-1].pose.position), pnt2cv(path.poses[l].pose.position), color_path,1,cv::LINE_8,0);
      cv::circle(img,pnt2cv(path.poses[l].pose.position),1,color_path,1);
    }
  //  color_path[0] = color_score;

    cv::line (img, pnt2cv(path.poses[path.poses.size()-1].pose.position), pnt2cv(base_pose.pose.position), color_path,1,cv::LINE_8,0);
  }
  count++;

}
std::vector<nav_msgs::Path> testdown(std::vector<std::vector<int>> cand_i_at_vlp_i,nav_msgs::Path vlp_with_candidates,nav_msgs::Path path_cand){
  std::vector<nav_msgs::Path> path_vector;
  nav_msgs::Path best_path,best_path_candidates;
  path_vector = sort_paths_in_vector(paths_segmented_hdng(vlp_with_candidates),"dst_2d");
  ROS_INFO("Path vector returned (%i)",path_vector.size());
  float best_dst0 = 30;
  float best_hdng = 0;
  float best_deltadst = 0;
  float best_candidates = 0;
  float best_score = 0;

  std::vector<nav_msgs::Path> paths;
  std::vector<nav_msgs::Path> cands;
  std::vector<float> score;

  for(int i = 0; i < path_vector.size(); i++){
    int vec_size = path_vector[i].poses.size();
    ROS_INFO("path_vector[%i],size: %i",i,vec_size);
    if(vec_size > 0){
      float hdng = get_shortest(get_hdng(path_vector[i].poses[0].pose.position,base_pose.pose.position),tf::getYaw(base_pose.pose.orientation));
      float dst0 = get_dst2d(path_vector[i].poses[0].pose.position,base_pose.pose.position);
      float dstN = get_dst2d(path_vector[i].poses[path_vector[i].poses.size()-1].pose.position,base_pose.pose.position);
      float delta_dst = dstN - dst0;
      int candidates = 0;
      nav_msgs::Path path_candidates;
      for(int k = 0; k < path_vector[i].poses.size(); k++){
        std::vector<int> candidates_at_vlp = cand_i_at_vlp_i[get_corresponding_index(vlp_with_candidates,path_vector[i].poses[k])];
        candidates += candidates_at_vlp.size();
        for(int l = 0; l < candidates_at_vlp.size();l++){
          path_candidates.poses.push_back(path_cand.poses[candidates_at_vlp[l]]);
        }
      }
      if(hdng < 0)
        hdng *= -1;
      float rel_dst0  = dst0 - best_dst0;
      float rel_hdng  = hdng - best_hdng;
      float rel_dst   = delta_dst - best_deltadst;
      int   rel_cands = candidates - best_candidates;
      float score_dst0 = rel_dst0 / -10;
      float score_hdng = rel_hdng * -1;
      float score_dst  = rel_dst / 10;
      float score_cands = float(rel_cands) /100.0;
      float score_tot = score_dst0 + score_hdng + score_dst + score_cands;
      ROS_INFO("    dst0      %.2f, hdng:       %.2f, dst:       %.2f, cands:       %i",dst0,hdng,delta_dst,candidates);
      ROS_INFO("rel_dst0      %.2f, rel_hdng:   %.2f, rel_dst:   %.2f, rel_cands:   %i",rel_dst0,rel_hdng,rel_dst,rel_cands);
      ROS_INFO("score_dst0    %.2f, score_hdng: %.2f, score_dst: %.2f, score_cands: %.2f tot: %.2f",score_dst0,score_hdng,score_dst,score_cands,score_tot);
      if(score_tot > best_score){
        best_dst0 = dst0;
        best_hdng = hdng;
        best_deltadst = delta_dst;
        best_path = path_vector[i];
        best_path_candidates = path_candidates;
      }
      if(path_vector[i].poses.size() > 2){
        paths.push_back(path_vector[i]);
        cands.push_back(path_candidates);
        score.push_back(score_tot);
      }
    }
  }
  cv::Scalar color_path;

  color_path[0]  = 255;
  color_path[1] = 255;
  draw_scores(paths,cands,score);
  ROS_INFO("count: %i",count);
  for(int k = 0; k < best_path_candidates.poses.size(); k++){
    img.at<cv::Vec3b>( y2r(best_path_candidates.poses[k].pose.position.y,img.rows,1),x2c(best_path_candidates.poses[k].pose.position.x,img.cols,1) )[0] = 100;
    img.at<cv::Vec3b>( y2r(best_path_candidates.poses[k].pose.position.y,img.rows,1),x2c(best_path_candidates.poses[k].pose.position.x,img.cols,1) )[1] = 100;
    img.at<cv::Vec3b>( y2r(best_path_candidates.poses[k].pose.position.y,img.rows,1),x2c(best_path_candidates.poses[k].pose.position.x,img.cols,1) )[2] = 0;
  }
  for(int l = 0; l < best_path.poses.size(); l++){
    if(l > 0)
      cv::line (img, pnt2cv(best_path.poses[l-1].pose.position), pnt2cv(best_path.poses[l].pose.position), color_path,1,cv::LINE_8,0);
    cv::circle(img,pnt2cv(best_path.poses[l].pose.position),1,color_path,1);
  }

  cv::imwrite("/home/nuc/brain/img"+std::to_string(count)+".png",img);

  std::vector<nav_msgs::Path> paths_out;
  paths_out.push_back(best_path);
  paths_out.push_back(best_path_candidates);
  return paths_out;
}

void draw_pose(geometry_msgs::PoseStamped pose,cv::Scalar color,int size){
	geometry_msgs::Point p1,p0;
	float yaw = tf::getYaw(pose.pose.orientation);
	p1.x = pose.pose.position.x + 1.5*size * cos(yaw);
	p1.y = pose.pose.position.y + 1.5*size * sin(yaw);
	cv::line (img_side, pnt2cv(pose.pose.position), pnt2cv(p1), color,size,cv::LINE_8,0);
	cv::circle(img_side,pnt2cv(pose.pose.position),size,color,1);
}

void draw_side(nav_msgs::Path best_path,nav_msgs::Path best_path_candidates){
  cv::Scalar color_path,color_cands;
  color_path  = get_color(0,0,150);
  color_cands = get_color(0,100,0);

  for(int i = 0; i < best_path.poses.size(); i++){
    if(i > 0)
      cv::line (img_side, pnt2cv(best_path.poses[i-1].pose.position), pnt2cv(best_path.poses[i].pose.position), color_path,1,cv::LINE_8,0);
    cv::circle(img_side,pnt2cv(best_path_candidates.poses[i].pose.position),1,color_path,1);
  }
  for(int i = 0; i < best_path_candidates.poses.size(); i++){
    cv::circle(img_side,pnt2cv(best_path_candidates.poses[i].pose.position),1,color_cands,1);
  }
}
void draw_down(nav_msgs::Path best_path,nav_msgs::Path best_path_candidates){
  cv::Scalar color_path,color_cands;
  color_path  = get_color(0,0,150);
  color_cands = get_color(0,100,0);

  for(int i = 0; i < best_path.poses.size(); i++){
    if(i > 0)
      cv::line (img_down, pnt2cv(best_path.poses[i-1].pose.position), pnt2cv(best_path.poses[i].pose.position), color_path,1,cv::LINE_8,0);
    cv::circle(img_down,pnt2cv(best_path_candidates.poses[i].pose.position),1,color_path,1);
  }
  for(int i = 0; i < best_path_candidates.poses.size(); i++){
    cv::circle(img_down,pnt2cv(best_path_candidates.poses[i].pose.position),1,color_cands,1);
  }
}


nav_msgs::Path testing_side(nav_msgs::Path pathin_vlp,nav_msgs::Path pathin_cands){
	std::vector<std::vector<int>> cand_i_at_vlp_i;
	nav_msgs::Path vlp_with_candidates;
	std::vector<nav_msgs::Path> paths_out;

  cand_i_at_vlp_i 		= get_candi_at_vlpi(pathin_vlp,pathin_cands);
  ROS_INFO("#1testing side");

  vlp_with_candidates = get_vlp_with_candidates(pathin_vlp,cand_i_at_vlp_i);
  ROS_INFO("#2testing side");

  cand_i_at_vlp_i     = get_updated_candi_at_vlpi(cand_i_at_vlp_i);
  ROS_INFO("#3testing side");
  paths_out           = testdown(cand_i_at_vlp_i,vlp_with_candidates,pathin_cands);
  ROS_INFO("#4testing side");

  nav_msgs::Path best_path,best_path_candidates;
  best_path = paths_out[0];
  best_path_candidates = paths_out[1];
  //draw_side(best_path,best_path_candidates);
  return best_path;
}
nav_msgs::Path testing_down(nav_msgs::Path pathin_vlp,nav_msgs::Path pathin_cands){
	std::vector<std::vector<int>> cand_i_at_vlp_i;
	nav_msgs::Path vlp_with_candidates;
	std::vector<nav_msgs::Path> paths_out;

  cand_i_at_vlp_i 		= get_candi_at_vlpi(pathin_vlp,pathin_cands);
  ROS_INFO("#1testing down");

  vlp_with_candidates = get_vlp_with_candidates(pathin_vlp,cand_i_at_vlp_i);
  ROS_INFO("#2testing down");

  cand_i_at_vlp_i     = get_updated_candi_at_vlpi(cand_i_at_vlp_i);
  ROS_INFO("#3testing down");
  paths_out           = testdown(cand_i_at_vlp_i,vlp_with_candidates,pathin_cands);
  ROS_INFO("#4testing down");
  nav_msgs::Path best_path,best_path_candidates;
  best_path = paths_out[0];
  best_path_candidates = paths_out[1];
  //draw_down(best_path,best_path_candidates);
  return best_path;
}

void pathcandnew_cb(const nav_msgs::Path::ConstPtr& msg){
  nav_msgs::Path path;
  if(msg->poses.size() > 0 && path_vlp.poses.size() > 0){
    count_down++;
    path=testing_down(path_vlp,*msg);
    //cv::imwrite("/home/nuc/brain/test_down_"+std::to_string(count_down)+".png",img_down);
  }
  path.header.frame_id = "map";
  pub_path_down.publish(path);
}
void pathcandsidesnew_cb(const nav_msgs::Path::ConstPtr& msg){
  nav_msgs::Path path;
  if(msg->poses.size() > 0 && path_vlp.poses.size() > 0){
    count_side++;
    path=testing_side(path_vlp,*msg);
    //cv::imwrite("/home/nuc/brain/test_side_"+std::to_string(count_side)+".png",img_side);
  }
  path.header.frame_id = "map";
  pub_path_side.publish(path);
}
void pathvlp_cb(const nav_msgs::Path::ConstPtr& msg){
  if(msg->poses.size() > 0)
    base_pose = msg->poses[0];
	path_vlp = *msg;
}
void pathvstd_cb(const nav_msgs::Path::ConstPtr& msg){
	cv::Scalar color;
	color[1] = 150;
	for(int i = path_visited.poses.size(); i < msg->poses.size(); i++){
		draw_pose(msg->poses[i],color,2);
	}
	path_visited = *msg;
}
int main(int argc, char **argv)
{
  ros::init(argc, argv, "tb_draw_node");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");

///	ros::Subscriber s2   = nh.subscribe("/tb_path/visited",10,pathvstd_cb);
	ros::Subscriber s1   = nh.subscribe("/tb_path/candidates",10,pathcandnew_cb);
	ros::Subscriber s12  = nh.subscribe("/tb_path/candidates_sides",10,pathcandsidesnew_cb);
	ros::Subscriber s11  = nh.subscribe("/tb_path/vlp_path",10,pathvlp_cb);
  pub_path_down = nh.advertise<nav_msgs::Path>("/tb_path/out_down",10);
	pub_path_side = nh.advertise<nav_msgs::Path>("/tb_path/out_side",10);

  ros::Rate rate(1.0);
  while(ros::ok()){

	  rate.sleep();
	  ros::spinOnce();
  }
  return 0;
}
