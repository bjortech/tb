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

ros::Publisher pub_target_dash, pub_path_best,pub_get_next_path,pub_cmd;
geometry_msgs::PoseStamped target,base_pose,target_last,target_final,target_dash;
ros::Time activity_change,last_tilt,path_complete_time;
std_msgs::Float64 arm1_tilt_msg;
geometry_msgs::Vector3 vlp_rpy;
geometry_msgs::Point poly_cleared_centroid,pos,pnt_midpoint,pnt_ref;
nav_msgs::Odometry odom;
nav_msgs::Path path_raw_side,path_raw_down,path_target_full,path_clear_vlp,path_down_best_in_poly,path_side_best_in_poly,path_full,path_world_visible,path_down_best,path_side_best,path_vlp,path_obs,path_side_full,path_down_full,path_targets,path_visited,path_cleared_full,path_obstacles_full,path_targets_sent,path_side,path_down;
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
int targets_down_in_poly,targets_side_in_poly;
int dash_stage = 0;
float poly_cleared_centroid_area;
std::vector<int> targets_complete;
tb_msgsrv::Paths paths_active_down,down_in_poly;
tb_msgsrv::Paths paths_active_side,side_in_poly;
geometry_msgs::PolygonStamped poly_cleared,target_final_poly;
tb_msgsrv::Paths paths_candidates,paths_full;
bool path_side_requested,path_down_requested,path_side_received,path_down_received;
float target_path_distance,target_final_hdng,target_hdng,target_distance;
std::string centroids[200][200][20];
bool got_path_raw_side,got_path_raw_down,got_path_vlp,got_path_obs,got_poly_cleared;

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
void update_centroid(geometry_msgs::Point pnt,std::string info){
  centroids[int(pnt.x / 5 + 100)][int(pnt.y / 5 + 100)][int(pnt.z / 2 + 5)] = info;
}
void check_centroid(){
  cv::Mat img(1000,1000,CV_8UC3,cv::Scalar(0, 0, 0)); //create image, set encoding and size, init pixels to default val
  int obstacles_count = 0;
  int cleared_count   = 0;
  int something_count = 0;
  for(int i = 0; i < 200; i++){
    for(int j = 0; j < 200; j++){
      bool got_something;
      geometry_msgs::Point pnt,prect0,prect1;
      pnt.x = i * 5 + -500;
      pnt.y = j * 5 + -500;
      prect0.x = pnt.x-2.5;
      prect0.y = pnt.y-2.5;
      prect1.x = pnt.x+2.5*2;
      prect1.y = pnt.y+2.5*2;
      cv::Scalar color;
      color[0] = 50;
      cv::rectangle(img, pnt2cv(prect0),pnt2cv(prect1),color,1,8,0);

      for(int k = 0; k < 20; k++){
        pnt.z = k * 2 - 25;
        std::string out = centroids[i][j][k];
        if(!got_something && out != "")
          got_something = true;
        if(out == "obstacle"){
          color[2] = 100;
          color[1] = 0;
          cv::circle(img,pnt2cv(pnt),2,color,1);
          obstacles_count++;
        }
        if(out == "cleared"){
          color[1] = 100;
          color[2] = 0;
          cv::circle(img,pnt2cv(pnt),1,color,1);
          cleared_count++;
        }
      }
      if(got_something)
        something_count++;
    }
  }
  cv::imwrite("/home/nuc/brain/"+std::to_string(count_target_paths)+"grids.png",img);
  //ROS_INFO("Checked centroid. total of obs: %i clear: %i something: %i",obstacles_count,cleared_count,something_count);
}


float get_slope(geometry_msgs::Point p1, geometry_msgs::Point p2){
  return (p2.z - p1.z) / get_dst2d(p1,p2);
}
float get_inclination(geometry_msgs::Point p1, geometry_msgs::Point p2){
  return atan2(p2.z - p1.z,get_dst2d(p1,p2));
}
float get_inclination_offset(geometry_msgs::Point pnt,float inclination_target){
  return (pnt.z - inclination_target*get_dst2d(pos,pnt) + pos.z);
}

void draw_pose(geometry_msgs::Point pnt,float yaw,float len, cv::Scalar color){
  geometry_msgs::Point pyaw;
  pyaw.x = pnt.x + len * cos(yaw);
  pyaw.y = pnt.y + len * sin(yaw);
  cv::circle(img,pnt2cv(pnt),2,color,1);
  cv::line (img, pnt2cv(pnt), pnt2cv(pyaw),color,1,cv::LINE_8,0);
}
void draw_poly(geometry_msgs::PolygonStamped polyin,cv::Scalar color){
  for(int i = 1; i < polyin.polygon.points.size(); i++){
    geometry_msgs::Point p1,p2;
    p1.x = polyin.polygon.points[i-1].x;
    p1.y = polyin.polygon.points[i-1].y;
    p2.x = polyin.polygon.points[i].x;
    p2.y = polyin.polygon.points[i].y;
    cv::line (img, pnt2cv(p1), pnt2cv(p2),color,1,cv::LINE_8,0);
  }
}

geometry_msgs::Point get_eval_area(geometry_msgs::Point midpoint,int dxy){
  geometry_msgs::Point pout;
  //create image, set encoding and size, init pixels to default val
  int obs_clr = 0;
  int obs   = 0;
  int clr = 0;
  int d_xy = int(dxy / 5);
  int i0 = int(midpoint.x / 5 + 100) - d_xy;
  int j0 = int(midpoint.y / 5 + 100) - d_xy;
  int k0 = int(midpoint.z / 2 + 5);
  int in = i0 + d_xy*2;
  int jn = j0 + d_xy*2;
  int cnt = 0; int none = 0;
  float obs_alt_sum = 0; float obs_clr_alt_sum = 0; float clr_alt_sum = 0;
  for(int i = i0; i < in; i++){
    for(int j = j0; j < jn; j++){
      bool got_cleared;
      bool got_obstacle;
      int obstacle_at = 0;
      int cleared_at = 0;
      cnt++;

      cv::Scalar color;

      for(int k = 0; k < 20; k++){
        std::string out = centroids[i][j][k];
        if(out == "obstacle"){
          got_obstacle = true;
          obstacle_at =  k * 2 + 10;
          color[2] = 100;
        }
        if(out == "cleared"){
          if(!got_cleared)
            cleared_at = k * 2 + 10;
          got_cleared = true;
          color[1] = 100;
        }
      }

      geometry_msgs::Point pnt,prect0,prect1;
      pnt.x = i * 5 + -500;
      pnt.y = j * 5 + -500;
      prect0.x = pnt.x-2.5;
      prect0.y = pnt.y-2.5;
      prect1.x = pnt.x+2.5*2;
      prect1.y = pnt.y+2.5*2;
      cv::rectangle(img, pnt2cv(prect0),pnt2cv(prect1),color,1,8,0);

      if(got_obstacle && got_cleared){
        obs_clr++;
        obs_clr_alt_sum += obstacle_at;
      }
      else if(got_obstacle){
        obs++;
        obs_alt_sum += obstacle_at;
      }
      else if(got_cleared){
        clr++;
        clr_alt_sum += cleared_at;
      }
      else{
        none++;
      }
    }
  }
  geometry_msgs::Point prect0,prect1;
  prect0.x = midpoint.x-dxy*0.5;
  prect0.y = midpoint.y-dxy*0.5;
  prect1.x = midpoint.x+dxy*0.5;
  prect1.y = midpoint.y+dxy*0.5;
  cv::Scalar color;
  color[2] = 100;
  cv::rectangle(img, pnt2cv(prect0),pnt2cv(prect1),color,1,8,0);
  float obs_alt = obs_alt_sum / fmax(obs,1);
  float clr_alt = clr_alt_sum / fmax(clr,1);
  float obs_clr_alt = obs_clr_alt_sum / fmax(obs_clr,1);
  float percent_obs = obs / cnt * 100;
  float percent_clr = clr / cnt * 100;
  float percent_obs_clr = obs_clr / cnt * 100;
  float percent_none = none / cnt * 100;
  //ROS_INFO("Checked centroid. total of obs: %i clear: %i something: %i, percent: obs / clr / obs_clr / else : [%.0f / %.0f / %.0f / %.0f]",obs,clr,obs_clr,percent_obs,percent_clr,percent_obs_clr,percent_none);
  pout.x = percent_none;
  pout.y = percent_clr;
  pout.z = percent_obs_clr;
  return pout;
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
void draw_pathside_at_img(nav_msgs::Path pathin,geometry_msgs::Point p0,
	 bool path_line,bool pose_yawline,bool pose_rectangle,bool pose_circle,bool pose_pnt,
  cv::Scalar color, int pose_size){
	geometry_msgs::Point pyaw,prect0,prect1,pnt;
  if(pathin.poses.size() < 1)
    return;
	if(p0.x == 0 && p0.y == 0)
		p0 = pathin.poses[0].pose.position;
	for(int i = 0; i < pathin.poses.size(); i++){
		pnt.x = get_dst2d(pathin.poses[i].pose.position,base_pose.pose.position);
		pnt.y = pathin.poses[i].pose.position.z;
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
  int farthest_i = 0;
  float farthest_dst = 100;
  if(pathin.poses.size() == 0)
    return farthest_i;
  for(int i = 0; i < pathin.poses.size(); i++){
    float dst = get_dst2d(pathin.poses[i].pose.position,pin);
     if(dst > farthest_dst){
       farthest_dst = dst;
       farthest_i = i;
     }
  }
  return farthest_i;
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
//  ROS_INFO("BRAIN: ARM - tilt from %.2f -> %.2f",arm1_tilt_msg.data,new_tilt);
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
geometry_msgs::PoseStamped get_ave_pose(nav_msgs::Path pathin){
  geometry_msgs::PoseStamped ps;
  float sum_x  = 0; float sum_y = 0; float sum_z  = 0; float sum_yaw = 0;
  float zmn = 100; float zmx = -100;
  int down_count = 0;
  for(int i = 0; i < pathin.poses.size(); i++){
    if(pathin.poses[i].pose.orientation.y == 0.7071)
      down_count++;
    else
      sum_yaw += tf::getYaw(pathin.poses[i].pose.orientation);
    sum_x += pathin.poses[i].pose.position.x;
    sum_y += pathin.poses[i].pose.position.y;
    sum_z += pathin.poses[i].pose.position.z;
    if(zmn > pathin.poses[i].pose.position.z)
      zmn = pathin.poses[i].pose.position.z;
    if(zmx > pathin.poses[i].pose.position.z)
      zmx = pathin.poses[i].pose.position.z;
  }
  int side_count = pathin.poses.size() - down_count;
  if(side_count > down_count)
    ps.pose.orientation = tf::createQuaternionMsgFromYaw(sum_yaw / side_count);
  else
    ps.pose.orientation.y = ps.pose.orientation.w = 0.7071;
  ps.pose.position.x = sum_x / pathin.poses.size();
  ps.pose.position.y = sum_y / pathin.poses.size();
  ps.pose.position.z = sum_z / pathin.poses.size();
  if(down_count > side_count){
    ps.pose.position.z = zmx + 4;
  }
  return ps;
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

void hdngclear_cb(const nav_msgs::Path::ConstPtr& msg){
  path_clear_vlp = *msg;
}
void eval_tilting_v2(){
  float lowest_dst = 100;
  float lowest_dst_hdng = 0;
  int lowest_dst_i;
  for(int i = 2; i < path_clear_vlp.poses.size(); i++){
    float dst  = get_dst3d(path_clear_vlp.poses[i].pose.position,path_clear_vlp.poses[0].pose.position);
    float hdng = get_hdng(path_clear_vlp.poses[i].pose.position,path_clear_vlp.poses[0].pose.position);
  //  ROS_INFO("dst %.0f hdng: %.2f",dst,hdng);
    if(hdng < 0.05 && hdng > -0.05){
      if(dst < lowest_dst){
        lowest_dst = dst;
        lowest_dst_i = i;
      }
    }
  }
  if(lowest_dst < 18){
    increment_tilt_degrees(-5);
  }
  else if(lowest_dst > 22){
    increment_tilt_degrees(5);
  }
  else{

  }
  ROS_INFO("Lowest_dst_i[%i],dst: %.0f, hdng: %.2f",lowest_dst_i,lowest_dst,lowest_dst_hdng);
}

nav_msgs::Path get_targets_remaining(nav_msgs::Path pathin){
  nav_msgs::Path pathout;
  pathout.header = hdr();
	int i0 = get_closest_i(pathin,pos);
	if(i0 < pathin.poses.size() && i0 > 0){
		for(int i = i0; i < path_targets.poses.size(); i++){
			pathout.poses.push_back(path_targets.poses[i]);
		}
	}
	else
		return pathin;
  return pathout;
}

float get_dst_path(nav_msgs::Path pathin){
  float dst_sum = 0;
  for(int i = 1; i < pathin.poses.size(); i++){
    dst_sum += get_dst3d(pathin.poses[i].pose.position,pathin.poses[i-1].pose.position);
  }
	return dst_sum;
}

void targetpose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
	target = *msg;
}

void targetpath_cb(const nav_msgs::Path::ConstPtr& msg){
	path_targets = *msg;
	if(msg->poses.size() > 2){
		get_targets_remaining(*msg);
	}
}
void lowrateodom_cb(const nav_msgs::Odometry::ConstPtr& msg){
  odom = *msg;
}

void pathvstd_cb(const nav_msgs::Path::ConstPtr& msg){
	path_visited = *msg;
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
	ROS_INFO("degrees: %i, pose_in: %i path_out: %i",degrees,pathin.poses.size(),pathout.poses.size());
	return pathout;
}
nav_msgs::Path cutoff_percentage(nav_msgs::Path pathin,int percentage){
  nav_msgs::Path pathout;
  pathout.header = hdr();
  int percent_length = int(round(pathin.poses.size() * percentage / 100));
  for(int i = 0; i < fmax(percent_length,fmin(pathin.poses.size(),3)); i++){
    pathout.poses.push_back(pathin.poses[i]);
  }
  ROS_INFO("Percent: %i, pose_in: %i, percent_len: %i, path_out: %i",percentage,pathin.poses.size(),percent_length,pathout.poses.size());
  return pathout;
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
void pathrawside_cb(const nav_msgs::Path::ConstPtr& msg){
	path_raw_side = *msg;
	got_path_raw_side = true;
}
void pathrawdown_cb(const nav_msgs::Path::ConstPtr& msg){
	path_raw_down = *msg;
	got_path_raw_down = true;
}
void mainstate_cb(const std_msgs::UInt8::ConstPtr& msg){
  mainstate = msg->data;
}
void pathvlp_cb(const nav_msgs::Path::ConstPtr& msg){
  path_vlp = *msg;
	got_path_vlp = true;
  for(int i = 0; i < path_vlp.poses.size(); i++){
    update_centroid(path_vlp.poses[i].pose.position,"cleared");
  }
  if(msg->poses.size() > 0)
    base_pose = msg->poses[0];
}

void pathvlpobs_cb(const nav_msgs::Path::ConstPtr& msg){
  path_obs = *msg;
	got_path_obs = true;
  for(int i = 0; i < path_obs.poses.size(); i++){
    update_centroid(path_obs.poses[i].pose.position,"obstacle");
  }
}

void poylcleared_cb(const geometry_msgs::PolygonStamped::ConstPtr& msg){
	got_poly_cleared = true;
	poly_cleared 							 = *msg;
  poly_cleared_centroid      = get_poly_centroidarea(poly_cleared);
  poly_cleared_centroid_area = poly_cleared_centroid.z;
  poly_cleared_centroid.z    = base_pose.pose.position.z;
}
void tiltctrlmode_cb(const std_msgs::String::ConstPtr& msg){

}
int main(int argc, char** argv)
{
  ros::init(argc, argv, "tb_control_node");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");
	private_nh.param("par_vlpmaxtilt",par_vlpmaxtilt, M_PI/10);
	private_nh.param("par_vlpmintilt",par_vlpmintilt, -M_PI/10);
	private_nh.param("par_vlptiltinterval",par_vlptiltinterval, M_PI/10);
	private_nh.param("takeoff_altlvl",par_takeoffaltitude, 5.0);
	tf2_ros::TransformListener tf2_listener(tfBuffer);
	std_msgs::Float64 target_alt_msg;

  ros::Subscriber s0 = nh.subscribe("/tb_path/cleared_poly",1,poylcleared_cb);
	ros::Subscriber s2 = nh.subscribe("/tb_path/obstacles",1,pathvlpobs_cb);
	ros::Subscriber ass5 = nh.subscribe("/tb_path/cleared",10,pathvlp_cb);
	ros::Subscriber s5 = nh.subscribe("/tb_path/visited",10,pathvstd_cb);
	ros::Subscriber ss5 = nh.subscribe("/tb_path/raw_down",10,pathrawdown_cb);
	ros::Subscriber sd5 = nh.subscribe("/tb_path/raw_side",10,pathrawside_cb);
	ros::Subscriber s19 = nh.subscribe("/tb_path/hdng_clear",10,hdngclear_cb);
	ros::Subscriber ss9 = nh.subscribe("/cmd_pose",10,targetpose_cb);
	ros::Subscriber a8 = nh.subscribe("/tb_behav/targets",100,&targetpath_cb);
	ros::Subscriber s8 = nh.subscribe("/tb_fsm/main_state",100,&mainstate_cb);
	ros::Subscriber s9 = nh.subscribe("/tb_nav/lowrate_odom",10,lowrateodom_cb);


	ros::Publisher pub_altcmd	 = nh.advertise<std_msgs::Float64>("/tb_cmd/alt_target", 100);
	ros::Publisher pub_tiltvlp = nh.advertise<std_msgs::Float64>("/tb_cmd/arm1_tilt", 10);
	ros::Publisher pub_tilt  	 = nh.advertise<std_msgs::Float64>("/tb_cmd/arm2_tilt", 10);
	ros::Publisher pub_pan	   = nh.advertise<std_msgs::Float64>("/tb_cmd/arm2_pan", 10);

	ros::Rate rate(2.0);

  while(ros::ok()){
    checktf();
		ROS_INFO("%i %i %i %i %i",got_path_raw_side ,got_path_raw_down ,got_path_vlp ,got_path_obs ,got_poly_cleared);
		if(got_path_raw_side && got_path_raw_down && got_path_vlp && got_path_obs && got_poly_cleared){
			got_path_raw_side  =false;
			ROS_INFO("Inspection type: %s, pathlen: %i ",inspection_type.c_str(),path_targets.poses.size());
			got_path_raw_down  =false;
			got_path_vlp =false;
			got_path_obs =false;
			got_poly_cleared =false;
			count_target_paths++;
			img_blank.copyTo(img);

			draw_path_at_img(path_clear_vlp,base_pose.pose.position,false,false,false,false,true,get_color(100,0,0),1);
			draw_path_at_img(path_vlp,base_pose.pose.position,false,true,false,false,true,get_color(30,30,30),1);
			draw_path_at_img(path_raw_side,base_pose.pose.position,false,true,false,false,true,get_color(0,100,0),1);
			draw_path_at_img(path_raw_down,base_pose.pose.position,false,false,false,false,true,get_color(100,0,0),1);
			draw_path_at_img(path_obs,base_pose.pose.position,false,false,false,true,false,get_color(0,0,200),1);
			draw_pose(pos,tf::getYaw(base_pose.pose.orientation),5,get_color(200,200,200));
			draw_pose(target.pose.position,tf::getYaw(base_pose.pose.orientation),5,get_color(155,155,255));
			draw_poly(poly_cleared,get_color(0,100,100));
			cv::imwrite("/home/nuc/brain/control/"+std::to_string(count_target_paths)+"_raw.png",img);
			img_blank.copyTo(img);

			path_vlp			 = cutoff_abs(sort_path(path_vlp,"hdng_abs"),40);
			path_raw_side	 = cutoff_abs(sort_path(path_raw_side,"hdng_abs"),40);
			path_raw_down	 = cutoff_abs(sort_path(path_raw_down,"hdng_abs"),40);
			path_obs			 = cutoff_abs(sort_path(path_obs,"hdng_abs"),40);
			path_clear_vlp = cutoff_abs(sort_path(path_clear_vlp,"hdng_abs"),40);
			float zmx_s = -1;
			float zmx_d = -1;
			float zmx_o = -1;
			float zmx_do = -1;
			float dst_sf,dst_sc,dst_of,dst_oc,dst_cf,dst_cc,dst_df,dst_dc;
			geometry_msgs::Point farthest_side,closest_side,farthest_down,closest_down,closest_obs,farthest_obs,farthest_clear,closest_clear;

			if(path_obs.poses.size() > 0){
				zmx_o = get_zmax(path_obs);
				zmx_do = zmx_o;
				closest_obs = path_obs.poses[get_closest_i(path_obs,pos)].pose.position;
				farthest_obs = path_obs.poses[get_farthest_i(path_obs,pos)].pose.position;
				dst_oc = get_dst3d(closest_obs,pos);
				dst_of = get_dst3d(farthest_obs,pos);
				ROS_INFO("closest  obst: [%.2f] %.2f %.2f %.2f",dst_oc,closest_obs.x,closest_obs.y,closest_obs.z);
				ROS_INFO("farthest obst [%.2f] %.2f %.2f %.2f",dst_of,farthest_obs.x,farthest_obs.y,farthest_obs.z);
			}
			if(path_raw_side.poses.size() > 0){
				zmx_s = get_zmax(path_raw_side);
				farthest_side = path_raw_side.poses[get_farthest_i(path_raw_side,pos)].pose.position;
				closest_side  = path_raw_side.poses[get_closest_i(path_raw_side,pos)].pose.position;
				dst_sc = get_dst3d(closest_side,pos);
				dst_sf = get_dst3d(farthest_side,pos);
				ROS_INFO("closest  side [%.2f] %.2f %.2f %.2f",dst_sc,closest_side.x,closest_side.y,closest_side.z);
				ROS_INFO("farthest  side [%.2f] %.2f %.2f %.2f",dst_sf,farthest_side.x,farthest_side.y,farthest_side.z);
			}
			if(path_raw_down.poses.size() > 0){
				zmx_d = get_zmax(path_raw_down);
				zmx_do = fmax(zmx_d,zmx_o);
				farthest_down = path_raw_down.poses[get_farthest_i(path_raw_down,pos)].pose.position;
				closest_down  = path_raw_down.poses[get_closest_i(path_raw_down,pos)].pose.position;
				dst_dc = get_dst3d(closest_down,pos);
				dst_df = get_dst3d(farthest_down,pos);
				ROS_INFO("closest down [%.2f] %.2f %.2f %.2f",dst_dc,closest_down.x,closest_down.y,closest_down.z);
				ROS_INFO("farthest down [%.2f] %.2f %.2f %.2f",dst_df,farthest_down.x,farthest_down.y,farthest_down.z);
			}
			if(path_clear_vlp.poses.size() > 0){
				farthest_clear = path_clear_vlp.poses[get_farthest_i(path_clear_vlp,pos)].pose.position;
				closest_clear  = path_clear_vlp.poses[get_closest_i(path_clear_vlp,pos)].pose.position;
				dst_cc = get_dst3d(closest_clear,pos);
				dst_cf = get_dst3d(farthest_clear,pos);
				ROS_INFO("closest clear [%.2f] %.2f %.2f %.2f",dst_cc,closest_clear.x,closest_clear.y,closest_clear.z);
				ROS_INFO("farthest clear [%.2f] %.2f %.2f %.2f",dst_cf,farthest_clear.x,farthest_clear.y,farthest_clear.z);
			}
			ROS_INFO("Zmax: %.2f %.2f %.2f -> zmx %.2f",zmx_d,zmx_s,zmx_o,zmx_do);

			if(path_obs.poses.size() == 0){
				increment_tilt_degrees(10);
			}
			else if(zmx_o < pos.z - 5){
				target_alt_msg.data = zmx_o + 3;
			}
			else if(zmx_o - pos.z > -1){
				set_tilt(0);
			}
			else if(poly_cleared_centroid_area < 500){
				increment_tilt_degrees(-10);
			}


			ROS_INFO("poly_area: %.0f pathsizes: RSIDE: %i RDOWN: %i vlp: %i clear: %i obs: %i ",poly_cleared_centroid_area,path_raw_side.poses.size(),path_raw_down.poses.size(),
			path_vlp.poses.size(),path_clear_vlp.poses.size(),path_obs.poses.size());

			for(int i = 0; i < path_raw_side.poses.size(); i++){

			}
			for(int i = 0; i < path_raw_down.poses.size(); i++){

			}
			for(int i = 0; i < path_vlp.poses.size(); i++){

			}
			for(int i = 0; i < path_clear_vlp.poses.size(); i++){

			}
			for(int i = 0; i < path_obs.poses.size(); i++){

			}
			draw_path_at_img(path_clear_vlp,base_pose.pose.position,true,false,false,false,true,get_color(100,0,0),1);
			draw_path_at_img(path_vlp,base_pose.pose.position,false,false,false,false,true,get_color(30,30,30),1);
			draw_path_at_img(path_raw_side,base_pose.pose.position,false,true,false,false,true,get_color(0,100,0),1);
			draw_path_at_img(path_raw_down,base_pose.pose.position,false,false,false,false,true,get_color(100,0,0),1);
			draw_path_at_img(path_obs,base_pose.pose.position,false,true,true,false,false,get_color(0,0,100),1);
			draw_path_at_img(path_targets,base_pose.pose.position,true,true,true,false,false,get_color(0,0,200),1);
			draw_pose(pos,tf::getYaw(base_pose.pose.orientation),5,get_color(200,200,200));
			draw_poly(poly_cleared,get_color(0,100,100));
			draw_pose(target.pose.position,tf::getYaw(base_pose.pose.orientation),5,get_color(155,155,255));
			cv::imwrite("/home/nuc/brain/control/"+std::to_string(count_target_paths)+"_done.png",img);
		}
		target_alt_msg.data = 20;
  //  	target_alt_msg.data = target.pose.position.z;
    pub_tiltvlp.publish(arm1_tilt_msg);
    pub_altcmd.publish(target_alt_msg);
    rate.sleep();
    ros::spinOnce();
  }
  return 0;
}
