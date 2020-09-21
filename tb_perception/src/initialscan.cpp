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
sensor_msgs::LaserScan scan_lo,scan_mi,scan_hi,scan_up,scan_dn,scan_st;

cv::Scalar c_lo,c_dn,c_mi,c_up,c_h,c_hi,c_sz;
geometry_msgs::PointStamped obs_p,obs_r,obs_l,obs_m,obs_b;

ros::Publisher pub_cmdexplore,pub_cmdmb,pub_target_dash,pub_path_best,pub_get_next_path,pub_cmd,pub_cmdpose;
geometry_msgs::PoseStamped pose_down,pose_side,target,base_pose,target_last,target_final,target_dash;
ros::Time activity_change,last_tilt,path_complete_time;
std_msgs::Float64 arm1_tilt_msg;
geometry_msgs::Vector3 vlp_rpy;
geometry_msgs::Point cmd_pos,poly_cleared_centroid,pos,pnt_midpoint,pnt_ref;
nav_msgs::Odometry odom;
nav_msgs::Path scanzone_path,path_ds_full,raw_full,path_raw_side,path_raw_down,path_target_full,path_clear_vlp,path_down_best_in_poly,path_side_best_in_poly,path_full,path_world_visible,path_down_best,path_side_best,path_vlp,path_obs,path_side_full,path_down_full,path_targets,path_visited,path_cleared_full,path_obstacles_full,path_targets_sent,path_side,path_down;
int mainstate;
int counter = 0;
int blankdraw_counter = 0;
int path_targets_i = 0;
int inspection_count = 0;
double par_vlpmaxtilt,par_vlpmintilt,par_vlptiltinterval,par_takeoffaltitude;
bool path_complete,tiltlimit_reached,tilting,side_empty;
cv::Mat img(1000,1000,CV_8UC3,cv::Scalar(0, 0, 0)); //create image, set encoding and size, init pixels to default val
cv::Mat img_super_down(1000,1000,CV_8UC3,cv::Scalar(0, 0, 0)); //create image, set encoding and size, init pixels to default val
cv::Mat img_super_side(1000,1000,CV_8UC3,cv::Scalar(0, 0, 0)); //create image, set encoding and size, init pixels to default val
cv::Mat img_super(1000,1000,CV_8UC3,cv::Scalar(0, 0, 0)); //create image, set encoding and size, init pixels to default val
cv::Mat img_blank(1000,1000,CV_8UC3,cv::Scalar(0, 0, 0)); //create image, set encoding and size, init pixels to default val
float rad2deg = 180.0/M_PI;
float hdng_cutoff = M_PI/8;
bool override = false;
int count_target_paths = 0;
int targets_down_in_poly,targets_side_in_poly;
int dash_stage = 0;
bool got_one,got_one_raw;
float altmax,dst_target,poly_cleared_centroid_area,pos_yaw,scanrange_actual;
std::vector<int> targets_complete;
tb_msgsrv::Paths paths_active_down,down_in_poly;
tb_msgsrv::Paths paths_active_side,side_in_poly;
geometry_msgs::PolygonStamped poly_cleared,target_final_poly;
tb_msgsrv::Paths paths_down,paths_side;
double par_zclearing,par_scanrange;
geometry_msgs::Point scanpoint_actual2,scanpoint_actual,scanpoint_actual_lo,scanpoint_actual_hi;
float target_angle,setpoint_altitude,setpoint_scanrange;
nav_msgs::Path path_lo,path_lo_mi;
nav_msgs::Path path_mi,path_mi_mi;
nav_msgs::Path path_hi,path_hi_mi;
nav_msgs::Path path_up,path_up_mi;
nav_msgs::Path path_dn,path_dn_mi;
nav_msgs::Path path_st,path_st_mi;
nav_msgs::Path path_up_ri,path_up_le,path_mi_ri,path_mi_le,path_hi_ri,path_hi_le,path_lo_ri,path_lo_le,path_dn_ri,path_dn_le,path_st_ri,path_st_le;
bool use_old;
bool use_raw_for_target = true;
int override_tilt = 0;
int override_alt = 0;
int scanzone_size_pnts;
float scanzone_size_area,scanzone_size_ddst,scanzone_size_dalt,scanzone_dst_min,scanzone_dst_max,scanzone_z_min,scanzone_z_max;
geometry_msgs::Point scanzone_centroid,scanzone_avepnt;
geometry_msgs::Point scanzone_bbmin,scanzone_bbmax;
std_msgs::Header hdr(){
  std_msgs::Header header;
  header.frame_id = "map";
  header.stamp    = ros::Time::now();
  return header;
}
double constrainAngle(double x){
  if(x > M_PI)
    return (x - M_PI*2);
  else if(x < -M_PI)
    return (x + M_PI*2);
  else
    return x;
}
float relval(float min,float max,float val){
  return 255*(val - min) / (max - min);
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

float get_hdng(geometry_msgs::Point p1,geometry_msgs::Point p0){
  float dx = p1.x - p0.x;
  float dy = p1.y - p0.y;
  return atan2(dy,dx);
}
float get_dst2d(geometry_msgs::Point p1, geometry_msgs::Point p2){
  return(sqrt(pow(p1.x-p2.x,2)+pow(p1.y-p2.y,2)));
}

float get_dst3d(geometry_msgs::Point p1, geometry_msgs::Point p2){
  return(sqrt(pow(p1.x-p2.x,2)+pow(p1.y-p2.y,2)+pow(p1.z-p2.z,2)));
}
float get_inclination(geometry_msgs::Point p2, geometry_msgs::Point p1){
  return atan2(p2.z - p1.z,get_dst2d(p1,p2));
}
float get_shortest(float target_heading,float actual_hdng){
  float a = target_heading - actual_hdng;
  if(a > M_PI)a -= M_PI*2;
  else if(a < -M_PI)a += M_PI*2;
  return a;
}

geometry_msgs::PolygonStamped get_polyfinal(float l0,float l1,float delta_hdng){
  geometry_msgs::PolygonStamped poly;
  poly.header.frame_id = "map";
  int num_rays    = 16;
  poly.polygon.points.resize(num_rays*2);
  float rads_pr_i = delta_hdng/num_rays;
  float hdng0     = get_hdng(target.pose.position,target_last.pose.position) - delta_hdng/2;
  for(int i = 0; i < num_rays; i++){
    float a = hdng0 + rads_pr_i * i;
    poly.polygon.points[i].x = target.pose.position.x + l0 * cos(a);
    poly.polygon.points[i].y = target.pose.position.y + l0 * sin(a);
    poly.polygon.points[poly.polygon.points.size()-1-i].x = target.pose.position.x + l1 * cos(a);
    poly.polygon.points[poly.polygon.points.size()-1-i].y = target.pose.position.y + l1 * sin(a);
  }
  return poly;
}

void draw(std::string type){
	cv::circle(img,pnt2cv(pos),2,get_color(255,255,0),1);
	count_target_paths++;
	cv::imwrite("/home/nuc/brain/draw/"+std::to_string(count_target_paths)+type+".png",img);
	img_blank.copyTo(img);
}
void draw_line(geometry_msgs::Point p0,float a,float r,cv::Scalar color){
  geometry_msgs::Point pyaw;
  pyaw.x = p0.x + r * cos(a);
  pyaw.y = p0.y + r * sin(a);
  cv::line (img, pnt2cv(pos), pnt2cv(pyaw),color,1,cv::LINE_8,0);
}

void initialscan(sensor_msgs::LaserScan scanin){
		geometry_msgs::Point current_object;
		std::vector<geometry_msgs::Point>cluster;
    std::vector<float> hdngs;
    std::vector<bool> is_clear;
    std::vector<float> hdng_start_obs;
    std::vector<float> hdng_end_obs;
    std::vector<float> obstacle_clusters_r;
    std::vector<float> obstacle_clusters_z;
    std::vector<float> cleared_clusters_ends;
    std::vector<float> obstacle_clusters_starts;
    std::vector<float> obstacle_clusters_ends;
    std::vector<float> cleared_clusters_starts;

    std::vector<geometry_msgs::Point> obstacle_clusters_center;
    std::vector<std::vector<geometry_msgs::Point>>cleared_clusters;
		std::vector<std::vector<geometry_msgs::Point>>obstacle_clusters;
    bool last_clear = false;
    float zmx = 0;
    bool clear;
    float r_sum = 0;
    int largest_cleared_cluster = 0;
    int largest_obstacle_cluster = 0;

    float cleared_lim = 30;
    if(std::isinf(scanin.ranges[0]))
      last_clear = true;
    if(last_clear)
      cleared_clusters_starts.push_back(scanin.angle_min);
    else
      obstacle_clusters_starts.push_back(scanin.angle_min);
    int count_ten = 0;
    for(int i  = 1; i < scanin.ranges.size()-1; i++){
      geometry_msgs::PointStamped pnt,pnt_out;
      float current_angle = scanin.angle_min + scanin.angle_increment * i;
      hdngs.push_back(current_angle);
      //ROS_INFO("Current angle: %.2f range: %.2f cluster: %i",current_angle,scanin.ranges[i],cluster.size());
      clear = false;
      pnt.point.x = scanin.ranges[i] * cos(current_angle);
      pnt.point.y = scanin.ranges[i] * sin(current_angle);
      pnt.header.frame_id = scanin.header.frame_id;
      pnt.header.stamp    = ros::Time();
      pnt_out = tfBuffer.transform(pnt, "map");
      if(scanin.ranges[i] > scanin.ranges[i+1] && scanin.ranges[i] > scanin.ranges[i-1] && scanin.ranges[i] > cleared_lim){
        obstacle_clusters_center.push_back(pnt_out.point);
        cv::circle(img,pnt2cv(pnt_out.point),1,get_color(200,200,200),1);
      }
      count_ten++;
      if(scanin.ranges[i] > cleared_lim)
        clear = true;
      if(count_ten > 10){
        count_ten = 0;
      if(clear)
        draw_line(pos,current_angle,scanin.ranges[i],get_color(relval(0,50,scanin.ranges[i]),relval(0,50,scanin.ranges[i]),0));
      else
        draw_line(pos,current_angle,scanin.ranges[i],get_color(0,0,relval(0,50,scanin.ranges[i])));
      }

    //  cv::circle(img,pnt2cv(pnt_out.point),1,get_color(200,200,0),1);

        // std::isinf(scanin.ranges[i-1]) && std::isinf(scanin.ranges[i]) && std::isinf(scanin.ranges[1+i]))


    //  cv::circle(img,pnt2cv(pnt_out.point),1,get_color(0,0,relval(0,50,pnt_out.point.z)),1);

      is_clear.push_back(clear);
      if(last_clear && !clear){
        zmx = 0;
        cleared_clusters_ends.push_back(current_angle);
        obstacle_clusters_starts.push_back(current_angle);
        cleared_clusters.push_back(cluster);
        if(cluster.size() > largest_cleared_cluster)
          largest_cleared_cluster = cluster.size();
        cluster.resize(0);
      }
      else if(!last_clear && clear){
        float ranges_ave = r_sum / cluster.size();
        r_sum = 0;
        obstacle_clusters_z.push_back(zmx);
        obstacle_clusters_r.push_back(ranges_ave);
        if(cluster.size() > largest_obstacle_cluster)
          largest_obstacle_cluster = cluster.size();
        obstacle_clusters_ends.push_back(current_angle);
        cleared_clusters_starts.push_back(current_angle);
        obstacle_clusters.push_back(cluster);
        cluster.resize(0);
      }
      else if(last_clear && clear){

      }
      else if(!last_clear && !clear){

      }
      else{
        ROS_INFO("ERROR - impossible combinatio (CLEAR vs OBST)");
      }

      cluster.push_back(pnt_out.point);
      last_clear = clear;
      if(!clear){
        r_sum += scanin.ranges[i];
        if(pnt_out.point.z > zmx)
          zmx = pnt_out.point.z;
      }
    }
  for(int i  = 0; i < cleared_clusters.size(); i++){
    int num  = cleared_clusters[i].size();
    float a0 = cleared_clusters_starts[i];
    float aN = cleared_clusters_ends[i];
    float a  = (aN+a0)/2.0;
    draw_line(pos,a,50,get_color(0,200*num/largest_cleared_cluster,0));
    float ad = aN - a0;
    ROS_INFO("Cluster[%i], size: %i a: %.2f (delta. %.2f rads) (a0 %.2f->%.2f an)",i,num,a,ad,a0,aN);
  }
  for(int i  = 0; i < obstacle_clusters.size(); i++){
    int num  = obstacle_clusters[i].size();
    float a0 = obstacle_clusters_starts[i];
    float aN = obstacle_clusters_ends[i];
    float a  = (aN+a0)/2.0;
    float r  = (aN+a0)/2.0;
    float z  = (aN+a0)/2.0;
    draw_line(pos,a,r,get_color(0,0,200*num/largest_obstacle_cluster));

    ROS_INFO("Obstacles[%i], size: %i a: %.2f, zmax: %.2f dst_ave: %.2f",i,num,a,obstacle_clusters_z[i],obstacle_clusters_r[i]);
  }
}

void initialscan2(sensor_msgs::LaserScan scanin){
		geometry_msgs::Point current_object;
		std::vector<geometry_msgs::Point>cluster;
    std::vector<float> hdngs;
    std::vector<bool> is_clear;
    std::vector<float> hdng_start_obs;
    std::vector<float> hdng_end_obs;
    std::vector<float> obstacle_clusters_r;
    std::vector<float> obstacle_clusters_z;
    std::vector<float> cleared_clusters_ends;
    std::vector<float> obstacle_clusters_starts;
    std::vector<float> obstacle_clusters_ends;
    std::vector<float> cleared_clusters_starts;

    std::vector<geometry_msgs::Point> obstacle_clusters_center;
    std::vector<std::vector<geometry_msgs::Point>>cleared_clusters;
		std::vector<std::vector<geometry_msgs::Point>>obstacle_clusters;
    bool last_clear = false;
    float zmx = 0;
    bool clear;
    float r_sum = 0;
    int largest_cleared_cluster = 0;
    int largest_obstacle_cluster = 0;

    scanin = get_ranges(scanin,15);
    float cleared_lim = 30;
    if(std::isinf(scanin.ranges[0]))
      last_clear = true;
    if(last_clear)
      cleared_clusters_starts.push_back(scanin.angle_min);
    else
      obstacle_clusters_starts.push_back(scanin.angle_min);
    int count_ten = 0;
    for(int i  = 1; i < scanin.ranges.size()-1; i++){
      geometry_msgs::PointStamped pnt,pnt_out;
      float current_angle = scanin.angle_min + scanin.angle_increment * i;
      hdngs.push_back(current_angle);
      //ROS_INFO("Current angle: %.2f range: %.2f cluster: %i",current_angle,scanin.ranges[i],cluster.size());
      clear = false;
      pnt.point.x = scanin.ranges[i] * cos(current_angle);
      pnt.point.y = scanin.ranges[i] * sin(current_angle);
      pnt.header.frame_id = scanin.header.frame_id;
      pnt.header.stamp    = ros::Time();
      pnt_out = tfBuffer.transform(pnt, "map");
      if(scanin.ranges[i] > scanin.ranges[i+1] && scanin.ranges[i] > scanin.ranges[i-1] && scanin.ranges[i] > cleared_lim && scanin.ranges[i] < scanin.range_max){
        obstacle_clusters_center.push_back(pnt_out.point);
        cv::circle(img,pnt2cv(pnt_out.point),1,get_color(200,200,200),1);
      }
      count_ten++;
      if(scanin.ranges[i] > cleared_lim)
        clear = true;
      if(count_ten > 10){
        count_ten = 0;
      if(clear)
        draw_line(pos,current_angle,scanin.ranges[i],get_color(relval(0,50,scanin.ranges[i]),relval(0,50,scanin.ranges[i]),0));
      else
        draw_line(pos,current_angle,scanin.ranges[i],get_color(0,0,relval(0,50,scanin.ranges[i])));
      }

    //  cv::circle(img,pnt2cv(pnt_out.point),1,get_color(200,200,0),1);

        // std::isinf(scanin.ranges[i-1]) && std::isinf(scanin.ranges[i]) && std::isinf(scanin.ranges[1+i]))


    //  cv::circle(img,pnt2cv(pnt_out.point),1,get_color(0,0,relval(0,50,pnt_out.point.z)),1);

      is_clear.push_back(clear);
      if(last_clear && !clear){
        zmx = 0;
        cleared_clusters_ends.push_back(current_angle);
        obstacle_clusters_starts.push_back(current_angle);
        cleared_clusters.push_back(cluster);
        if(cluster.size() > largest_cleared_cluster)
          largest_cleared_cluster = cluster.size();
        cluster.resize(0);
      }
      else if(!last_clear && clear){
        float ranges_ave = r_sum / cluster.size();
        r_sum = 0;
        obstacle_clusters_z.push_back(zmx);
        obstacle_clusters_r.push_back(ranges_ave);
        if(cluster.size() > largest_obstacle_cluster)
          largest_obstacle_cluster = cluster.size();
        obstacle_clusters_ends.push_back(current_angle);
        cleared_clusters_starts.push_back(current_angle);
        obstacle_clusters.push_back(cluster);
        cluster.resize(0);
      }
      else if(last_clear && clear){

      }
      else if(!last_clear && !clear){

      }
      else{
        ROS_INFO("ERROR - impossible combinatio (CLEAR vs OBST)");
      }

      cluster.push_back(pnt_out.point);
      last_clear = clear;
      if(!clear){
        r_sum += scanin.ranges[i];
        if(pnt_out.point.z > zmx)
          zmx = pnt_out.point.z;
      }
    }
  for(int i  = 0; i < cleared_clusters.size(); i++){
    int num  = cleared_clusters[i].size();
    float a0 = cleared_clusters_starts[i];
    float aN = cleared_clusters_ends[i];
    float a  = (aN+a0)/2.0;
    draw_line(pos,a,50,get_color(0,200*num/largest_cleared_cluster,0));
    float ad = aN - a0;
    ROS_INFO("Cluster[%i], size: %i a: %.2f (delta. %.2f rads) (a0 %.2f->%.2f an)",i,num,a,ad,a0,aN);
  }
  for(int i  = 0; i < obstacle_clusters.size(); i++){
    int num  = obstacle_clusters[i].size();
    float a0 = obstacle_clusters_starts[i];
    float aN = obstacle_clusters_ends[i];
    float a  = (aN+a0)/2.0;
    float r  = (aN+a0)/2.0;
    float z  = (aN+a0)/2.0;
    draw_line(pos,a,r,get_color(0,0,200*num/largest_obstacle_cluster));

    ROS_INFO("Obstacles[%i], size: %i a: %.2f, zmax: %.2f dst_ave: %.2f",i,num,a,obstacle_clusters_z[i],obstacle_clusters_r[i]);
  }
}
void scan_cb(const sensor_msgs::LaserScan scanin){
	initialscan(scanin);
	draw("laser");
}
int main(int argc, char** argv)
{
  ros::init(argc, argv, "tb_behavior_node");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");
	tf2_ros::TransformListener tf2_listener(tfBuffer);

  ros::Subscriber a6        = nh.subscribe("/scan_stabilized",10,scan_cb);
  return 0;
}
