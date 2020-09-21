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
tf2_ros::Buffer tfBuffer;

using namespace octomap;
using namespace std;
shared_ptr<DynamicEDTOctomap> edf_ptr;
AbstractOcTree* abs_octree;
shared_ptr<OcTree> octree;
string par_workdir_path;
geometry_msgs::Point bbmin_custom,bbmax_custom,bbmin_octree,bbmax_octree,pos,last_pos;
int xmin,ymin,zmin,xmax,ymax,zmax,range_x,range_y,range_z,zmin_global,zmax_global;
int zlvl;
double par_maprad,par_maxobs,par_minobs,last_yaw,par_zjump;
std::vector<int> z_lvls;
bool got_map,par_unknownAsOccupied;
geometry_msgs::PointStamped pnt_closest,pnt_midpoint;
geometry_msgs::PolygonStamped poly_safe,poly_hdng,poly_vstd,poly_vlp;
nav_msgs::Path path_vlp,path_candidates_sides,path_visited,path_lo,path_mid,path_hi,path_candidates,path_floor,path_floor_grid;
float pos_yaw;
ros::Publisher pub_path,path_pub,pub_path_down,pub_path_side;
geometry_msgs::Vector3 vlp_rpy;
/////////////GLOBAL IMG/////////////////////7
geometry_msgs::Point hdng_maxhdng_maxincline;


int area_sidelength,grid_sidelength,num_gridsprside;
int num = 0;
double par_grid_size,par_area_sidelength;

cv::Mat mapimg_copy(300,300,CV_8UC3,cv::Scalar(0, 0, 0)); //create image, set encoding and size, init pixels to default val
cv::Mat img(300,300,CV_8UC3,cv::Scalar(0, 0, 0)); //create image, set encoding and size, init pixels to default val
cv::Mat img_copy(300,300,CV_8UC3,cv::Scalar(0, 0, 0)); //create image, set encoding and size, init pixels to default val
cv::Mat mapimg_sides(300,300,CV_8UC3,cv::Scalar(0, 0, 0)); //create image, set encoding and size, init pixels to default val
cv::Mat mapimg(300,300,CV_8UC3,cv::Scalar(0, 0, 0)); //create image, set encoding and size, init pixels to default val
cv::Mat mapimg_down(300,300,CV_8UC3,cv::Scalar(0, 0, 0)); //create image, set encoding and size, init pixels to default val

std::vector<geometry_msgs::PolygonStamped> objects_polys;
std::vector<geometry_msgs::PolygonStamped> cleared_polys;
std::vector<geometry_msgs::PolygonStamped> polygon_grids;
std::vector<geometry_msgs::PoseStamped> targetcmds_sent;
std::vector<geometry_msgs::Point> grids_centroids;
std::vector<std::vector<int>> grids_at_ranges;
cv::Scalar visited_color,candidate_color,target_color,grid_color,obstacle_color,building_color,cleared_color,path_color;
geometry_msgs::PoseStamped last_target,target;
bool in_poly(geometry_msgs::PolygonStamped polyin, float x, float y){
	ros::Time t0 = ros::Time::now();
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
//	float dt = (ros::Time::now() - t0).toSec();
//	ROS_INFO("sr: %.5f",dt);
  return bool(cross % 2);
}

void define_colors(){
  visited_color[0] = 150;
  visited_color[1] = 50;
  visited_color[2] = 0;

  target_color[0] = 0;
  target_color[1] = 100;
  target_color[2] = 150;

  candidate_color[0] = 100;
  candidate_color[1] = 100;
  candidate_color[2] = 0;

  grid_color[0] =  0;
  grid_color[1] =  20;
  grid_color[2] =  0;

  building_color[0] = 0;
  building_color[1] = 0;
  building_color[2] = 100;

  obstacle_color[0] = 0;
  obstacle_color[1] = 0;
  obstacle_color[2] = 150;

  cleared_color[0] = 100;
  cleared_color[1] = 150;
  cleared_color[2] = 0;

  path_color[0] = 150;
  path_color[1] = 150;
  path_color[2] = 150;
}
float y2r(float y, float rows,float res){
  return (rows / 2 - y / res);
}
float x2c(float x, float cols,float res){
  return (x / res + cols/2);
}
int x2gx(float x){
  return int(round((x+area_sidelength/2) / grid_sidelength));
}
int y2gy(float y){
  return int(round((y+area_sidelength/2) / grid_sidelength));
}
int gxgy2gn(int gx, int gy){
  return num_gridsprside * gy + gx;
}
geometry_msgs::Point gn2gxgy(int gn){
  int gy = gn / num_gridsprside;
  int gx = gn - gy * num_gridsprside;
  geometry_msgs::Point pout;
  pout.x = gx;
  pout.y = gy;
  return pout;
}
int xy2gn(geometry_msgs::Point pnt){
  int gx = x2gx(pnt.x);
  int gy = y2gy(pnt.y);
  ROS_INFO("g_x %i, y: %i",gx,gy);
  return gxgy2gn(gx,gy);
}
int xy2gn32(geometry_msgs::Point32 pnt){
  int gx = x2gx(pnt.x);
  int gy = y2gy(pnt.y);
  return gxgy2gn(gx,gy);
}
int r2y(float r, float rows,float res){
  return int((rows / 2 - r) * res);
}
int c2x(float c, float cols,float res){
  return int((c - cols / 2) * res);
}

cv::Point pnt322cv(geometry_msgs::Point32 pin){
	int c = x2c(pin.x,img.cols,1);
	int r = y2r(pin.y,img.rows,1);
	return cv::Point(c,r);
}
void draw_line32(geometry_msgs::Point32 pnt0,geometry_msgs::Point32 pnt1, cv::Scalar color,bool copy){
  if(copy)
    cv::line (img_copy, pnt322cv(pnt0), pnt322cv(pnt1), color,1,cv::LINE_8,0);
  else
    cv::line (img, pnt322cv(pnt0), pnt322cv(pnt1), color,1,cv::LINE_8,0);
}
void draw_rectangle32(geometry_msgs::Point32 p0,geometry_msgs::Point32 p1,cv::Scalar color,bool copy){
  if(copy)
    cv::rectangle(img_copy, pnt322cv(p0),pnt322cv(p1),color,1,8,0);
  else
    cv::rectangle(img, pnt322cv(p0),pnt322cv(p1),color,1,8,0);
}
void draw_circle32(geometry_msgs::Point32 pnt,int size,cv::Scalar color,bool copy){
  if(copy)
    cv::circle(img_copy,pnt322cv(pnt),size,color,1);
  else
    cv::circle(img,pnt322cv(pnt),size,color,1);
}
void draw_grids(std::vector<int> grids_to_draw,cv::Scalar color,bool copy){
	if(grids_to_draw.size() < 2)
		return;
	for(int i = 0; i < grids_to_draw.size()-1; i++){
    draw_rectangle32(polygon_grids[grids_to_draw[i]].polygon.points[1],polygon_grids[grids_to_draw[i]].polygon.points[3],color,copy);
  }
	ROS_INFO("GridsToDraw: %i",grids_to_draw.size());
}
void draw_polygon(geometry_msgs::PolygonStamped polyin,cv::Scalar color,bool copy){
  ROS_INFO("draw_polygond");
	if(polyin.polygon.points.size() > 2){
		for(int i = 0; i < polyin.polygon.points.size()-1; i++){
			draw_line32(polyin.polygon.points[i],polyin.polygon.points[i+1],color,copy);
		}
	}
	ROS_INFO("PolygonToDraw: %i pints",polyin.polygon.points.size());
}

cv::Point pnt2cv(geometry_msgs::Point pin){
	int c = x2c(pin.x,img.cols,1);
	int r = y2r(pin.y,img.rows,1);
	return cv::Point(c,r);
}


void draw_rectangle(geometry_msgs::Point p0,geometry_msgs::Point p1,cv::Scalar color,bool copy){
  if(copy)
    cv::rectangle(img_copy, pnt2cv(p0),pnt2cv(p1),color,1,8,0);
  else
    cv::rectangle(img, pnt2cv(p0),pnt2cv(p1),color,1,8,0);
}
void draw_circle(geometry_msgs::Point pnt,float size,cv::Scalar color,bool copy){
  if(copy)
    cv::circle(img_copy,pnt2cv(pnt),int(round(size)),color,1);
  else
    cv::circle(img,pnt2cv(pnt),int(round(size)),color,1);
}
void draw_line(geometry_msgs::Point pnt0,geometry_msgs::Point pnt1, cv::Scalar color,bool copy){
  if(copy)
    cv::line (img_copy, pnt2cv(pnt0), pnt2cv(pnt1), color,1,cv::LINE_8,0);
  else
    cv::line (img, pnt2cv(pnt0), pnt2cv(pnt1), color,1,cv::LINE_8,0);
}
void draw_pose2d(geometry_msgs::Point p0,float size,cv::Scalar color,bool copy,double yaw){
  geometry_msgs::Point p1;
  p1.x = p0.x + size*2 * cos(yaw);
  p1.y = p0.y + size*2 * sin(yaw);
  draw_circle(p0,size,color,copy);
  draw_line(p0,p1,color,copy);
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

void create_polygon_grids(geometry_msgs::Point midpoint,float rad){
	geometry_msgs::Point p;
  grids_centroids.resize(0);
  polygon_grids.resize(0);
  for(int y = 0; y < num_gridsprside; y++){
    for(int x = 0; x < num_gridsprside; x++){
			p.x      = x * grid_sidelength - area_sidelength / 2 + grid_sidelength/2;
      p.y      = y * grid_sidelength - area_sidelength / 2 + grid_sidelength/2;
      if(p.x > pnt_midpoint.point.x - rad && p.y > pnt_midpoint.point.y - rad &&
         p.x < pnt_midpoint.point.x + rad && p.y < pnt_midpoint.point.y + rad){
  			grids_centroids.push_back(p);
  			polygon_grids.push_back(createpoly_square(p,grid_sidelength));
      }
		}
  }
}
std::vector<int> grids_in_poly_safe(geometry_msgs::PolygonStamped polyin){
	std::vector<int> grids_out;
	if(polyin.polygon.points.size() == 0)
		return grids_out;
	for(int i = 0; i < grids_centroids.size(); i++){
    int not_in_poly = 0;
    for(int k = 0; k < polygon_grids[i].polygon.points.size(); k++){
      if(!in_poly(polyin,polygon_grids[i].polygon.points[k].x,polygon_grids[i].polygon.points[k].y)){
        not_in_poly++;
      }
    }
    if(not_in_poly == 0)
      grids_out.push_back(i);
	}
	if(grids_out.size() > 2)
		ROS_INFO("GridsInPoly: %i -> %i, total %i points from %i points large poly",grids_out.size(),grids_out[0],grids_out[grids_out.size()-1],polyin.polygon.points.size());
	return grids_out;
}

std::vector<int> grids_in_poly(geometry_msgs::PolygonStamped polyin){
	std::vector<int> grids_out;
	if(polyin.polygon.points.size() == 0)
		return grids_out;
	for(int i = 0; i < grids_centroids.size(); i++){
		if(in_poly(polyin,grids_centroids[i].x,grids_centroids[i].y))
			grids_out.push_back(i);
	}
	if(grids_out.size() > 2)
		ROS_INFO("GridsInPoly: %i -> %i, total %i points from %i points large poly",grids_out.size(),grids_out[0],grids_out[grids_out.size()-1],polyin.polygon.points.size());
	return grids_out;
}
void draw_path(nav_msgs::Path pathin,int size, cv::Scalar color,bool copy, bool draw_lines){
  for(int k = 0; k < pathin.poses.size(); k++){
    if(k < pathin.poses.size()-1 && draw_lines)
      draw_line(pathin.poses[k].pose.position,pathin.poses[k+1].pose.position,color,copy);
    draw_pose2d(pathin.poses[k].pose.position,2,color,copy, tf::getYaw(pathin.poses[k].pose.orientation));
  }
}
void draw_paths(bool copy,bool draw_lines){
  if(path_lo.poses.size() > 1){
    draw_path(path_lo,2,candidate_color,copy,draw_lines);
  }
  if(path_mid.poses.size() > 1){
    draw_path(path_mid,3,visited_color,copy,draw_lines);
  }
  if(path_hi.poses.size() > 1){
    draw_path(path_hi,3,target_color,copy,draw_lines);
  }
}
int get_gridnum(float x, float y){
  for(int i = 0; i < grids_centroids.size(); i++){
    if(x <= polygon_grids[i].polygon.points[0].x && x >= polygon_grids[i].polygon.points[2].x
    && y <= polygon_grids[i].polygon.points[0].y && y >= polygon_grids[i].polygon.points[2].y)
      return i;
  }
  ROS_INFO("ERROR; NOT RETURNED GRIDNUMBER FOR POINT %.2f %.2f",x,y);
  return 0;
}
void draw_grid(int i,cv::Scalar color,bool copy){
  draw_rectangle32(polygon_grids[i].polygon.points[1],polygon_grids[i].polygon.points[3],color,copy);
}
void draw_grids_from_path(nav_msgs::Path pathin){
  for(int i = 0; i < pathin.poses.size(); i++){
    cv::Scalar obstacle_color2;
    obstacle_color2 = obstacle_color;
    obstacle_color2[2] = pathin.poses[i].pose.position.z * 10;
    int ii = get_gridnum(pathin.poses[i].pose.position.x,pathin.poses[i].pose.position.y);
		if(ii > 0)
    	draw_grid(ii,obstacle_color2,false);
  }
}

/////////////GLOBAL IMG/////////////////////7

std_msgs::Header hdr(){
  std_msgs::Header header;
  header.frame_id = "map";
  header.stamp    = ros::Time::now();
  return header;
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

float get_dst2d(geometry_msgs::Point p1, geometry_msgs::Point p2){
  return(sqrt(pow(p1.x-p2.x,2)+pow(p1.y-p2.y,2)));
}
float get_dst3d(geometry_msgs::Point p1, geometry_msgs::Point p2){
  return(sqrt(pow(p1.x-p2.x,2)+pow(p1.y-p2.y,2)+pow(p1.z-p2.z,2)));
}

bool sort_dst_pair(const std::tuple<int,float>& a,
               const std::tuple<int,float>& b)
{
    return (std::get<1>(a) < std::get<1>(b));
}

nav_msgs::Path sort_path_by_dst(nav_msgs::Path pathin,geometry_msgs::Point pnt0,bool use_3d){
  nav_msgs::Path pathout;
  if(pathin.poses.size() <= 1)
    return pathin;
  pathout.header = pathin.header;
  std::vector<std::tuple<int,float>>i_dst;
  for(int i = 0; i < pathin.poses.size(); i++){
    if(use_3d)
      i_dst.push_back(std::make_tuple(i,get_dst3d(pnt0,pathin.poses[i].pose.position)));
    else
      i_dst.push_back(std::make_tuple(i,get_dst2d(pnt0,pathin.poses[i].pose.position)));
  }
  sort(i_dst.begin(),i_dst.end(),sort_dst_pair);
  for(int i = 0; i < i_dst.size(); i++){
    pathout.poses.push_back(pathin.poses[std::get<0>(i_dst[i])]);
  }
	return pathout;
}

nav_msgs::Path sort_path_by_zabs(nav_msgs::Path pathin){
  nav_msgs::Path pathout;
  if(pathin.poses.size() <= 1)
    return pathin;
  pathout.header = pathin.header;
  std::vector<std::tuple<int,float>>i_dst;
  for(int i = 0; i < pathin.poses.size(); i++){
      i_dst.push_back(std::make_tuple(i,abs(pathin.poses[i].pose.position.z - pos.z)));
  }
  sort(i_dst.begin(),i_dst.end(),sort_dst_pair);
  for(int i = 0; i < i_dst.size(); i++){
    pathout.poses.push_back(pathin.poses[std::get<0>(i_dst[i])]);
  }
	return pathout;
}
nav_msgs::Path sort_path_by_hdng(nav_msgs::Path pathin){
  float rad2deg = 180.0/M_PI;
  nav_msgs::Path pathout;
  if(pathin.poses.size() <= 1)
    return pathin;
  pathout.header = pathin.header;
  std::vector<std::tuple<int,float>>i_dst;
  for(int i = 0; i < pathin.poses.size(); i++){
    	float hdng_shortest = abs(get_shortest(get_hdng(pathin.poses[i].pose.position,pos),pos_yaw) * rad2deg);
      i_dst.push_back(std::make_tuple(i,hdng_shortest));
  }
  sort(i_dst.begin(),i_dst.end(),sort_dst_pair);
  for(int i = 0; i < i_dst.size(); i++){
    pathout.poses.push_back(pathin.poses[std::get<0>(i_dst[i])]);
  }
	return pathout;
}

void checktf(){
  geometry_msgs::TransformStamped transformStamped;
  try{
    transformStamped = tfBuffer.lookupTransform("map","base_stabilized",
                             ros::Time(0));
  }
  catch (tf2::TransformException &ex) {
    ROS_WARN("%s",ex.what());
    ros::Duration(1.0).sleep();
  }

	pos.x = transformStamped.transform.translation.x;
	pos.y = transformStamped.transform.translation.y;
	pos.z = transformStamped.transform.translation.z;
	pos_yaw = tf::getYaw(transformStamped.transform.rotation);
}

void midpoint_cb(const geometry_msgs::PointStamped::ConstPtr& msg){
    pnt_midpoint = *msg;
}
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
float getinpath_closestdst(nav_msgs::Path pathin,geometry_msgs::PoseStamped pose_to_check,bool use_3d){
  float lowest_dist = 1000;  float dst;
  if(pathin.poses.size() == 0){
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
nav_msgs::Path constrain_path_vstd(nav_msgs::Path pathin,nav_msgs::Path pathin_vstd,float cutoff_dst,bool use_3d){
  nav_msgs::Path pathout;
  pathout.header = hdr();
  if(fmin(pathin.poses.size(),pathin_vstd.poses.size()) == 0){
    return pathin;
  }
  for(int i = 0; i < pathin.poses.size(); i++){
    if(getinpath_closestdst(pathin_vstd,pathin.poses[i],use_3d) > cutoff_dst)
      pathout.poses.push_back(pathin.poses[i]);
  }
  return pathout;
}

bool pose_is_side(geometry_msgs::PoseStamped pose_in){
  if(pose_in.pose.orientation.y = 0.7071)
    return false;
  else
    return true;
}
nav_msgs::Path get_path_side(nav_msgs::Path pathin,bool get_side){
  nav_msgs::Path pathout;
  pathout.header = hdr();
  for(int i = 0; i < pathin.poses.size(); i++){
    if(get_side && pose_is_side(pathin.poses[i]))
      pathout.poses.push_back(pathin.poses[i]);
    if(!get_side && !pose_is_side(pathin.poses[i]))
      pathout.poses.push_back(pathin.poses[i]);
  }
  return pathout;
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

void polyvlp_cb(const geometry_msgs::PolygonStamped::ConstPtr& msg){
	poly_vlp = *msg;
}
int closest_in_path(nav_msgs::Path pathin,geometry_msgs::Point pnt){
  float res,dst;
  res = 1000;
  int res_i;
  if(pathin.poses.size() == 0)
    return -1;
  for(int i = 1; i < pathin.poses.size(); i++){
    dst = get_dst3d(pathin.poses[i].pose.position,pnt);
    if(dst < res){
      res = dst;
      res_i = i;
    }
  }
  return res_i;
}


nav_msgs::Path create_path(int num_xy, int num_z, float dxy, float dz){
	nav_msgs::Path pathout;
	for(int z = 0; z < num_z; z++){
		for(int y = 0; y < num_xy; y++){
			for(int x = 0; x < num_xy; x++){
				geometry_msgs::PoseStamped pose;
				pose.pose.position.x    = x * dxy - num_xy/2 * dxy;
				pose.pose.position.y    = y * dxy - num_xy/2 * dxy;
				pose.pose.position.z    = z * dz;
				pose.pose.orientation.w = 1;
				pose.header.frame_id     = "map";
				pose.header.stamp       = ros::Time::now();
				pathout.poses.push_back(pose);
			}
		}
	}
	pathout.header.frame_id     = "map";
	return pathout;
}
nav_msgs::Path cutoff_percentage(nav_msgs::Path pathin,int percentage,std::string frame){
  nav_msgs::Path pathout;
  pathout.header = hdr();
	cv::Mat mapimg(300,300,CV_8UC3,cv::Scalar(0, 0, 0)); //create image, set encoding and size, init pixels to default val
	cv::Mat mapimg_side(300,300,CV_8UC3,cv::Scalar(0, 0, 0)); //create image, set encoding and size, init pixels to default val
  int percent_length = int(round(pathin.poses.size() * percentage / 100));
	int percent_end = fmax(percent_length,fmin(pathin.poses.size(),3));
	int tot_len = pathin.poses.size();
  for(int i = 0; i < percent_end; i++){
    pathout.poses.push_back(pathin.poses[i]);
		mapimg.at<cv::Vec3b>( y2r(pathin.poses[i].pose.position.y,mapimg.rows,1),x2c(pathin.poses[i].pose.position.x,mapimg.cols,1) )[1] = (1.0 - i / tot_len) * 100;
		mapimg_side.at<cv::Vec3b>( y2r(pathin.poses[i].pose.position.z,mapimg.rows,1),x2c(get_dst2d(pathin.poses[i].pose.position,pos),mapimg.cols,1) )[0] = (1.0 - i / tot_len) * 100;
  }
	for(int i = percent_end; i < pathin.poses.size(); i++){
		mapimg.at<cv::Vec3b>( y2r(pathin.poses[i].pose.position.y,mapimg.rows,1),x2c(pathin.poses[i].pose.position.x,mapimg.cols,1) )[0] = (1.0 - i / tot_len) * 100;
		mapimg_side.at<cv::Vec3b>( y2r(pathin.poses[i].pose.position.z,mapimg.rows,1),x2c(get_dst2d(pathin.poses[i].pose.position,pos),mapimg.cols,1) )[1] = (1.0 - i / tot_len) * 100;
	}
	cv::circle(mapimg,pnt2cv(pos),3,visited_color,1);
	cv::imwrite("/home/nuc/brain" + frame + "_path_img.png",mapimg);
	cv::imwrite("/home/nuc/brain" + frame + "_path_img_side.png",mapimg_side);
  ROS_INFO("Percent: %i, pose_in: %i, percent_len: %i, path_out: %i",percentage,pathin.poses.size(),percent_length,pathout.poses.size());
  return pathout;
}
void test(){
	nav_msgs::Path path;
	path.header = hdr();
}

void draw_path_sides(nav_msgs::Path pathin,int size, int dst_pos){
	for(int i = 0; i < pathin.poses.size(); i++){
		cv::Scalar color_tar;
		cv::Scalar color_obs;
		geometry_msgs::Point p1,p2;
		color_tar[1] = i * 5;
		color_obs[2] = i * 5;
		float yaw = tf::getYaw(pathin.poses[i].pose.orientation);
		p1.x = pathin.poses[i].pose.position.x + size * cos(yaw);
		p1.y = pathin.poses[i].pose.position.y + size * sin(yaw);
		p2.x = pathin.poses[i].pose.position.x + dst_pos * cos(yaw);
		p2.y = pathin.poses[i].pose.position.y + dst_pos * sin(yaw);
		cv::line (img_copy, pnt2cv(pathin.poses[i].pose.position), pnt2cv(p1), color_obs,1,cv::LINE_8,0);
		cv::circle(img_copy,pnt2cv(pathin.poses[i].pose.position),size,color_obs,1);
		cv::circle(img_copy,pnt2cv(p2),size,color_tar,1);
		if(i > 1)
			cv::line (img_copy, pnt2cv(pathin.poses[i-1].pose.position), pnt2cv(pathin.poses[i].pose.position), color_obs,size,cv::LINE_8,0);
	}
}
void draw_path_floor(nav_msgs::Path pathin,int size){
	for(int i = 0; i < pathin.poses.size(); i++){
		cv::Scalar color;
		geometry_msgs::Point p1,p0;
		p0.x = path_floor.poses[i].pose.position.x-size;
		p0.y = path_floor.poses[i].pose.position.y-size;
		p1.x = p0.x + size*2;
		p1.y = p0.y + size*2;
		color[0] = path_floor.poses[i].pose.position.z * 10;
		cv::rectangle(img, pnt2cv(p0),pnt2cv(p1),color,1,8,0);
	}
}
void draw_pos(int size){
	cv::Scalar color;
	geometry_msgs::Point p1,p0;

	color[0] = 100;
	color[1] = 100;

	p1.x = pos.x + 3 * cos(pos_yaw);
	p1.y = pos.y + 3 * sin(pos_yaw);

	cv::line (mapimg, pnt2cv(pos), pnt2cv(p1), color,1,cv::LINE_8,0);
	cv::circle(mapimg,pnt2cv(pos),1,color,1);
}
void draw_target(geometry_msgs::PoseStamped target_pose,int size){
	cv::Scalar color,color_tar;
	geometry_msgs::Point p1,p0,p2;

	color[2] = 255 - pos.z * 10;
	color[0] = target_pose.pose.position.z * 10;
	color_tar[2] = 255;
	if(target_pose.pose.orientation.y == 0.7071){
		p0.x = target_pose.pose.position.x-size;
		p0.y = target_pose.pose.position.y-size;
		p1.x = target_pose.pose.position.x + size*2;
		p1.y = target_pose.pose.position.y + size*2;
		cv::rectangle(mapimg, pnt2cv(p0),pnt2cv(p1),color,1,8,0);
	}
	else{
		float yaw_tar = tf::getYaw(target_pose.pose.orientation);
		p1.x = target_pose.pose.position.x + size*2 * cos(yaw_tar);
		p1.y = target_pose.pose.position.y + size*2 * sin(yaw_tar);

		p2.x = target_pose.pose.position.x + 10 * cos(yaw_tar);
		p2.y = target_pose.pose.position.y + 10 * sin(yaw_tar);

		cv::line (mapimg, pnt2cv(target_pose.pose.position), pnt2cv(p1), color,1,cv::LINE_8,0);
		cv::circle(mapimg,pnt2cv(target_pose.pose.position),size,color,1);

		cv::circle(mapimg,pnt2cv(p2),1,color_tar,1);
	}
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
void drawtest(nav_msgs::Path pathin){
	img_copy.copyTo(img);
	int size=1;
	cv::Scalar color,color_tar;
	color[1] = 255;
	geometry_msgs::Point cur_tar = target.pose.position;
	float cur_yaw = tf::getYaw(target.pose.orientation);

	geometry_msgs::Point cur_obs = get_pose_pair(target);
	cv::circle(img,pnt2cv(cur_tar),size,color,1);
	cv::circle(img,pnt2cv(cur_obs),size,color,1);

	for(int i = 0; i < pathin.poses.size(); i++){
			geometry_msgs::Point cand_tar  = pathin.poses[i].pose.position;
			geometry_msgs::Point cand_obs  = get_pose_pair(pathin.poses[i]);
			float dst_pos = get_dst2d(cur_tar,cand_tar);
			float dst_obs = get_dst2d(cur_obs,cand_obs);
			int points = (dst_pos + dst_pos);
			color[0] = points;
			color[1] = 0;
			color[2] = 100;
			cv::line (img, pnt2cv(cur_tar), pnt2cv(cand_tar), color,1,cv::LINE_8,0);
		//	cv::line (img, pnt2cv(cur_obs), pnt2cv(cand_obs), color,1,cv::LINE_8,0);
	//		cv::circle(img,pnt2cv(cand_obs),size,color,1);
			cv::circle(img,pnt2cv(cand_tar),size,color,1);
			ROS_INFO("Dst_pos %.0f dst_obs: %.0f points: %i",dst_pos,dst_obs,points);
	}
	cv::imwrite("/home/nuc/brain/drawtesthdng.png",img);

}
/*
nav_msgs::Path get_path(nav_msgs::Path pathin){
	nav_msgs::Path pathout;
	pathout.header = hdr();
	std::vector<int> vec_i;
	std::vector<float> vec_dst;
	geometry_msgs::PoseStamped current_pose;
	geometry_msgs::Point current_tar = target.pose.position;
	geometry_msgs::Point current_obs = get_pose_pair(target);
	float current_yaw = tf::getYaw(target.pose.orientation);
	float dst_sum = 0;
	float dst_ave = 0;
	while(vec_i.size() < pathin.poses.size()){
		int new_i = get_closest_pose_not_in_vec(pathin,vec_i,current_tar,current_obs,current_yaw);
		float dst = get_dst3d(current_tar,pathin.poses[new_i].pose.position);
		current_tar = pathin.poses[new_i].pose.position;
		current_obs = get_pose_pair(pathin.poses[new_i]);
		current_yaw = tf::getYaw(pathin.poses[new_i].pose.orientation);
		if(dst_sum > 10 && dst > dst_ave * 3){
			ROS_INFO("Path BREAK after %i poses, %.0f meters average: %.0f m/pr_pose",vec_i.size(),dst_sum,dst_ave);
			break;
		}
		vec_i.push_back(new_i);
		dst_sum += dst;
		dst_ave = dst / vec_i.size();
	}
	ROS_INFO("Pathout: %i poses",pathout.poses.size());
	return pathout;
}*/
int get_closest_pose_not_in_vec(nav_msgs::Path pathin,std::vector<int> vec,geometry_msgs::Point cur_tar, geometry_msgs::Point cur_obs, float cur_yaw, float cur_hdng){
	float best_tot = 1000;
	int best_tot_i;
	for(int i = 0; i < pathin.poses.size(); i++){
		if(!in_vec(vec,i)){
			geometry_msgs::Point cand_tar  = pathin.poses[i].pose.position;
			geometry_msgs::Point cand_obs  = get_pose_pair(pathin.poses[i]);
			float 			    		 cand_yaw  = tf::getYaw(pathin.poses[i].pose.orientation);
			float 							 cand_hdng = get_hdng(cand_tar,cur_tar);
			float dst_hdng = get_shortest(cand_hdng,cur_hdng);
			float dst_yaw  = get_shortest(cand_yaw,cur_yaw);
			float dst_pos  = get_dst3d(cur_tar,cand_tar);
			float dst_obs  = get_dst3d(cur_obs,cand_obs);
			float dst_tot  = dst_pos + dst_obs;
			if(dst_tot < best_tot && abs(dst_hdng) < 3){
				best_tot_i = i;
				best_tot = dst_tot;
			}
		}
	}
	return best_tot_i;
}
nav_msgs::Path get_path(nav_msgs::Path pathin){
	nav_msgs::Path pathout;
	pathout.header = hdr();
	std::vector<int> vec_i;
	geometry_msgs::PoseStamped current_pose;
	geometry_msgs::Point current_tar = target.pose.position;
	geometry_msgs::Point current_obs = get_pose_pair(target);
	float current_yaw  = tf::getYaw(target.pose.orientation);
	float current_hdng = get_hdng(current_tar,last_target.pose.position);
	while(vec_i.size() < pathin.poses.size()){
		int new_i 	 = get_closest_pose_not_in_vec(pathin,vec_i,current_tar,current_obs,current_yaw,current_hdng);
		if(new_i == 1000)
			break;
		current_hdng = get_hdng(pathin.poses[new_i].pose.position,current_tar);
		current_tar  = pathin.poses[new_i].pose.position;
		current_obs  = get_pose_pair(pathin.poses[new_i]);
		current_yaw  = tf::getYaw(pathin.poses[new_i].pose.orientation);
		vec_i.push_back(new_i);
	}
	float dst,dst_sum,dst_ave;
	geometry_msgs::Point p0,p1;
	dst_sum = 0;
	for(int i = 0; i < vec_i.size(); i++){
/*		if(pathout.poses.size() > 0){
			dst = get_dst2d(pathout.poses[pathout.poses.size()-1].pose.position,pathin.poses[vec_i[i]].pose.position);
			dst_sum += dst;
			dst_ave = dst_sum / i;
			ROS_INFO("pathin[%i] dst: %,0f dst_ave: %.0f",vec_i[i],dst,dst_ave);
			if(dst_sum > 20 && dst_sum > dst_ave * 3){
				ROS_INFO("Pathout: %i poses",pathout.poses.size());
				return pathout;
			}
		}*/
		pathout.poses.push_back(pathin.poses[vec_i[i]]);
	}
	ROS_INFO("Pathout: %i poses",pathout.poses.size());
	return pathout;
}
nav_msgs::Path cutoff_at_zabs(nav_msgs::Path pathin,int cutoff_zabs){
	nav_msgs::Path pathout;
  pathout.header = hdr();
	for(int i = 0; i < pathin.poses.size(); i++){
		if(abs(pathin.poses[i].pose.position.z - pos.z) < cutoff_zabs)
			pathout.poses.push_back(pathin.poses[i]);
		else
			return pathout;
	}
	return pathout;
}
int find_percentage(nav_msgs::Path pathin){
	int current_zabs = 0;
	int last_change = 0;
	int i_return;
	for(int i = 0; i < pathin.poses.size(); i++){
		int zabs = abs(pathin.poses[i].pose.position.z - pos.z);
		if(zabs != current_zabs){
			if(current_zabs == 0)
				i_return = i;
			int percent_len = i - last_change;
			int percent_int = percent_len / pathin.poses.size() * 100;
			ROS_INFO("zabs change %i -> %i after %i poses (%i percent)",current_zabs,zabs,percent_len,percent_int);
			current_zabs = zabs;
			last_change  = i;
		}
	}
	return i_return;
}
nav_msgs::Path cutoff_abs(nav_msgs::Path pathin){
	nav_msgs::Path pathout;
	pathout.header = hdr();
	int cutoff_at = find_percentage(pathin);
	for(int i = 0; i < fmin(cutoff_at,pathin.poses.size()); i++){
		pathout.poses.push_back(pathin.poses[i]);
	}
	int percen_int = pathout.poses.size() / pathin.poses.size() * 100;
	ROS_INFO("Percent: percent_len: %i, path_in: %i path_out: %i",percen_int,pathin.poses.size(),pathout.poses.size());
	return pathout;
}



nav_msgs::Path cutoff_percentage2(nav_msgs::Path pathin,int percentage){
  nav_msgs::Path pathout;
  pathout.header = hdr();
  int percent_length = int(round(pathin.poses.size() * percentage / 100));
  for(int i = 0; i < fmax(percent_length,fmin(pathin.poses.size(),3)); i++){
    pathout.poses.push_back(pathin.poses[i]);
  }
  ROS_INFO("Percent: %i, pose_in: %i, percent_len: %i, path_out: %i",percentage,pathin.poses.size(),percent_length,pathout.poses.size());
  return pathout;
}
nav_msgs::Path work_path2(nav_msgs::Path pathin){
  pathin = sort_path_by_zabs(pathin);
  pathin = cutoff_percentage2(pathin,30);
	draw_path_sides(pathin,1,10);
	cv::imwrite("/home/nuc/brain/"+std::to_string(num)+"zabs.png",img);
  pathin = sort_path_by_hdng(pathin);
  pathin = cutoff_percentage2(pathin,30);
	cv::imwrite("/home/nuc/brain/"+std::to_string(num)+"hdng.png",img);
  pathin = sort_path_by_dst(pathin,pos,true);
  return pathin;
}

int dst_point_in_path_lim_index(nav_msgs::Path pathin,geometry_msgs::Point pin,float lim){
  float res,dst;
  if(pathin.poses.size() == 0)
    return res;
  for(int i = 0; i < pathin.poses.size(); i++){
     if(get_dst2d(pathin.poses[i].pose.position,pin) < lim)
        return i;
  }
  return 0;
}

void pathfloor_cb(const nav_msgs::Path::ConstPtr& msg){
  path_floor = *msg;
}
void pathvstd_cb(const nav_msgs::Path::ConstPtr& msg){
	cv::Scalar color;
	for(int i = 0; i < msg->poses.size(); i++){
		color[0] = msg->poses[i].pose.position.z;
		color[2] = 100;
		cv::circle(img,pnt2cv(msg->poses[i].pose.position),1,color,1);
	}
	path_visited = *msg;
}
void pathcand_cb(const nav_msgs::Path::ConstPtr& msg){
	if(msg->poses.size() > 0){
		nav_msgs::Path path;
		img_copy.copyTo(img);
		//path = cutoff_percentage2(sort_path_by_zabs(*msg),70);
//		path = cutoff_abs(sort_path_by_zabs(*msg));
		path = cutoff_at_zabs(sort_path_by_zabs(*msg),3);
	//	draw_path_sides(path,1,10);
	//	cv::imwrite("/home/nuc/brain/"+std::to_string(num)+"test_in.png",img);
	//	img_copy.copyTo(img);
		pub_path.publish(get_path(path));
//		draw_path_sides(path,1,10);
//		cv::imwrite("/home/nuc/brain/"+std::to_string(num)+"test_out.png",img);

	//	drawtest(*msg);
		//		work_path2(*msg);
	//	draw_path_sides(*msg,1,10);
	}
	else{
		draw_path_floor(path_floor,1);
	}
//	geometry_msgs::Point p1;

//	nav_msgs::Path path;7
//	path = work_path(*msg);
}
void pathout_cb(const nav_msgs::Path::ConstPtr& msg){
	if(msg->poses.size() > 0){
		img.copyTo(img_copy);
		draw_path_sides(*msg,1,10);
		cv::imwrite("/home/nuc/brain/"+std::to_string(num)+"test_in.png",img_copy);
	}
	else
		draw_path_floor(path_floor,1);
}

void cmdpose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
	last_target = target;
	target = *msg;
	draw_target(*msg,1);
	num++;

	//cv::imwrite("/home/nuc/brain/"+std::to_string(num)+"brain_img.png",img);
	//img_copy.copyTo(img);
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


nav_msgs::Path create_paths_by_heading(nav_msgs::Path pathin_vlp,nav_msgs::Path pathin_cands){
	int zero_count = 0;

//	pathin_vlp = sort_path_by_hdng(pathin_vlp);
//	float current_heading = get_hdng(pathin_vlp.poses[pathin_vlp.poses[0].pose.position)
		std::vector<nav_msgs::Path> paths_at_hdngs;
		std::vector<int> obstacles_at_paths;
		int obstacle_count = 0;
		int highest_obstacle_count = 0;
		std::vector<int> index_hits;
		float grid_sidelength3 = 5;
		int highest_index = 0;
		nav_msgs::Path path_at_hdngs;
		int highest_obstacle_count_path = 0;

		cv::Scalar color_pos;
		geometry_msgs::Point pos_0,pos_0_yaw;
		pos_0 = path_vlp.poses[0].pose.position;
		float yaw_0 = tf::getYaw(path_vlp.poses[0].pose.orientation);

		color_pos[1] = 255;

		pos_0_yaw.x = pos_0.x + 4 * cos(yaw_0);
		pos_0_yaw.y = pos_0.y + 4 * sin(yaw_0);

		cv::line (mapimg_down, pnt2cv(pos), pnt2cv(pos_0_yaw), color_pos,1,cv::LINE_8,0);
		cv::circle(mapimg_down,pnt2cv(pos),2,color_pos,1);
		if(pathin_cands.poses.size() == 0){
			ROS_INFO("Down - retun pathin poses 0");
			return pathin_cands;
		}
	if(pathin_cands.poses.size() > 0 && pathin_vlp.poses.size() > 0){
		index_hits.resize(pathin_vlp.poses.size());
		for(int i = 0; i < pathin_cands.poses.size(); i++){
	//		mapimg_down.at<cv::Vec3b>( y2r(msg->poses[i].pose.position.y,mapimg_down.rows,1),x2c(msg->poses[i].pose.position.x,mapimg_down.cols,1) )[2] = 100;
			int closest = get_closest_pose(pathin_vlp,pathin_cands.poses[i].pose.position,6);
			if(closest > 0){
				index_hits[closest] = index_hits[closest] + 1;
				if(highest_index < index_hits[closest]){
					highest_index = index_hits[closest];
				}
			}
			else
				zero_count++;
		}
		int counting = 0;
		for(int i = 0; i < index_hits.size(); i++){
			cv::Scalar color;
			if(index_hits[i] > 0){
				counting++;
				if(index_hits[i] > highest_index * 0.7){
					color[1] = 255 * index_hits[i]/highest_index;
				}
				else
					color[2] = 255 * index_hits[i]/highest_index;
				cv::circle(mapimg_down,pnt2cv(path_vlp.poses[i].pose.position),1,color,1);
			}
		}
		ROS_INFO("DRAW: Pathvlp: %i, Obstacles: %i, highest_index: %i, total_indexes: %i zero: %i",pathin_vlp.poses.size(),pathin_cands.poses.size(),highest_index,counting,zero_count);

		float yaw = tf::getYaw(pathin_vlp.poses[0].pose.orientation);
		ROS_INFO("yaw: %.2f",yaw);

		path_at_hdngs.header = hdr();
		if(pathin_vlp.poses.size() < 2)
			return pathin_vlp;
		float hdng = get_hdng(pathin_vlp.poses[1].pose.position,pathin_vlp.poses[0].pose.position);

		for(int i = 0; i < pathin_vlp.poses.size(); i++){
			float new_hdng = get_hdng(pathin_vlp.poses[i].pose.position,pathin_vlp.poses[0].pose.position);
		//	ROS_INFO("new_hdng: %.2f,hdng: %.2f",new_hdng,hdng);
			obstacle_count += index_hits[i];
			if(round(hdng*10) != round(new_hdng*10)){
				hdng = new_hdng;
				if(path_at_hdngs.poses.size() > 1){
					path_at_hdngs = sort_path_by_dst(path_at_hdngs,pathin_vlp.poses[0].pose.position,false);
					float tot_dst = get_dst2d(path_at_hdngs.poses[path_at_hdngs.poses.size() - 1].pose.position,pathin_vlp.poses[0].pose.position);
					ROS_INFO("Old hdng %.2f [(%i poses, dst: %.0f, obstacles: %i] (New heading: %.2f)",hdng,path_at_hdngs.poses.size(),tot_dst,obstacle_count,new_hdng);
				}
				if(obstacle_count > highest_obstacle_count){
					highest_obstacle_count = obstacle_count;
					highest_obstacle_count_path = paths_at_hdngs.size();
					ROS_INFO("highest_obstacle_count %i, path#%i",highest_obstacle_count,highest_obstacle_count_path);
				}
				obstacles_at_paths.push_back(obstacle_count);
				paths_at_hdngs.push_back(path_at_hdngs);
				obstacle_count = 0;
				path_at_hdngs.poses.resize(0);
			}
			path_at_hdngs.poses.push_back(pathin_vlp.poses[i]);
		}
	}
	if(highest_obstacle_count == 0){
		path_at_hdngs.poses.resize(0);
		return path_at_hdngs;
	}
		ROS_INFO("Paths at hdngs: %i",paths_at_hdngs.size());
	for(int i = 0; i < paths_at_hdngs.size(); i++){

		cv::Scalar color;
		if(obstacles_at_paths[i] > highest_obstacle_count * 0.7)
			color[0] = 255 * obstacles_at_paths[i]/highest_index;
		else
			color[2] = 255 * obstacles_at_paths[i]/highest_index;
			if(paths_at_hdngs[i].poses.size() > 0)
				cv::line (mapimg_down, pnt2cv(paths_at_hdngs[i].poses[paths_at_hdngs[i].poses.size()-1].pose.position), pnt2cv(paths_at_hdngs[i].poses[0].pose.position), color,0,cv::LINE_8,0);

//	for(int k = 1; k < paths_at_hdngs[i].poses.size(); k++){

		//	cv::circle(mapimg_down,pnt2cv(paths_at_hdngs[i].poses[k].pose.position),1,color,1);
	//		cv::line (mapimg_down, pnt2cv(paths_at_hdngs[i].poses[k-1].pose.position), pnt2cv(paths_at_hdngs[i].poses[k].pose.position), color,0,cv::LINE_8,0);
	//	}
	}
	ROS_INFO("paths_at_hdngs[%i].poses.size() = ");
	if(paths_at_hdngs.size() > highest_obstacle_count_path){
		cv::Scalar color;
		color[0] = 50;
		color[2] = 50;
		for(int i = 0; i < paths_at_hdngs[highest_obstacle_count_path].poses.size(); i++){
			color[0] =paths_at_hdngs[highest_obstacle_count_path].poses[i].pose.position.z * 10;

			cv::circle(mapimg_down,pnt2cv(paths_at_hdngs[highest_obstacle_count_path].poses[i].pose.position),2,color_pos,1);
			if(i > 0)
				cv::line (mapimg_down, pnt2cv(paths_at_hdngs[highest_obstacle_count_path].poses[i].pose.position), pnt2cv(paths_at_hdngs[highest_obstacle_count_path].poses[i-1].pose.position), color,0,cv::LINE_8,0);
		}
		ROS_INFO("Down return %i",paths_at_hdngs[highest_obstacle_count_path].poses.size());

		return paths_at_hdngs[highest_obstacle_count_path];
	}
	else{
		path_at_hdngs.poses.resize(0);
		ROS_INFO("Down return0 ");

		return path_at_hdngs;
	}
}
nav_msgs::Path create_paths_by_heading_sides(nav_msgs::Path pathin_vlp,nav_msgs::Path pathin_cands){
	int zero_count = 0;
	ROS_INFO("SIDES");
//	pathin_vlp = sort_path_by_hdng(pathin_vlp);
//	float current_heading = get_hdng(pathin_vlp.poses[pathin_vlp.poses[0].pose.position)
	if(pathin_cands.poses.size() == 0)
		return pathin_cands;
		std::vector<nav_msgs::Path> paths_at_hdngs;
		std::vector<int> obstacles_at_paths;
		int obstacle_count = 0;
		int highest_obstacle_count = 0;
		std::vector<int> index_hits;
		float grid_sidelength3 = 5;
		int highest_index = 0;
		nav_msgs::Path path_at_hdngs;
		int highest_obstacle_count_path = 0;

		cv::Scalar color_pos;
		geometry_msgs::Point pos_0,pos_0_yaw;
		pos_0 = path_vlp.poses[0].pose.position;
		float yaw_0 = tf::getYaw(path_vlp.poses[0].pose.orientation);

		color_pos[1] = 255;

		pos_0_yaw.x = pos_0.x + 4 * cos(yaw_0);
		pos_0_yaw.y = pos_0.y + 4 * sin(yaw_0);

		cv::line (mapimg_sides, pnt2cv(pos), pnt2cv(pos_0_yaw), color_pos,1,cv::LINE_8,0);
		cv::circle(mapimg_sides,pnt2cv(pos),2,color_pos,1);

	if(pathin_cands.poses.size() > 0 && pathin_vlp.poses.size() > 0){
		index_hits.resize(pathin_vlp.poses.size());
		for(int i = 0; i < pathin_cands.poses.size(); i++){
	//		mapimg_sides.at<cv::Vec3b>( y2r(msg->poses[i].pose.position.y,mapimg_sides.rows,1),x2c(msg->poses[i].pose.position.x,mapimg_sides.cols,1) )[2] = 100;
			int closest = get_closest_pose(pathin_vlp,pathin_cands.poses[i].pose.position,6);
			if(closest > 0){
				index_hits[closest] = index_hits[closest] + 1;
				if(highest_index < index_hits[closest]){
					highest_index = index_hits[closest];
				}
			}
			else
				zero_count++;
		}
		int counting = 0;
		for(int i = 0; i < index_hits.size(); i++){
			cv::Scalar color;
			if(index_hits[i] > 0){
				counting++;
				if(index_hits[i] > highest_index * 0.7){
					color[1] = 255 * index_hits[i]/highest_index;
				}
				else
					color[2] = 255 * index_hits[i]/highest_index;
				cv::circle(mapimg_sides,pnt2cv(path_vlp.poses[i].pose.position),1,color,1);
			}
		}
		ROS_INFO("DRAW: Pathvlp: %i, Obstacles: %i, highest_index: %i, total_indexes: %i zero: %i",pathin_vlp.poses.size(),pathin_cands.poses.size(),highest_index,counting,zero_count);

		float yaw = tf::getYaw(pathin_vlp.poses[0].pose.orientation);
		ROS_INFO("sides yaw: %.2f",yaw);

		path_at_hdngs.header = hdr();
		if(pathin_vlp.poses.size() < 2)
			return pathin_vlp;
		float hdng = get_hdng(pathin_vlp.poses[1].pose.position,pathin_vlp.poses[0].pose.position);

		for(int i = 0; i < pathin_vlp.poses.size(); i++){
			float new_hdng = get_hdng(pathin_vlp.poses[i].pose.position,pathin_vlp.poses[0].pose.position);
		//	ROS_INFO("new_hdng: %.2f,hdng: %.2f",new_hdng,hdng);
			obstacle_count += index_hits[i];
			if(round(hdng*10) != round(new_hdng*10)){
				hdng = new_hdng;
				if(path_at_hdngs.poses.size() > 1){
					path_at_hdngs = sort_path_by_dst(path_at_hdngs,pathin_vlp.poses[0].pose.position,false);
					float tot_dst = get_dst2d(path_at_hdngs.poses[path_at_hdngs.poses.size() - 1].pose.position,pathin_vlp.poses[0].pose.position);
					ROS_INFO("sides Old hdng %.2f [(%i poses, dst: %.0f, obstacles: %i] (New heading: %.2f)",hdng,path_at_hdngs.poses.size(),tot_dst,obstacle_count,new_hdng);
				}
				if(obstacle_count > highest_obstacle_count){
					highest_obstacle_count = obstacle_count;
					highest_obstacle_count_path = paths_at_hdngs.size();
					ROS_INFO("sides highest_obstacle_count %i, path#%i",highest_obstacle_count,highest_obstacle_count_path);
				}
				obstacles_at_paths.push_back(obstacle_count);
				paths_at_hdngs.push_back(path_at_hdngs);
				obstacle_count = 0;
				path_at_hdngs.poses.resize(0);
			}
			path_at_hdngs.poses.push_back(pathin_vlp.poses[i]);
		}
	}
	if(highest_obstacle_count == 0){
		path_at_hdngs.poses.resize(0);
		return path_at_hdngs;
	}
	for(int i = 0; i < paths_at_hdngs.size(); i++){
		ROS_INFO("sides Paths at hdngs: %i",i);
		cv::Scalar color;
		if(obstacles_at_paths[i] > highest_obstacle_count * 0.7)
			color[0] = 255 * obstacles_at_paths[i]/highest_index;
		else
			color[2] = 255 * obstacles_at_paths[i]/highest_index;
//		for(int k = 1; k < paths_at_hdngs[i].poses.size(); k++){
			if(paths_at_hdngs[i].poses.size() > 0)
				cv::line (mapimg_sides, pnt2cv(paths_at_hdngs[i].poses[paths_at_hdngs[i].poses.size()-1].pose.position), pnt2cv(paths_at_hdngs[i].poses[0].pose.position), color,0,cv::LINE_8,0);

		//	cv::circle(mapimg_sides,pnt2cv(paths_at_hdngs[i].poses[k].pose.position),1,color,1);
	//		cv::line (mapimg_sides, pnt2cv(paths_at_hdngs[i].poses[k-1].pose.position), pnt2cv(paths_at_hdngs[i].poses[k].pose.position), color,0,cv::LINE_8,0);
//		}
	}
	ROS_INFO("sides1 Paths at hdngs: %i",paths_at_hdngs.size());

	if(paths_at_hdngs.size() > highest_obstacle_count_path){
		cv::Scalar color;
		color[0] = 50;
		color[2] = 50;
		for(int i = 0; i < paths_at_hdngs[highest_obstacle_count_path].poses.size(); i++){
			color[0] =paths_at_hdngs[highest_obstacle_count_path].poses[i].pose.position.z * 10;

			cv::circle(mapimg_sides,pnt2cv(paths_at_hdngs[highest_obstacle_count_path].poses[i].pose.position),2,color_pos,1);
			if(i > 0)
				cv::line (mapimg_sides, pnt2cv(paths_at_hdngs[highest_obstacle_count_path].poses[i].pose.position), pnt2cv(paths_at_hdngs[highest_obstacle_count_path].poses[i-1].pose.position), color,0,cv::LINE_8,0);
		}
		ROS_INFO("Sides return %i",paths_at_hdngs[highest_obstacle_count_path].poses.size());
		return paths_at_hdngs[highest_obstacle_count_path];
	}
	else{
		path_at_hdngs.poses.resize(0);
		ROS_INFO("Sides return0 ");

		return path_at_hdngs;
	}
}
void pathcandnew_cb(const nav_msgs::Path::ConstPtr& msg){
	if(path_vlp.poses.size() > 0){
		nav_msgs::Path pathout;
		img.copyTo(mapimg_down);
		pathout = create_paths_by_heading(path_vlp,*msg);
		cv::imwrite("/home/nuc/brain/test_out.png",mapimg_down);
		pub_path_down.publish(pathout);
	}
	else{
		ROS_INFO("PATH_VLP EMPTY");
	}
}
void pathcandsidesnew_cb(const nav_msgs::Path::ConstPtr& msg){
	if(path_vlp.poses.size() > 0){
		nav_msgs::Path pathout;
		img.copyTo(mapimg_sides);
		pathout = create_paths_by_heading_sides(path_vlp,*msg);
		cv::imwrite("/home/nuc/brain/test_outside.png",mapimg_sides);
		pub_path_side.publish(pathout);
	}
	else{
		ROS_INFO("PATH_VLP EMPTY");
	}
}

void pathvlp_cb(const nav_msgs::Path::ConstPtr& msg){
	path_vlp = *msg;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "tb_braindraw_node");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");
  private_nh.param("workdir_path",par_workdir_path);//*2.0);
  private_nh.param("par_maprad",  par_maprad, 30.0);//*2.0);

  /////////////GLOBAL IMG/////////////////////7
  private_nh.param("map_sidelength_min",  par_maprad, 50.0);
  private_nh.param("max_sidelength",      par_area_sidelength, 1000.0);
  private_nh.param("par_zjump",           par_zjump, 3.0);//*2.0);
	private_nh.param("grid_size",		        par_grid_size, 5.0);

  area_sidelength = int(round(par_area_sidelength));
  grid_sidelength = int(round(par_grid_size));
  num_gridsprside  = int(round(par_area_sidelength / par_grid_size));
  ROS_INFO("par_area_sidelength: %.2f par_grid_size %.2f",par_area_sidelength,par_grid_size);
  ROS_INFO("Area side: %i grid_side %i num_gridsprside: %i",area_sidelength,grid_sidelength,num_gridsprside);
  define_colors();

  tf2_ros::TransformListener tf2_listener(tfBuffer);

	//ros::Subscriber s5  = nh.subscribe("/tb_nav/lowrate_odom",10,lowrateodom_cb);
	ros::Subscriber s2   = nh.subscribe("/tb_path/visited",10,pathvstd_cb);
	ros::Subscriber s1   = nh.subscribe("/tb_path/candidates",10,pathcandnew_cb);
	ros::Subscriber s12  = nh.subscribe("/tb_path/candidates_sides",10,pathcandsidesnew_cb);
	ros::Subscriber s11  = nh.subscribe("/tb_path/vlp_path",10,pathvlp_cb);
  ros::Subscriber s6   = nh.subscribe("/tb_path/poly_vlp",10,polyvlp_cb);
  ros::Subscriber s9   = nh.subscribe("/tb_path/midpoint",10,midpoint_cb);
	ros::Subscriber s7   = nh.subscribe("/tb_path/floor",10,pathfloor_cb);

	//ros::Subscriber s3  = nh.subscribe("/tb_path/pathout",10,pathout_cb);
	ros::Subscriber s4  = nh.subscribe("/cmd_pose",10,cmdpose_cb);
	pub_path_down = nh.advertise<nav_msgs::Path>("/tb_path/out_down",10);
	pub_path_side = nh.advertise<nav_msgs::Path>("/tb_path/out_side",10);
	pub_path 			= nh.advertise<nav_msgs::Path>("/tb_path_tester",10);

	test();
  ROS_INFO("Ready to convert octomaps.");
  ros::Rate rate(1.0);
  while(ros::ok()){
		checktf();
		//draw_pos(3);

	/*	float rad_grids = 50;
		create_polygon_grids(pnt_midpoint.point,rad_grids);
		draw_grids_from_path(get_path_side(*msg,false));
		path_visited = constrain_path_bbpoly(*msg,poly_vlp);*/
		float size = 1;
/*Æ	for(int k = 0; k < path_floor.poses.size(); k++){

		}
		img.copyTo(img_copy);

		for(int k = 0; k < path_candidates.poses.size(); k++){
			float yaw = tf::getYaw(path_candidates.poses[k].pose.orientation);
			geometry_msgs::Point p1,p0;
			p0 = path_candidates.poses[k].pose.position;
			p1.x = p0.x + size*2 * cos(yaw);
			p1.y = p0.y + size*2 * sin(yaw);
			cv::Scalar color;
			color[0] = path_candidates.poses[k].pose.position.z * 10;
			color[2] = 0;
			cv::circle(img_copy,pnt2cv(p0),3,visited_color,1);
			cv::line (img_copy, pnt2cv(p0), pnt2cv(p1),color,3,cv::LINE_8,0);

		//    if(k < mainpath.poses.size()-1 && draw_lines)
		//      draw_line(pathin.poses[k].pose.position,pathin.poses[k+1].pose.position,color,copy);
	}*/

	  rate.sleep();
	  ros::spinOnce();
  }
  return 0;
}
