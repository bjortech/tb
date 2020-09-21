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
#include <geometry_msgs/Twist.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_listener.h>
#include "tf2_ros/message_filter.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
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
ros::Publisher img_pub;
bool got_map,par_inspect_top;
double par_num_levels,par_maprad,par_hiabs,par_loabs,par_lookahead_distance,par_eval_maxrange,par_grid_size,par_zjump,par_area_sidelength,par_takeoffaltitude;
tf2_ros::Buffer tfBuffer;
int xmin,ymin,zmin,xmax,ymax,zmax,range_x,range_y,range_z,zmin_global,zmax_global;
geometry_msgs::PointStamped closest_obstacle,closest_obstacle_plane;
float closest_obstacle_plane_dist,closest_obstacle_dist;
geometry_msgs::Point pos;
geometry_msgs::PointStamped last_pos_check;
geometry_msgs::Point bbmin_custom,bbmax_custom,bbmin_octree,bbmax_octree;
nav_msgs::Path path_visited_tar01;
ros::Publisher pub_tarpose,visited_path_pub,invoke_pub,targetalt_pub;
geometry_msgs::PoseStamped last_pose,last_target,target;

std_msgs::Float64 cmdarm_msg;
std_msgs::Float64 target_alt;
float area_range,delta_z;
std_msgs::UInt8 state_msg;

cv::Mat mapimg(1000,1000,CV_8UC3,cv::Scalar(0, 0, 0)); //create image, set encoding and size, init pixels to default val
cv::Mat mapimg_copy(1000,1000,CV_8UC3,cv::Scalar(0, 0, 0)); //create image, set encoding and size, init pixels to default val
bool use_mono = true;;
float zneg = -10;
float zpos = 100;
float collision_radius = 1.1;
std::string par_workdir;
bool idle,par_live;
ros::Time last_twist,last_rosinfo;
cv::Mat img(1000,1000,CV_8UC3,cv::Scalar(0, 0, 0)); //create image, set encoding and size, init pixels to default val
cv::Mat img_copy(1000,1000,CV_8UC3,cv::Scalar(0, 0, 0)); //create image, set encoding and size, init pixels to default val
float pos_yaw;
bool get_initialscan_from_scan,need_candidates,need_visited;
int last_i;
nav_msgs::Odometry odom;
ros::Time last_scan;

std::vector<int> z_lvls;
int zlvl;

// Define Infinite (Using INT_MAX caused overflow problems)

std::vector<geometry_msgs::PolygonStamped> objects_polys;
std::vector<geometry_msgs::PolygonStamped> cleared_polys;
std::vector<geometry_msgs::PolygonStamped> polygon_grids;
std::vector<geometry_msgs::PoseStamped> targetcmds_sent;
std::vector<geometry_msgs::Point> grids_centroids;
std::vector<std::vector<int>> grids_at_ranges;
std::vector<nav_msgs::Path> paths_cand_at_lvl;
std::vector<nav_msgs::Path> paths_vstd_at_lvl;
std::vector<std::vector<int>> xy;
cv::Scalar visited_color,candidate_color,target_color,grid_color,obstacle_color,building_color,cleared_color,path_color;
std::vector<int> grids_in_range;
geometry_msgs::Point tar0,tar1,tar_01_normalized;
int area_sidelength,grid_sidelength,num_gridsprside;
sensor_msgs::LaserScan scan_copy;
double get_shortest(double target_hdng,double actual_hdng){
  double a = target_hdng - actual_hdng;
  if(a > M_PI)a -= M_PI*2;
  else if(a < -M_PI)a += M_PI*2;
  return a;
}
void octomap_callback(const octomap_msgs::Octomap& msg){
  abs_octree=octomap_msgs::fullMsgToMap(msg);
  octree.reset(dynamic_cast<octomap::OcTree*>(abs_octree));
  got_map = true;
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
float get_hdng32(geometry_msgs::Point32 p1,geometry_msgs::Point32 p0){
  float dx = p1.x - p0.x;
  float dy = p1.y - p0.y;
  return atan2(dy,dx);
}
cv::Point pnt322cv(geometry_msgs::Point32 pin){
	int c = x2c(pin.x,mapimg.cols,1);
	int r = y2r(pin.y,mapimg.rows,1);
	return cv::Point(c,r);
}
cv::Point pnt2cv(geometry_msgs::Point pin){
	int c = x2c(pin.x,mapimg.cols,1);
	int r = y2r(pin.y,mapimg.rows,1);
	return cv::Point(c,r);
}
double constrainAngle(double x){
  if(x > M_PI)
    return (x - M_PI*2);
  else if(x < -M_PI)
    return (x + M_PI*2);
  else
    return x;
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

bool update_edto(geometry_msgs::Point midpoint,float collision_radius,float maprad,float z0,float z1,bool unknownAsOccupied,bool sides){
  if(!got_map)
		return false;
  int zmid = (z1+z0)/2;


  octree.get()->getMetricMin(bbmin_octree.x,bbmin_octree.y,bbmin_octree.z);
  octree.get()->getMetricMax(bbmax_octree.x,bbmax_octree.y,bbmax_octree.z);
  zmin_global = int(round(bbmin_octree.z));
  zmax_global = int(round(bbmax_octree.z));

  z0 = fmin(z0,bbmax_octree.z-2);
  z1 = fmin(z1,bbmax_octree.z);
  bbmin_custom.x = midpoint.x-maprad;
  bbmin_custom.y = midpoint.y-maprad;
  bbmin_custom.z = z0;

  bbmax_custom.x = midpoint.x+maprad;
  bbmax_custom.y = midpoint.y+maprad;
  bbmax_custom.z = z1;
  float zmin_touse = fmax(bbmin_custom.z,bbmin_octree.z);
  float zmax_touse = fmin(bbmax_custom.z,bbmax_octree.z);
  if(zmax_touse < zmin_touse){
    zmax_touse = fmax(bbmin_custom.z,bbmin_octree.z);
    zmin_touse = fmin(bbmax_custom.z,bbmax_octree.z);
  }

  octomap::point3d boundary_min(fmax(bbmin_custom.x,bbmin_octree.x),fmax(bbmin_custom.y,bbmin_octree.y),zmin_touse);
  octomap::point3d boundary_max(fmin(bbmax_custom.x,bbmax_octree.x),fmin(bbmax_custom.y,bbmax_octree.y),zmax_touse);

  edf_ptr.reset (new DynamicEDTOctomap(collision_radius,octree.get(),
      boundary_min,
      boundary_max,
      unknownAsOccupied));
  edf_ptr.get()->update();
  xmin    = int(round(boundary_min.x()))+1;  ymin    = int(round(boundary_min.y()))+1; zmin    = int(round(boundary_min.z()));
  xmax    = int(round(boundary_max.x()))-1;  ymax    = int(round(boundary_max.y()))-1; zmax    = int(round(boundary_max.z()));
  range_x = xmax - xmin;                     range_y = ymax - ymin;                    range_z = zmax - zmin;
  int vol = range_x*range_y*range_z;
  if(vol <= 0){
		ROS_INFO("FAILED update_edto FAILED: xmin: %i, ymin: %i, zmin: %i, xmax: %i, ymax: %i, zmax: %i, range_x: %i, range_y: %i, range_z: %i ",xmin,ymin,zmin,xmax,ymax,zmax,range_x,range_y,range_z);
		return false;
	}
  return true;
}

bool check_if_inside_bb(std::vector<geometry_msgs::Point> l0l1,geometry_msgs::Point p){
	if(l0l1[0].x < p.x && l0l1[0].y < p.y && l0l1[1].x > p.x && l0l1[1].y > p.y)
		return true;
	else
		return false;
}

geometry_msgs::Point32 get_poly_centroid(geometry_msgs::PolygonStamped polyin){
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

    return centroid;
}
geometry_msgs::TransformStamped get_tf(std::string from, std::string to){
	geometry_msgs::TransformStamped transformStamped;
  try{
    transformStamped = tfBuffer.lookupTransform(from,to,
                             ros::Time(0));
  }
  catch (tf2::TransformException &ex) {
    ROS_WARN("%s",ex.what());
    ros::Duration(1.0).sleep();
  }
	return transformStamped;
}
void get_zdown(){
  octomap::point3d closestObst;
  for(int y = ymin; y < ymax; y++){
    for(int x = xmin; x < xmax; x++){
      int z = zmax;
      float dst = 1.1;
      while(dst > (1.1-1) && z > bbmin_octree.z){
        octomap::point3d p(x,y,z);
        edf_ptr.get()->getDistanceAndClosestObstacle(p,dst,closestObst);
        z--;
      }
			int r = y2r(y,mapimg.rows,1);
			int c = x2c(x,mapimg.cols,1);
			img.at<cv::Vec3b>(r,c)[2] = (z - bbmin_octree.z);
    }
  }
	ROS_INFO("get_zdown complete");
}
geometry_msgs::PolygonStamped poly_addpoint(geometry_msgs::PolygonStamped polyin,geometry_msgs::Point point_to_add){
	geometry_msgs::PolygonStamped polyout;
	polyout.header = polyin.header;
	geometry_msgs::Point32 pin,centroid;
	pin.x = point_to_add.x;
	pin.y = point_to_add.y;
	pin.z = point_to_add.z;
	int len = polyin.polygon.points.size();
	if(len < 3){
		polyin.polygon.points.push_back(pin);
		return polyin;
	}
	centroid = get_poly_centroid(polyin);
	float centroid_hdng = get_hdng32(centroid,pin);
	float last_hdng = get_hdng32(centroid,polyin.polygon.points[0]);
	int index;
	for(int i = 1; i < len+1; i++){
		float hdng = get_hdng32(centroid,polyin.polygon.points[i]);
		if(last_hdng < centroid_hdng && hdng > centroid_hdng){
			polyout.polygon.points.push_back(pin);
			index = i;
			i++;
			ROS_INFO("Polypoint added: %i",i);
		}
		else
			polyout.polygon.points.push_back(polyin.polygon.points[i]);
	}
	return polyout;
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
void draw_line32(geometry_msgs::Point32 pnt0,geometry_msgs::Point32 pnt1, cv::Scalar color,bool copy){
  if(copy)
    cv::line (img_copy, pnt322cv(pnt0), pnt322cv(pnt1), color,1,cv::LINE_8,0);
  else
    cv::line (img, pnt322cv(pnt0), pnt322cv(pnt1), color,1,cv::LINE_8,0);
}

void colorize_grid_dst(int i,cv::Scalar color){
  if(polygon_grids.size() < i)
    return;
	int rmin = y2r(polygon_grids[i].polygon.points[1].y,img.rows,1);
	int cmin = x2c(polygon_grids[i].polygon.points[1].x,img.cols,1);
	int rmax = y2r(polygon_grids[i].polygon.points[3].y,img.rows,1);
	int cmax = x2c(polygon_grids[i].polygon.points[3].x,img.cols,1);
	for(int r = rmin+1; r < rmax-1; r++){
		for(int c = cmin+1; c < cmax-1; c++){
      img.at<cv::Vec3b>(r,c)[0] = color[0];
      img.at<cv::Vec3b>(r,c)[1] = color[1];
      img.at<cv::Vec3b>(r,c)[2] = color[2];
		}
	}
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
void draw_pose2d(geometry_msgs::Point p0,float size,cv::Scalar color,bool copy,double yaw){
  geometry_msgs::Point p1;
  p1.x = p0.x + size*2 * cos(yaw);
  p1.y = p0.y + size*2 * sin(yaw);
  draw_circle(p0,size,color,copy);
  draw_line(p0,p1,color,copy);
}
void draw_grid(int i,cv::Scalar color,bool copy){
  draw_rectangle32(polygon_grids[i].polygon.points[1],polygon_grids[i].polygon.points[3],color,copy);
}
void draw_grids(std::vector<int> grids_to_draw,cv::Scalar color,bool copy){
	if(grids_to_draw.size() < 2)
		return;
	for(int i = 0; i < grids_to_draw.size()-1; i++){
    draw_rectangle32(polygon_grids[grids_to_draw[i]].polygon.points[1],polygon_grids[grids_to_draw[i]].polygon.points[3],color,copy);
  }
	ROS_INFO("GridsToDraw: %i",grids_to_draw.size());
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
std::vector<int> grids_on_poly(geometry_msgs::PolygonStamped polyin){
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
void create_polygon_grids(){
	geometry_msgs::Point p;
  grids_centroids.resize(0);
  polygon_grids.resize(0);
  for(int y = 0; y < num_gridsprside; y++){
    for(int x = 0; x < num_gridsprside; x++){
			p.x      = x * grid_sidelength - area_sidelength / 2 + grid_sidelength/2;
      p.y      = y * grid_sidelength - area_sidelength / 2 + grid_sidelength/2;
      if(p.x > xmin && p.y > ymin &&
         p.x < xmax && p.y < ymax){
  			grids_centroids.push_back(p);
  			polygon_grids.push_back(createpoly_square(p,grid_sidelength));
      }
		}
  }
}
geometry_msgs::PolygonStamped polyCircle(geometry_msgs::Point centroid,float radius, int resolution){
  geometry_msgs::PolygonStamped polyout;
  polyout.header.frame_id = "map";
  for(int i = 0; i < resolution; i++){
    geometry_msgs::Point32 p;
    p.x = centroid.x + radius*cos(-M_PI + (M_PI*2/resolution)*i);
    p.y = centroid.y + radius*sin(-M_PI + (M_PI*2/resolution)*i);
    polyout.polygon.points.push_back(p);
  }
  return polyout;
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

geometry_msgs::PolygonStamped scan2poly(sensor_msgs::LaserScan scan,geometry_msgs::Point midpoint){
  geometry_msgs::PolygonStamped polyout;
  polyout.header.frame_id = "map";
  polyout.polygon.points.resize(scan.ranges.size());
  for(int i = 0; i < scan.ranges.size();i++){
    geometry_msgs::Point32 p0,p1,ps;
    float r = fmin(scan.ranges[i],scan.range_max);
    float a =	constrainAngle(scan.angle_increment * i + scan.angle_min + pos_yaw);
    polyout.polygon.points[i].x = round(midpoint.x) + r * cos(a);
    polyout.polygon.points[i].y = round(midpoint.y) + r * sin(a);
    polyout.polygon.points[i].z = midpoint.z;
  }
  return polyout;
}
void scan_cb(const sensor_msgs::LaserScan::ConstPtr& scan){
  scan_copy = *scan;
}
void scanproc(bool copy){
  ROS_INFO("Scan cb %i",scan_copy.ranges.size());
  geometry_msgs::Point empty;
  std::vector<int> possible_grids;
  std::vector<int> hits;
  std::vector<int> grids;
	geometry_msgs::Point32 p0,p1,ps;
	for(int i = 0; i < scan_copy.ranges.size(); i++){
    if(scan_copy.ranges[i] < scan_copy.range_max){
			float r = scan_copy.ranges[i];
			float a =	constrainAngle(scan_copy.angle_increment * i + scan_copy.angle_min + pos_yaw);
      p0.x = round(pos.x) + r * cos(a);
      p0.y = round(pos.y) + r * sin(a);
      int ii = get_gridnum(p0.x,p0.y);
      draw_grid(ii,obstacle_color,copy);
      hits.push_back(ii);
	  }
	}/*
    sort(hits.begin(),hits.end());
     int hitcount = 0;
      if(hits.size() > 0){
        int current_number = hits[0];
      for(int i = 0; i < hits.size(); i++){
        if(current_number != hits[i]){
          hitcount = 0;
          current_number = hits[i];
        }
        else
          hitcount++;
      }
    }
  std::vector<int> grids_to_draw;
    for(int i = 0; i < hits.size(); i++){
      if(hitcount > 5){
        ROS_INFO("Hits at gridnumber: %i",hits[i]);
        grids_to_draw.push_back(hits[i]);
      }
    }
    if(grids_to_draw.size() > 1)
      draw_grids(grids_to_draw,obstacle_color);
	*/
}



geometry_msgs::Point get_normalized_dst(geometry_msgs::Point actual,geometry_msgs::Point setpoint){
  Eigen::Vector3f pnt1_vec(actual.x,actual.y,actual.z);
  Eigen::Vector3f pnt2_vec(setpoint.x,setpoint.y,setpoint.z);
  Eigen::Vector3f cur_vec = pnt1_vec;
  float error_length = (pnt2_vec - pnt1_vec).norm();
  Eigen::Vector3f stride_vec;
  stride_vec = (pnt2_vec - pnt1_vec).normalized();
  geometry_msgs::Point res;
  res.x = stride_vec.x();
  res.y = stride_vec.y();
  res.z = stride_vec.z();
  return res;
}
std::vector<int> get_points_not_in_both(std::vector<int> v1,std::vector<int> v2){
  std::vector<int> vout;
  for(int i = 0; i < v1.size(); i++){
    bool in_both;
    for(int k = 0; k < v2.size(); k++){
      if(v1[i] == v2[k]){
        in_both = true;
      }
    }
    if(!in_both)
      vout.push_back(v1[i]);
  }
  return vout;
}
void draw_grids_at_ranges(geometry_msgs::Point  midpoint){
  ROS_INFO("draw_grids_at_ranges");

  for(int i = 0; i < par_eval_maxrange/par_grid_size; i++){
    //grids_at_ranges_ramge.push_back(par_grid_size*(i+1));
    grids_at_ranges.push_back(get_points_not_in_both(grids_in_poly(polyCircle( midpoint,par_grid_size*i,8)),grids_in_poly(polyCircle( midpoint,par_grid_size*i+1,8))));
  }
  for(int i = 0; i < grids_at_ranges.size(); i++){
    for(int k = 0; k < grids_at_ranges[i].size(); k++){
      draw_grids(grids_at_ranges[i],cv::Scalar(20,20,100),true);
    }
  }
}
void draw_path(nav_msgs::Path pathin,int size, cv::Scalar color,bool copy, bool draw_lines){
  for(int k = 0; k < pathin.poses.size(); k++){
    if(k < pathin.poses.size()-1 && draw_lines)
      draw_line(pathin.poses[k].pose.position,pathin.poses[k+1].pose.position,color,copy);
    draw_pose2d(pathin.poses[k].pose.position,2,target_color,true, tf::getYaw(pathin.poses[k].pose.orientation));
  }
}

void draw_path_vstdcnd(int zn0,int zn1,bool copy){
  cv::Scalar color_visited;
  cv::Scalar color_candidates;
  color_visited = visited_color;
  color_candidates = candidate_color;
  for(int i = zn0; i < zn1; i++){
    if(paths_cand_at_lvl[i].poses.size() > 1){
      color_visited[2] = i * 10;
      draw_path(paths_cand_at_lvl[i],2,candidate_color,copy,false);
    }
    if(paths_vstd_at_lvl[i].poses.size() > 1){
      color_candidates[0] = i * 10;
      draw_path(paths_vstd_at_lvl[i],3,visited_color,copy,true);
    }
  }
}

void get_zdown_grids(bool copy){
  octomap::point3d closestObst;
	int gn = 0;
  for(int i = 0; i < grids_centroids.size(); i++){
		geometry_msgs::Point xy;
		xy = grids_centroids[i];
		if(xy.x > xmin && xy.y > ymin &&
			 xy.x < xmax && xy.y < ymax){
	      int z = zmax;
	      float dst = 5.0;
	      while(dst > 3 && z > zmin){
	        octomap::point3d p(grids_centroids[i].x,grids_centroids[i].y,z);
	        edf_ptr.get()->getDistanceAndClosestObstacle(p,dst,closestObst);
	        z--;
	      }
				int colout = z - zmin;
        if(z < pos.z)
				  draw_circle(grids_centroids[i],2,cv::Scalar(0,colout*5,0),copy);
        else
          draw_circle(grids_centroids[i],2,cv::Scalar(0,colout*5,250),copy);
    	}
		}
    //
	ROS_INFO("get_zdown_grids complete");
}
void path01_cb(const nav_msgs::Path::ConstPtr& msg){
  ROS_INFO("Pathvstd callback %i",msg->poses.size());
  if(msg->poses.size() < path_visited_tar01.poses.size()){
    for(int i  = 0; i < path_visited_tar01.poses.size(); i++){
      paths_vstd_at_lvl[zlvl].poses.push_back(path_visited_tar01.poses[i]);
    }
  }
  path_visited_tar01 = *msg;
}

void pathcandupd_cb(const nav_msgs::Path::ConstPtr& msg){
  ROS_INFO("Pathcand callback %i",msg->poses.size());
  if(msg->poses.size() > 0){
    for(int i = 0; i < msg->poses.size(); i++){
      int zlvl_in  = round(msg->poses[i].pose.position.z / par_zjump) + 3;
      paths_cand_at_lvl[zlvl_in].poses.push_back(msg->poses[i]);
    }
  }
}

void get_and_draw_targets(){
 float circle_rad = 2;
  if(targetcmds_sent.size() > 2){
      for(int i = 0; i < targetcmds_sent.size()-2; i++){
        draw_line(targetcmds_sent[i+1].pose.position,targetcmds_sent[i].pose.position,target_color,true);
        draw_pose2d(target.pose.position,circle_rad,target_color,true,tf::getYaw(targetcmds_sent[i].pose.orientation));
    }
  }
}
void cmd_cb(const geometry_msgs::Point::ConstPtr& msg){
  cv::Scalar target_color_alt;
  target_color_alt = target_color;
  target_color_alt[0] = msg->z * 5;
  draw_line(target.pose.position,*msg,target_color,false);
  last_target          = target;
  target.pose.position = *msg;
  targetcmds_sent.push_back(target);
  get_and_draw_targets();
//  img.copyTo(img_copy)
/*

  get_normalized_dst(tar0,tar)
  d.x = posein.pose.position.x - t.x;
  d.y = posein.pose.position.y - t.y;
  d.z = posein.pose.position.z - t.z;
  t.x = posein.pose.position.x;
  t.y = posein.pose.position.y;
  t.z = posein.pose.position.z;
  tyaw = tf::getYaw(posein.pose.orientation);
t_yaw_0  = t_yaw_1;
t_hdng_0 = t_hdng_1;
t_dst_0  = t_dst_1;
geometry_msgs::PoseStamped last_target;
tardelta_norm = get_normalized_dst(tar,msg->pose.position);
t_dst         = get_dst3d(tar,msg->pose.position);
t_hdng        = get_hdng(tar,msg->pose.position);
*/
  ROS_INFO("Resetting image");
}

void zlvl_cb(const std_msgs::UInt8::ConstPtr& msg){
  zlvl = msg->data;
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
int main(int argc, char** argv)
{
  ros::init(argc, argv, "tb_global_node");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");
  private_nh.getParam("workdir_path",     par_workdir);
  private_nh.param("map_sidelength_min",  par_maprad, 50.0);
  private_nh.param("max_sidelength",      par_area_sidelength, 1000.0);
  private_nh.param("par_zjump",           par_zjump, 3.0);//*2.0);
	private_nh.param("grid_size",		        par_grid_size, 5.0);
	private_nh.param("lookahead_distance",  par_lookahead_distance, 50.0);
  private_nh.param("par_eval_maxrange",		par_eval_maxrange, 25.0);
  private_nh.param("inspect_top",		      par_inspect_top, false);
  tf2_ros::TransformListener tf2_listener(tfBuffer);
  area_sidelength = int(round(par_area_sidelength));
  grid_sidelength = int(round(par_grid_size));
  num_gridsprside  = int(round(par_area_sidelength / par_grid_size));
  ROS_INFO("par_area_sidelength: %.2f par_grid_size %.2f",par_area_sidelength,par_grid_size);
  ROS_INFO("Area side: %i grid_side %i num_gridsprside: %i",area_sidelength,grid_sidelength,num_gridsprside);
  nav_msgs::Path path_template;
  path_template.header.frame_id = "map";
  target.pose.position.z = par_takeoffaltitude;
  draw_pose2d(target.pose.position,2,target_color,false,tf::getYaw(target.pose.orientation));
  for(int i = 0; i < 20; i++){
    z_lvls.push_back(int(round(-par_zjump*3 + par_zjump*i)));
    paths_cand_at_lvl.push_back(path_template);
    paths_vstd_at_lvl.push_back(path_template);
  }
  define_colors();
ros::Subscriber s22 = nh.subscribe("/tb_fsm/altlvl",100,&zlvl_cb);


ros::Subscriber s1  = nh.subscribe("/octomap_full",1,octomap_callback);
ros::Subscriber s46 = nh.subscribe("/tb_path_filtered",1,&pathcandupd_cb);
ros::Subscriber s8  = nh.subscribe("/cmd_pos", 10,&cmd_cb);
ros::Subscriber s10  = nh.subscribe("/tb_nav/path_visited", 10,&path01_cb);

ros::Subscriber s3  = nh.subscribe("/scan_base_alt",  100,&scan_cb);

ros::Rate rate(1);
bool done = true;
bool everyother;
ros::Time start = ros::Time::now();
ros::Time last_check = ros::Time::now();
bool par_get_zdown;
bool first = true;
int c = 0;
  while(ros::ok()){
    geometry_msgs::TransformStamped transformStamped;
    try{
      transformStamped = tfBuffer.lookupTransform("map","base_stabilized",
                               ros::Time(0));
    }
    catch (tf2::TransformException &ex) {
      ROS_WARN("%s",ex.what());
      ros::Duration(1.0).sleep();
    }
      pos.x  = transformStamped.transform.translation.x;
      pos.y  = transformStamped.transform.translation.y;
      pos.z  = transformStamped.transform.translation.z;
      pos_yaw = tf::getYaw(transformStamped.transform.rotation);

    if(c == 5)
      update_edto(pos,5,par_maprad,pos.z-10,pos.z+50,false,false);
    else if(c > 5)
      c = 0;
    else
      c++;

    std::vector<int> safe_grids;
    geometry_msgs::PolygonStamped poly;
    create_polygon_grids();

    img.copyTo(img_copy);
    scanproc(false);

    get_zdown_grids(true);
    draw_path_vstdcnd(2,10,true);
    poly       = scan2poly(scan_copy,pos);
    safe_grids = grids_in_poly_safe(poly);

    draw_polygon(poly,obstacle_color,true);
    draw_grids(safe_grids,cleared_color,false);

    cv::imwrite(par_workdir+"/global_img_copy.png",img_copy);
    cv::imwrite(par_workdir+"/global_img.png",img);

	  rate.sleep();
	  ros::spinOnce();
  }
  return 0;
}
