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
double par_num_levels,par_maprad,par_hiabs,par_loabs,par_lookahead_distance,par_takeoffaltitude;
tf2_ros::Buffer tfBuffer;
int xmin,ymin,zmin,xmax,ymax,zmax,range_x,range_y,range_z,zmin_global,zmax_global;
geometry_msgs::PointStamped closest_obstacle,closest_obstacle_plane;
float closest_obstacle_plane_dist,closest_obstacle_dist;
geometry_msgs::Point pos;
geometry_msgs::PointStamped last_pos_check;
geometry_msgs::Point bbmin_custom,bbmax_custom,bbmin_octree,bbmax_octree;
nav_msgs::Path path_visited,path_candidates;
ros::Publisher pub_tarpose,visited_path_pub,invoke_pub,targetalt_pub;
geometry_msgs::PoseStamped last_pose,target;
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
std::vector<geometry_msgs::PolygonStamped> objects_polys;
std::vector<geometry_msgs::PolygonStamped> cleared_polys;
std::vector<geometry_msgs::PolygonStamped> polygon_grids;
std::vector<geometry_msgs::Point> grids_centroids;

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
geometry_msgs::PolygonStamped createpoly_square(geometry_msgs::Point pin, float centroid_sidelength){
  geometry_msgs::PolygonStamped poly;
  poly.polygon.points.resize(5);
  poly.header.frame_id = "map";
  poly.polygon.points[0].x = round(pin.x + centroid_sidelength);
  poly.polygon.points[0].y = round(pin.y + centroid_sidelength);
  poly.polygon.points[1].x = round(pin.x - centroid_sidelength);
  poly.polygon.points[1].y = round(pin.y + centroid_sidelength);
  poly.polygon.points[2].x = round(pin.x - centroid_sidelength);
  poly.polygon.points[2].y = round(pin.y - centroid_sidelength);
  poly.polygon.points[3].x = round(pin.x + centroid_sidelength);
  poly.polygon.points[3].y = round(pin.y - centroid_sidelength);
  poly.polygon.points[4]   = poly.polygon.points[0];
  return poly;
}

void colorize_grid_dst(int i,int color){
  if(polygon_grids.size() < i)
    return;
	int rmin = y2r(polygon_grids[i].polygon.points[1].y,img.rows,1);
	int cmin = x2c(polygon_grids[i].polygon.points[1].x,img.cols,1);
	int rmax = y2r(polygon_grids[i].polygon.points[3].y,img.rows,1);
	int cmax = x2c(polygon_grids[i].polygon.points[3].x,img.cols,1);
	for(int r = rmin+1; r < rmax-1; r++){
		for(int c = cmin+1; c < cmax-1; c++){
			img.at<cv::Vec3b>(r,c)[1] = color;
		}
	}
}

void draw_polygon(geometry_msgs::PolygonStamped polyin,std::string name){
	if(polyin.polygon.points.size() > 2){
		for(int i = 0; i < polyin.polygon.points.size()-1; i++){
			cv::line (img_copy, pnt322cv(polyin.polygon.points[i]), pnt322cv(polyin.polygon.points[i+1]), cv::Scalar(0,155,155),1,cv::LINE_8,0);
		}
	}
	ROS_INFO("PolygonToDraw: %i pints",polyin.polygon.points.size());
}
void draw_path(nav_msgs::Path pathin,int r, int g, int b){
	for(int i = 0; i < pathin.poses.size();i++){
		if(get_dst2d(pathin.poses[i].pose.position,pos) < par_maprad && abs(pathin.poses[i].pose.position.z - pos.z) < 10){
			cv::circle(img_copy,pnt2cv(pathin.poses[i].pose.position),5 - abs(pathin.poses[i].pose.position.z - pos.z)/2,cv::Scalar(r,g,b),-1);
		}
	}
}
void draw_grids(std::vector<int> grids_to_draw,int r, int g, int b){
	if(grids_to_draw.size() < 2)
		return;
	for(int i = 0; i < grids_to_draw.size()-1; i++){
			cv::rectangle(img, pnt322cv(polygon_grids[grids_to_draw[i]].polygon.points[1]), pnt322cv(polygon_grids[grids_to_draw[i]].polygon.points[3]), cv::Scalar(r,g,b), 1,8,0);
	}
	ROS_INFO("GridsToDraw: %i grids drawn in RGB: %i %i %i",grids_to_draw.size(),r,g,b);
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
void create_polygon_grids(float area_sidelength,float area_alt,float centroid_sidelength){
  int num_centroids = int(round(area_sidelength / centroid_sidelength));
	std::vector<int> grids_to_draw;
	geometry_msgs::Point p;
  for(int y = 0; y < num_centroids; y++){
    for(int x = 0; x < num_centroids; x++){
			p.x      = pow(-1,y) * (x * centroid_sidelength - area_sidelength / 2 + centroid_sidelength/2);
      p.y      = -1 * (y * centroid_sidelength - area_sidelength / 2 + centroid_sidelength/2);
			grids_to_draw.push_back(polygon_grids.size());
			grids_centroids.push_back(p);
			polygon_grids.push_back(createpoly_square(p,centroid_sidelength));
		}
  }
	ROS_INFO("path grid %i",grids_centroids.size());
	draw_grids(grids_to_draw,0,20,20);
}


void draw_scans_polys(){
	ROS_INFO("draw_scans_polys %i",objects_polys.size());
	for(int i = 0; i < objects_polys.size(); i++){
		draw_grids(grids_in_poly(objects_polys[i]),100,0,0);
	}
}
void draw_clear_polys(){
	ROS_INFO("draw_clear_polys %i",objects_polys.size());
	for(int i = 0; i < cleared_polys.size(); i++){
		draw_grids(grids_in_poly(cleared_polys[i]),0,100,0);
	}
}
void work_odom(){
	if(get_dst2d(odom.pose.pose.position,last_pos_check.point) > 5 || (last_pos_check.header.stamp - odom.header.stamp).toSec() > 5.0) {
		last_pos_check.point = odom.pose.pose.position;
		last_pos_check.header = odom.header;
		cv::imwrite(par_workdir+"/img.png",img_copy);
		img.copyTo(img_copy);
		need_visited = true;
		need_candidates = true;
		ROS_INFO("Resetting image");
	}
	float true_heading = atan2(odom.twist.twist.linear.y,odom.twist.twist.linear.x);
	geometry_msgs::Point relvel,relvel0;
	float scalarvel = sqrt(odom.twist.twist.linear.y*odom.twist.twist.linear.y+odom.twist.twist.linear.x*odom.twist.twist.linear.x);
	relvel.x = scalarvel * cos(true_heading);
	relvel.y = scalarvel * sin(true_heading);
	cv::line (img_copy, pnt2cv(relvel0), pnt2cv(relvel), cv::Scalar(100,100,100),1,cv::LINE_8,0);
}

void transform_candidates(){
	geometry_msgs::TransformStamped map2pos;
	map2pos=get_tf("map","base_stabilized");
	for(int i = 0; i < path_candidates.poses.size(); i++){
		tf2::doTransform(path_candidates.poses[i], path_candidates.poses[i], map2pos);
	}
}
void transform_visited(){
	geometry_msgs::TransformStamped map2pos;
	map2pos=get_tf("map","base_stabilized");
	for(int i = 0; i < path_visited.poses.size(); i++){
		tf2::doTransform(path_visited.poses[i], path_visited.poses[i], map2pos);
	}
}

void pathcand_cb(const nav_msgs::Path::ConstPtr& msg){
	if(need_candidates && msg->poses.size() > path_candidates.poses.size()){
		need_candidates = false;
		ROS_INFO("pathin: %i ", msg->poses.size());
		path_candidates = *msg;
		transform_candidates();
		draw_path(path_candidates,200,0,0);
	}
}
void pathvstd_cb(const nav_msgs::Path::ConstPtr& msg){
	if(need_visited && msg->poses.size() > path_visited.poses.size()){
		need_visited = false;
		ROS_INFO("pathin: %i ", msg->poses.size());
		path_visited = *msg;
		transform_visited();
		draw_path(path_visited,0,0,200);
	}
}
void scan_cb(const sensor_msgs::LaserScan::ConstPtr& scan){
	if((scan->header.stamp - last_scan).toSec() > 1.0){
		objects_polys.resize(0);
		cleared_polys.resize(0);
		geometry_msgs::PolygonStamped poly;
		geometry_msgs::PolygonStamped poly_clear;
		geometry_msgs::Point32 p0,p1,ps;
		last_scan = ros::Time::now();
		for(int i = 0; i < scan->ranges.size(); i++){
			float r = scan->ranges[i];
			float a =	scan->angle_increment * i + scan->angle_min;
			p0.x = r * cos(a);
			p0.y = r * sin(a);
			float clear_rad = poly_clear.polygon.points.size() * scan->angle_increment;
			float obs_rad = poly_clear.polygon.points.size() * scan->angle_increment;
			if(scan->ranges[i] < scan->range_max){
				poly.polygon.points.push_back(p0);
				if(clear_rad > 1.0 && poly_clear.polygon.points.size() >= 2){
					p0 = poly_clear.polygon.points[0];
			  	p1 = poly_clear.polygon.points[poly_clear.polygon.points.size()-1];
					cleared_polys.push_back(poly_clear);
					ROS_INFO("InitialScan[rads: %.2f]: Object[#%i of %i pnts] found between %.0f %.0f and %.0f %.0f",clear_rad,cleared_polys.size(),poly_clear.polygon.points.size(),p0.x,p0.y,p1.x,p1.y);
					cv::line (img_copy, pnt322cv(ps), pnt322cv(p0), cv::Scalar(0,0,200),1,cv::LINE_8,0);
					cv::line (img_copy, pnt322cv(ps), pnt322cv(p1), cv::Scalar(0,0,200),1,cv::LINE_8,0);
				}
				poly_clear.polygon.points.resize(0);
			}
			else{
				poly_clear.polygon.points.push_back(p0);
				if(clear_rad > 0.2 && poly.polygon.points.size() >= 2){
					p0 = poly.polygon.points[0];
					p1 = poly.polygon.points[poly_clear.polygon.points.size()-1];
					ROS_INFO("InitialScan[rads: %.2f]: Object[#%i of %i pnts] found between %.0f %.0f and %.0f %.0f",obs_rad,objects_polys.size(),poly.polygon.points.size(),p0.x,p0.y,p1.x,p1.y);
					cv::line (img_copy, pnt322cv(ps), pnt322cv(p0), cv::Scalar(0,200,0),1,cv::LINE_8,0);
					cv::line (img_copy, pnt322cv(ps), pnt322cv(p1), cv::Scalar(0,200,0),1,cv::LINE_8,0);
					objects_polys.push_back(poly);
					poly.polygon.points.resize(0);
				}
			}
		}
	}
}
void odom_cb(const nav_msgs::Odometry::ConstPtr& msg){
	if((msg->header.stamp - odom.header.stamp).toSec() > 0.5){
		odom = *msg;
		work_odom();
	}
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "tb_structures_node");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");
  private_nh.getParam("workdir_path", par_workdir);
	private_nh.param("map_sidelength_min",  par_maprad, 50.0);
	private_nh.param("ffill_hiabs",  par_hiabs, 2.0);
  private_nh.param("ffill_loabs",  par_loabs, 3.0);
	private_nh.param("takeoff_altitude",		par_takeoffaltitude, 15.0);
	private_nh.param("lookahead_distance",  par_lookahead_distance, 50.0);
  private_nh.param("is_live",		par_live, false);
  private_nh.param("inspect_top",		par_inspect_top, false);
  tf2_ros::TransformListener tf2_listener(tfBuffer);

ros::Subscriber s1 = nh.subscribe("/octomap_full",1,octomap_callback);
ros::Subscriber s6 = nh.subscribe("/tb_path_filtered",1,&pathcand_cb);
ros::Subscriber s2 = nh.subscribe("/tb_nav/path_visited",1,&pathvstd_cb);
ros::Subscriber s3 = nh.subscribe("/scan_base_alt",  100,&scan_cb);
ros::Subscriber s4 = nh.subscribe("/odom",  100,&odom_cb);
ros::Rate rate(1);
bool done = true;
bool everyother;
ros::Time start = ros::Time::now();
ros::Time last_check = ros::Time::now();
bool par_get_zdown;
  while(ros::ok()){
	//	if((ros::Time::now() - last_check).toSec() > 2 && got_map && done){
		if(polygon_grids.size() == 0)
			create_polygon_grids(100,10,5);
		//transform_candidates();
		//transform_visited();

		if(par_get_zdown){
			update_edto(pos,collision_radius,par_maprad,pos.z-10,pos.z+50,false,false);
			get_zdown();
		}
		draw_scans_polys();
		draw_clear_polys();
	  rate.sleep();
	  ros::spinOnce();
  }
  return 0;
}
