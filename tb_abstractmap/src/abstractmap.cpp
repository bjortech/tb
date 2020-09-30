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
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/point_cloud2_iterator.h>
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

cv::Mat img(1000,1000,CV_8UC3,cv::Scalar(0, 0, 0)); //create image, set encoding and size, init pixels to default val
cv::Mat img_temp(1000,1000,CV_8UC3,cv::Scalar(0, 0, 0)); //create image, set encoding and size, init pixels to default val
cv::Mat img_blank(1000,1000,CV_8UC3,cv::Scalar(0, 0, 0)); //create image, set encoding and size, init pixels to default val
cv::Mat img_distances(1000,1000,CV_8UC3,cv::Scalar(0, 0, 0)); //create image, set encoding and size, init pixels to default val
cv::Mat img_draw(1000,1000,CV_8UC3,cv::Scalar(0, 0, 0)); //create image, set encoding and size, init pixels to default val
///********FRONTIER*************////////
double par_gridsize,par_res,par_imwrite_interval;
tf2_ros::Buffer tfBuffer;
ros::Time process_start;
std::string active_process;
int naming_count = 0;
nav_msgs::Path g_path_target_candidates;
geometry_msgs::PolygonStamped g_poly_boundary;
geometry_msgs::PoseStamped g_pose,g_target_pose;
geometry_msgs::Point info_point;
int info_interval = 20;

void start_process(std::string name){
  float dt = (ros::Time::now() - process_start).toSec();
  if(active_process != "")
    ROS_INFO("ABMAP: Process: %s took %.4f sec",active_process.c_str(),dt);
  active_process = name;
  process_start  = ros::Time::now();
}

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
float y2r(float y){
  return (img.rows / 2 - y / par_res);
}
float x2c(float x){
  return (x / par_res + img.cols/2);
}
int r2y(float r){
  return int((img.rows / 2 - r) * par_res);
}
int c2x(float c){
  return int((c - img.cols / 2) * par_res);
}
cv::Point pnt2cv(geometry_msgs::Point pin){
	int c = x2c(pin.x);
	int r = y2r(pin.y);
	return cv::Point(c,r);
}
cv::Point pnt322cv(geometry_msgs::Point32 pin){
	int c = x2c(pin.x);
	int r = y2r(pin.y);
	return cv::Point(c,r);
}

float get_dst2d(geometry_msgs::Point p2, geometry_msgs::Point p1){
  return(sqrt(pow(p1.x-p2.x,2)+pow(p1.y-p2.y,2)));
}
float get_dst3d(geometry_msgs::Point p2, geometry_msgs::Point p1){
  return(sqrt(pow(p1.x-p2.x,2)+pow(p1.y-p2.y,2)+pow(p1.z-p2.z,2)));
}
bool isXYinPoly(geometry_msgs::PolygonStamped polyin, float x, float y)
{
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
nav_msgs::Path constrain_path_bbpoly(nav_msgs::Path pathin,geometry_msgs::PolygonStamped poly_bb,bool get_in_poly){
  nav_msgs::Path path_inpoly,path_outpoly;
  path_outpoly.header = path_inpoly.header = hdr();
  if(pathin.poses.size() == 0 || poly_bb.polygon.points.size() == 0)
    return pathin;
  for(int i = 0; i < pathin.poses.size(); i++){
    if(isXYinPoly(poly_bb,pathin.poses[i].pose.position.x,pathin.poses[i].pose.position.y))
      path_inpoly.poses.push_back(pathin.poses[i]);
		else
			path_outpoly.poses.push_back(pathin.poses[i]);
  }
	if(get_in_poly)
  	return path_inpoly;
	else
		return path_outpoly;
}
cv::Scalar get_color(int base_blue,int base_green, int base_red){
	cv::Scalar color;
	color[0] = base_blue;
	color[1] = base_green;
	color[2] = base_red;
	return color;
}

void draw_rectangle(geometry_msgs::Point midpoint,float size, cv::Scalar color){
  geometry_msgs::Point p0,p1;
  p0.x = midpoint.x-size; p1.x = midpoint.x+size*2;
  p0.y = midpoint.y-size; p1.y = midpoint.y+size*2;
  cv::rectangle(img, pnt2cv(p0),pnt2cv(p1),color,1,8,0);
}
void draw_line(geometry_msgs::Point p0,float a,float r,cv::Scalar color){
  geometry_msgs::Point pyaw;
  pyaw.x = p0.x + r * cos(a);
  pyaw.y = p0.y + r * sin(a);
  cv::line (img, pnt2cv(p0), pnt2cv(pyaw),color,1,cv::LINE_8,0);
}

void draw_pnt(geometry_msgs::Point pnt, int rectangle_size, int circle_size, int yaw_size, bool pixel,float yaw, cv::Scalar color){
	if(circle_size > 0)
		cv::circle(img,pnt2cv(pnt),circle_size,color,1);
	if(rectangle_size > 0)
		draw_rectangle(pnt,rectangle_size,color);
	if(yaw_size > 0)
		draw_line(pnt,yaw,yaw_size,color);
	if(pixel){
		if(color[0] > 0)
			img.at<cv::Vec3b>( y2r(pnt.y),x2c(pnt.x) )[0] = color[0];
		if(color[1] > 0)
			img.at<cv::Vec3b>( y2r(pnt.y),x2c(pnt.x) )[1] = color[1];
		if(color[2] > 0)
			img.at<cv::Vec3b>( y2r(pnt.y),x2c(pnt.x) )[2] = color[2];
	}
}
void draw_info(std::string info, int rectangle_size, int circle_size, int yaw_size, bool pixel,float yaw, cv::Scalar color){
	if(info_point.y > -450){
		info_point.y -= info_interval;
		geometry_msgs::Point info_point_beside;
		info_point_beside.x = info_point.x - info_interval / 2;
		info_point_beside.y = info_point.y + info_interval / 2;
		draw_pnt(info_point_beside,rectangle_size,circle_size,yaw_size,pixel,yaw,color);
		putText(img,info,pnt2cv(info_point),
				cv::FONT_HERSHEY_COMPLEX_SMALL, 0.8, color, 1, CV_AA);
	}
}
void draw_path(std::string info,nav_msgs::Path pathin, int rectangle_size, int circle_size, int yaw_size, bool pixel, cv::Scalar color, bool draw_line){
	ROS_INFO("DRAWING PATH: %s %i",info.c_str(),pathin.poses.size());
	if(pathin.poses.size() == 0)
		return;
	for(int i = 0; i < pathin.poses.size(); i++){
		draw_pnt(pathin.poses[i].pose.position,rectangle_size,circle_size,yaw_size,pixel,tf::getYaw(pathin.poses[i].pose.orientation),color);
		if(draw_line && i > 0)
			cv::line (img, pnt2cv(pathin.poses[i-1].pose.position), pnt2cv(pathin.poses[i].pose.position),color,1,cv::LINE_8,0);
	}
	draw_info(info,rectangle_size,circle_size,yaw_size,pixel,0.0,color);
}

void draw_poly(std::string info, geometry_msgs::PolygonStamped poly, int rectangle_size, int circle_size, int yaw_size, bool pixel, cv::Scalar color, bool draw_lines){
//	ROS_INFO("DRAWING POLY: %s %i",info.c_str(),poly.polygon.points.size());
	if(poly.polygon.points.size() < 3)
		return;
	for(int i = 0; i < poly.polygon.points.size(); i++){
		geometry_msgs::Point p;
		p.x = poly.polygon.points[i].x;
		p.y = poly.polygon.points[i].y;
		draw_pnt(p,rectangle_size,circle_size,0,pixel,0,color);

		if(draw_lines && i > 0){
		//	ROS_INFO("i: %i i-1: %i",i,i-1);
		//	ROS_INFO("p.x2: %.0f y: %.0f",p.x,p.y);
			p.x = poly.polygon.points[i-1].x;
			p.y = poly.polygon.points[i-1].y;
		//	ROS_INFO("p.x2: %.0f y: %.0f",p.x,p.y);

			cv::line (img, pnt322cv(poly.polygon.points[i-1]), pnt322cv(poly.polygon.points[i]),color,1,cv::LINE_8,0);
		}
	}
	draw_info(info,rectangle_size,circle_size,yaw_size,pixel,0.0,color);
}
double constrainAngle(double x){
  if(x > M_PI)
    return (x - M_PI*2);
  else if(x < -M_PI)
    return (x + M_PI*2);
  else
    return x;
}
void draw_pose(std::string info,geometry_msgs::Point pnt, int rectangle_size, int circle_size, int yaw_size, bool pixel, float yaw,cv::Scalar color){
	draw_pnt(pnt,rectangle_size,circle_size,yaw_size,pixel,yaw,color);
	draw_info(info,rectangle_size,circle_size,yaw_size,pixel,yaw,color);
	if(info == "pos"){
		draw_line(g_pose.pose.position,constrainAngle(tf::getYaw(g_pose.pose.orientation) + M_PI/3),50,color);
		draw_line(g_pose.pose.position,constrainAngle(tf::getYaw(g_pose.pose.orientation) - M_PI/3),50,color);
	}
}
std::vector<geometry_msgs::Point> getinpath_boundingbox(nav_msgs::Path pathin){
  std::vector<geometry_msgs::Point> bbmnbbmx;

  geometry_msgs::Point bbtotmax,bbtotmin;
  bbtotmax.x = bbtotmax.y = bbtotmax.z = -100;
  bbtotmin.x = bbtotmin.y = bbtotmin.z = 100;
  for(int i = 0; i < pathin.poses.size(); i++){
    if(pathin.poses[i].pose.position.x > bbtotmax.x)bbtotmax.x = pathin.poses[i].pose.position.x;
    if(pathin.poses[i].pose.position.y > bbtotmax.y)bbtotmax.y = pathin.poses[i].pose.position.y;
    if(pathin.poses[i].pose.position.z > bbtotmax.z)bbtotmax.z = pathin.poses[i].pose.position.z;
    if(pathin.poses[i].pose.position.x < bbtotmin.x)bbtotmin.x = pathin.poses[i].pose.position.x;
    if(pathin.poses[i].pose.position.y < bbtotmin.y)bbtotmin.y = pathin.poses[i].pose.position.y;
    if(pathin.poses[i].pose.position.z < bbtotmin.z)bbtotmin.z = pathin.poses[i].pose.position.z;
  }
  bbmnbbmx.push_back(bbtotmin);
  bbmnbbmx.push_back(bbtotmax);

  float diag = sqrt(pow(bbtotmax.x-bbtotmin.x,2)+pow(bbtotmax.y-bbtotmin.y,2)+pow(bbtotmax.z-bbtotmin.z,2));
  //ROS_INFO("GENERICNODE: LIMITS: diagonal: %.2f,max(%.2f %.2f %.2f) min(%.2f %.2f %.2f)",diag,bbtotmax.x,bbtotmax.y,bbtotmax.z,bbtotmin.x,bbtotmin.y,bbtotmin.z);
  return bbmnbbmx;
}

void draw_path_boundary(nav_msgs::Path pathin,cv::Scalar color){
	if(pathin.poses.size() == 0)
		return;
	std::vector<geometry_msgs::Point> bbmnmx = getinpath_boundingbox(pathin);
	geometry_msgs::Point bbmin_scan = bbmnmx[0];
	geometry_msgs::Point bbmax_scan = bbmnmx[1];
	cv::rectangle(img, pnt2cv(bbmin_scan),pnt2cv(bbmax_scan),color,1,8,0);
}
void draw_and_reset_img(){
	info_point.x = 100;
	info_point.y = 100;
	cv::Mat img_new2;
	cv::resize(img, img_new2, cv::Size(), 6.0, 6.0);
	naming_count++;
	img_blank.copyTo(img);
	cv::imwrite("/home/nuc/brain/fullpath/"+std::to_string(naming_count)+"navigator.png",img_new2);

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

int get_float(float val){
	return int(round(val));
}
std::string write_xyz(geometry_msgs::Point pnt){
	return " x: " + std::to_string(get_float(pnt.x)) + " x: " + std::to_string(get_float(pnt.y)) + " z: " + std::to_string(get_float(pnt.z));
}
geometry_msgs::Point get_ave_pnt_poly(geometry_msgs::PolygonStamped polyin){
	geometry_msgs::Point pnt;
	if(polyin.polygon.points.size() == 0)
		return pnt;
	for(int i = 0; i < polyin.polygon.points.size(); i++){
		pnt.x += polyin.polygon.points[i].x;
		pnt.y += polyin.polygon.points[i].y;
		pnt.z += polyin.polygon.points[i].z;
	}
  pnt.x /= polyin.polygon.points.size();
  pnt.y /= polyin.polygon.points.size();
  pnt.z /= polyin.polygon.points.size();
  return pnt;
}
void draw_polys(std::string info,std::vector<geometry_msgs::PolygonStamped> polysin,int color_strength){
	if(polysin.size() == 0)
		return;
	int count_color = 0;
	for(int i = 0; i < polysin.size(); i++){
		if(polysin[i].polygon.points.size() > 2){
			geometry_msgs::Point p_cen = get_ave_pnt_poly(polysin[i]);
			std::string s1 = std::to_string(polysin[i].polygon.points.size());
			std::string s2 = write_xyz(p_cen);
			std::string s3 = std::to_string(i);
			std::string info_new = info + s1 + s2 + s3;
			count_color++;
			if(count_color == 3)
				count_color = 0;
			draw_poly(info,polysin[i],0,0,0,true,get_shifting_color(count_color,color_strength),true);
		}
	}
}
void draw_polys_by_score(std::vector<geometry_msgs::PolygonStamped> polysin,std::vector<int> scores){
	if(polysin.size() == 0)
		return;
	int count_color = 0;
	int score_min = 100; int score_max = -100;
	for(int i = 0; i < scores.size(); i++){
		if(scores[i] > score_max)
			score_max = scores[i];
		if(scores[i] < score_min)
			score_min = scores[i];
	}
	float score_range = score_max - score_min;
	for(int i = 0; i < polysin.size(); i++){
		if(polysin[i].polygon.points.size() > 2){
			float rel_score = float(scores[i] - score_min)/score_range;
			int rel_color = int(round(255.0 * rel_score));
			std::string info = "score: "+std::to_string(rel_color);
			if(rel_color > 200){
				count_color++;
				draw_poly(info,polysin[i],0,1,1,true,cv::Scalar(0,0,rel_color),true);
			}
		}
	}
	ROS_INFO("DRAWN %i polys",count_color);
}
geometry_msgs::PolygonStamped createpoly_square(geometry_msgs::Point pin, float sidelength){
  geometry_msgs::PolygonStamped poly;
  poly.polygon.points.resize(5);
  poly.header.frame_id = "map";
  poly.polygon.points[0].x = round(pin.x + sidelength/2);
  poly.polygon.points[0].y = round(pin.y + sidelength/2);
  poly.polygon.points[1].x = round(pin.x - sidelength/2);
  poly.polygon.points[1].y = round(pin.y + sidelength/2);
  poly.polygon.points[2].x = round(pin.x - sidelength/2);
  poly.polygon.points[2].y = round(pin.y - sidelength/2);
  poly.polygon.points[3].x = round(pin.x + sidelength/2);
  poly.polygon.points[3].y = round(pin.y - sidelength/2);
  poly.polygon.points[4]   = poly.polygon.points[0];
//  ROS_INFO("x %.2f y %.2f x %.2f y %.2f x %.2f y %.2f x %.2f y %.2f x %.2f y %.2f",poly.polygon.points[0].x,poly.polygon.points[0].y,poly.polygon.points[1].x,poly.polygon.points[1].y,poly.polygon.points[2].x,poly.polygon.points[2].y,poly.polygon.points[3].x,poly.polygon.points[3].y,poly.polygon.points[4].x,poly.polygon.points[4].y);
 return poly;
}
void drawimg(){
	draw_pose("target"+std::to_string(get_float(get_dst3d(g_target_pose.pose.position,g_pose.pose.position))),g_target_pose.pose.position,2,2,5,false,tf::getYaw(g_target_pose.pose.orientation),cv::Scalar(200,200,100));
	draw_pose("pos",g_pose.pose.position,2,2,5,false,tf::getYaw(g_pose.pose.orientation),cv::Scalar(200,200,200));
	draw_path("path_targets",g_path_target_candidates,0,5,1,true,cv::Scalar(100,43,200),false);
	draw_poly("g_poly_boundary",g_poly_boundary,0,0,0,false,cv::Scalar(0,0,200),true);
	ROS_INFO("***************done drawing****************");
	draw_and_reset_img();
}
bool in_vec(std::vector<int> vec,int k){
	if(vec.size() == 0)
		return false;
	//	//ROS_INFO("in vec: %i",k);
	for(int i = 0; i < vec.size(); i++){
		if(vec[i] == k)
			return true;
	}
	return false;
}

int getGridDistancefield(geometry_msgs::Point midpoint, int radlen_xy){
  int pnts = 0; int pnts_tot = 0;
	int tot_sum = 0;
	int min_dst = 100;
	for(int c = x2c(midpoint.x)-radlen_xy; c < x2c(midpoint.x)+radlen_xy; c++){
    for(int r = y2r(midpoint.y)-radlen_xy; r < y2r(midpoint.y)+radlen_xy; r++){
      if(img_distances.at<cv::Vec3b>(r,c)[2] > 0){
				pnts++;
				tot_sum += img_distances.at<cv::Vec3b>(r,c)[2];
				if(img_distances.at<cv::Vec3b>(r,c)[2] < min_dst)
					min_dst = img_distances.at<cv::Vec3b>(r,c)[2];
			}
		}
  }
	int average_dst = tot_sum / fmax(pnts,1);
//	ROS_INFO("getGridDistancefield[%.0f,%.0f]: ave: %i,min: %i",midpoint.x,midpoint.y,average_dst,min_dst);
//	if(min_dst < 10)
//		return 255;
//	else
		return average_dst;
}
int getGriVisitedield(geometry_msgs::Point midpoint, int radlen_xy){
  int pnts = 0;
	int tot_sum = 0;
	for(int c = x2c(midpoint.x)-radlen_xy; c < x2c(midpoint.x)+radlen_xy; c++){
    for(int r = y2r(midpoint.y)-radlen_xy; r < y2r(midpoint.y)+radlen_xy; r++){
      if(img_distances.at<cv::Vec3b>(r,c)[1] > 0){
				pnts++;
				tot_sum += img_distances.at<cv::Vec3b>(r,c)[2];
			}
		}
  }
	int average_dst = tot_sum / fmax(pnts,1);
//	ROS_INFO("getGriVisitedield[%.0f,%.0f]: ave: %i",midpoint.x,midpoint.y,average_dst);
	return average_dst;
}
int getGridElevation(geometry_msgs::Point midpoint, int radlen_xy){
  int pnts = 0;
	int max_elevation = 0;
	int elevation_sum = 0;
	for(int c = x2c(midpoint.x)-radlen_xy; c < x2c(midpoint.x)+radlen_xy; c++){
    for(int r = y2r(midpoint.y)-radlen_xy; r < y2r(midpoint.y)+radlen_xy; r++){
			if(img_distances.at<cv::Vec3b>(r,c)[0] > 0){
				pnts++;
				elevation_sum += img_distances.at<cv::Vec3b>(r,c)[0];
				if(img_distances.at<cv::Vec3b>(r,c)[0] > max_elevation)
					max_elevation = img_distances.at<cv::Vec3b>(r,c)[0];
			}
		}
  }
	int average_elevation = elevation_sum / fmax(pnts,1);
	//ROS_INFO("getGridElevation[%.0f,%.0f]: ave: %i max: %i",midpoint.x,midpoint.y,average_elevation,max_elevation);
	return average_elevation;
}
int getGridCoverage(geometry_msgs::Point midpoint, int radlen_xy){
  int pnts = 0; int pnts_tot = 0;
	for(int c = x2c(midpoint.x)-radlen_xy; c < x2c(midpoint.x)+radlen_xy; c++){
    for(int r = y2r(midpoint.y)-radlen_xy; r < y2r(midpoint.y)+radlen_xy; r++){
      pnts_tot++;
      if(img_distances.at<cv::Vec3b>(r,c)[0] > 0)
        pnts++;
    }
  }
	int coverage = 100 * pnts / (pnts_tot+1);
	//ROS_INFO("getGridCoverage[%.0f,%.0f]: %i",midpoint.x,midpoint.y,coverage);
  return coverage;
}
void distanceCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
	cv::imwrite("/home/nuc/brain/fullpath/"+std::to_string(naming_count)+"img_basis.png",img_distances);
	img_blank.copyTo(img_distances);
	ROS_INFO("ABMAP: distanceCloudCallback");
	if(msg->data.size() > 10 && msg->header.frame_id == "map")
	{
		sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x");
		sensor_msgs::PointCloud2ConstIterator<float> iter_y(*msg, "y");
		sensor_msgs::PointCloud2ConstIterator<float> iter_z(*msg, "z");
		sensor_msgs::PointCloud2ConstIterator<uint8_t> iter_rgb(*msg, "rgb");

		for (int i = 0; iter_x != iter_x.end(); ++i, ++iter_x, ++iter_y, ++iter_z, ++iter_rgb)
		{
			if (!std::isnan(*iter_x) && !std::isnan(*iter_y) && !std::isnan(*iter_z) &&
					!std::isnan(iter_rgb[0]) && !std::isnan(iter_rgb[1]) && !std::isnan(iter_rgb[2]))
	    {
	      int r = y2r(*iter_y);
		    int c = x2c(*iter_x);
				img_distances.at<cv::Vec3b>(r,c)[2] = iter_rgb[2]; //dst_obst
				img_distances.at<cv::Vec3b>(r,c)[1] = iter_rgb[1]; //dst_vstd
				img_distances.at<cv::Vec3b>(r,c)[0] = iter_rgb[0]; //registered_elevation
			}
		}
	}

	ROS_INFO("ABMAP: distanceCloudCallback_complete");

}
geometry_msgs::PoseStamped getPoseInMap(std::string frame){
	geometry_msgs::TransformStamped transformStamped;
	try{
		transformStamped = tfBuffer.lookupTransform("map",frame,
														 ros::Time(0));
	}
	catch (tf2::TransformException &ex) {
		ROS_WARN("%s",ex.what());
		ros::Duration(1.0).sleep();
	}
	geometry_msgs::PoseStamped psout;
	psout.pose.position.x  = transformStamped.transform.translation.x;
	psout.pose.position.y  = transformStamped.transform.translation.y;
	psout.pose.position.z  = transformStamped.transform.translation.z;
	psout.pose.orientation = transformStamped.transform.rotation;
	psout.header 					 = transformStamped.header;
	return psout;
}
void gridsActiveCallback(const nav_msgs::Path::ConstPtr& msg){
	g_path_target_candidates = *msg;
}
float find_pathspread(nav_msgs::Path pathin){

	float lowest_dist = 100;
	for(int i = 1; i < pathin.poses.size(); i++){
		float dx = pathin.poses[i].pose.position.x-pathin.poses[i-1].pose.position.x;
		float dy = pathin.poses[i].pose.position.y-pathin.poses[i-1].pose.position.y;
		if(dx < lowest_dist && dx > 0)
			lowest_dist = dx;
		if(dy < lowest_dist && dy > 0)
			lowest_dist = dy;
	}
//	ROS_INFO("ABMAP: lowest_dist: %.0f",lowest_dist);
	return lowest_dist;
}

std::vector<geometry_msgs::PolygonStamped> getGridpathPolys(nav_msgs::Path pathin,float grid_sidelength){
	std::vector<geometry_msgs::PolygonStamped> polysout;
	polysout.resize(pathin.poses.size());
	for(int i = 0; i < pathin.poses.size(); i++){
		polysout[i] = createpoly_square(pathin.poses[i].pose.position,grid_sidelength/2);
	}
//	ROS_INFO("Created %i polys",polysout.size());
	return polysout;
}
int getGridCombined(geometry_msgs::Point grid_midpoint,int radlen_xy){
	int grid_Distancefield = getGridDistancefield(grid_midpoint,radlen_xy);
	int grid_isitedield 	 = getGriVisitedield(grid_midpoint,radlen_xy);
	int grid_Elevation 		 = getGridElevation(grid_midpoint,radlen_xy);
	int grid_Coverage 		 = getGridCoverage(grid_midpoint,radlen_xy);
	int score = grid_isitedield - grid_Coverage - grid_Distancefield;
	ROS_INFO("getGridElevation[%.0f,%.0f]: dstfield: %i visited: %i elevation: %i coverage: %i - score: %i",grid_midpoint.x,grid_midpoint.y,grid_Distancefield,grid_isitedield,grid_Elevation,grid_Coverage,score);
	return score;
}
std::vector<int> getScores(nav_msgs::Path pathin, float radlen_xy){
	std::vector<int> scores;
	scores.resize(pathin.poses.size());
	for(int i = 0; i < pathin.poses.size(); i++){
		scores[i] = getGridCombined(pathin.poses[i].pose.position,int(round(radlen_xy)));
	}

	return scores;
}
int main(int argc, char** argv)
{
  ros::init(argc, argv, "tb_abstractmap_node");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");
	private_nh.param("resolution", par_res, 1.0);
	private_nh.param("write_heightimage_interval", par_imwrite_interval, 2.0);
	private_nh.param("grid_sidelength", par_gridsize, 5.0);
	tf2_ros::TransformListener tf2_listener(tfBuffer);
	g_target_pose.pose.orientation.w = 1.0;
	g_pose.pose.orientation.w = 1.0;
	ros::Subscriber s1  = nh.subscribe("/tb_distancecloud/generated_cloud_2d",10,distanceCloudCallback);
	ros::Subscriber s2  = nh.subscribe("/tb_distancecloud/grids_active",10,gridsActiveCallback);
	ros::Publisher pub_target  = nh.advertise<geometry_msgs::PoseStamped>("/tb_abstractmap/target_candidate", 100);

	ros::Rate rate(5.0);
	ros::Time last_drawimg = ros::Time::now();

  while(ros::ok()){
		g_pose = getPoseInMap("base_stabilized");
		float grid_sidelength = find_pathspread(g_path_target_candidates);
		ROS_INFO("ABMAP: grid_side: %.0f",grid_sidelength);
		std::vector<geometry_msgs::PolygonStamped> polys = getGridpathPolys(g_path_target_candidates,grid_sidelength);
		std::vector<int> scores = getScores(g_path_target_candidates,grid_sidelength);
		draw_polys_by_score(polys,scores);
		int best_i = 0;
		for(int i = 1; i < scores.size(); i++){
			if(scores[i] > scores[best_i])
				best_i = i;
		}
		if(best_i > 0 && best_i < g_path_target_candidates.poses.size())
			pub_target.publish(g_path_target_candidates.poses[best_i]);
		if((ros::Time::now()-last_drawimg).toSec() > 3.0){
			last_drawimg = ros::Time::now();
			drawimg();
		}
		rate.sleep();
		ros::spinOnce();
	}
  return 0;
}
	//
