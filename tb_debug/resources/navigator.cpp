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
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
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
#include <sensor_msgs/point_cloud2_iterator.h>

tf2_ros::Buffer tfBuffer;
std::string par_workdir = "/home/nuc/brain/pathfull/heights0.png";
cv::Mat img(1000,1000,CV_8UC3,cv::Scalar(0, 0, 0)); //create image, set encoding and size, init pixels to default val
cv::Mat img_height(1000,1000,CV_8UC3,cv::Scalar(0, 0, 0)); //create image, set encoding and size, init pixels to default val
std::string state_scanpoint = "target_gp";
std::string state_perfalt   = "no_data";
std::string state_gp_pnt    = "no_data";
std::string state_pos_pnt   = "no_data";
std::string state_forwpah   = "filling";
float state_octomap_zmax = 0;
float state_octomap_zmin = 0;
float rad2deg = 180.0/M_PI;
float deg2rad = M_PI/180;
///////////********CTRL***********//////////
int count_target_paths = 0;
int mainstate = 0;
bool got_map = false;
bool cansee_proximity = false;
bool par_debug_img;
double par_maprad,par_elevbb,par_zclearing,par_arearad;
///////////********CTRL***********//////////

std::string state_mb = "idle";
float state_globalplan_elevz = 0;
std::vector<int> targetindexes_sent;
nav_msgs::Path path_forwsim,path_starsquare,path_global_plan,path_visited,path_st,path_scan;
ros::Publisher pub_path_frontier,pub_poly_scanzone,pub_path_global,pub_poly_proximity,pub_path_elevated,pub_winner,pub_perfectalt_elevation,pub_map_updates,pub_setpoint,pub_mbtarget,pub_setodom;
geometry_msgs::PolygonStamped poly_mb,poly_scanzone,poly_proximity;
ros::Time process_start;
std::string active_process;
using namespace octomap;
using namespace std;
shared_ptr<DynamicEDTOctomap> edf_ptr;
AbstractOcTree* abs_octree;
shared_ptr<OcTree> octree;
geometry_msgs::PointStamped scanpoint_mid,closest_obstacle,last_mbmidpnt;
float closest_obstacle_dst = 10;
float min_dst = 5.0;
float max_dst = 10.0;
geometry_msgs::PointStamped perfalt_pnt,gp_pnt,scantarget_pnt,pos_pnt;
geometry_msgs::Point pos;
geometry_msgs::Vector3 vlp_rpy;
float pos_yaw;
float par_res = 1.0;
geometry_msgs::PoseStamped ps_forwsim,pose_setpoint,targetmb;
nav_msgs::Odometry odom_global;
std::deque<int> zvals_forwsim;
int costmap_elevation_meters = 0;
sensor_msgs::LaserScan scan_frontier;
geometry_msgs::PointStamped point_global_target;
geometry_msgs::PointStamped point_elevation_checked;
geometry_msgs::PolygonStamped path_poly,get_down_poly,get_side_poly,get_poly_poly;
nav_msgs::Path path_frontier,path_down,path_side,path_base,path_cmd;
std_msgs::Header hdr(){
  std_msgs::Header header;
  header.frame_id = "map";
  header.stamp    = ros::Time::now();
  return header;
}
nav_msgs::Path create_starsquare(float area_rad){
  nav_msgs::Path path;
  path.poses.resize(9);
  path.header = hdr();
  float z = 0;
  path.poses[0].pose.position.x = 0;
  path.poses[0].pose.position.y = 0;
  path.poses[0].header= hdr();
  path.poses[0].pose.orientation.w = 1;
  path.poses[0].pose.position.z =z;

  path.poses[1].pose.position.x = -area_rad;
  path.poses[1].pose.position.y = area_rad;
  path.poses[1].header= hdr();
  path.poses[1].pose.orientation.w = 1;
  path.poses[1].pose.position.z =z;

  path.poses[2].pose.position.x = area_rad;
  path.poses[2].pose.position.y = 0;
  path.poses[2].header= hdr();
  path.poses[2].pose.orientation.w = 1;
  path.poses[2].pose.position.z =z;

  path.poses[3].pose.position.x = -area_rad;
  path.poses[3].pose.position.y = -area_rad;
  path.poses[3].header= hdr();
  path.poses[3].pose.orientation.w = 1;
  path.poses[3].pose.position.z =z;

  path.poses[4].pose.position.x = area_rad;
  path.poses[4].pose.position.y = area_rad;
  path.poses[4].header= hdr();
  path.poses[4].pose.orientation.w = 1;
  path.poses[4].pose.position.z =z;

  path.poses[5].pose.position.x = -area_rad;
  path.poses[5].pose.position.y = 0;
  path.poses[5].header= hdr();
  path.poses[5].pose.orientation.w = 1;
  path.poses[5].pose.position.z =z;

  path.poses[6].pose.position.x = area_rad;
  path.poses[6].pose.position.y = -area_rad;
  path.poses[6].header= hdr();
  path.poses[6].pose.orientation.w = 1;
  path.poses[6].pose.position.z =z;;


  path.poses[6].pose.position.x = 0;
  path.poses[6].pose.position.y = area_rad;
  path.poses[6].header= hdr();
  path.poses[6].pose.orientation.w = 1;
  path.poses[6].pose.position.z =z;

  path.poses[7].pose.position.x = 0;
  path.poses[7].pose.position.y = -area_rad;
  path.poses[7].header= hdr();
  path.poses[7].pose.orientation.w = 1;
  path.poses[7].pose.position.z =z;;


  path.poses[8].pose.position.x = 0;
  path.poses[8].pose.position.y = 0;
  path.poses[8].header= hdr();
  path.poses[8].pose.orientation.w = 1;
  path.poses[8].pose.position.z =z;
  return path;
}



void start_process(std::string name){
  float dt = (ros::Time::now() - process_start).toSec();
  if(active_process != "")
    ROS_INFO("Process: %s took %.4f sec",active_process.c_str(),dt);
  active_process = name;
  process_start  = ros::Time::now();
}
bool sort_dst_pair(const std::tuple<int,float>& a,
               const std::tuple<int,float>& b)
{
    return (std::get<1>(a) < std::get<1>(b));
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
double constrainAngle(double x){
  if(x > M_PI)
    return (x - M_PI*2);
  else if(x < -M_PI)
    return (x + M_PI*2);
  else
    return x;
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
std::vector<int> constrain_path_indexes_bbpoly(nav_msgs::Path pathin,geometry_msgs::PolygonStamped poly_bb){
  std::vector<int> indexes_out;
  if(pathin.poses.size() == 0 || poly_bb.polygon.points.size() == 0)
    return indexes_out;
  for(int i = 0; i < pathin.poses.size(); i++){
    if(in_poly(poly_bb,pathin.poses[i].pose.position.x,pathin.poses[i].pose.position.y)){
      indexes_out.push_back(i);
    }
  }
  return indexes_out;
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
std::vector<float> vec_to_min_max_ave(std::vector<float> vec_in){
  std::vector<float> min_max_ave;
  min_max_ave.push_back(143131);
  min_max_ave.push_back(-143131);
  min_max_ave.push_back(0);
  if(vec_in.size() == 0){
    min_max_ave[0] = 0;
    min_max_ave[1] = 0;
    return min_max_ave;
  }

  float vec_sum = 0;
  for(int i = 0; i < vec_in.size(); i++){
    vec_sum += vec_in[i];
    if(vec_in[i] > min_max_ave[1])
     min_max_ave[1] = vec_in[i];
    if(vec_in[i] < min_max_ave[0])
     min_max_ave[0] = vec_in[i];
  }
    min_max_ave[2] = vec_sum / vec_in.size();
  return min_max_ave;
}
float y2r(float y){
  return (img_height.rows / 2 - y / par_res);
}
float x2c(float x){
  return (x / par_res + img_height.cols/2);
}
int r2y(float r){
  return int((img_height.rows / 2 - r) * par_res);
}
int c2x(float c){
  return int((c - img_height.cols / 2) * par_res);
}
cv::Point pnt2cv(geometry_msgs::Point pin){
	int c = x2c(pin.x);
	int r = y2r(pin.y);
	return cv::Point(c,r);
}

void draw_path_by_score(nav_msgs::Path pathin,std::vector<float> score,int primary_color,int secondary_color,int tertiary_color,float color_intensity){
  std::vector<float> mma = vec_to_min_max_ave(score);
  if(pathin.poses.size() < 1)
    return;
	for(int i = 0; i < pathin.poses.size(); i++){
		geometry_msgs::Point pnt = pathin.poses[i].pose.position;
    cv::Scalar color;
    float rel_score = color_intensity * (score[i] - mma[0]) / (mma[1]-mma[0]);
    if(rel_score > 0.9)
      color[tertiary_color] = 255*rel_score;
    if(rel_score > 0.75)
      color[secondary_color] = 255*rel_score;
    if(rel_score > 0.3)
      color[primary_color] = 255*rel_score;
    if(rel_score < 0.3)
      color[primary_color] = 100*rel_score;
    if(pathin.poses.size() > 25){
      img.at<cv::Vec3b>( y2r(pnt.y),x2c(pnt.x) )[0] = color[0];
      img.at<cv::Vec3b>( y2r(pnt.y),x2c(pnt.x) )[1] = color[1];
      img.at<cv::Vec3b>( y2r(pnt.y),x2c(pnt.x) )[2] = color[2];
    }
    else
      cv::circle(img,pnt2cv(pnt),2,color,1);
	}
}
void draw_path(nav_msgs::Path pathin,cv::Scalar color, int size){
	for(int i = 0; i < pathin.poses.size(); i++){
		geometry_msgs::Point pnt = pathin.poses[i].pose.position;
    if(size == 0){
      img.at<cv::Vec3b>( y2r(pnt.y),x2c(pnt.x) )[0] = color[0];
      img.at<cv::Vec3b>( y2r(pnt.y),x2c(pnt.x) )[1] = color[1];
      img.at<cv::Vec3b>( y2r(pnt.y),x2c(pnt.x) )[2] = color[2];
    }
    else
      cv::circle(img,pnt2cv(pnt),size,color,1);
	}
}

cv::Scalar get_color(int base_blue,int base_green, int base_red){
	cv::Scalar color;
	color[0] = base_blue;
	color[1] = base_green;
	color[2] = base_red;
	return color;
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
  if(polyin.polygon.points.size() > 2){
    geometry_msgs::Point p1,p2;
    p1.x = polyin.polygon.points[polyin.polygon.points.size()-1].x;
    p1.y = polyin.polygon.points[polyin.polygon.points.size()-1].y;
    p2.x = polyin.polygon.points[0].x;
    p2.y = polyin.polygon.points[0].y;
    cv::line (img, pnt2cv(p1), pnt2cv(p2),color,1,cv::LINE_8,0);
  }
}
float get_zmax_grid_custom(geometry_msgs::Point pnt1,geometry_msgs::Point pnt2,float radlen_xy){
  int c1 = x2c(pnt1.x-radlen_xy);
  int c0 = x2c(pnt2.x+radlen_xy);
  int r1 = y2r(pnt1.y-radlen_xy);
  int r0 = y2r(pnt2.y+radlen_xy);
  float zmx = 0;
  for(int c = fmin(c1,c0); c < fmax(c1,c0); c++){
    for(int r = fmin(r1,r0); r < fmax(r1,r0); r++){
      if(img_height.at<cv::Vec3b>(r,c)[2] > zmx)
        zmx = img_height.at<cv::Vec3b>(r,c)[2];
    }
  }
  return zmx;
}
void draw_grid_within(geometry_msgs::Point pnt1,geometry_msgs::Point pnt2,float radlen_xy){
  float zmax = get_zmax_grid_custom(pnt1,pnt2,radlen_xy);
  int c1 = x2c(pnt1.x-radlen_xy);
  int c0 = x2c(pnt2.x+radlen_xy);
  int r1 = y2r(pnt1.y-radlen_xy);
  int r0 = y2r(pnt2.y+radlen_xy);
  for(int c = fmin(c1,c0); c < fmax(c1,c0); c++){
    for(int r = fmin(r1,r0); r < fmax(r1,r0); r++){
      float zval = img_height.at<cv::Vec3b>(r,c)[2];
      if(zval > zmax * 0.66)
        img.at<cv::Vec3b>(r,c)[2] = 255*(zval / zmax);
      else if(zval > zmax * 0.33)
        img.at<cv::Vec3b>(r,c)[1] = 255*(zval / zmax);
      else
        img.at<cv::Vec3b>(r,c)[0] = 255*(zval / zmax);
    }
  }
  return;
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
  cv::line (img, pnt2cv(pos), pnt2cv(pyaw),color,1,cv::LINE_8,0);
}
void draw_path_poses(nav_msgs::Path pathin,cv::Scalar color, int size){
	for(int i = 0; i < pathin.poses.size(); i++){
		geometry_msgs::Point pnt = pathin.poses[i].pose.position;
		draw_line(pnt,tf::getYaw(pathin.poses[i].pose.orientation),3,color);
    cv::circle(img,pnt2cv(pnt),size,color,1);
	}
}
void create_mapelevation_visualization(geometry_msgs::Point startpnt){
  int ffillMode = 1;  int connectivity = 8;  int newMaskVal = 255;
  int flags = connectivity + (newMaskVal << 8) +
       (ffillMode == 1 ? cv::FLOODFILL_FIXED_RANGE : 0);
  cv::Rect ccomp;
	cv::Mat img_filled(1000,1000,CV_8U,cv::Scalar(0)); //create image, set encoding and size, init pixels to default val
	for(int c = 0; c < img_filled.cols; c++){
		for(int r = 0; r < img_filled.rows; r++){
			img_filled.at<uchar>(r,c) = img.at<cv::Vec3b>(r,c)[2];
		}
	}
	int hi = costmap_elevation_meters - img_filled.at<uchar>(y2r(pos.y),x2c(pos.x));
  int lo = img_filled.at<uchar>(y2r(pos.y),x2c(pos.x))-1;
  if(hi < 0)
    hi = 0;
  if(lo < 0)
    lo = 0;
  int area  = cv::floodFill(img_filled,cv::Point(x2c(pos.x),y2r(pos.y)),cv::Scalar(150), &ccomp, cv::Scalar(lo),cv::Scalar(hi), flags);
  for(int c = 0; c < img_filled.cols; c++){
    for(int r = 0; r < img_filled.rows; r++){
      if(img_filled.at<uchar>(r,c) == 150){
        img.at<cv::Vec3b>(r,c)[0] = 50;
      }
    }
  }
}

void draw_scan(cv::Scalar color){
	std::vector<geometry_msgs::Point> bbmnmx = getinpath_boundingbox(path_scan);
	geometry_msgs::Point bbmin_scan = bbmnmx[0];
	geometry_msgs::Point bbmax_scan = bbmnmx[1];
	draw_path(path_scan,color,0);

	cv::rectangle(img, pnt2cv(bbmin_scan),pnt2cv(bbmax_scan),color,1,8,0);
	cv::circle(img,pnt2cv(scanpoint_mid.point),2,color,1);
	putText(img,"sp:"+std::to_string(int(scanpoint_mid.point.z)), pnt2cv(scanpoint_mid.point),
			cv::FONT_HERSHEY_COMPLEX_SMALL, 0.8, color, 1, CV_AA);
	draw_poly(poly_proximity,color);
	draw_poly(poly_scanzone,color);
	draw_grid_within(bbmin_scan,bbmax_scan,10.0);
}
void draw_pos(cv::Scalar color){
	draw_line(pos,constrainAngle(vlp_rpy.z + M_PI/3),50,color);
	draw_line(pos,constrainAngle(vlp_rpy.z - M_PI/3),50,color);
	draw_line(pos,constrainAngle(vlp_rpy.z),50,color);
	cv::circle(img,pnt2cv(pos),4,color,1);
	putText(img,"posz: "+std::to_string(int(round(pos.z))), pnt2cv(pos),
			cv::FONT_HERSHEY_COMPLEX_SMALL, 0.8, color, 1, CV_AA);
}
void draw_target(cv::Scalar color){
	draw_rectangle(point_global_target.point,3,color);
	cv::line (img, pnt2cv(pos), pnt2cv(point_global_target.point),color,1,cv::LINE_8,0);
	cv::circle(img,pnt2cv(point_global_target.point),4,color,1);
	putText(img,"costmap_elevation:"+std::to_string(costmap_elevation_meters), pnt2cv(point_global_target.point),
			cv::FONT_HERSHEY_COMPLEX_SMALL, 0.8, color, 1, CV_AA);
}

void draw_gp(cv::Scalar color){
	draw_path(path_global_plan,color,0);
	if(state_gp_pnt == "got_grid" || state_gp_pnt == "got_edto"){
		putText(img,"gp:"+std::to_string(int(gp_pnt.point.z)), pnt2cv(gp_pnt.point),
				cv::FONT_HERSHEY_COMPLEX_SMALL, 0.8, color, 1, CV_AA);
		cv::circle(img,pnt2cv(gp_pnt.point),2,color,1);
		if(state_gp_pnt == "got_edto")
			draw_rectangle(gp_pnt.point,3,color);
	}
}
void draw_sidedown(){
	cv::Scalar color_side = get_color(0,200,100);
	cv::Scalar color_down = get_color(200,0,100);
	draw_path(path_down,color_down,1);
	draw_poly(get_down_poly,color_down);
	draw_path_poses(path_side,color_side,2);
	draw_poly(get_side_poly,color_side);
	if(path_side.poses.size() > 3){
		putText(img,"hori: "+std::to_string(path_side.poses.size()), pnt2cv(path_side.poses[path_side.poses.size()/2].pose.position),
				cv::FONT_HERSHEY_COMPLEX_SMALL, 0.8, color_side, 1, CV_AA);
	}
	if(path_down.poses.size() > 3){
		putText(img,"hori: "+std::to_string(path_down.poses.size()), pnt2cv(path_down.poses[path_down.poses.size()/2].pose.position),
				cv::FONT_HERSHEY_COMPLEX_SMALL, 0.8, color_down, 1, CV_AA);
	}
}
void draw_misc_paths(){
	draw_path(path_starsquare,get_color(200,0,200),3);
	draw_path(path_visited,get_color(100,200,0),2);
	draw_path(path_base,get_color(0,100,150),2);
	draw_path(path_cmd,get_color(0,100,200),2);
}

void drawimg(){
  img_height.copyTo(img);


	create_mapelevation_visualization(pos);
	draw_path(path_frontier,get_color(0,200,200),1);
	draw_scan(get_color(0,0,200));
	draw_gp(get_color(45,95,55));
	draw_pos(get_color(200,200,200));
	draw_target(get_color(100,100,200));
	draw_sidedown();
  cv::Mat img_new2;
  cv::resize(img, img_new2, cv::Size(), 2.0, 2.0);
  count_target_paths++;
  cv::imwrite("/home/nuc/brain/fullpath/"+std::to_string(count_target_paths)+"navigator.png",img_new2);
}

//std::deque<int> q{5, 1, 3};
//std::deque<int>::iterator it = std::min_element(q.begin(), q.end());
//std::vector<int>::iterator result = std::min_element(v.begin(), v.end());
//std::cout << *it << std::endl;
////////////////////////////**********************************/////////////////////////////
////////////////////////////**********************************/////////////////////////////
////////////////////////////**********************************/////////////////////////////
nav_msgs::Path merge_paths(std::vector<nav_msgs::Path> paths){
  nav_msgs::Path pathout;
  for(int i = 0; i < paths.size(); i++){
    for(int k = 0; k < paths[i].poses.size(); k++){
      pathout.poses.push_back(paths[i].poses[k]);
    }
  }
  pathout.header = hdr();
  return pathout;
}
int get_area_coverage(geometry_msgs::Point midpoint, int radlen_xy){
  int c1 = x2c(midpoint.x-radlen_xy);
  int c0 = x2c(midpoint.x+radlen_xy);
  int r1 = y2r(midpoint.y-radlen_xy);
  int r0 = y2r(midpoint.y+radlen_xy);
  int pnts = 0; int pnts_tot = 0;
  for(int c = fmin(c1,c0); c < fmax(c1,c0); c++){
    for(int r = fmin(r1,r0); r < fmax(r1,r0); r++){
      pnts_tot++;
      if(img_height.at<cv::Vec3b>(r,c)[1] > 0)
        pnts++;
    }
  }
  int coverage = 100 * pnts / (pnts_tot+1);
  return coverage;
}
int get_zmax_path(nav_msgs::Path pathin){
  float zmx = 0;
  for(int i = 0; i < pathin.poses.size(); i++){
    if(pathin.poses[i].pose.position.z > zmx){
      zmx = pathin.poses[i].pose.position.z;
    }
  }
  return int(zmx);
}

int get_zmax_grid(geometry_msgs::Point midpoint, int radlen_xy){
  int c1 = x2c(midpoint.x-radlen_xy);
  int c0 = x2c(midpoint.x+radlen_xy);
  int r1 = y2r(midpoint.y-radlen_xy);
  int r0 = y2r(midpoint.y+radlen_xy);
  int zmx = 0;
  for(int c = fmin(c1,c0); c < fmax(c1,c0); c++){
    for(int r = fmin(r1,r0); r < fmax(r1,r0); r++){
      if(img_height.at<cv::Vec3b>(r,c)[2] > zmx)
        zmx = img_height.at<cv::Vec3b>(r,c)[2];
    }
  }
  return zmx;
}


////////////////////////////**********************************/////////////////////////////
////////////////////////////**********************************/////////////////////////////

////////////////////////////**********************************/////////////////////////////
////////////////////////////**********************************/////////////////////////////
float find_zmin_clearing(geometry_msgs::Point pnt,float radlen_xy_min,float z_dn,float z_up, int z_interval){
  if(!got_map)
    return -1;
  geometry_msgs::Point bbmin_custom,bbmax_custom,bbmin_octree,bbmax_octree,bbmin,bbmax;
  float collision_radius = radlen_xy_min + 5 + z_interval;
  bbmin_custom.x = pnt.x-radlen_xy_min;
  bbmin_custom.y = pnt.y-radlen_xy_min;
  bbmin_custom.z = pnt.z-z_dn;
  bbmax_custom.x = pnt.x+radlen_xy_min;
  bbmax_custom.y = pnt.y+radlen_xy_min;
  bbmax_custom.z = pnt.z+z_up;
  octree.get()->getMetricMin(bbmin_octree.x,bbmin_octree.y,bbmin_octree.z);
  octree.get()->getMetricMax(bbmax_octree.x,bbmax_octree.y,bbmax_octree.z);

  if(bbmin_custom.z >= bbmax_octree.z-2)
    return (bbmax_octree.z+2);
	bbmin.x = fmax(bbmin_custom.x,bbmin_octree.x);
	bbmin.y = fmax(bbmin_custom.y,bbmin_octree.y);
	bbmin.z = fmax(bbmin_custom.z,bbmin_octree.z);

	bbmax.x = fmin(bbmax_custom.x,bbmax_octree.x);
	bbmax.y = fmin(bbmax_custom.y,bbmax_octree.y);
	bbmax.z = fmin(bbmax_custom.z,bbmax_octree.z);

  int zrnge = round(bbmax.z-bbmin.z)-1;
  int zn = zrnge / z_interval;
  for(int z = 1; z < zn; z++){
    float z0 = bbmin.z + (z-1)*z_interval;
    float z1 = bbmin.z + z*z_interval;
    octomap::point3d boundary_min(bbmin.x,bbmin.y,z0);
    octomap::point3d boundary_max(bbmax.x,bbmax.y,z1);
    edf_ptr.reset (new DynamicEDTOctomap(collision_radius,octree.get(),
            boundary_min,
            boundary_max,
            false));
    edf_ptr.get()->update();
    point3d closestObst;
    point3d p(pnt.x,pnt.y,(z0+z1)/2);
    float dst = collision_radius;
    edf_ptr.get()->getDistanceAndClosestObstacle(p, dst, closestObst);

    //ROS_INFO("Zinterval[%i]: p.z(): %.0f (z0:%.0f->z1:%.0f), dst: %.2f, obst-z: %.0f",z,p.z(),z0,z1,dst,closestObst.z());
    if(dst > radlen_xy_min)
      return p.z();
  }
  ROS_ERROR("Zinterval[FAILED]: pnt: %.0f %.0f %.0f minxy: %.0f z0:%.0f,z1:%.0f z_interval: %i",pnt.x,pnt.y,pnt.z,radlen_xy_min,z_dn,z_up,z_interval);

  return -1;
}
float get_inclination_target(){
  return get_inclination(scantarget_pnt.point,pos);
}
float get_inclination_error(float inclination_target){
  return get_shortest(get_inclination_target(),vlp_rpy.y);
}
float get_rmnmx_interval(sensor_msgs::LaserScan scanin,int i0, int iN,bool max){
  float mn = scanin.range_max;
  float mx = 0;
  for(int i = i0; i < iN; i++){
    if(!std::isinf(scanin.ranges[i])){
      if(scanin.ranges[i] < mn)
        mn = scanin.ranges[i];
      if(scanin.ranges[i] > mx)
        mx = scanin.ranges[i];
    }
  }
  if(max)
    return fmin(mx,scanin.range_max);
  else
    return mn;
}
void create_proximity_polygon(sensor_msgs::LaserScan scan){
  int num_is = 64;
  int scans_pr_i = scan.ranges.size() / num_is;
  if(scans_pr_i < 0){
    scans_pr_i = 1;
    num_is = scan.ranges.size();
  }
  poly_proximity.polygon.points.resize(num_is);
  for(int i = 0; i < num_is; i++){
    int i0  = fmax(scans_pr_i*i-scans_pr_i*2,0);
    int in  = fmin(scans_pr_i*i+scans_pr_i*2,scan.ranges.size());
    float a = constrainAngle(vlp_rpy.z + scan.angle_min + scans_pr_i * i  * scan.angle_increment);
    float r = get_rmnmx_interval(scan,i0,in,false);
    poly_proximity.polygon.points[i].z = pos.z;
    poly_proximity.polygon.points[i].x = pos.x + r * cos(a);
    poly_proximity.polygon.points[i].y = pos.y + r * sin(a);
  }
  poly_proximity.header = hdr();
  pub_poly_proximity.publish(poly_proximity);
}

void create_scanzone_polygon(sensor_msgs::LaserScan scanin){
  geometry_msgs::PointStamped p1_out,p2_out,p1,p2;
  p1.header = scanin.header;
  p2.header = scanin.header;
  int scans_pr_i = 100;
  float last_rmx,last_rmn;
  last_rmx = scanin.range_max-10;
  last_rmn = scanin.range_min+10;
  poly_scanzone.polygon.points.resize(2*scanin.ranges.size());
  for(int i = 0; i < scanin.ranges.size(); i++){
    int i0  = fmax(i-scans_pr_i,0);
    int in  = fmin(i+scans_pr_i,scanin.ranges.size());
    float rmx = get_rmnmx_interval(scanin,i0,in,true);
    float rmn = get_rmnmx_interval(scanin,i0,in,false);
    if(rmx == 0){
      rmx = last_rmx;
      rmn = last_rmn;
    }
    if(rmn == 0 && rmx > 10)
      rmn = rmx-10;
    else if(rmn == rmx && rmx > 20)
      rmn = rmx-10;
    else if(rmn == rmx && rmn > 5)
      rmx = rmn+10;
    last_rmn = rmn;
    last_rmx = rmx;
    float a = scanin.angle_min + scanin.angle_increment * i;
    p1.point.x = rmn * cos(a);
    p1.point.y = rmn * sin(a);
    p2.point.x = rmx * cos(a);
    p2.point.y = rmx * sin(a);
    try{
        p1_out = tfBuffer.transform(p1, "map");
        p2_out = tfBuffer.transform(p2, "map");
        poly_scanzone.polygon.points[i].z = p1_out.point.z;
        poly_scanzone.polygon.points[i].x = p1_out.point.x;
        poly_scanzone.polygon.points[i].y = p1_out.point.y;
        poly_scanzone.polygon.points[poly_scanzone.polygon.points.size()-1-i].z = p2_out.point.z;
        poly_scanzone.polygon.points[poly_scanzone.polygon.points.size()-1-i].x = p2_out.point.x;
        poly_scanzone.polygon.points[poly_scanzone.polygon.points.size()-1-i].y = p2_out.point.y;
    }
    catch (tf2::TransformException &ex) {
          ROS_WARN("%s",ex.what());
          ros::Duration(1.0).sleep();
          continue;
    }
  }
  poly_scanzone.header = hdr();
  pub_poly_scanzone.publish(poly_scanzone);
}
void create_scan_polygon(sensor_msgs::LaserScan scanin){
  geometry_msgs::PointStamped p1_out,p1;
  p1.header = scanin.header;
  int scans_pr_i = 10;
  poly_scanzone.polygon.points.resize(scanin.ranges.size()+1);
  for(int i = 0; i < scanin.ranges.size(); i++){
    int i0  = fmax(i-scans_pr_i,0);
    int in  = fmin(i+scans_pr_i,scanin.ranges.size());
    float rmx = get_rmnmx_interval(scanin,i0,in,true);
    float a    = scanin.angle_min + scanin.angle_increment * i;
    p1.point.x = rmx * cos(a);
    p1.point.y = rmx * sin(a);
    try{
        p1_out = tfBuffer.transform(p1, "map");
        poly_scanzone.polygon.points[i].z = p1_out.point.z;
        poly_scanzone.polygon.points[i].x = p1_out.point.x;
        poly_scanzone.polygon.points[i].y = p1_out.point.y;
    }
    catch (tf2::TransformException &ex) {
          ROS_WARN("%s",ex.what());
          ros::Duration(1.0).sleep();
          continue;
    }
  }
  poly_scanzone.polygon.points[poly_scanzone.polygon.points.size()-1].x = pos.x;
  poly_scanzone.polygon.points[poly_scanzone.polygon.points.size()-1].y = pos.x;
  poly_scanzone.polygon.points[poly_scanzone.polygon.points.size()-1].z = pos.z;
  poly_scanzone.polygon.points[0] =   poly_scanzone.polygon.points[poly_scanzone.polygon.points.size()-1];
  poly_scanzone.header = hdr();
  pub_poly_scanzone.publish(poly_scanzone);
}
nav_msgs::Path get_pathfinal(sensor_msgs::LaserScan scanin){
  nav_msgs::Path pathout;
  geometry_msgs::PointStamped pnt,pnt_out;
  pathout.header = hdr();
  geometry_msgs::TransformStamped transformStamped;
//  transformStamped = tfBuffer.lookupTransform("map", scanin.header.frame_id, scanin.header.stamp);
  pnt.header.frame_id = scanin.header.frame_id;
  for(int i = 0; i < scanin.ranges.size(); i++){
    if(std::isinf(scanin.ranges[i])){
    }
    else{
      float a = scanin.angle_min + scanin.angle_increment * i;
      pnt.point.x = scanin.ranges[i] * cos(a);
      pnt.point.y = scanin.ranges[i] * sin(a);
      pnt.header.stamp  = scanin.header.stamp;
      try{
          pnt_out = tfBuffer.transform(pnt, "map");
          geometry_msgs::PoseStamped ps;
          ps.header = hdr();
          ps.pose.position    = pnt_out.point;
          ps.pose.orientation = tf::createQuaternionMsgFromYaw(get_hdng(pnt_out.point,pos));
          pathout.poses.push_back(ps);
      }
      catch (tf2::TransformException &ex) {
            ROS_WARN("%s",ex.what());
            ros::Duration(1.0).sleep();
            continue;
      }
    ///  pnt_out = tfBuffer.transform("map",scanin.header.stamp,pnt,"map",pnt_out);

    }
  }
  return pathout;
}

void update_pos_vlp(){
  geometry_msgs::TransformStamped transformStamped;
  try{
    transformStamped = tfBuffer.lookupTransform("map","velodyne_aligned",
                             ros::Time(0));
  }
  catch (tf2::TransformException &ex) {
    ROS_WARN("%s",ex.what());
    ros::Duration(1.0).sleep();
  }
  pos.x = transformStamped.transform.translation.x;
  pos.y = transformStamped.transform.translation.y;
  pos.z = transformStamped.transform.translation.z;
  tf2::Matrix3x3 q(tf2::Quaternion(transformStamped.transform.rotation.x, transformStamped.transform.rotation.y, transformStamped.transform.rotation.z, transformStamped.transform.rotation.w));
  q.getRPY(vlp_rpy.x,vlp_rpy.y,vlp_rpy.z);
  vlp_rpy.y *= -1;
  pos_yaw = vlp_rpy.z;
}

void global_plan_cb(const nav_msgs::Path::ConstPtr& msg){
  path_global_plan = *msg;
}
void get_elevation_to_point_cb(const geometry_msgs::PointStamped::ConstPtr& msg){
	point_elevation_checked = *msg;
}
void costmap_elevation_cb(const std_msgs::UInt8::ConstPtr& msg){
	costmap_elevation_meters = msg->data;
}
void target_cb(const geometry_msgs::PointStamped::ConstPtr& msg){
	point_global_target = *msg;
}
void getplanfailed_cb(const std_msgs::Empty::ConstPtr& msg){
	path_global_plan.header.frame_id = "failed";
}
void get_down_cb(const geometry_msgs::PolygonStamped::ConstPtr& msg){
	get_down_poly = *msg;
}
void get_side_cb(const geometry_msgs::PolygonStamped::ConstPtr& msg){
	get_side_poly = *msg;
}
void get_poly_cb(const geometry_msgs::PolygonStamped::ConstPtr& msg){
	get_poly_poly = *msg;
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
nav_msgs::Path add_path(nav_msgs::Path path_base,nav_msgs::Path pathin,float cutoff){
	for(int i = 0; i < pathin.poses.size(); i++){
		if(dst_point_in_path_lim(path_base,pathin.poses[i].pose.position,cutoff))
			path_base.poses.push_back(pathin.poses[i]);
	}
	return path_base;
}
void path_down_cb(const nav_msgs::Path::ConstPtr& msg){
  path_down = add_path(path_down,*msg,1.0);
}
void path_side_cb(const nav_msgs::Path::ConstPtr& msg){
  path_side = add_path(path_side,*msg,1.0);
}
void path_base_cb(const nav_msgs::Path::ConstPtr& msg){
	path_base = *msg;
}
void path_cmd_cb(const nav_msgs::Path::ConstPtr& msg){
  path_cmd = *msg;
}
void vstd_cb(const nav_msgs::Path::ConstPtr& msg){
  path_visited = *msg;
}



void path_poly_cb(const geometry_msgs::PolygonStamped::ConstPtr& msg){
  path_poly = *msg;
}

void octomap_callback(const octomap_msgs::Octomap& msg){
  abs_octree=octomap_msgs::fullMsgToMap(msg);
  octree.reset(dynamic_cast<octomap::OcTree*>(abs_octree));
  got_map = true;
}

void odomglobal_cb(const nav_msgs::Odometry::ConstPtr& msg){
  if((ros::Time::now() - odom_global.header.stamp).toSec() > 0.2){
    odom_global = *msg;
  }
}

void scanvlp_cb(const sensor_msgs::LaserScan::ConstPtr& msg){
  path_scan = get_pathfinal(*msg);
  int half_size = msg->ranges.size()/2;
  int best_dst =1111;
  float best_rng = 1111;
  float best_ang = 0;
  for(int i = 0;i < msg->ranges.size(); i++){
    int dst_from_mid = abs(i - half_size);
    if(dst_from_mid < best_dst && msg->ranges[i] < best_dst*1.3){
      best_ang = msg->angle_min + msg->angle_increment * i;
      best_dst = dst_from_mid;
      best_rng = msg->ranges[i];
    }
  }
  geometry_msgs::PointStamped pnt;
  pnt.point.x = best_rng * cos(best_ang);
  pnt.point.y = best_rng * sin(best_ang);
  pnt.header.stamp   = ros::Time(0);
  pnt.header.frame_id = msg->header.frame_id;
  scanpoint_mid = tfBuffer.transform(pnt, "map");
//  create_scanzone_polygon(*msg);
  create_scan_polygon(*msg);

}
void scan_stab_cb(const sensor_msgs::LaserScan::ConstPtr& msg){
  if(cansee_proximity && (ros::Time::now() - poly_proximity.header.stamp).toSec() > 1.0){
    create_proximity_polygon(*msg);
  }
  //path_st = get_pathfinal(*msg);
}
int get_first_index_above_dst(nav_msgs::Path pathin,float mindst){
  for(int i = 0; i < pathin.poses.size(); i++){
    if(get_dst2d(pos,pathin.poses[i].pose.position) > mindst)
      return i;
  }
  return -1;
}
bool get_gp_elevbbpnt(float radlen_xy){
  gp_pnt.point.x = perfalt_pnt.point.x;
  gp_pnt.point.y = perfalt_pnt.point.y;
  if(path_global_plan.poses.size() > 0){
    nav_msgs::Path path_global_scanzone = constrain_path_bbpoly(path_global_plan,poly_scanzone);
    float dst_scanzone0 = 0;
    float dst_scanzoneN = 0;
    if(path_global_scanzone.poses.size() > 0){
      dst_scanzone0 = get_dst2d(path_global_scanzone.poses[0].pose.position,perfalt_pnt.point);
      dst_scanzoneN = get_dst2d(path_global_scanzone.poses[path_global_scanzone.poses.size()-1].pose.position,perfalt_pnt.point);
    }
    float max_dst_gp_pnt = par_maprad;
    if(dst_scanzone0 < par_maprad && dst_scanzoneN > par_maprad)
      max_dst_gp_pnt = par_maprad + radlen_xy*2;
    for(int i = 0; i < path_global_plan.poses.size(); i++){
      float dstpnt = get_dst2d(path_global_plan.poses[i].pose.position,perfalt_pnt.point);
      float z_grid = get_zmax_grid(path_global_plan.poses[i].pose.position,radlen_xy);
      if(dstpnt < max_dst_gp_pnt && z_grid > 0){
        gp_pnt.point.x = path_global_plan.poses[i].pose.position.x;
        gp_pnt.point.y = path_global_plan.poses[i].pose.position.y;
        gp_pnt.header = hdr();
      }
    }
  }
  if(gp_pnt.point.x == perfalt_pnt.point.x && gp_pnt.point.y == perfalt_pnt.point.y && gp_pnt.point.z == perfalt_pnt.point.z)
    return false;
  else
    return true;
}
geometry_msgs::PointStamped get_combined_zdata(float x, float y, float radlen_xy){
  geometry_msgs::PointStamped pnt;
  pnt.point.x = x;
  pnt.point.y = y;
  float z_grid = get_zmax_grid(pnt.point,radlen_xy);
  int a_grid   = get_area_coverage(pnt.point,radlen_xy);
//  ROS_INFO("Point %.0f %.0f - zmax: %.0f a_grid: %i",x,y,z_grid,a_grid);
  if(z_grid > 0 || a_grid > 0){
    pnt.header.frame_id = "got_grid";
    pnt.point.z = z_grid;
    float z_edto = find_zmin_clearing(pnt.point,radlen_xy,3,20,3);
    if(z_edto > 0){
      pnt.header.frame_id = "got_edto";
      pnt.point.z = z_edto;
    }
  }
  return pnt;
}
void update_gp(float radlen_xy){
  if(get_gp_elevbbpnt(radlen_xy)){
    gp_pnt        = get_combined_zdata(gp_pnt.point.x,gp_pnt.point.y,radlen_xy);
    state_gp_pnt  = gp_pnt.header.frame_id;
    gp_pnt.header = hdr();
  }
  else
    state_gp_pnt = "no_gp";
}

void update_pospnt(float radlen_xy){
  pos_pnt  = get_combined_zdata(pos.x,pos.y,radlen_xy);
  state_pos_pnt  = pos_pnt.header.frame_id;
  pos_pnt.header = hdr();
  pos_pnt.header.frame_id = "map";
}
void update_scanstate(){
  state_scanpoint = "target_perfalt";
  if(state_gp_pnt == "got_grid" && state_perfalt == "got_edto")
    state_scanpoint = "target_gp";
}
void update_scantarget(){
  scantarget_pnt.header = hdr();
  if(state_scanpoint == "target_gp")
    scantarget_pnt.point = gp_pnt.point;
  else
    scantarget_pnt.point = perfalt_pnt.point;
}
void pc2_cb(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
	if(msg->data.size() > 10 && msg->header.frame_id == "map"){
		sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x");
		sensor_msgs::PointCloud2ConstIterator<float> iter_y(*msg, "y");
		sensor_msgs::PointCloud2ConstIterator<float> iter_z(*msg, "z");
		for(int i = 0; iter_x != iter_x.end(); ++i, ++iter_x, ++iter_y, ++iter_z)
		{
			if(!std::isnan(*iter_x) && !std::isnan(*iter_y) && !std::isnan(*iter_z))
	    {
	      int r = y2r(*iter_y);
	      int c = x2c(*iter_x);
	      int z = fmax(*iter_z,1);
	      if(z > img_height.at<uchar>(r,c))
	        img_height.at<uchar>(r,c) = z;
	    }
		}
	}
}
nav_msgs::Path create_gridpath(geometry_msgs::Point midpoint,float area_sidelength,float radlen_xy){
  float centroid_sides = 2*radlen_xy;
  int num_grids = area_sidelength / centroid_sides;
  nav_msgs::Path pathout;
  pathout.header = hdr();
  ////ROS_INFO("Area side: %.2f radlen: %.2f num_grids: %i centroid_sides: %.2f",area_sidelength,radlen_xy,num_grids,centroid_sides);
  int i = 0;
  for (int y = 0; y < num_grids; y++)
  {
    for (int x = 0; x < num_grids; x++)
    {
     geometry_msgs::PoseStamped ps;
     ps.pose.position.x = midpoint.x + float(area_sidelength*-0.5 + x * centroid_sides+centroid_sides*0.5);
     ps.pose.position.y = midpoint.y + float(area_sidelength*-0.5 + y * centroid_sides+centroid_sides*0.5);
     ps.pose.position.z = get_zmax_grid(ps.pose.position,2.0);
     ps.pose.orientation.w = 1.0;
     ps.header = hdr();
     pathout.poses.push_back(ps);
    }
  }
  return pathout;
}
///********FRONTIER*************////////
///********FRONTIER*************////////
void create_frontier(int num_is){
  geometry_msgs::Point p0;
  sensor_msgs::LaserScan scan_old = scan_frontier;
  float rads_pr_i = 2*M_PI / num_is;
  path_frontier.header = hdr();
  path_frontier.poses.resize(num_is);
  scan_frontier.ranges.resize(0);
  scan_frontier.ranges.resize(num_is);
  scan_frontier.angle_min = -M_PI;
  scan_frontier.angle_max = M_PI;
  scan_frontier.angle_increment = rads_pr_i;
  scan_frontier.header = hdr();
  scan_frontier.range_min = 1.0;
  scan_frontier.range_max = 500;
	for(int i = 0; i < scan_frontier.ranges.size(); i++){
		if(scan_frontier.ranges[i] < 10.0)
		scan_frontier.ranges[i] = 10.0;
	}
  for(int i = 0; i < path_frontier.poses.size(); i++){
    int i_at_rad = round((get_hdng(path_frontier.poses[i].pose.position,p0) - scan_frontier.angle_min ) / scan_frontier.angle_increment);
    float dst = get_dst2d(path_frontier.poses[i].pose.position,p0);
    if(scan_frontier.ranges[i_at_rad] == 0 || scan_frontier.ranges[i_at_rad] > dst)
       scan_frontier.ranges[i_at_rad]  = dst;
  }
  for(int i = 0; i < scan_old.ranges.size(); i++){
    if(scan_frontier.ranges[i] < scan_old.ranges[i])
      scan_frontier.ranges[i] = scan_old.ranges[i];
  }
	path_frontier.poses.resize(0);
  for(int i = 0; i < num_is; i++){
    geometry_msgs::Point midpoint;
    float a = scan_frontier.angle_min + i * scan_frontier.angle_increment;
    path_frontier.poses[i].header = hdr();
    path_frontier.poses[i].pose.orientation = tf::createQuaternionMsgFromYaw(a);
    path_frontier.poses[i].pose.position.x = p0.x + scan_frontier.ranges[i] * cos(a);
    path_frontier.poses[i].pose.position.y = p0.y + scan_frontier.ranges[i] * sin(a);
    path_frontier.poses[i].pose.position.z = 0;
  }
}
std::vector<int> get_indexes_around(nav_msgs::Path pathin,int i_in,float cutoff){
  std::vector<int> indexes_around;
  for(int i=0;i<pathin.poses.size();i++){
    if(get_dst2d(pathin.poses[i].pose.position,pathin.poses[i_in].pose.position) < cutoff)
      indexes_around.push_back(i);
  }
  return indexes_around;
}
void find_frontier(int coverage_lo,int coverage_hi){
  //start_process("create_frontier");
  geometry_msgs::Point p0;
  float area_sidelength = 300;
  float radlen_xy = 5;
  nav_msgs::Path gridpath = create_gridpath(p0,area_sidelength,radlen_xy);
	path_frontier.poses.resize(0);
  std::vector<int> coverage;
  std::vector<std::vector<int>> indexes_around;
  std::vector<std::string> index_state;
  for(int i = 0; i < gridpath.poses.size(); i++){
    int coverage_percent = get_area_coverage(gridpath.poses[i].pose.position,radlen_xy);
    indexes_around.push_back(get_indexes_around(gridpath,i,radlen_xy*3));
    coverage.push_back(coverage_percent);
    std::string state;
    if(coverage_percent < coverage_lo)
      state = "unknown";
    else if(coverage_percent < coverage_hi)
      state = "frontier";
    else
      state = "known";
    index_state.push_back(state);
  }
  for(int i = 0; i < gridpath.poses.size(); i++){
    int frontier_count = 0; int unknown_count = 0; int known_count = 0;
    for(int k =0; k < indexes_around[i].size(); k++){
      if(index_state[indexes_around[i][k]] == "unknown")
        unknown_count++;
      if(index_state[indexes_around[i][k]] == "frontier")
        frontier_count++;
      if(index_state[indexes_around[i][k]] == "known")
        known_count++;
    }
      ///unknown_count < indexes_around[i].size()-2 &&
    if(unknown_count >= 2 && known_count >= 2){
      path_frontier.poses.push_back(gridpath.poses[i]);
    }
  }
  create_frontier(32);
//  start_process("");
}
void mainstate_cb(const std_msgs::UInt8::ConstPtr& msg){
  mainstate = msg->data;
	count_target_paths++;
	if(count_target_paths >= 5){
		count_target_paths = 0;
		find_frontier(65,95);
	}
	else if(path_frontier.poses.size() > 10){
		path_frontier.header = hdr();
		pub_path_frontier.publish(path_frontier);
	}
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "tb_behavior_node");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");
  private_nh.param("mapbb_radius",    par_maprad, 25.0);
  private_nh.param("elvbb_radius",    par_elevbb, 5.0);
  private_nh.param("z_clearing",      par_zclearing, 3.0);
  private_nh.param("mission_arearad", par_arearad, 75.0);
  private_nh.param("write_debug_img", par_debug_img, true);

  private_nh.getParam("workdir_path", par_workdir);//*2.0);
//img_height = cv::imread(par_workdir,CV_LOAD_IMAGE_COLOR);
  //  srand (time(NULL));
  //   5 + rand() % 25;

	tf2_ros::TransformListener tf2_listener(tfBuffer);
  pub_poly_proximity  = nh.advertise<geometry_msgs::PolygonStamped>("/tb_world/proximity_polygon",10);
  pub_poly_scanzone   = nh.advertise<geometry_msgs::PolygonStamped>("/tb_world/scanzone_polygon",10);
  ros::Subscriber s1  = nh.subscribe("/octomap_full",1,octomap_callback);
  ros::Subscriber s2 = nh.subscribe("/move_base/NavfnROS/plan",  100,&global_plan_cb);
  ros::Subscriber s3 = nh.subscribe("/odom_global",1,odomglobal_cb);
	ros::Subscriber s4 = nh.subscribe("/tb_world/path_visited",10,vstd_cb);
	ros::Subscriber s5 = nh.subscribe("/tb_path/path_to_interpolate",10,path_base_cb);
	ros::Subscriber s6 = nh.subscribe("/tb_cmd/path",10,path_cmd_cb);

	ros::Subscriber s7 = nh.subscribe("/tb_mbmap/get_elevation_to_point",10,get_elevation_to_point_cb);
	ros::Subscriber s8 = nh.subscribe("/tb_mbmap/costmap_elevation",10,costmap_elevation_cb);
	ros::Subscriber s9 = nh.subscribe("/tb_plan/target_point",10,target_cb);
//
	ros::Subscriber a1 = nh.subscribe("/tb_edto/get_down",10,get_down_cb);
	ros::Subscriber a2 = nh.subscribe("/tb_edto/get_side",10,get_side_cb);
	ros::Subscriber a3 = nh.subscribe("/tb_edto/get_poly",10,get_poly_cb);

	ros::Subscriber as4 = nh.subscribe("/tb_edto/down",10,path_down_cb);
	ros::Subscriber as5 = nh.subscribe("/tb_edto/side",10,path_side_cb);
	ros::Subscriber as6 = nh.subscribe("/tb_edto/poly",10,path_poly_cb);
  ros::Subscriber s01 = nh.subscribe("/tb_fsm/main_state",10,mainstate_cb);
	ros::Subscriber s11   = nh.subscribe("/assembled_cloud2",10,pc2_cb);
  ros::Subscriber s12 = nh.subscribe("/velodyne_scan",10,scanvlp_cb);
  ros::Subscriber s13  = nh.subscribe("/scan_stabilized",10,scan_stab_cb);

 	pub_path_frontier = nh.advertise<nav_msgs::Path>("/tb_world/path_frontier",10);

  ros::Publisher pub_pnt_gp      = nh.advertise<geometry_msgs::PointStamped>("/tb_nav/gp",10);
  ros::Publisher pub_pnt_scan    = nh.advertise<geometry_msgs::PointStamped>("/tb_nav/scanpoint_mid",10);
  ros::Publisher pub_pnt_scantar = nh.advertise<geometry_msgs::PointStamped>("/tb_nav/scanptarget",10);

  path_starsquare = create_starsquare(par_arearad);
  targetindexes_sent.push_back(0);
  ros::Rate rate(5.0);
  float radlen_xy = 10;
  while(ros::ok()){
    update_pos_vlp();
    update_gp(radlen_xy);
    update_pospnt(radlen_xy);
    update_scanstate();
    update_scantarget();

    pub_pnt_scan.publish(scanpoint_mid);
    pub_pnt_scantar.publish(scantarget_pnt);
    pub_pnt_gp.publish(gp_pnt);

    drawimg();

    rate.sleep();
    ros::spinOnce();
  }
  return 0;
}
//
