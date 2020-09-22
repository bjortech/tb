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
std::string par_workdir = "/home/nuc/brain/pathfull/heights0.png";
cv::Mat img(1000,1000,CV_8UC3,cv::Scalar(0, 0, 0)); //create image, set encoding and size, init pixels to default val
cv::Mat img_blank(1000,1000,CV_8UC3,cv::Scalar(0, 0, 0)); //create image, set encoding and size, init pixels to default val
cv::Mat img_height(1000,1000,CV_8UC3,cv::Scalar(0, 0, 0)); //create image, set encoding and size, init pixels to default val
cv::Mat img_update(1000,1000,CV_8UC3,cv::Scalar(0, 0, 0)); //create image, set encoding and size, init pixels to default val
cv::Mat img_new(1000,1000,CV_8UC3,cv::Scalar(0, 0, 0)); //create image, set encoding and size, init pixels to default val
cv::Mat img_update_copy(1000,1000,CV_8UC3,cv::Scalar(0, 0, 0)); //create image, set encoding and size, init pixels to default val

cv::Mat img_height_m(1000,1000,CV_8U,cv::Scalar(0)); //create image, set encoding and size, init pixels to default val
cv::Mat img_height_mi(1000,1000,CV_8U,cv::Scalar(0)); //create image, set encoding and size, init pixels to default val
cv::Mat slopeimage_mono(1000,1000,CV_8U,cv::Scalar(0)); //create image, set encoding and size, init pixels to default val
cv::Mat slopeimage_mono_mid(1000,1000,CV_8U,cv::Scalar(0)); //create image, set encoding and size, init pixels to default val
cv::Mat slopeimage_mono_mid_copy(1000,1000,CV_8U,cv::Scalar(0)); //create image, set encoding and size, init pixels to default val
cv::Mat slopeimage_mono_copy(1000,1000,CV_8U,cv::Scalar(0)); //create image, set encoding and size, init pixels to default val

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
ros::Publisher pub_poly_scanzone,pub_path_global,pub_poly_proximity,pub_path_elevated,pub_winner,pub_perfectalt_elevation,pub_map_updates,pub_setpoint,pub_mbtarget,pub_setodom;
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
geometry_msgs::PoseStamped target;



std_msgs::Header hdr(){
  std_msgs::Header header;
  header.frame_id = "map";
  header.stamp    = ros::Time::now();
  return header;
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

std::vector<float> get_vec_attribute(nav_msgs::Path pathin,std::string type){
  std::vector<float> vec_out;
  geometry_msgs::Point p0;
  for(int i = 0; i < pathin.poses.size(); i++){
    float val = 0;
    if(type == "dst_2d0")
      val = get_dst2d(p0,pathin.poses[i].pose.position);
    else if(type == "inclination")
      val = get_inclination(pathin.poses[i].pose.position,pos);
    else if(type == "hdng_abs")
   		val = abs(get_shortest(get_hdng(pathin.poses[i].pose.position,pos),vlp_rpy.z) * rad2deg);
    else if(type == "dst_2d")
      val = get_dst2d(pos,pathin.poses[i].pose.position);
    else if(type == "dst_3d")
      val = get_dst3d(pos,pathin.poses[i].pose.position);
    else if(type == "z")
      val = pathin.poses[i].pose.position.z;
    else if(type == "zrel")
      val = pathin.poses[i].pose.position.z - pos.z;
       vec_out.push_back(val);
  }
  return vec_out;
}
nav_msgs::Path sort_path(nav_msgs::Path pathin,std::string sort_by){
  nav_msgs::Path pathout;
  if(pathin.poses.size() <= 1)
    return pathin;
  pathout.header = pathin.header;
  std::vector<std::tuple<int,float>>i_dst;
  for(int i = 0; i < pathin.poses.size(); i++){
		if(sort_by == "dst_2d")
      i_dst.push_back(std::make_tuple(i,get_dst2d(pos,pathin.poses[i].pose.position)));
    else if(sort_by == "inclination")
      i_dst.push_back(std::make_tuple(i,get_inclination(pathin.poses[i].pose.position,pos)));
    else if(sort_by == "dst_3d")
      i_dst.push_back(std::make_tuple(i,get_dst3d(pos,pathin.poses[i].pose.position)));
		else if(sort_by == "hdng_abs")
			i_dst.push_back(std::make_tuple(i,abs(get_shortest(get_hdng(pathin.poses[i].pose.position,pos),vlp_rpy.z) * rad2deg)));
    else if(sort_by == "hdng")
      i_dst.push_back(std::make_tuple(i,get_shortest(get_hdng(pathin.poses[i].pose.position,pos),vlp_rpy.z)));
    else if(sort_by == "z")
      i_dst.push_back(std::make_tuple(i,pathin.poses[i].pose.position.z));
	}
  sort(i_dst.begin(),i_dst.end(),sort_dst_pair);
  for(int i = 0; i < i_dst.size(); i++){
    pathout.poses.push_back(pathin.poses[std::get<0>(i_dst[i])]);
  }
	return pathout;
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

//******************************************************************************************************************//
//***************************FRONTIER END******************************************//
//***************************HEIGHTPATHS START******************************************//
//*******************************bb***********************************************************************************//
float get_zmax(nav_msgs::Path pathin){
  float zmx = 0;
  for(int i = 0; i < pathin.poses.size(); i++){
    if(pathin.poses[i].pose.position.z > zmx){
      zmx = pathin.poses[i].pose.position.z;
    }
  }
  return zmx;
}
geometry_msgs::PolygonStamped create_linepoly(geometry_msgs::Point p0, float line_length,float line_heading,float poly_width){
  geometry_msgs::PolygonStamped poly;
  poly.polygon.points.resize(5);
  poly.header = hdr();
  geometry_msgs::Point p1;
  p1.x = p0.x + line_length * cos(line_heading);
  p1.y = p0.y + line_length * sin(line_heading);

  poly.polygon.points[0].x = p0.x + poly_width/2*cos(line_heading+M_PI/2);
  poly.polygon.points[0].y = p0.y + poly_width/2*sin(line_heading+M_PI/2);
  poly.polygon.points[1].x = p0.x + poly_width/2*cos(line_heading-M_PI/2);
  poly.polygon.points[1].y = p0.y + poly_width/2*sin(line_heading-M_PI/2);
  poly.polygon.points[2].x = p1.x + poly_width/2*cos(line_heading-M_PI/2);
  poly.polygon.points[2].y = p1.y + poly_width/2*sin(line_heading-M_PI/2);
  poly.polygon.points[3].x = p1.x + poly_width/2*cos(line_heading+M_PI/2);
  poly.polygon.points[3].y = p1.y + poly_width/2*sin(line_heading+M_PI/2);
  poly.polygon.points[4] = poly.polygon.points[0];
 return poly;
}

nav_msgs::Path create_linepath(geometry_msgs::Point p0, float line_length,float line_heading,float interval_meters){
  nav_msgs::Path pathout;
  pathout.header = hdr();
  int num_intervals = line_length / interval_meters;
  for(int i = 0; i < num_intervals+1; i++){
    geometry_msgs::PoseStamped ps;
    ps.header = hdr();
    ps.pose.position.x = p0.x + interval_meters * i * cos(line_heading);
    ps.pose.position.y = p0.y + interval_meters * i * sin(line_heading);
    ps.pose.orientation = tf::createQuaternionMsgFromYaw(line_heading);
    pathout.poses.push_back(ps);
  }
 return pathout;
}
geometry_msgs::Point get_linepoly_p0(geometry_msgs::PolygonStamped polyin){
  geometry_msgs::Point start;
  if(polyin.polygon.points.size() > 2){
    start.x = (polyin.polygon.points[0].x + polyin.polygon.points[1].x)/2;
    start.y = (polyin.polygon.points[0].y + polyin.polygon.points[1].y)/2;
  }
  else{
    ROS_INFO("LINEPOLY P0 ERROR");
  }
  return start;
}
geometry_msgs::Point get_linepoly_p1(geometry_msgs::PolygonStamped polyin){
  geometry_msgs::Point end;
  if(polyin.polygon.points.size() > 3){
    end.x   = (polyin.polygon.points[2].x + polyin.polygon.points[3].x)/2;
    end.y   = (polyin.polygon.points[2].y + polyin.polygon.points[3].y)/2;
  }
  else{
    ROS_INFO("LINEPOLY P1 ERROR");
  }
  return end;
}

float get_linepoly_heading(geometry_msgs::PolygonStamped polyin){
  geometry_msgs::Point end,start;
  start = get_linepoly_p0(polyin);
  end   = get_linepoly_p1(polyin);
  return get_hdng(end,start);
}
float get_linepoly_length(geometry_msgs::PolygonStamped polyin){
  geometry_msgs::Point end,start;
  start = get_linepoly_p0(polyin);
  end   = get_linepoly_p1(polyin);
  return get_dst2d(end,start);
}

std::vector<nav_msgs::Path> segment_paths_with_linepolys(nav_msgs::Path pathin,std::vector<geometry_msgs::PolygonStamped> linepolys){
  std::vector<nav_msgs::Path> pathsout;
  pathsout.resize(linepolys.size());
  for(int i = 0; i < linepolys.size(); i++){
    pathsout[i] = constrain_path_bbpoly(pathin,linepolys[i]);
  }
  return pathsout;
}
std::vector<geometry_msgs::PolygonStamped> create_linepolys(geometry_msgs::Point p0,float a0,float aN,float len_polys,float width_polys,int num_is){
  std::vector<geometry_msgs::PolygonStamped> linepolys;
  linepolys.resize(num_is);
  float rads_pr_i = (aN-a0)/num_is;
  for(int i = 0; i < num_is; i++){
    float a_i    = constrainAngle(a0 + rads_pr_i * i);
    linepolys[i] = create_linepoly(p0,len_polys,a_i,width_polys);
  }
  return linepolys;
}
std::vector<float> create_angles(float a0,float aN,int num_is){
  std::vector<float> angles;
  angles.resize(num_is);
  float rads_pr_i = (aN-a0)/num_is;
  for(int i = 0; i < num_is; i++){
    float a_i    = constrainAngle(a0 + rads_pr_i * i);
    angles[i] = a_i;
  }
  return angles;
}
std::vector<nav_msgs::Path> create_linepaths(geometry_msgs::Point p0,float a0,float aN,float len_polys,float interval_meters,int num_is,bool get_shortest_angle){
  std::vector<nav_msgs::Path> linepaths;
  linepaths.resize(num_is);
  float rads_pr_i = (aN-a0)/num_is;
  if(get_shortest_angle)
    rads_pr_i = get_shortest(aN,a0) / num_is;
  for(int i = 0; i < num_is; i++){
    float a_i    = constrainAngle(a0 + rads_pr_i * i);
    linepaths[i] = create_linepath(p0,len_polys,a_i,interval_meters);
  }
  return linepaths;
}
std::vector<std::vector<int>> get_intervals(std::vector<float> vec_dst2d,float m_pr_interval){
  std::vector<std::vector<int>> is_in_intervals;

  //ROS_INFO("dst0 -> N : %i dsts %.0f %.0f mprint: %.0f",vec_dst2d.size(),vec_dst2d[0],vec_dst2d[vec_dst2d.size()-1],m_pr_interval);
  int intervals_d2d = 1+(vec_dst2d[vec_dst2d.size()-1]) / m_pr_interval;

  is_in_intervals.resize(intervals_d2d);
  for(int i = 0; i < vec_dst2d.size();i++){
    int interval = vec_dst2d[i] / m_pr_interval;
    is_in_intervals[interval].push_back(i);
  }
  return is_in_intervals;
}

std::vector<nav_msgs::Path> get_path_intervals(nav_msgs::Path pathin,std::vector<std::vector<int>> intervals){
  std::vector<nav_msgs::Path> path_intervals;
  path_intervals.resize(intervals.size());
  for(int k = 0; k < intervals.size(); k++){
    for(int i = 0; i < intervals[k].size(); i++){
      path_intervals[k].poses.push_back(pathin.poses[intervals[k][i]]);
    }
  }
  return path_intervals;
}
tb_msgsrv::Paths remove_empty_intervals(std::vector<nav_msgs::Path> pathsin){
  tb_msgsrv::Paths pathsout;
  for(int k = 0; k < pathsin.size(); k++){
    nav_msgs::Path pathout;
    pathout.header = hdr();
    for(int i = 0; i < pathsin[k].poses.size(); i++){
      if(pathsin[k].poses[i].pose.position.z > 0)
        pathout.poses.push_back(pathsin[k].poses[i]);
    }
    if(pathout.poses.size() > 0)
      pathsout.paths.push_back(pathout);
  }
  return pathsout;
}

std::vector<nav_msgs::Path> create_heightpaths(nav_msgs::Path pathin,int num_polys,float len_polys,float width_polys,float a1,float a2){
//  ROS_INFO("pathin: %i m_pr_interval: %.2f num_polys %i len_polys: %.2f width_polys: %.2f a1: %.2f,a2: %.2f",pathin.poses.size(),m_pr_interval,num_polys,len_polys,width_polys,a1,a2);

  std::vector<geometry_msgs::PolygonStamped> linepolys  = create_linepolys(pos,a1,a2,len_polys,width_polys,num_polys);
  std::vector<nav_msgs::Path>            path_segments  = segment_paths_with_linepolys(pathin,linepolys);
  ROS_INFO("CREATED path_segments: %i",path_segments.size());

  for(int i = 0; i < path_segments.size(); i++){
    path_segments[i]  = sort_path(path_segments[i],"dst_2d");
  }
 return path_segments;
}

//******************************************************************************************************************//
//***************************HEIGHTPATHS END******************************************//
//***************************FULLPATH START******************************************//
//******************************************************************************************************************//
cv::Scalar get_random_color(){
  int rg = rand() % 255;
  int rb = rand() % 255;
  int rr = rand() % 255;

  return get_color(rg,rb,rr);
}
nav_msgs::Path cutoff_abs(nav_msgs::Path pathin,std::string type,float val_0,float val_N){
  if(pathin.poses.size() < 3)
    return pathin;
  nav_msgs::Path pathout;
  pathout.header = hdr();
  if(type == "hdng_rel"){
    for(int i = 0; i < pathin.poses.size(); i++){
      float hdng_rel = get_shortest(get_hdng(pathin.poses[i].pose.position,pos),vlp_rpy.z);
      if(hdng_rel >= val_0 && hdng_rel <= val_N)
        pathout.poses.push_back(pathin.poses[i]);
    }
  }
  if(type == "hdng_back"){
    for(int i = 0; i < pathin.poses.size(); i++){
      float hdng_rel = get_shortest(get_hdng(pathin.poses[i].pose.position,pos),vlp_rpy.z);
      if(hdng_rel > M_PI/2 || hdng_rel < -M_PI/2)
        pathout.poses.push_back(pathin.poses[i]);
    }
  }
  if(type == "z"){
    for(int i = 0; i < pathin.poses.size(); i++){
      if(val_0 >= pathin.poses[i].pose.position.z && pathin.poses[i].pose.position.z <= val_N){
        pathout.poses.push_back(pathin.poses[i]);
      }
    }
  }
  if(type == "dst"){
    for(int i = 0; i < pathin.poses.size(); i++){
      float dst = get_dst2d(pathin.poses[i].pose.position,pos);
      if(dst >= val_0 && dst <= val_N){
        pathout.poses.push_back(pathin.poses[i]);
      }
    }
  }
  return pathout;
}
std::vector<float> analyze_path(nav_msgs::Path pathin){
  std::vector<float> vals;
  int num_z0 = 0;
  float zmax = 0;
  float closest_zpos = 1000;
  float farthest = 0;
  for(int i = 0; i < pathin.poses.size(); i++){
    float z = pathin.poses[i].pose.position.z;
    float dst = get_dst2d(pathin.poses[i].pose.position,pos);
    if(z >= pos.z && dst < closest_zpos)
      closest_zpos = dst;
    else if(closest_zpos == 1000 && dst > farthest)
      farthest = dst;
    if(z == 0)
      num_z0++;
    if(z > zmax)
      zmax = z;
  }
  vals.push_back(zmax);
  vals.push_back(farthest);
  vals.push_back(float(num_z0));
  vals.push_back(closest_zpos);
  return vals;
}

void drawimg(nav_msgs::Path gridpath){
  img_blank.copyTo(img);
  nav_msgs::Path gp,gp_0;
  float grid_s;
  float grid_s1 = gridpath.poses[1].pose.position.x - gridpath.poses[0].pose.position.x;
  float grid_s2 = gridpath.poses[1].pose.position.y - gridpath.poses[0].pose.position.y;
  if(grid_s1 > 0)
    grid_s = grid_s1;
  else
    grid_s = grid_s2;
  srand (time(NULL));

  for(int i = 0; i < gridpath.poses.size(); i++){
    if(gridpath.poses[i].pose.position.z == 0)
      gp_0.poses.push_back(gridpath.poses[i]);
    else
      gp.poses.push_back(gridpath.poses[i]);
  }
  bool use_old = false;
  if(use_old){
    int num_is = 16;
    std::vector<nav_msgs::Path> heightpaths = create_heightpaths(gridpath,num_is,80.0,15.0,-M_PI,M_PI);
    std::vector<float> angles = create_angles(-M_PI,M_PI,num_is);

    std::vector<float> h_l;
    std::vector<float> h_z;
    std::vector<float> h_n;
    std::vector<float> h_a;

    h_l.resize(heightpaths.size());
    h_z.resize(heightpaths.size());
    h_n.resize(heightpaths.size());
    for(int i = 0; i < heightpaths.size(); i++){
      h_a.push_back(abs(rad2deg*get_shortest(angles[i],vlp_rpy.z)));
      bool ended = false;
      int num = 0;
      for(int k = 0; k < heightpaths[i].poses.size(); k++){
        if(!ended){
          float z = heightpaths[i].poses[k].pose.position.z;
          if(z >= pos.z){
            ended = true;
            ROS_INFO("Heightpaths[%i] - len: %.0f zmax: %.0f num0: %i",i,h_l[i],h_z[i],h_n[i]);
          }
          else if(z == 0)
            num++;
          else{
            if(z > h_z[i])
              h_z[i] = z;
            h_l[i] = get_dst2d(pos,heightpaths[i].poses[k].pose.position);
          }
        }
      }
      h_n[i] = float(num);
    }
    std::vector<float> l_mma = vec_to_min_max_ave(h_l);
    std::vector<float> z_mma = vec_to_min_max_ave(h_z);
    std::vector<float> n_mma = vec_to_min_max_ave(h_n);
    std::vector<float> a_mma = vec_to_min_max_ave(h_a);
    float l_range = l_mma[1]-l_mma[0];
    float z_range = z_mma[1]-z_mma[0];
    float n_range = n_mma[1]-n_mma[0];
    float a_range = a_mma[1]-a_mma[0];
    float z_w = -1.0;
    float n_w = 1.0;
    float l_w = 1.0;
    float a_w = -1.0;
    std::vector<float> scores;
    scores.resize(heightpaths.size());
    float a_rel,l_rel,z_rel,n_rel;
    for(int i = 0; i < heightpaths.size(); i++){
        a_rel = (h_a[i] - a_mma[0])/a_range;
      if(l_range > 0)
        l_rel = (h_l[i] - l_mma[0])/l_range;
      if(z_range > 0)
        z_rel = (h_z[i] - z_mma[0])/z_range;
      if(n_range > 0)
        n_rel = (h_n[i] - n_mma[0])/n_range;
      scores[i] = l_rel * l_w + z_rel * z_w + n_rel * n_w * a_rel * a_w;
    }
    std::vector<float> s_mma = vec_to_min_max_ave(scores);
    float s_range = s_mma[1]-s_mma[0];
    nav_msgs::Path path_closest = cutoff_abs(gridpath,"dst",0,12);
    bool got_target = false;
    for(int i = 0; i < heightpaths.size(); i++){
      float rel_score = (scores[i]-s_mma[0])/s_range;
      ROS_INFO("a: %.2f l: %.0f z: %.0f n: %.0f,score: %.2f",h_a[i],h_l[i],h_z[i],h_n[i],scores[i]);
      if(rel_score > 0.9){
        got_target = true;
        target.pose.position.z  = fmax(h_z[i],get_zmax(path_closest));
        float hdng = get_hdng(heightpaths[i].poses[1].pose.position,target.pose.position);
        float dhdng = get_shortest(hdng,tf::getYaw(target.pose.orientation));
        if(dhdng < 0)
          dhdng *= -1;
        if(dhdng < M_PI/2)
          target.pose.position = heightpaths[i].poses[heightpaths[i].poses.size()-1].pose.position;
        target.pose.orientation = tf::createQuaternionMsgFromYaw(dhdng);
        draw_path(heightpaths[i],get_color(0,200,0),2);
      }
      if(rel_score < 0.1){
        draw_path(heightpaths[i],get_color(0,0,200),2);
      }
    }
  }

  nav_msgs::Path pf = cutoff_abs(gridpath,"hdng_rel",-M_PI/5,M_PI/5);
  nav_msgs::Path pr = cutoff_abs(gridpath,"hdng_rel",-M_PI/2,-M_PI/6);
  nav_msgs::Path pl = cutoff_abs(gridpath,"hdng_rel",M_PI/6,M_PI/2);
  nav_msgs::Path pb = cutoff_abs(gridpath,"hdng_back",0,0);
  nav_msgs::Path pc = cutoff_abs(gridpath,"dst",0,12);

  nav_msgs::Path vstd = cutoff_abs(path_visited,"dst",0,50);
  nav_msgs::Path pfv = cutoff_abs(vstd,"hdng_rel",-M_PI/6,M_PI/6);
  nav_msgs::Path prv = cutoff_abs(vstd,"hdng_rel",-M_PI/2,-M_PI/6);
  nav_msgs::Path plv = cutoff_abs(vstd,"hdng_rel",M_PI/6,M_PI/2);
  nav_msgs::Path pbv = cutoff_abs(vstd,"hdng_back",0,0);
  int vstd_f = pfv.poses.size();
  int vstd_r = prv.poses.size();
  int vstd_l = plv.poses.size();
  int vstd_b = pbv.poses.size();
  std::vector<float> pf_v = analyze_path(pf);
  std::vector<float> pr_v = analyze_path(pr);
  std::vector<float> pl_v = analyze_path(pl);
  std::vector<float> pb_v = analyze_path(pb);
  std::vector<float> pc_v = analyze_path(pc);

  ROS_INFO("Zmax0: f: %.0f r: %.0f l: %.0f b: %.0f c: %.0f",pf_v[0],pr_v[0],pl_v[0],pb_v[0],pc_v[0]);
  ROS_INFO("DstN:  f: %.0f r: %.0f l: %.0f b: %.0f c: %.0f",pf_v[1],pr_v[1],pl_v[1],pb_v[1],pc_v[1]);
  ROS_INFO("Numz0: f: %.0f r: %.0f l: %.0f b: %.0f c: %.0f",pf_v[2],pr_v[2],pl_v[2],pb_v[2],pc_v[2]);
  ROS_INFO("DstZ:  f: %.0f r: %.0f l: %.0f b: %.0f c: %.0f",pf_v[3],pr_v[3],pl_v[3],pb_v[3],pc_v[3]);

  draw_path(pr,get_color(200,200,0),2);
  draw_path(pl,get_color(0,200,200),2);
  draw_path(pf,get_color(0,0,200),1);
  draw_path(pb,get_color(0,200,200),2);

  int percent_0 = int(round(100.0 * float(gp_0.poses.size()) / float(gridpath.poses.size())));
  int percent   =  int(round(100.0 * float(gp.poses.size()) / float(gridpath.poses.size())));
  ROS_INFO("grid sidelength: %.0f, 0/z: %i / %i percent_0: %i / %i (total: %i)",grid_s,gp_0.poses.size(),gp.poses.size(),percent_0,percent,gridpath.poses.size());

  cv::Scalar color_zero = get_color(130,55,255);
  cv::Scalar color_gp   = get_color(45,95,55);
  cv::Scalar color      = get_color(245,95,55);

  //draw_path(gp_0,color_zero,2);
  //draw_path(gp,color_gp,1);

  std::vector<geometry_msgs::Point> bbmnmx = getinpath_boundingbox(gridpath);
  geometry_msgs::Point bbmin_scan = bbmnmx[0];
  geometry_msgs::Point bbmax_scan = bbmnmx[1];
  cv::rectangle(img, pnt2cv(bbmin_scan),pnt2cv(bbmax_scan),color_gp,1,8,0);
  cv::circle(img,pnt2cv(pos),2,color,1);
  count_target_paths++;
  geometry_msgs::Point pyaw,pyaw_mn,pyaw_mx;
  float a1 = constrainAngle(vlp_rpy.z + M_PI/3);
  float a2 = constrainAngle(vlp_rpy.z - M_PI/3);
  float a0 = fmin(a1,a2);
  float aN = fmax(a1,a2);
  draw_line(pos,a0,100,get_color(200,200,200));
  draw_line(pos,a1,100,get_color(200,200,200));
  draw_line(pos,vlp_rpy.z,100,get_color(100,100,0));
  cv::Mat img_new2;

  cv::resize(img, img_new2, cv::Size(), 2.0, 2.0);
  count_target_paths++;
  cv::imwrite("/home/nuc/brain/fullpath/"+std::to_string(count_target_paths)+"navigator.png",img_new2);
  //if(!got_target){
  //  target.pose.position.z  = get_zmax(path_closest);
  //}
  pub_setpoint.publish(target);
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
  target.pose.position = pos;
  target.pose.orientation.w = 1.0;
  target.header = transformStamped.header;
  tf2::Matrix3x3 q(tf2::Quaternion(transformStamped.transform.rotation.x, transformStamped.transform.rotation.y, transformStamped.transform.rotation.z, transformStamped.transform.rotation.w));
  q.getRPY(vlp_rpy.x,vlp_rpy.y,vlp_rpy.z);
  vlp_rpy.y *= -1;
}
void gridpath_cb(const nav_msgs::Path::ConstPtr& msg){
  update_pos_vlp();
  drawimg(*msg);
}

void vstd_cb(const nav_msgs::Path::ConstPtr& msg){
  path_visited = *msg;
}
int main(int argc, char** argv)
{
  ros::init(argc, argv, "tb_gridpath_node");
  ros::NodeHandle nh;
  tf2_ros::TransformListener tf2_listener(tfBuffer);

  ros::Subscriber s3   = nh.subscribe("/tb_env/gridpath",1,gridpath_cb);
 ros::Subscriber as2 = nh.subscribe("/tb_world/path_visited",10,vstd_cb);
 pub_setpoint      = nh.advertise<geometry_msgs::PoseStamped>("/tb_nav/setpoint_recom",10);


  ros::spin();

  return 0;
}
//
