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
cv::Mat img_blank(1000,1000,CV_8UC3,cv::Scalar(0, 0, 0)); //create image, set encoding and size, init pixels to default val
cv::Mat img_height(1000,1000,CV_8U,cv::Scalar(0)); //create image, set encoding and size, init pixels to default val
///////////********CTRL***********//////////
ros::Time process_start;
std::string active_process;
///********FRONTIER*************////////
sensor_msgs::LaserScan scan_frontier;
nav_msgs::Path path_frontier,path_unknown,path_known;
int lead_num = 1;
bool target_active = false;
ros::Time motionstate_change,mbstate_change,start;
geometry_msgs::PoseStamped exploration_start;
int last_frontier_i = 0;
int z_highest = 0;
int mainstate = 0;
double par_res;
int count_target_paths= 0;
///********FRONTIER*************////////

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
int get_area_coverage(geometry_msgs::Point midpoint, int radlen_xy){
  int c1 = x2c(midpoint.x-radlen_xy);
  int c0 = x2c(midpoint.x+radlen_xy);
  int r1 = y2r(midpoint.y-radlen_xy);
  int r0 = y2r(midpoint.y+radlen_xy);
  int pnts = 0; int pnts_tot = 0;
  for(int c = fmin(c1,c0); c < fmax(c1,c0); c++){
    for(int r = fmin(r1,r0); r < fmax(r1,r0); r++){
      pnts_tot++;
      if(img_height.at<uchar>(r,c) > 0)
        pnts++;
    }
  }
  int coverage = 100 * pnts / (pnts_tot+1);
  return coverage;
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
				if(z > z_highest)
					z_highest = z;
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
     ps.pose.orientation.w = 1.0;
     ps.header = hdr();
     pathout.poses.push_back(ps);
    }
  }
  return pathout;
}
std::vector<int> get_indexes_around(nav_msgs::Path pathin,int i_in,float cutoff){
  std::vector<int> indexes_around;
  for(int i=0;i<pathin.poses.size();i++){
    if(get_dst2d(pathin.poses[i].pose.position,pathin.poses[i_in].pose.position) < cutoff)
      indexes_around.push_back(i);
  }
  return indexes_around;
}
void create_frontier(nav_msgs::Path pathin,int num_is){
  geometry_msgs::Point p0;
  sensor_msgs::LaserScan scan_old = scan_frontier;
  float rads_pr_i = 2*M_PI / num_is;
  path_frontier.header = hdr();
	path_unknown.header = hdr();
	path_known.header = hdr();
  path_frontier.poses.resize(num_is);
  scan_frontier.ranges.resize(0);
  scan_frontier.ranges.resize(num_is);
  scan_frontier.angle_min = -M_PI;
  scan_frontier.angle_max = M_PI;
  scan_frontier.angle_increment = rads_pr_i;
  scan_frontier.header = hdr();
  scan_frontier.range_min = 1.0;
  scan_frontier.range_max = 500;


  for(int i = 0; i < pathin.poses.size(); i++){
    int i_at_rad = round((get_hdng(pathin.poses[i].pose.position,p0) - scan_frontier.angle_min ) / scan_frontier.angle_increment);
    float dst = get_dst2d(pathin.poses[i].pose.position,p0);
    if(scan_frontier.ranges[i_at_rad] == 0 || scan_frontier.ranges[i_at_rad] > dst)
       scan_frontier.ranges[i_at_rad]  = dst;
  }
  for(int i = 0; i < scan_old.ranges.size(); i++){
    if(scan_frontier.ranges[i] < scan_old.ranges[i])
      scan_frontier.ranges[i] = scan_old.ranges[i];
  }
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

void draw_alt_img(){
	for(int r = 0; r < img_height.rows;r++){
		for(int c = 0; c < img_height.cols;c++){
			img.at<cv::Vec3b>(r,c)[0] = 255*img_height.at<uchar>(r,c) / fmax(z_highest,10);
		}
	}
}

void find_frontier(int coverage_lo,int coverage_hi){
  start_process("create_frontier");
  geometry_msgs::Point p0;
  float area_sidelength = 300;
  float radlen_xy = 5;
	path_frontier.poses.resize(0);
	path_unknown.poses.resize(0);
	path_known.poses.resize(0);
	nav_msgs::Path path_frontier_temp;
  img_blank.copyTo(img);
	draw_alt_img();
  nav_msgs::Path gridpath = create_gridpath(p0,area_sidelength,radlen_xy);
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
			cv::Scalar color;
		if(state == "unknown")
			color[0] = coverage_percent;
		if(state == "frontier")
			color[2] = coverage_percent;
		if(state == "known")
			color[1] = 100;
    index_state.push_back(state);
		cv::circle(img,pnt2cv(gridpath.poses[i].pose.position),2,color,1);
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
      path_frontier_temp.poses.push_back(gridpath.poses[i]);
    }
		else if(unknown_count > 5){
			path_unknown.poses.push_back(gridpath.poses[i]);
		}
		else if(known_count >= 4){
			path_known.poses.push_back(gridpath.poses[i]);
		}
  }
  create_frontier(path_frontier_temp,32);
  start_process("");
}
///********FRONTIER*************////////
///********FRONTIER*************////////
void draw_height_atstart(int radlen_xy,int zval){
  for(int x = -radlen_xy; x < radlen_xy; x++){
    for(int y = -radlen_xy; y < radlen_xy; y++){
      img_height.at<uchar>(y2r(y),x2c(x)) = zval;
    }
  }
}
void mainstate_cb(const std_msgs::UInt8::ConstPtr& msg){
  mainstate = msg->data;
}
int main(int argc, char** argv)
{
  ros::init(argc, argv, "tb_abmap_frontier_node");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");
	private_nh.param("resolution", par_res, 1.0);

  ros::Subscriber s01 = nh.subscribe("/tb_fsm/main_state",10,mainstate_cb);
	ros::Subscriber s1   = nh.subscribe("/assembled_cloud2",10,pc2_cb);

  ros::Publisher pub_scan_frontier = nh.advertise<sensor_msgs::LaserScan>("/tb_world/scan_frontier",10);
  ros::Publisher pub_path_frontier = nh.advertise<nav_msgs::Path>("/tb_world/path_frontier",10);
	ros::Publisher pub_path_unknown = nh.advertise<nav_msgs::Path>("/tb_world/path_unknown",10);
	ros::Publisher pub_path_known = nh.advertise<nav_msgs::Path>("/tb_world/path_known",10);
	draw_height_atstart(5,1);
  ros::Rate rate(2.0);
  ros::Time last_radialupdate = ros::Time::now();
  while(ros::ok()){
    rate.sleep();
    ros::spinOnce();
    if(mainstate == 1 && (ros::Time::now() - last_radialupdate).toSec() > 0.75){
      last_radialupdate = ros::Time::now();
      find_frontier(15,55);
			pub_path_frontier.publish(path_frontier);
			pub_scan_frontier.publish(scan_frontier);
			pub_path_unknown.publish(path_unknown);
			pub_path_known.publish(path_known);
			for(int i = 1; i < path_frontier.poses.size(); i++){
				cv::line (img,  pnt2cv(path_frontier.poses[i-1].pose.position), pnt2cv(path_frontier.poses[i].pose.position),cv::Scalar(200,200,200),1,cv::LINE_8,0);
			}
			count_target_paths++;
			cv::imwrite("/home/nuc/brain/abmap/"+std::to_string(count_target_paths)+"frontier.png",img);
    }
  }
  return 0;
}
//
