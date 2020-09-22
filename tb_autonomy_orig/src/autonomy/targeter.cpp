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
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib_msgs/GoalID.h>
#include <nav_msgs/GetPlan.h>

ros::Publisher pub_set_z,pub_set_yaw,pub_set_xy,pub_get_poly,pub_get_side,pub_get_sphere,pub_get_down,pub_set_mbpose,pub_get_elevation,pub_costmapupdate_edto,pub_costmapupdate_img;
geometry_msgs::Point p,pos;
std::string state_target = "idle";
geometry_msgs::PointStamped setpoint_xy,new_target_pnt;
geometry_msgs::PoseStamped setpose;
std_msgs::Float64 setpoint_z,setpoint_yaw;
bool use_mb;
float pos_yaw;
tf2_ros::Buffer tfBuffer;
std::vector<int> targetindexes_sent;
std::vector<int> blacklist;
int costmap_elevation_meters;
ros::Time process_start;
geometry_msgs::PolygonStamped poly_roi2d,poly_roi;
nav_msgs::Path path_starsquare,path_cmd;
std::string active_process;
std::vector<float> zmin_vec;
cv::Mat img(1000,1000,CV_8UC3,cv::Scalar(0, 0, 0)); //create image, set encoding and size, init pixels to default val
cv::Mat img_blank(1000,1000,CV_8UC3,cv::Scalar(0, 0, 0)); //create image, set encoding and size, init pixels to default val
float zmin_ave;
int que_size = 10;
float pos_z_min = 5.0;
geometry_msgs::PointStamped pos_front;
geometry_msgs::Vector3 vlp_rpy;
int path_cmd_i = 0;
int count_target_paths = 0;
nav_msgs::Odometry odom_global;
float par_res = 1.0;
nav_msgs::Path path_visited;
void vstd_cb(const nav_msgs::Path::ConstPtr& msg){
  path_visited = *msg;
}
std_msgs::Header hdr(){
  std_msgs::Header header;
  header.frame_id = "map";
  header.stamp    = ros::Time::now();
  return header;
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

void draw_line(geometry_msgs::Point p0,float a,float r,cv::Scalar color){
  geometry_msgs::Point pyaw;
  pyaw.x = p0.x + r * cos(a);
  pyaw.y = p0.y + r * sin(a);
  cv::line (img, pnt2cv(pos), pnt2cv(pyaw),color,1,cv::LINE_8,0);
}

double constrainAngle(double x){
  if(x > M_PI)
    return (x - M_PI*2);
  else if(x < -M_PI)
    return (x + M_PI*2);
  else
    return x;
}
void drawimg(){
  img_blank.copyTo(img);

	draw_path(path_starsquare,cv::Scalar(200,0,200),3);
	draw_path(path_visited,cv::Scalar(100,50,50),1);

  draw_line(pos,constrainAngle(pos_yaw + M_PI/3),50,cv::Scalar(200,200,200));
  draw_line(pos,constrainAngle(pos_yaw - M_PI/3),50,cv::Scalar(200,200,200));
  cv::circle(img,pnt2cv(pos),4,cv::Scalar(200,200,200),1);
  cv::Mat img_new2;
  cv::resize(img, img_new2, cv::Size(), 2.0, 2.0);
  count_target_paths++;
  cv::imwrite("/home/nuc/brain/fullpath/"+std::to_string(count_target_paths)+"navigator.png",img_new2);
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
int get_random_i(int nummax){
  srand (time(NULL));
  return rand() % nummax;
}
geometry_msgs::Point get_random_pnt(nav_msgs::Path pathin){
	std::vector<int> possible_i;
	for(int i = 0; i < pathin.poses.size(); i++){
		if(!in_vec(blacklist,i))
			possible_i.push_back(i);
	}
	int iout = get_random_i(possible_i.size());
	int iact = possible_i[iout];
	blacklist.push_back(iact);
	return pathin.poses[iact].pose.position;
}
void send_setpoint(geometry_msgs::Point pnt){
	float new_yaw = get_hdng(pnt,pos);

	setpose.header = hdr();
	setpoint_xy.header = hdr();
	setpose.pose.position 	= pnt;
	setpose.pose.position.z = 0;
	setpose.pose.orientation = tf::createQuaternionMsgFromYaw(new_yaw);
	setpoint_xy.point.x = pnt.x;
	setpoint_xy.point.y = pnt.y;
	setpoint_z.data 		= pnt.z;
	setpoint_yaw.data = new_yaw;
	new_target_pnt.point.z = 0;
	if(use_mb)
		pub_set_mbpose.publish(setpose);
}
geometry_msgs::PolygonStamped create_poly2d(geometry_msgs::Point pin, float rad){
	geometry_msgs::PolygonStamped poly;
	poly.header = hdr();
	poly.polygon.points.resize(5);
	poly.polygon.points[0].x = round(pin.x + rad);
	poly.polygon.points[0].y = round(pin.y + rad);
	poly.polygon.points[1].x = round(pin.x - rad);
	poly.polygon.points[1].y = round(pin.y + rad);
	poly.polygon.points[2].x = round(pin.x - rad);
	poly.polygon.points[2].y = round(pin.y - rad);
	poly.polygon.points[3].x = round(pin.x + rad);
	poly.polygon.points[3].y = round(pin.y - rad);
	poly.polygon.points[4]   = poly.polygon.points[0];
	return poly;
}
void create_poly_volume(geometry_msgs::Point pin, float sides,float z0,float z1){
	poly_roi2d = create_poly2d(pin,sides/2);
	poly_roi = poly_roi2d;
	for(int i = 0; i < poly_roi.polygon.points.size(); i++){
		poly_roi.polygon.points[i].z = z0;
	}
	poly_roi.polygon.points.push_back(poly_roi.polygon.points[0]);
	poly_roi.polygon.points.push_back(poly_roi.polygon.points[1]);
	poly_roi.polygon.points.push_back(poly_roi.polygon.points[2]);
	poly_roi.polygon.points.push_back(poly_roi.polygon.points[3]);
	poly_roi.polygon.points.push_back(poly_roi.polygon.points[4]);

	for(int i = 5; i < poly_roi.polygon.points.size(); i++){
		poly_roi.polygon.points[i].z = z1;
	}
	for(int i = 0; i < poly_roi.polygon.points.size(); i++){
		if(poly_roi.polygon.points[i].z > z1)
			poly_roi.polygon.points[i].z = z1;
		if(poly_roi.polygon.points[i].z < z0)
			poly_roi.polygon.points[i].z = z0;
	}
//  ROS_INFO("x %.2f y %.2f x %.2f y %.2f x %.2f y %.2f x %.2f y %.2f x %.2f y %.2f",poly.polygon.points[0].x,poly.polygon.points[0].y,poly.polygon.points[1].x,poly.polygon.points[1].y,poly.polygon.points[2].x,poly.polygon.points[2].y,poly.polygon.points[3].x,poly.polygon.points[3].y,poly.polygon.points[4].x,poly.polygon.points[4].y);
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
void update_polygon_of_interest(float m_in_front,float dxy, float dz){
	pos_front.header = hdr();
	pos_front.point.x = pos.x + m_in_front * cos(pos_yaw);
	pos_front.point.y = pos.y + m_in_front * sin(pos_yaw);
	float z0 = pos.z - dz;
	float z1 = pos.z + dz;
	create_poly_volume(pos,dxy,z0,z1);
}
void update_pos_vlp(){
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
	tf2::Matrix3x3 q(tf2::Quaternion(transformStamped.transform.rotation.x, transformStamped.transform.rotation.y, transformStamped.transform.rotation.z, transformStamped.transform.rotation.w));
	geometry_msgs::Vector3 v;
	q.getRPY(v.x,v.y,v.z);
	pos_yaw = v.z;
}
void mainstate_cb(const std_msgs::UInt8::ConstPtr& msg){

}

void odomglobal_cb(const nav_msgs::Odometry::ConstPtr& msg){
  //ROS_INFO("odomglobal_cb");
  if((ros::Time::now() - odom_global.header.stamp).toSec() > 0.2){
    odom_global = *msg;
  }
}
void costmap_elevation_cb(const std_msgs::UInt8::ConstPtr& msg){
	if(new_target_pnt.point.z == 0){
		new_target_pnt.point.z = msg->data;
		pub_costmapupdate_img.publish(poly_roi2d);
	}
}

void map_updates_cb(const map_msgs::OccupancyGridUpdate::ConstPtr& msg){
	if(new_target_pnt.point.z > 0)
		send_setpoint(new_target_pnt.point);
}
void get_setpoint(){
	new_target_pnt.point  = get_random_pnt(path_starsquare);
	new_target_pnt.point.z = 0;
	new_target_pnt.header = hdr();
	pub_get_elevation.publish(new_target_pnt);
}
float get_zmax_que(){
	float zmx = 0;
	float zmin_zum = 0;
	for(int i = 0; i < zmin_vec.size(); i++){
		zmin_zum += zmin_vec[i];
		if(zmx < zmin_vec[i])
			zmx = zmin_vec[i];
	}
	zmin_ave = zmin_zum / zmin_vec.size();
	return zmx;
}
void point_elevated_cb(const geometry_msgs::PointStamped::ConstPtr& msg){
	float zmin = msg->point.z;
	zmin_vec.push_back(msg->point.z);
	if(zmin_vec.size() > que_size)
		zmin_vec.erase(zmin_vec.begin());
	pos_z_min = get_zmax_que();
	ROS_INFO("pos_z_min: %.0f",pos_z_min);
}

void pathcmd_i_cb(const std_msgs::UInt8::ConstPtr& msg){
	path_cmd_i = msg->data;
}

void path_cmd_cb(const nav_msgs::Path::ConstPtr& msg){
	if(msg->poses.size() > path_cmd_i){
		path_cmd 		 = *msg;
		path_cmd.header.stamp = ros::Time::now();
		state_target = "path_cmd";
	}
	else if(state_target == "waiting_cmd"){
		state_target = "path_cmd";
	}
	if(state_target == "path_cmd" && (ros::Time::now()-path_cmd.header.stamp).toSec() > 2.0){
		state_target = "idle";
	}
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "tb_autonomy_node");
  ros::NodeHandle nh;
	ros::NodeHandle private_nh("~");
	tf2_ros::TransformListener tf2_listener(tfBuffer);

	ros::Subscriber s0 			= nh. subscribe("/tb_fsm/main_state",100,&mainstate_cb);
	ros::Subscriber s1 			= nh. subscribe("/tb_mbmap/costmap_elevation",100,&costmap_elevation_cb);
	ros::Subscriber s2 			= nh. subscribe("/map_updates",100,&map_updates_cb);

	ros::Subscriber s5 			= nh.subscribe("/tb_cmd/path_active_index", 100,&pathcmd_i_cb);
	ros::Subscriber s6 			= nh.subscribe("/tb_cmd/path_active",  100,&path_cmd_cb);
	ros::Subscriber s4 = nh.subscribe("/tb_world/path_visited",10,vstd_cb);

	pub_set_z	  = nh.advertise<std_msgs::Float64>("/tb_cmd/set_z",10);
	pub_set_yaw	= nh.advertise<std_msgs::Float64>("/tb_cmd/set_yaw",10);
	pub_set_xy	= nh.advertise<geometry_msgs::PointStamped>("/tb_cmd/set_xy",10);

	ros::Publisher pub_roi_poly	  = nh.advertise<geometry_msgs::PolygonStamped>("/tb_roi/poly",10);
	ros::Publisher pub_roi2d_poly	= nh.advertise<geometry_msgs::PolygonStamped>("/tb_roi/poly2d",10);
	pub_get_poly	  = nh.advertise<geometry_msgs::PolygonStamped>("/tb_edto/get_poly",10);
	pub_get_side	  = nh.advertise<geometry_msgs::PolygonStamped>("/tb_edto/get_side",10);
	pub_get_sphere	= nh.advertise<geometry_msgs::PolygonStamped>("/tb_edto/get_sphere",10);
	pub_get_down		= nh.advertise<geometry_msgs::PolygonStamped>("/tb_edto/get_down",10);

	pub_set_mbpose  = nh.advertise<geometry_msgs::PoseStamped>("/tb_md/pose_target",10);
	pub_get_elevation	 			= nh.advertise<geometry_msgs::PointStamped>("/tb_mbmap/get_elevation_to_point",10);
	pub_costmapupdate_edto  = nh.advertise<geometry_msgs::PolygonStamped>("/tb_mbmap/costmap_update_EDTO",10);
	pub_costmapupdate_img   = nh.advertise<geometry_msgs::PolygonStamped>("/tb_mbmap/costmap_update_heightimg",10);
	ros::Publisher pub_point_to_elevate	= nh.advertise<geometry_msgs::PointStamped>("/tb_abmap/get_elevation_point",10);

	ros::Subscriber s3   = nh.subscribe("/tb_abmap/elevated_point",10,point_elevated_cb);

	ros::Rate rate(2.0);
  ros::Time last_radialupdate = ros::Time::now();
	int count = 0;
	path_starsquare = create_starsquare(75);
	get_setpoint();

	ros::Time time_last = ros::Time::now();
	while(ros::ok()){
		pub_point_to_elevate.publish(pos_front);
    rate.sleep();
    ros::spinOnce();
		update_pos_vlp();
		update_polygon_of_interest(5,50,5);
		geometry_msgs::Point p0;
		float area_sidelength = 300;
		float radlen_xy = 5;
		nav_msgs::Path gridpath = create_gridpath(p0,area_sidelength,radlen_xy);

		if(new_target_pnt.point.z == setpoint_xy.point.x && new_target_pnt.point.y == setpoint_xy.point.y){
			float dst_remaining = get_dst2d(setpoint_xy.point,pos);
			if(dst_remaining < 10.0){
				get_setpoint();
			}
		}

		if(count == 1)
			pub_get_poly.publish(poly_roi);
		else if(count == 2)
			pub_get_side.publish(poly_roi);
		else if(count == 3)
			pub_get_down.publish(poly_roi);
		else if(count == 4)
			pub_get_sphere.publish(poly_roi);
		else
			count = 0;
		count++;
		if(!use_mb){
			if(pos_z_min < 3)
				pos_z_min = 10.0;
			setpoint_z.data = pos_z_min + 5;
			pub_set_z.publish(setpoint_z);
			pub_set_yaw.publish(setpoint_yaw);
			pub_set_xy.publish(setpoint_xy);
		}
		drawimg();
		pub_roi_poly.publish(poly_roi);
		pub_roi2d_poly.publish(poly_roi2d);
  }
  return 0;
}
