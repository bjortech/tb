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
cv::Mat img_height_m(1000,1000,CV_8U,cv::Scalar(0)); //create image, set encoding and size, init pixels to default val
cv::Mat img_height_mi(1000,1000,CV_8U,cv::Scalar(0)); //create image, set encoding and size, init pixels to default val

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
double par_zmin,par_maprad,par_elevbb,par_zclearing,par_arearad;
///////////********CTRL***********//////////
ros::Publisher pub_path_elevated,pub_gridpath,pub_gp_elevated,pub_fw_elevated,pub_setpoint,pub_map_updates;
ros::Time process_start;
std::string active_process;
using namespace octomap;
using namespace std;
shared_ptr<DynamicEDTOctomap> edf_ptr;
AbstractOcTree* abs_octree;
shared_ptr<OcTree> octree;
nav_msgs::Odometry odom_global;
float par_res = 1.0;
geometry_msgs::PoseStamped setpoint_candidate,pose_setpoint;
geometry_msgs::Point pos;
geometry_msgs::Vector3 vlp_rpy;

bool new_setpoint;
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

    ////ROS_INFO("Zinterval[%i]: p.z(): %.0f (z0:%.0f->z1:%.0f), dst: %.2f, obst-z: %.0f",z,p.z(),z0,z1,dst,closestObst.z());
    if(dst > radlen_xy_min)
      return p.z();
  }
//  ROS_ERROR("Zinterval[FAILED]: pnt: %.0f %.0f %.0f minxy: %.0f z0:%.0f,z1:%.0f z_interval: %i",pnt.x,pnt.y,pnt.z,radlen_xy_min,z_dn,z_up,z_interval);

  return -1;
}
int get_zmax_grid_m(int r0,int c0, int radlen_xy){
  int zmx = 0;
  for(int c = c0-radlen_xy; c < c0+radlen_xy; c++){
    for(int r = r0-radlen_xy; r < r0+radlen_xy; r++){
      if(img_height_m.at<uchar>(r,c) > zmx)
        zmx = img_height_m.at<uchar>(r,c);
    }
  }
  return zmx;
}
void inflate_monoheight(int inflation_radlen_xy){
  for(int c = inflation_radlen_xy*2; c < img_height_m.cols-inflation_radlen_xy*2; c++){
    for(int r = inflation_radlen_xy*2; r < img_height_m.rows-inflation_radlen_xy*2; r++){
      img_height_mi.at<uchar>(r,c) = get_zmax_grid_m(r,c,inflation_radlen_xy);
    }
  }
}
void update_monoheight(){
  for(int c = 0; c < img_height_m.cols; c++){
    for(int r = 0; r < img_height_m.rows; r++){
      img_height_m.at<uchar>(r,c) = fmax(img_height.at<cv::Vec3b>(r,c)[1],img_height.at<cv::Vec3b>(r,c)[2]);
    }
  }
}
int get_zmi(geometry_msgs::Point midpoint){
  return img_height_mi.at<uchar>(y2r(midpoint.y),x2c(midpoint.x));
}
void update_mbmap(float z){
  map_msgs::OccupancyGridUpdate update;

  int xmin = -400; //bbxyxy[0];
  int ymin = -400; //bbxyxy[1];
  int xmax = 400; //bbxyxy[2];
  int ymax = 400; //bbxyxy[3];
  int xrnge = xmax-xmin;
  int yrnge = ymax-ymin;
  //ROS_INFO("BB: xmin,ymin:(%i,%i)->xmax,ymax:(%i,%i)  [%i x %i]",xmin,ymin,xmax,ymax,xmax-xmin,ymax-ymin);
  if(xrnge < 10 || yrnge < 10){
    ROS_ERROR("BB rnge low");
    return;
  }
  update.header.stamp = ros::Time::now();
  update.header.frame_id = "map";
  update.x = 500+xmin;
  update.y = 500+ymin;
  update.width  = xrnge;
  update.height = yrnge;
  update.data.resize(update.width * update.height);
  int zlim = int(round(z));
  unsigned int i = 0;
  for (int y = ymin; y < ymax; y++){
    for (int x = xmin; x < xmax; x++){
      if(img_height_mi.at<uchar>(y2r(y),x2c(x)) > zlim)
        update.data[i++] = 100;
      else
        update.data[i++] = 0;
    }
  }
//  start_process("");
  pub_map_updates.publish(update);
}
void update_edto_zmap(std::vector<int> bbmnmnx_rc,float collision_radius){
  geometry_msgs::Point bbmin_custom,bbmax_custom,bbmin_octree,bbmax_octree,bbmin,bbmax;

  float z0 = bbmnmnx_rc[4];
  float z1 = bbmnmnx_rc[5];
  bbmin.x = c2x(bbmnmnx_rc[1]);
  bbmin.y = r2y(bbmnmnx_rc[2]);
  bbmax.x = c2x(bbmnmnx_rc[3]);
  bbmax.y = r2y(bbmnmnx_rc[0]);
  bbmin.z = bbmnmnx_rc[4];
  bbmax.z = bbmnmnx_rc[5];
  if(bbmin.x < -400 || bbmin.y < -400 || bbmax.x > 400 || bbmax.y > 400 || bbmin.z == 0)
    return;
  //ROS_INFO(" bbmin: %.2f, : %.2f, : %.2f, bbmax: %.2f, : %.2f, : %.2f ",bbmin.x,bbmin.y,bbmin.z,bbmax.x,bbmax.y,bbmax.z);
  ////ROS_INFO("UPDAT_EDTO")
  float dz = bbmax.z - bbmin.z;
  if(dz < 5){
    if(abs(pos.z-bbmax.z) < abs(pos.z-bbmin.z)){
      bbmax.z += (5-dz);
      bbmin.z = bbmax.z - 5;
    }
    else{
      bbmin.z -= (5-dz);
      bbmax.z = bbmin.z + 5;
    }
  }
  octree.get()->getMetricMin(bbmin_octree.x,bbmin_octree.y,bbmin_octree.z);
  octree.get()->getMetricMax(bbmax_octree.x,bbmax_octree.y,bbmax_octree.z);


  //ROS_INFO("bbmin.z: %.2f->%.2f bbmin_octree: %.2f, : %.2f, : %.2f, bbmax_octree: %.2f, : %.2f, : %.2f ",bbmin.z,bbmax.z,bbmin_octree.x,bbmin_octree.y,bbmin_octree.z,bbmax_octree.x,bbmax_octree.y,bbmax_octree.z);
  if(bbmin.z > bbmax_octree.z)
    return;
  octomap::point3d boundary_min(fmax(bbmin.x,bbmin_octree.x),fmax(bbmin.y,bbmin_octree.y),fmax(bbmin.z,bbmin_octree.z));
  octomap::point3d boundary_max(fmin(bbmax.x,bbmax_octree.x),fmin(bbmax.y,bbmax_octree.y),fmin(bbmax.z,bbmax_octree.z));

  edf_ptr.reset (new DynamicEDTOctomap(collision_radius,octree.get(),
          boundary_min,
          boundary_max,
          false));
  edf_ptr.get()->update();
  int xmin    = int(round(boundary_min.x()))+1;  int ymin    = int(round(boundary_min.y()))+1; int zmin    = int(round(boundary_min.z()));
  int xmax    = int(round(boundary_max.x()))-1;  int ymax    = int(round(boundary_max.y()))-1; int zmax    = int(round(boundary_max.z()));
  int range_x = xmax - xmin;                     int range_y = ymax - ymin;                   int range_z = zmax - zmin;
  //ROS_INFO("xyz -> %i %i %i -> %i %i %i ",xmin,ymin,zmin,xmax,ymax,zmax);
  int vol = range_x*range_y*range_z;
  if(vol <= 0)
    return;
  for(int z = zmin; z < zmax; z++){
    for(int y = ymin; y < ymax; y++){
      for(int x = xmin; x < xmax; x++){
        octomap::point3d p(x,y,z);
        //if(img_height.at<cv::Vec3b>(y2r(y),x2c(x))[2] < z && edf_ptr.get()->getDistance(p) < collision_radius){
        if(edf_ptr.get()->getDistance(p) < collision_radius){
          img_height.at<cv::Vec3b>(y2r(y),x2c(x))[1] = z;
        }
      }
    }
  }
}
void merge_update_with_grid(){
  img_blank.copyTo(img_new);
  std::vector<int> bbvec; //rmn_cmn_rmx_cmx
  bbvec.resize(6);
  bbvec[0] = img_height.rows;  bbvec[1] = img_height.cols;  bbvec[2] = 0;
  bbvec[3] = 0;  bbvec[4] = 0;  bbvec[5] = 0;
  bool first = true;
  for(int c = 0; c < img_height.cols; c++){
    for(int r = 0; r < img_height.rows; r++){
      if(img_update.at<uchar>(r,c) > 0){
        if(r < bbvec[0])
          bbvec[0] = r;
        if(c < bbvec[1])
          bbvec[1] = c;
        if(c > bbvec[3])
          bbvec[3] = c;
        if(r > bbvec[2])
          bbvec[2] = r;
        if(img_update.at<uchar>(r,c) > 0){
          if(first){
            first =false;
            bbvec[4] = img_update.at<uchar>(r,c)-1;
            bbvec[5] = img_update.at<uchar>(r,c)+1;
          }
          else{
            if(img_update.at<uchar>(r,c) > bbvec[5])
              bbvec[5] = img_update.at<uchar>(r,c);
            if(img_update.at<uchar>(r,c) < bbvec[4])
              bbvec[4] = img_update.at<uchar>(r,c);
            if(img_update.at<uchar>(r,c) > img_height.at<cv::Vec3b>(r,c)[2])
              img_height.at<cv::Vec3b>(r,c)[2] = img_update.at<uchar>(r,c);
          }
        }
      }
    }
  }
  //start_process("merge");
  if(got_map)
    update_edto_zmap(bbvec,1.5);
//  start_process("");
  img_blank.copyTo(img_update);
  update_monoheight();
  inflate_monoheight(5);
  count_target_paths++;
  if(par_debug_img){
    cv::imwrite("/home/nuc/brain/fullpath/"+ std::to_string(count_target_paths) + "img_mono_inflatied" + ".png",img_height_mi);
    cv::imwrite("/home/nuc/brain/fullpath/"+ std::to_string(count_target_paths) + "img_mono" + ".png",img_height_m);
    cv::imwrite("/home/nuc/brain/fullpath/"+ std::to_string(count_target_paths) + "img_upd" + ".png",img_update);
  }
}
void update_gridpath(geometry_msgs::Point32 pnt){
  int r = y2r(pnt.y);
  int c = x2c(pnt.x);
  int z = fmax(pnt.z,1);
  if(z > img_update.at<uchar>(r,c))
    img_update.at<uchar>(r,c) = z;
}
void pc1_assembly_cb(const sensor_msgs::PointCloud::ConstPtr& msg){
  for(int i= 0; i < msg->points.size(); i++){
    update_gridpath(msg->points[i]);
  }
  merge_update_with_grid();
}
void pc1_cb(const sensor_msgs::PointCloud::ConstPtr& msg){
  for(int i= 0; i < msg->points.size(); i++){
    update_gridpath(msg->points[i]);
  }
//  merge_update_with_grid();
}
void octomap_callback(const octomap_msgs::Octomap& msg){
  abs_octree=octomap_msgs::fullMsgToMap(msg);
  octree.reset(dynamic_cast<octomap::OcTree*>(abs_octree));
  got_map = true;
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
     ps.pose.position.z = get_zmi(ps.pose.position);
     ps.pose.orientation.w = 1.0;
     ps.header = hdr();
     pathout.poses.push_back(ps);
    }
  }
  return pathout;
}
nav_msgs::Path elevate_path(nav_msgs::Path pathin){
  for(int i = 0; i < pathin.poses.size(); i++){
    pathin.poses[i].pose.position.z = get_zmi(pathin.poses[i].pose.position);
  }
  return pathin;
}
void pathelevate_cb(const nav_msgs::Path::ConstPtr& msg){
  pub_path_elevated.publish(elevate_path(*msg));
}
void global_plan_cb(const nav_msgs::Path::ConstPtr& msg){
  pub_gp_elevated.publish(elevate_path(*msg));
}
void path_forwsim_cb(const nav_msgs::Path::ConstPtr& msg){
  pub_fw_elevated.publish(elevate_path(*msg));
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
  //ROS_INFO("Create gridpath");
  pub_gridpath.publish(create_gridpath(pos,100,5));
}
geometry_msgs::PoseStamped check_setpoint(nav_msgs::Odometry odom_in,geometry_msgs::PoseStamped setpoint_in){
  float vz   = odom_in.twist.twist.linear.z;
  float vxy  = sqrt(pow(odom_in.twist.twist.linear.x,2)+pow(odom_in.twist.twist.linear.y,2));
  float vxyz = sqrt(pow(odom_in.twist.twist.linear.x,2)+pow(odom_in.twist.twist.linear.y,2)+pow(odom_in.twist.twist.linear.z,2));
  float vaz  = odom_in.twist.twist.angular.z;
//  ROS_INFO("Velocity hdng: %.2f incl: %.2f speed: vxyz: %.2f vxy: %.2f vz: %.2f angular: %.2f",rpy_vel.z,rpy_vel.y,vxyz,vxy,vz,vaz);
  geometry_msgs::Point forward_point;
  geometry_msgs::Point setpoint;

  forward_point.x = odom_in.twist.twist.linear.x * 3;//  pos.x + 15 * cos(vlp_rpy.z);
  forward_point.y = odom_in.twist.twist.linear.y * 3;// pos.y + 15 * sin(vlp_rpy.z);
  forward_point.z = odom_in.twist.twist.linear.z * 3;// pos.y + 15 * sin(vlp_rpy.z);
  //TODO: gJØRE FERDIG SIKKERHETSVENTIL MED EDTO
  return setpoint_in;
}
void mapelevation_cb(const std_msgs::Float64::ConstPtr& msg){
  //ROS_INFO("mapelevation_cb");
//  start_process("Update mbmap");
  update_mbmap(msg->data);
}
void setp_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
  //ROS_INFO("setp_cb");
  setpoint_candidate = *msg;
  //new_setpoint = true;
  pub_setpoint.publish(check_setpoint(odom_global,setpoint_candidate));

}
void mbtarget_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
  float zmi_target = get_zmi(msg->pose.position)+2;
  float zmi_pos    = get_zmi(pos);
  update_mbmap(fmax(zmi_target,zmi_pos));
}

void odomglobal_cb(const nav_msgs::Odometry::ConstPtr& msg){
  //ROS_INFO("odomglobal_cb");
  if((ros::Time::now() - odom_global.header.stamp).toSec() > 0.2 || new_setpoint){
    odom_global = *msg;
    update_pos_vlp();
    if(new_setpoint){
      new_setpoint = false;
      pub_setpoint.publish(check_setpoint(*msg,setpoint_candidate));
    }
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "tb_heightmapper_node");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");
  private_nh.param("mapbb_radius",    par_maprad, 25.0);
  private_nh.param("z_zero",          par_zmin, -10.0);
  private_nh.param("write_debug_img", par_debug_img, true);
  private_nh.param("z_clearing", par_zclearing, 5.0);

  private_nh.getParam("workdir_path", par_workdir);//*2.0);
//img_height = cv::imread(par_workdir,CV_LOAD_IMAGE_COLOR);
  //  srand (time(NULL));
  //   5 + rand() % 25;

	tf2_ros::TransformListener tf2_listener(tfBuffer);

  ros::Subscriber s1  = nh.subscribe("/octomap_full",1,octomap_callback);
  ros::Subscriber a2  = nh.subscribe("/velodyne_pc1",10,pc1_cb);
  ros::Subscriber a3  = nh.subscribe("/tb_obs/assembled_pc1",10,pc1_assembly_cb);
  ros::Subscriber s55 = nh.subscribe("/odom_global",1,odomglobal_cb);
  ros::Subscriber s7  = nh.subscribe("/move_base/NavfnROS/plan",100,&global_plan_cb);
  ros::Subscriber s8  = nh.subscribe("/tb_nav/path_forwsim",100,&path_forwsim_cb);
  ros::Subscriber s0  = nh.subscribe("/tb_env/elevate_path",100,&pathelevate_cb);

  ros::Subscriber as3 = nh.subscribe("/tb_nav/setpoint",10,setp_cb);
  ros::Subscriber as4 = nh.subscribe("/tb_env/mapelevation",10,mapelevation_cb);

  ros::Subscriber a4  = nh.subscribe("/tb_cmd/posemb_exeq",10,mbtarget_cb);

  pub_gridpath    = nh.advertise<nav_msgs::Path>("/tb_env/gridpath",10);
  pub_gp_elevated = nh.advertise<nav_msgs::Path>("/tb_env/gp_elevated",10);
  pub_fw_elevated = nh.advertise<nav_msgs::Path>("/tb_env/fw_elevated",10);
  pub_path_elevated = nh.advertise<nav_msgs::Path>("/tb_env/path_elevated",10);

  pub_setpoint    = nh.advertise<geometry_msgs::PoseStamped>("/tb_setpoint",10);
  pub_map_updates = nh.advertise<map_msgs::OccupancyGridUpdate>("/map_updates",100);

  ros::spin();

  return 0;
}
//
