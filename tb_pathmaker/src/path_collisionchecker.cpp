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

tf2_ros::Buffer tfBuffer;
using namespace octomap;
using namespace std;
shared_ptr<DynamicEDTOctomap> edf_ptr;
AbstractOcTree* abs_octree;
shared_ptr<OcTree> octree;
bool got_map = false;
int oct_xmin,oct_ymin,oct_zmin,oct_xmax,oct_ymax,oct_zmax,oct_range_x,oct_range_y,oct_range_z;
ros::Publisher pub_path;

bool par_debug_img;
int count_target_paths = 0;
cv::Mat img(1000,1000,CV_8UC3,cv::Scalar(0, 0, 0)); //create image, set encoding and size, init pixels to default val
cv::Mat img_blank(1000,1000,CV_8UC3,cv::Scalar(0, 0, 0)); //create image, set encoding and size, init pixels to default val
float par_res = 1.0;

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
bool update_edto_bbvec(std::vector<float> bbvec,float collision_radius){
  if(!got_map)
    return false;
  geometry_msgs::Point bbmin_octree,bbmax_octree;
  octree.get()->getMetricMin(bbmin_octree.x,bbmin_octree.y,bbmin_octree.z);
  octree.get()->getMetricMax(bbmax_octree.x,bbmax_octree.y,bbmax_octree.z);

  octomap::point3d boundary_min(fmax(bbvec[0],bbmin_octree.x),fmax(bbvec[1],bbmin_octree.y),fmax(bbvec[2],bbmin_octree.z));
  octomap::point3d boundary_max(fmin(bbvec[3],bbmax_octree.x),fmin(bbvec[4],bbmax_octree.y),fmin(bbvec[5],bbmax_octree.z));
  edf_ptr.reset (new DynamicEDTOctomap(collision_radius,octree.get(),
          boundary_min,
          boundary_max,
          false));
  edf_ptr.get()->update();
  oct_xmin    = int(round(boundary_min.x()))+1;  oct_ymin    = int(round(boundary_min.y()))+1; oct_zmin    = int(round(boundary_min.z()))+1;
  oct_xmax    = int(round(boundary_max.x()))-1;  oct_ymax    = int(round(boundary_max.y()))-1; oct_zmax    = int(round(boundary_max.z()))-1;
  oct_range_x = oct_xmax - oct_xmin;                     oct_range_y = oct_ymax - oct_ymin;                    oct_range_z = oct_zmax - oct_zmin;
  int vol = oct_range_x*oct_range_y*oct_range_z;

  if(vol <= 0)
		return false;
	return true;
}
std::vector<float> getinpath_boundingbox(nav_msgs::Path pathin){
  std::vector<float> bbvec;
  bbvec.resize(6);
  bbvec[3] = bbvec[4] = bbvec[5] = -100;
  bbvec[0] = bbvec[1] = bbvec[2] = 100;
  for(int i = 0; i < pathin.poses.size(); i++){
    if(pathin.poses[i].pose.position.x > bbvec[3])bbvec[3] = pathin.poses[i].pose.position.x;
    if(pathin.poses[i].pose.position.y > bbvec[4])bbvec[4] = pathin.poses[i].pose.position.y;
    if(pathin.poses[i].pose.position.z > bbvec[5])bbvec[5] = pathin.poses[i].pose.position.z;
    if(pathin.poses[i].pose.position.x < bbvec[0])bbvec[0] = pathin.poses[i].pose.position.x;
    if(pathin.poses[i].pose.position.y < bbvec[1])bbvec[1] = pathin.poses[i].pose.position.y;
    if(pathin.poses[i].pose.position.z < bbvec[2])bbvec[2] = pathin.poses[i].pose.position.z;
  }
  return bbvec;
}
std::vector<float> inflate_bbvec(std::vector<float> bbvec,float inflation_xy,float inflation_z){
  bbvec[0] = bbvec[0] - inflation_xy;
  bbvec[3] = bbvec[3] + inflation_z;
  bbvec[1] = bbvec[1] - inflation_xy;
  bbvec[4] = bbvec[4] + inflation_xy;
  bbvec[2] = bbvec[2] - inflation_xy;
  bbvec[5] = bbvec[5] + inflation_z;
  return bbvec;
}
void draw_point_and_obstacle(geometry_msgs::Point pnt,point3d obs_p){
	geometry_msgs::Point obs;
	obs.x = obs_p.x(); obs.y = obs_p.y(); obs.z = obs_p.z();
	cv::circle(img,pnt2cv(pnt),2,cv::Scalar(200,100,0),1);
	if(obs.x == 0 && obs.y == 0)
		return;
	cv::circle(img,pnt2cv(obs),2,cv::Scalar(100,100,200),1);
}
nav_msgs::Path check_path_edto(nav_msgs::Path pathin,float collision_radius,float min_dst){
  std::vector<float> bbvec = inflate_bbvec(getinpath_boundingbox(pathin),7.0,3.0);
  int poses_changed = 0;
  if(update_edto_bbvec(bbvec,collision_radius)){
    for(int i = 0; i < pathin.poses.size(); i++){
      geometry_msgs::Point pnt = pathin.poses[i].pose.position;
      if(pnt.x < float(oct_xmax) && pnt.x > float(oct_xmin)
      && pnt.y < float(oct_ymax) && pnt.y > float(oct_ymin)
      && pnt.z < float(oct_zmax) && pnt.z > float(oct_zmin)){
        point3d closestObst;
        point3d p(pnt.x,pnt.y,pnt.z);
        float dst = collision_radius;
        edf_ptr.get()->getDistanceAndClosestObstacle(p, dst, closestObst);
				if(par_debug_img)
					draw_point_and_obstacle(pnt,closestObst);

        if(dst < min_dst && dst > 0){
					float pz_start = pnt.z;
          while(dst < min_dst){
            p.z() += 1.0;
            edf_ptr.get()->getDistanceAndClosestObstacle(p, dst, closestObst);
          }
					if(par_debug_img){
						float pz_delta = p.z()-pz_start;
						cv::circle(img,pnt2cv(pathin.poses[i].pose.position),2,cv::Scalar(100,100,100),1);
					}
          poses_changed++;
          pathin.poses[i].pose.position.z = p.z();
        }
      }
    }
  }
  ROS_INFO("CHECKING PATH: Path Active Update - EDTO %i/%i changes",poses_changed,pathin.poses.size());
  return pathin;
}
void octomap_callback(const octomap_msgs::Octomap& msg){
  abs_octree=octomap_msgs::fullMsgToMap(msg);
  octree.reset(dynamic_cast<octomap::OcTree*>(abs_octree));
  got_map = true;
}
void path_to_check_cb(const nav_msgs::Path::ConstPtr& msg){
/*	if(got_map && par_debug_img){
		nav_msgs::Path path_out = check_path_edto(*msg,10,5);
		for(int i = 0; i < path_out.poses.size(); i++){

		}
	}*/
	if(par_debug_img){
		count_target_paths++;
		cv::imwrite("/home/nuc/brain/path_planner/collision"+std::to_string(count_target_paths)+ ".png",img);
		img_blank.copyTo(img);
	}
	if(got_map)
		pub_path.publish(check_path_edto(*msg,10,5));
}
int main(int argc, char** argv)
{
  ros::init(argc, argv, "tb_pathedto_node");
  ros::NodeHandle nh;
	ros::NodeHandle private_nh("~");
	private_nh.param("write_debug_img", par_debug_img, true);

	ros::Subscriber s1  = nh.subscribe("/octomap_full",1,octomap_callback);
	ros::Subscriber s2  = nh.subscribe("/tb_path/check_collision",1,octomap_callback);
  pub_path 					  = nh.advertise<nav_msgs::Path>("/tb_path/check_collision",10);
	ros::spin();
  return 0;
}
