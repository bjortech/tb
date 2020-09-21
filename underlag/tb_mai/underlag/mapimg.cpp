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
double par_maprad,par_lookahead_distance,par_visited_rad,par_img_enhance;
tf2_ros::Buffer tfBuffer;
std::string par_workdir;
int img_enhance,xmin,ymin,zmin,xmax,ymax,zmax,range_x,range_y,range_z,zmin_global,zmax_global;
float closest_obstacle_plane_dist,closest_obstacle_dist;
geometry_msgs::PointStamped closest_obstacle,closest_obstacle_plane;
geometry_msgs::Point pos,bbmin_custom,bbmax_custom,bbmin_octree,bbmax_octree;

cv::Mat mapimg(1000,1000,CV_8UC3,cv::Scalar(0, 0, 0));
float zneg = -10;
float zpos = 100;
float collision_radius = 1.1;
int last_i,last_r,last_c;
bool got_map;
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
geometry_msgs::PolygonStamped get_poly_from_mapimg(geometry_msgs::Point midpoint,int tot_length,int num_rays){
	geometry_msgs::PolygonStamped poly;
	poly.polygon.points.resize(num_rays);
	poly.header.frame_id  = "map";
	for(int k  = 0; k < num_rays; k++){
		bool found_end = false;
		for(int i = 0; i < tot_length; i++){
      int r = y2r(midpoint.y + i*sin(-M_PI + k*(2*M_PI / num_rays)),mapimg.rows,1);
      int c = x2c(midpoint.x + i*cos(-M_PI + k*(2*M_PI / num_rays)),mapimg.cols,1);
			if(!found_end){
				if(//mapimg.at<cv::Vec3b>( r,c )[0] == 0&&
           mapimg.at<cv::Vec3b>( r,c )[1] < 20
				&& mapimg.at<cv::Vec3b>( r,c )[1] > 0 ){
					poly.polygon.points[k].x	= c2x(c,mapimg.cols,1);
					poly.polygon.points[k].y	= r2y(r,mapimg.rows,1);
					poly.polygon.points[k].z  = midpoint.z;
				}
				else{
					found_end = true;
					mapimg.at<cv::Vec3b>(r,c)[0] = int(midpoint.z);
					ROS_INFO("Found end (r,c): %i %i at %.2f %.2f %.2f",r,c,poly.polygon.points[i].x,poly.polygon.points[i].y,poly.polygon.points[i].z);
				}
			}
		}
	}
	return poly;
}

geometry_msgs::Point lengthedit_ray(geometry_msgs::Point p0,geometry_msgs::Point p1,float meters_to_add){
  Eigen::Vector3f stride_vec;
	Eigen::Vector3f pnt1_vec(p0.x,p0.y,p1.z);
	Eigen::Vector3f pnt2_vec(p1.x,p1.y,p1.z);
	float tot_length = (pnt2_vec - pnt1_vec).norm();
  stride_vec = (pnt2_vec - pnt1_vec).normalized();
  Eigen::Vector3f cur_vec = pnt1_vec + stride_vec * (tot_length+meters_to_add);
	geometry_msgs::Point new_endpoint;
	new_endpoint.x = cur_vec.x();
	new_endpoint.y = cur_vec.y();
	new_endpoint.z = cur_vec.z();
	ROS_INFO("P0 %.2f %.2f %.2f -> P1 %.2f %.2f %.2f:  [%.2f m], adding: %.2f m, endpoint: %.2f %.2f %.2f",p0.x,p0.y,p0.z,p1.x,p1.y,p1.z,tot_length,meters_to_add,new_endpoint.x,new_endpoint.y,new_endpoint.z);
  return new_endpoint;
}

geometry_msgs::PolygonStamped start_point(){
	geometry_msgs::PolygonStamped poly;
	poly = get_poly_from_mapimg(lengthedit_ray(pos,closest_obstacle_plane.point,-5),36,36);
	return poly;
}

bool update_edto(geometry_msgs::Point midpoint,float collision_radius,float maprad,float z0,float z1,bool unknownAsOccupied,bool sides){
  if(got_map){
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
    if(vol <= 0)
			return false;
    octomap::point3d p(midpoint.x,midpoint.y,midpoint.z);
    octomap::point3d closestObst;
		float dst;
    edf_ptr.get()->getDistanceAndClosestObstacle(p, dst, closestObst);

		if(sides){
			closest_obstacle_plane.point.x=closestObst.x();
			closest_obstacle_plane.point.y=closestObst.y();
			closest_obstacle_plane.point.z=closestObst.z();
			closest_obstacle_plane_dist = dst;
		}
		else{
			closest_obstacle.point.x=closestObst.x();
			closest_obstacle.point.y=closestObst.y();
			closest_obstacle.point.z=closestObst.z();
			closest_obstacle_dist = dst;
		}
    return true;
  }
  else
    return false;
}

void get_zdown(float collision_radius){
  for(int y = ymin; y < ymax; y++){
    for(int x = xmin; x < xmax; x++){
			int z = zmax;
			float dst = collision_radius;
			while(dst > (collision_radius-1) && z > bbmin_octree.z){
				octomap::point3d p(x,y,z);
				dst = edf_ptr.get()->getDistance(p);
				z--;
			}
			int r = y2r(y,mapimg.rows,1);
			int c = x2c(x,mapimg.cols,1);
			mapimg.at<cv::Vec3b>(r,c)[2] = z;
  	}
	}
	ROS_INFO("get_zdown complete");
}

void get_zsideways(float collision_radius,float zval){
  for(int y = ymin; y < ymax; y++){
    for(int x = xmin; x < xmax; x++){
			mapimg.at<cv::Vec3b>( y2r(y,mapimg.rows,1),x2c(x,mapimg.cols,1) )[1] = 0;
      octomap::point3d p(x,y,zval);
			float d = edf_ptr.get()->getDistance(p);
			if(d >= collision_radius)
				mapimg.at<cv::Vec3b>( y2r(y,mapimg.rows,1),x2c(x,mapimg.cols,1) )[1] = 50;
			else if(d < 0)
				mapimg.at<cv::Vec3b>( y2r(y,mapimg.rows,1),x2c(x,mapimg.cols,1) )[1] = 100;
			else
				mapimg.at<cv::Vec3b>( y2r(y,mapimg.rows,1),x2c(x,mapimg.cols,1) )[1] = d;
    }
  }
}
void update_edtos_down(){
	collision_radius = 1.1;
	float maprad = par_maprad;
	update_edto(pos,collision_radius,maprad,pos.z-10,pos.z+50,false,false);
	get_zdown(collision_radius);
}
void update_edtos_side(){
	float collision_radius_sides = 20;
	float maprad = par_maprad;
	update_edto(pos,collision_radius_sides,maprad,pos.z-1,pos.z,false,true);
	get_zsideways(collision_radius_sides,pos.z);
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
	int r0 = y2r(pos.y,mapimg.rows,1);
	int c0 = x2c(pos.x,mapimg.cols,1);
	if(r0 != last_r || c0 != last_c){
		for(int r = fmax(r0 - par_visited_rad,0); r < fmin(r0 + par_visited_rad,mapimg.rows); r++){
			for(int c = fmax(c0 - par_visited_rad,0); c <  fmin(c0 + par_visited_rad,mapimg.cols); c++){
				mapimg.at<cv::Vec3b>(r,c)[0] = int(pos.z);
			}
		}
		last_r = r0; last_c = c0;
	}
}
int main(int argc, char** argv)
{
  ros::init(argc, argv, "tb_polymake_node");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");
  private_nh.getParam("workdir_path", par_workdir);
	private_nh.param("map_sidelength_min",  par_maprad, 150.0);
	private_nh.param("visited_radius",  par_visited_rad, 5.0);
	private_nh.param("lookahead_distance",  par_lookahead_distance, 50.0);
	private_nh.param("lookahead_distance",  par_img_enhance, 7.0);
  tf2_ros::TransformListener tf2_listener(tfBuffer);
	img_enhance = int(round(par_img_enhance));
	ros::Subscriber s1 = nh.subscribe("/octomap_full",1,octomap_callback);
	ros::Rate rate(1);
	ros::Publisher pub = nh.advertise<sensor_msgs::Image>("/tb_bld/mapimg", 1);
	ros::Publisher poly_pub = nh.advertise<geometry_msgs::PolygonStamped>("/tb_bld/poly", 1);
	int count = 0;
	bool done = true;
	bool first = true;
	geometry_msgs::PolygonStamped poly;

  while(ros::ok()){
		checktf();
		if(got_map && done){
			done = false;
			count++;

			if(count == 3){
				count = 0;
				update_edtos_down();
			}
			else
				update_edtos_side();
			if(count == 0 && first){
		//			poly = start_point();
			//poly = get_poly_from_mapimg(lengthedit_ray(pos,closest_obstacle_plane.point,-5),36,36);
			}
		cv_bridge::CvImage cv_image;
			sensor_msgs::Image ros_image;
			cv_image.image = mapimg;
			cv_image.encoding = "bgr8";
			cv_image.toImageMsg(ros_image);
			pub.publish(ros_image);
			done = true;
		}
		poly.header.frame_id = "map";
		poly_pub.publish(poly);
	  rate.sleep();
	  ros::spinOnce();
	}

  return 0;
}
