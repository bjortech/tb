#include "ros/ros.h"
#include <std_msgs/Float64.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Point.h>
#include <iostream>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/concatenate.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_listener.h>
#include "tf2_ros/message_filter.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "pcl_ros/transforms.h"
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_ros/buffer.h>
#include <tf2/transform_datatypes.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>

bool got_map,got_map_full,par_is_normalized;
sensor_msgs::PointCloud2 octomap;
tf2_ros::Buffer tfBuffer;
sensor_msgs::PointCloud pc1;
geometry_msgs::Vector3 vlp_rpy;
geometry_msgs::Point pos;
double par_res = 1.0;
cv::Mat mapimg;
std::string par_pathstring;
double par_sensor_incl,par_cloud_radius,par_zmax,par_sensor_hdng;
float y2r(float y){
  return (mapimg.rows / 2 - y / par_res);
}
float x2c(float x){
  return (x / par_res + mapimg.cols/2);
}
int r2y(float r){
  return int((mapimg.rows / 2 - r) * par_res);
}
int c2x(float c){
  return int((c - mapimg.cols / 2) * par_res);
}
float get_dst2d(geometry_msgs::Point32 p2, geometry_msgs::Point p1){
  return(sqrt(pow(p1.x-p2.x,2)+pow(p1.y-p2.y,2)));
}
float get_dst3d(geometry_msgs::Point32 p2, geometry_msgs::Point p1){
  return(sqrt(pow(p1.x-p2.x,2)+pow(p1.y-p2.y,2)+pow(p1.z-p2.z,2)));
}
float get_hdng(geometry_msgs::Point32 p1,geometry_msgs::Point p0){
  float dx = p1.x - p0.x;
  float dy = p1.y - p0.y;
  return atan2(dy,dx);
}
double get_shortest(double target_hdng,double actual_hdng){
  double a = target_hdng - actual_hdng;
  if(a > M_PI)a -= M_PI*2;
  else if(a < -M_PI)a += M_PI*2;
  return a;
}
float get_inclination(geometry_msgs::Point32 p2, geometry_msgs::Point p1){
  return atan2(p2.z - p1.z,get_dst2d(p2,p1));
}
void checktf(){
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
}

sensor_msgs::PointCloud2 pc1_to_pc2(sensor_msgs::PointCloud pc1_in){
  sensor_msgs::PointCloud2 pc2_out;
  pc2_out.header = pc1_in.header;
  //pc2_out.data.resize(pc1_in.points.size());
  sensor_msgs::PointCloud2Modifier modifier(pc2_out);
  modifier.setPointCloud2FieldsByString(1,"xyz");
  modifier.resize(pc1_in.points.size());
	sensor_msgs::PointCloud2Iterator<float> iter_x(pc2_out, "x");
	sensor_msgs::PointCloud2Iterator<float> iter_y(pc2_out, "y");
	sensor_msgs::PointCloud2Iterator<float> iter_z(pc2_out, "z");

  for (unsigned int i = 0; i < pc1_in.points.size();
       ++i, ++iter_x, ++iter_y, ++iter_z)
  {
    iter_x[0] = float(pc1_in.points[i].x);
    iter_x[1] = float(pc1_in.points[i].y);
    iter_x[2] = float(pc1_in.points[i].z);
  }
  return pc2_out;
}

sensor_msgs::PointCloud2 transform_pointcloud2(sensor_msgs::PointCloud2 pc2_in,std::string frame){
  geometry_msgs::TransformStamped transformStamped;
  try{
		transformStamped = tfBuffer.lookupTransform("map",pc2_in.header.frame_id,
												ros::Time(0));
		tf2::doTransform(pc2_in, pc2_in, transformStamped);
	}
	catch (tf2::TransformException &ex) {
		ROS_WARN("%s",ex.what());
		ros::Duration(1.0).sleep();
	}
	return pc2_in;
}

sensor_msgs::PointCloud2 get_pc_within(){
	sensor_msgs::PointCloud pc_out;
	pc_out.header.frame_id = "map";
	pc_out.header.stamp = ros::Time::now();
	for(int x = fmax((pos.x - par_cloud_radius),-mapimg.cols/2); x < fmin((pos.x + par_cloud_radius),mapimg.cols/2); x++){
		for(int y = fmax((pos.y - par_cloud_radius),-mapimg.rows/2); y < fmin((pos.y + par_cloud_radius),mapimg.rows/2); y++){
			geometry_msgs::Point32 p_map;
			p_map.x = x;
			p_map.y = y;
			float hdng_shortest = get_shortest(get_hdng(p_map,pos),vlp_rpy.z);
			if(hdng_shortest < par_sensor_hdng && hdng_shortest > -par_sensor_hdng){
				float alt = 0;
				if(par_is_normalized)
					alt = (float(mapimg.at<cv::Vec3b>(y2r(y),x2c(x))[0]) / 255.0)*par_zmax;
				else
					alt = float(mapimg.at<cv::Vec3b>(y2r(y),x2c(x))[0]);
				for(int z = 0; z < alt; z++){
					p_map.z = z;
					float pitch_shortest = get_shortest(get_inclination(p_map,pos),vlp_rpy.y);
					if(pitch_shortest < par_sensor_incl && pitch_shortest > -par_sensor_incl){
						pc_out.points.push_back(p_map);
					}
				}
			}
		}
	}
	return pc1_to_pc2(pc_out);
	//return transform_pointcloud2(pc1_to_pc2(pc_out),"velodyne");
}

int main(int argc, char **argv)
{
  ros::init(argc, argv,"tb_simulate_img2sensor_node");
  ros::NodeHandle nh;
	ros::NodeHandle private_nh("~");
	private_nh.param("sensor_range_of_view", par_cloud_radius, 25.0);
	private_nh.param("sensor_angle_of_view_vertical", par_sensor_incl, M_PI/7);
	private_nh.param("sensor_angle_of_view_horisontal", par_sensor_hdng, M_PI);
	private_nh.param("normalized_image_zmax", par_zmax, 50.0);
	private_nh.param("image_is_normalized", par_is_normalized, true);
	private_nh.getParam("image_path", par_pathstring);//*2.0);
	mapimg = cv::imread(par_pathstring,CV_LOAD_IMAGE_COLOR);
	tf2_ros::TransformListener tf2_listener(tfBuffer);

	ros::Publisher pub2 = nh.advertise<sensor_msgs::PointCloud2> ("/velodyne_points", 1);
	if(mapimg.rows > 0 && mapimg.cols > 0)
		got_map = true;
  ros::Rate rate(1);
	while(ros::ok()){
		rate.sleep();
		ros::spinOnce();
		if(got_map){
			checktf();
      pub2.publish(get_pc_within());
		}
  }
  return 0;
}
