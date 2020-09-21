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
#include <tb_msgsrv/Paths.h>
#include <tb_msgsrv/Polygons.h>
#include <tb_msgsrv/PathsStamped.h>
#include <tb_msgsrv/PolygonsStamped.h>

cv::Mat img(1000,1000,CV_8UC3,cv::Scalar(0, 0, 0)); //create image, set encoding and size, init pixels to default val
cv::Mat img_blank(1000,1000,CV_8UC3,cv::Scalar(0, 0, 0)); //create image, set encoding and size, init pixels to default val
tb_msgsrv::PathsStamped paths_f,paths_2;
int count_target_paths= 0;
std::vector<int> blacklist;
geometry_msgs::PolygonStamped poly_side,poly_roi2d,poly_roi;
ros::Time request_time;
ros::Time path_sent_time;
int count_colorshift = 0;
bool received_clusters_ffill,received_clusters;
int down_size;
float dt_down;
nav_msgs::Path path_raw,path_raw_sent;
float dt2,dtf;
ros::Publisher pub_path_to_cluster;
bool ready = true;
///********FRONTIER*************////////

std_msgs::Header hdr(){
  std_msgs::Header header;
  header.frame_id = "map";
  header.stamp    = ros::Time::now();
  return header;
}
bool sort_dst_pair(const std::tuple<int,float>& a,
               const std::tuple<int,float>& b)
{
    return (std::get<1>(a) < std::get<1>(b));
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
  return (img.rows / 2 - y);
}
float x2c(float x){
  return (x + img.cols/2);
}
int r2y(float r){
  return int((img.rows / 2 - r));
}
int c2x(float c){
  return int((c - img.cols / 2));
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
void draw_poly(geometry_msgs::PolygonStamped polyin,cv::Scalar color){
  for(int i = 1; i < polyin.polygon.points.size(); i++){
		cv::line (img, pnt322cv(polyin.polygon.points[i-1]), pnt322cv(polyin.polygon.points[i]),color,1,cv::LINE_8,0);
  }
	cv::line (img, pnt322cv(polyin.polygon.points[polyin.polygon.points.size()-1]), pnt322cv(polyin.polygon.points[0]),color,1,cv::LINE_8,0);
}
void draw_circle(float x,float y, float size,cv::Scalar color){
	geometry_msgs::Point p1;
	p1.x = x; p1.y = y;
	cv::circle(img,pnt2cv(p1),size,color,1);
}
void draw_rectangle(float x,float y,float size,cv::Scalar color){
	geometry_msgs::Point p1,p2;
	p1.x = x-size;
	p1.y = y-size;
	p2.x = x+size*2;
	p2.y = y+size*2;
	cv::rectangle(img, pnt2cv(p1),pnt2cv(p2),color,1,8,0);
}
void draw_path(nav_msgs::Path pathin,cv::Scalar color){
	for(int i = 0; i < pathin.poses.size(); i++){
		geometry_msgs::Point pnt = pathin.poses[i].pose.position;
		float yaw = tf::getYaw(pathin.poses[i].pose.orientation);
		if(color[0] > 0)
    	img.at<cv::Vec3b>( y2r(pnt.y),x2c(pnt.x) )[0] = color[0];
		if(color[1] > 0)
	    img.at<cv::Vec3b>( y2r(pnt.y),x2c(pnt.x) )[1] = color[1];
		if(color[2] > 0)
	  	img.at<cv::Vec3b>( y2r(pnt.y),x2c(pnt.x) )[2] = color[2];
		geometry_msgs::Point pyaw;
		pyaw.x = pnt.x + 3 * cos(yaw);
		pyaw.y = pnt.y + 2 * sin(yaw);
		cv::line (img,  pnt2cv(pnt), pnt2cv(pyaw),color,1,cv::LINE_8,0);
	}
}
void draw_path2(nav_msgs::Path pathin,cv::Scalar color){
	for(int i = 0; i < pathin.poses.size(); i++){
		geometry_msgs::Point pnt = pathin.poses[i].pose.position;
		float yaw = tf::getYaw(pathin.poses[i].pose.orientation);
		draw_rectangle(pnt.x,pnt.y,3,color);
		geometry_msgs::Point pyaw;
		pyaw.x = pnt.x + 3 * cos(yaw);
		pyaw.y = pnt.y + 3 * sin(yaw);
		cv::line (img,  pnt2cv(pnt), pnt2cv(pyaw),color,1,cv::LINE_8,0);
	}
}
void draw_pathf(nav_msgs::Path pathin,cv::Scalar color){
	for(int i = 0; i < pathin.poses.size(); i++){
		geometry_msgs::Point pnt = pathin.poses[i].pose.position;
		float yaw = tf::getYaw(pathin.poses[i].pose.orientation);
		draw_circle(pnt.x,pnt.y,3,color);
		geometry_msgs::Point pyaw;
		pyaw.x = pnt.x + 3 * cos(yaw);
		pyaw.y = pnt.y + 3 * sin(yaw);
		cv::line (img,  pnt2cv(pnt), pnt2cv(pyaw),color,1,cv::LINE_8,0);
	}
}

void clusters_ffill_cb(const tb_msgsrv::PathsStamped::ConstPtr& msg){
	ROS_INFO("clusters_ffill %i paths",msg->paths.size());
	paths_f = *msg;
	dtf = (ros::Time::now() - path_sent_time).toSec();
	received_clusters_ffill = true;
}
void clusters_cb(const tb_msgsrv::PathsStamped::ConstPtr& msg){
	ROS_INFO("clusters %i paths",msg->paths.size());
	paths_2 = *msg;
	dt2 = (ros::Time::now() - path_sent_time).toSec();
	received_clusters = true;
}

cv::Scalar get_shifting_color(){
  cv::Scalar color;
  color[count_colorshift] = 255;
	count_colorshift++;
	if(count_colorshift >= 3)
		count_colorshift = 0;
  return color;
}
geometry_msgs::Point get_ave_pnt(nav_msgs::Path pathin){
  geometry_msgs::Point pnt;
  for(int i = 0; i < pathin.poses.size(); i++){
    pnt.x += pathin.poses[i].pose.position.x;
    pnt.y += pathin.poses[i].pose.position.y;
    pnt.z += pathin.poses[i].pose.position.z;
  }
  pnt.x /= pathin.poses.size();
  pnt.y /= pathin.poses.size();
  pnt.z /= pathin.poses.size();
  return pnt;
}
std::vector<int> sort_path(nav_msgs::Path pathin){
	std::vector<int> indexes;
  if(pathin.poses.size() <= 1){
		indexes.resize(pathin.poses.size());
		return indexes;
	}
  std::vector<std::tuple<int,float>>i_dst;
	geometry_msgs::Point centroid;
  for(int i = 0; i < pathin.poses.size(); i++){
		i_dst.push_back(std::make_tuple(i,get_hdng(pathin.poses[i].pose.position,centroid)));
	}
  sort(i_dst.begin(),i_dst.end(),sort_dst_pair);
  for(int i = 0; i < i_dst.size(); i++){
	  indexes.push_back(std::get<0>(i_dst[i]));
  }
	return indexes;
}
nav_msgs::Path get_master_path(tb_msgsrv::PathsStamped pathsin){
	nav_msgs::Path pathout;
	pathout.header = hdr();
	for(int i = 0; i < pathsin.paths.size(); i++){
		for(int k = 0; k < pathsin.paths[i].poses.size(); k++){
			pathout.poses.push_back(pathsin.paths[i].poses[k]);
		}
	}
	return pathout;
}
nav_msgs::Path get_ave_path(tb_msgsrv::PathsStamped pathsin){
	nav_msgs::Path pathout;
	pathout.header = hdr();
	pathout.poses.resize(pathsin.paths.size());
	for(int i = 0; i < pathsin.paths.size(); i++){
		pathout.poses[i].pose.position = get_ave_pnt(pathsin.paths[i]);
		pathout.poses[i].header = hdr();
	}
	return pathout;
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
nav_msgs::Path get_new_path(nav_msgs::Path path_base,nav_msgs::Path pathin,float cutoff){
	nav_msgs::Path pathout;
	pathout.header = hdr();
	for(int i = 0; i < pathin.poses.size(); i++){
		if(dst_point_in_path_lim(path_base,pathin.poses[i].pose.position,cutoff))
			pathout.poses.push_back(pathin.poses[i]);
	}
	return pathout;
}

void path_cb(const nav_msgs::Path::ConstPtr& msg){
	nav_msgs::Path path_new = get_new_path(path_raw,*msg,1);
	for(int i = 0; i < path_new.poses.size(); i++){
		path_raw.poses.push_back(path_new.poses[i]);
	}
	ROS_INFO("received: %i poses path_new: %i path_raw: %i",msg->poses.size(),path_new.poses.size(),path_raw.poses.size());
	if(path_raw.poses.size() - path_raw_sent.poses.size() > 100 && ready){
		path_sent_time = ros::Time::now();
		received_clusters_ffill = false;
		received_clusters = false;
		path_raw_sent = path_raw;
		ROS_INFO("Publishing: %i poses RAW",path_raw.poses.size());
		pub_path_to_cluster.publish(path_raw);
	}
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "tb_debug_clusterviz_node");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");

	pub_path_to_cluster = nh.advertise<nav_msgs::Path>("/tb_test/path_raw",10);

	ros::Subscriber s3 = nh.subscribe("/tb_edto/path2d_side",10,path_cb);
//	ros::Subscriber s3 = nh.subscribe("/tb_edto/path_side",10,path_cb);

	ros::Subscriber s7 = nh.subscribe("/tb_path/superpath_down",1,clusters_ffill_cb);
	ros::Subscriber s4 = nh.subscribe("/tb_path/superpath_side",1,clusters_cb);

	ros::Publisher pub_ffill_master = nh.advertise<nav_msgs::Path>("/tb_test/ffill_master",10);
	ros::Publisher pub_clust_master = nh.advertise<nav_msgs::Path>("/tb_test/clust_master",10);
	ros::Publisher pub_ffill_segment = nh.advertise<nav_msgs::Path>("/tb_test/ffill_segment",10);
	ros::Publisher pub_clust_segment = nh.advertise<nav_msgs::Path>("/tb_test/clust_segment",10);

  ros::Rate rate(1.0);
  ros::Time last_radialupdate = ros::Time::now();
	bool first = true;
	bool use_random = false;
	int count_segment = 0;
	cv::Scalar c2 = cv::Scalar(0,200,0);
	cv::Scalar cf = cv::Scalar(0,0,200);
	nav_msgs::Path path_master_f,path_master_2,path_ave_f,path_ave_2;
	ready = true;
  while(ros::ok()){
    rate.sleep();
    ros::spinOnce();
		if(paths_f.paths.size() > 0 && paths_2.paths.size() > 0){

			path_master_f = get_master_path(paths_f);
			path_master_2 = get_master_path(paths_2);

			path_ave_f = get_ave_path(paths_f);
			path_ave_2 = get_ave_path(paths_2);

			std::vector<int> indexes_f = sort_path(path_ave_f);
			std::vector<int> indexes_2 = sort_path(path_ave_2);

			ROS_INFO("FFILL: %i clusters, %i total length, %.4f sec (raw_sent: %i)",path_ave_f.poses.size(),path_master_f.poses.size(),dtf,path_raw_sent.poses.size());
			ROS_INFO("CLUST: %i clusters, %i total length, %.4f sec (raw_sent: %i)",path_ave_2.poses.size(),path_master_2.poses.size(),dt2,path_raw_sent.poses.size());

			int num = 16;
			float rad_pr_i = 2*M_PI/num;
			geometry_msgs::Point p0;

			bool inc2,incf;
			int i2 = 0;
			int iif = 0;
			int ind2,indf;
			float af,a2;
			for(int i = 0; i < num; i++){
				if(i2 < indexes_2.size())
					ind2 = indexes_2[i2];
				if(iif < indexes_f.size())
					indf = indexes_f[iif];
				if(ind2 < path_ave_2.poses.size())
					a2 = get_hdng(path_ave_2.poses[ind2].pose.position,p0);
				if(indf < path_ave_f.poses.size())
					af = get_hdng(path_ave_f.poses[indf].pose.position,p0);

				float a = -M_PI+rad_pr_i * i;
				ROS_INFO("index[#%i]: a: %.2f",i,a);
				if(a > a2){
					i2++;
					if(inc2 && !incf){
						ind2 = indexes_2[i2];
						if(ind2 < paths_2.paths.size()){
							geometry_msgs::Point p2 = path_ave_2.poses[ind2].pose.position;
							ROS_INFO("At angle: %.2f cluster2[%i]: %i poses cen: %.0f %.0f %.0f ",a,ind2,paths_2.paths[ind2].poses.size(),p2.x,p2.y,p2.z);
						}
					}
					else{
						inc2 = true;
					}
				}
				if(a > af){
					iif++;
					if(incf && !inc2){
						indf = indexes_f[iif];
						if(indf < paths_f.paths.size()){
							geometry_msgs::Point pf = path_ave_f.poses[indf].pose.position;
							ROS_INFO("At angle: %.2f clusterf[%i]: %i poses cen: %.0f %.0f %.0f ",a,indf,paths_f.paths[indf].poses.size(),pf.x,pf.y,pf.z);
						}
					}
					else{
						incf = true;
					}
				}
				if(inc2 && incf){
					inc2 = false;
					incf = false;
					if(ind2 < paths_2.paths.size() && indf < paths_f.paths.size()){
						geometry_msgs::Point p2 = path_ave_2.poses[ind2].pose.position;

						geometry_msgs::Point pf = path_ave_f.poses[indf].pose.position;

						ROS_INFO("At angle: %.2f cluster2[%i]: %i poses cen: %.0f %.0f %.0f ",a,ind2,paths_2.paths[ind2].poses.size(),p2.x,p2.y,p2.z);
						ROS_INFO("At angle: %.2f clusterf[%i]: %i poses cen: %.0f %.0f %.0f ",a,indf,paths_f.paths[indf].poses.size(),pf.x,pf.y,pf.z);
						draw_path(paths_2.paths[ind2],c2);
						draw_path(paths_f.paths[indf],cf);
					}
				}
			}
			received_clusters_ffill = false;
			received_clusters = false;
			ready = true;
			count_target_paths++;
		}
		count_segment++;
		draw_path2(path_ave_2,c2);
		draw_pathf(path_ave_f,cf);
		cv::imwrite("/home/nuc/brain/clusters/process"+std::to_string(count_target_paths)+".png",img);


		img_blank.copyTo(img);
		for(int i =0; i < paths_2.paths.size(); i++){
			draw_path(paths_2.paths[i],get_shifting_color());
		}
		cv::imwrite("/home/nuc/brain/clusters/process"+std::to_string(count_target_paths)+"_p2.png",img);
		img_blank.copyTo(img);
		for(int i =0; i < paths_f.paths.size(); i++){
			draw_path(paths_f.paths[i],get_shifting_color());
		}
		cv::imwrite("/home/nuc/brain/clusters/process"+std::to_string(count_target_paths)+"_pf.png",img);

		if(count_segment < paths_2.paths.size()){
			paths_2.paths[count_segment].header = hdr();
			pub_clust_segment.publish(paths_2.paths[count_segment]);
		}
		if(count_segment < paths_f.paths.size()){
			paths_f.paths[count_segment].header = hdr();
			pub_ffill_segment.publish(paths_f.paths[count_segment]);
		}
		if(fmin(paths_2.paths.size(),paths_f.paths.size()) <= count_segment)
			count_segment = 0;

		pub_ffill_master.publish(path_master_f);
		pub_clust_master.publish(path_master_2);
	}
	return 0;
}
//
