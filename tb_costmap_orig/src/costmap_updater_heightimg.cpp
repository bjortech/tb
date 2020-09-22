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


bool par_debug_img;
int count_target_paths = 0;
cv::Mat img(1000,1000,CV_8UC3,cv::Scalar(0, 0, 0)); //create image, set encoding and size, init pixels to default val
cv::Mat img_height(1000,1000,CV_8U,cv::Scalar(0)); //create image, set encoding and size, init pixels to default val
cv::Mat img_blank(1000,1000,CV_8UC3,cv::Scalar(0, 0, 0)); //create image, set encoding and size, init pixels to default val

ros::Time last_inflation;
ros::Publisher pub_map_updates;
float par_res = 1.0;
int mbmap_elevation;
float mbmap_elevation_dz = 1.5;
bool mbmap_elevation_is_set = false;
bool par_use_fixed_costmap_elevation;
float y2r(float y){
  return (img_height.rows / 2 - y / par_res);
}
float x2c(float x){
  return (x / par_res + img_height.cols/2);
}
cv::Point pnt2cv(geometry_msgs::Point pin){
	int c = x2c(pin.x);
	int r = y2r(pin.y);
	return cv::Point(c,r);
}
int get_z(float x, float y, int radlen_xy){
  int zmx = 0;
  for(int c = x2c(x)-radlen_xy; c < x2c(x)+radlen_xy; c++){
    for(int r = y2r(y)-radlen_xy; r < y2r(y)+radlen_xy; r++){
      if(img_height.at<uchar>(r,c) > zmx)
        zmx = img_height.at<uchar>(r,c);
    }
  }
  return zmx;
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
std::vector<int> floats_to_ints(std::vector<float> bbvecin){
	std::vector<int> bbvec_out;
	for(int i = 0; i < bbvecin.size(); i++){
		bbvec_out.push_back(int(round(bbvecin[i])));
	}
	return bbvec_out;
}
void update_mbmap_heightimg(std::vector<float> bbvecin){

	std::vector<int> bbvec = floats_to_ints(bbvecin);
	map_msgs::OccupancyGridUpdate update;
	if(fmin((bbvec[2]-bbvec[0]),(bbvec[3]-bbvec[1])) < 10)
		return;
	update.header.stamp = ros::Time::now();
	update.header.frame_id = "map";
	update.x = 500+bbvec[0];
	update.y = 500+bbvec[1];
	update.width  = bbvec[2]-bbvec[0];
	update.height = bbvec[3]-bbvec[1];
	update.data.resize(update.width * update.height);
	unsigned int i = 0;
	for (int y = bbvec[1]; y < bbvec[3]; y++){
		for (int x = bbvec[0]; x < bbvec[2]; x++){
			if(get_z(x,y,2.0) > mbmap_elevation-1){
				update.data[i++] = 100;
				if(par_debug_img){
					geometry_msgs::Point p; p.x = x; p.y = y;
					cv::circle(img,pnt2cv(p),2,cv::Scalar(50,50,200),1);
				}
			}
			else
				update.data[i++] = 0;
		}
	}
	if(par_debug_img){
		geometry_msgs::Point p0,p1;
		p0.x = bbvec[0];
		p0.y = bbvec[1];
		p1.x = bbvec[2];
		p1.y = bbvec[3];
		cv::rectangle(img, pnt2cv(p0),pnt2cv(p1),cv::Scalar(150,150,250),1,8,0);
		count_target_paths++;
		cv::imwrite("/home/nuc/brain/costmapupdates/heightimg"+std::to_string(count_target_paths)+ ".png",img);
		img_blank.copyTo(img);
	}
	pub_map_updates.publish(update);
}

void set_mbmap_elevation_cb(const std_msgs::UInt8::ConstPtr& msg){
	mbmap_elevation_is_set = true;
	mbmap_elevation 			 = msg->data;
}

void update_mbmap_heightimg_cb(const geometry_msgs::PolygonStamped::ConstPtr& msg){
	std::vector<float> bbvec;
	bbvec.resize(6);
	bbvec[0] = bbvec[1] = bbvec[2] = 100;
	bbvec[3] = bbvec[4] = bbvec[5] = -100;
	for(int i = 0; i < msg->polygon.points.size(); i++){
		if(msg->polygon.points[i].x < bbvec[0])
			bbvec[0] = msg->polygon.points[i].x;
		if(msg->polygon.points[i].y < bbvec[1])
			bbvec[1] = msg->polygon.points[i].y;
		if(msg->polygon.points[i].z > bbvec[2])
			bbvec[2] = msg->polygon.points[i].z;
		if(msg->polygon.points[i].x > bbvec[3])
			bbvec[3] = msg->polygon.points[i].x;
		if(msg->polygon.points[i].y < bbvec[4])
			bbvec[4] = msg->polygon.points[i].y;
		if(msg->polygon.points[i].z > bbvec[5])
			bbvec[5] = msg->polygon.points[i].z;
	}
	mbmap_elevation = (bbvec[2]+bbvec[5])/2;
	update_mbmap_heightimg(bbvec);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "tb_costmapupdateheightimg_node");
  ros::NodeHandle nh;
	ros::NodeHandle private_nh("~");
	private_nh.param("write_debug_img", par_debug_img, true);
	private_nh.param("fixed_costmap_elevation", par_use_fixed_costmap_elevation, false);

	ros::Subscriber s1  = nh.subscribe("/assembled_cloud2",10,pc2_cb);
	ros::Subscriber s2  = nh.subscribe("/tb_costmap/update_heightimg",1,update_mbmap_heightimg_cb);
	ros::Subscriber s3  = nh.subscribe("/tb_costmap/elevation",1,set_mbmap_elevation_cb);
	pub_map_updates   = nh.advertise<map_msgs::OccupancyGridUpdate>("/map_updates",100);
  int count = 0;
	ros::spin();

  return 0;
}
//
