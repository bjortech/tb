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

cv::Mat img_height(1000,1000,CV_8U,cv::Scalar(0)); //create image, set encoding and size, init pixels to default val
///********FRONTIER*************////////
double par_res,par_maprad;
ros::Publisher pub_heightcloud;
sensor_msgs::PointCloud heightcloud;
geometry_msgs::Point pos;
bool everyother;
int count_image_name= 0;
tf2_ros::Buffer tfBuffer;
std_msgs::Header hdr(){
  std_msgs::Header header;
  header.frame_id = "map";
  header.stamp    = ros::Time::now();
  return header;
}
float get_hdng(geometry_msgs::Point p1,geometry_msgs::Point p0){
  float dx = p1.x - p0.x;
  float dy = p1.y - p0.y;
  return atan2(dy,dx);
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
void publish_ordered_pointcloud(){
	//TODO: This function can be integrated directly in the below callback. Only thing to check before doing so is that the going from x,y to i is done in right order. Probably doesn't take that long anyway
	ros::Time t0 = ros::Time::now();
	int i = 0;
	for(int y = -img_height.rows/2; y < img_height.rows/2-1; y++){
  	for(int x = -img_height.rows/2; x < img_height.rows/2-1; x++){
			i++;
			heightcloud.points[i].x = x;
			heightcloud.points[i].y = y;
			heightcloud.points[i].z = img_height.at<uchar>(y2r(y),x2c(x));
    }
  }
	float dt = (ros::Time::now()-t0).toSec();
	ROS_INFO("create_and_publish_ordered_pointcloud took %.4f sec",dt);
	heightcloud.header = hdr();
	pub_heightcloud.publish(heightcloud);
}
sensor_msgs::PointCloud get_heights_aroundpoint(geometry_msgs::Point midpoint,int radlen_xy){
	sensor_msgs::PointCloud pc1_out;
	geometry_msgs::Point32 p;
	int i = 0;
	for(int y = midpoint.y - radlen_xy; y < midpoint.y + radlen_xy; y++){
  	for(int x = midpoint.y - radlen_xy; x < midpoint.y + radlen_xy; x++){
			if(img_height.at<uchar>(y2r(y),x2c(x)) > 0){
				p.x = x;
				p.y = y;
				p.z = img_height.at<uchar>(y2r(y),x2c(x));
				pc1_out.points.push_back(p);
			}
    }
  }
	pc1_out.header = hdr();
	return pc1_out;
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
				if(z > img_height.at<uchar>(r,c)){
					//if(img_height.at<uchar>(r,c) == 0){

			//		}
					img_height.at<uchar>(r,c) = z;
					//updated_points.push_b
				}
	    }
		}
	}
	if(everyother){
		everyother = false;
		count_image_name++;
		cv::imwrite("/home/nuc/brain/"+std::to_string(count_image_name)+"heightimage.png",img_height);
	}
	else{
		everyother = true;
		publish_ordered_pointcloud();
	}
}
void update_pos(){
	geometry_msgs::TransformStamped transformStamped;
	try{
		transformStamped = tfBuffer.lookupTransform("map","base_perfect_alt",
														 ros::Time(0));
	}
	catch (tf2::TransformException &ex) {
		ROS_WARN("%s",ex.what());
		ros::Duration(1.0).sleep();
	}
	pos.x = transformStamped.transform.translation.x;
	pos.y = transformStamped.transform.translation.y;
	pos.z = transformStamped.transform.translation.z;
}
int main(int argc, char** argv)
{
  ros::init(argc, argv, "tb_heightmapper_node");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");
	private_nh.param("resolution", par_res, 1.0);
	private_nh.param("area_radius", par_maprad, 1.0);
	tf2_ros::TransformListener tf2_listener(tfBuffer);
	heightcloud.points.resize(1000*1000+2);
	ros::Subscriber s1   								= nh.subscribe("/assembled_cloud2",10,pc2_cb);
	pub_heightcloud	 										= nh.advertise<sensor_msgs::PointCloud>("/tb_heightmapper/heightcloud",10);
	ros::Publisher pub_heightcloud_area = nh.advertise<sensor_msgs::PointCloud>("/tb_heightmapper/heightcloud_around_pos",10);
	ros::Rate rate(5.0);
  while(ros::ok()){
		update_pos();
		sensor_msgs::PointCloud heightcloud_area = get_heights_aroundpoint(pos,100);
		pub_heightcloud_area.publish(heightcloud_area);
		rate.sleep();
		ros::spinOnce();
	}
  return 0;
}
//
