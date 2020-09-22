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

tf2_ros::Buffer tfBuffer;
cv::Mat img_height_m(1000,1000,CV_8U,cv::Scalar(0)); //create image, set encoding and size, init pixels to default val
cv::Mat img_height_mi(1000,1000,CV_8U,cv::Scalar(0)); //create image, set encoding and size, init pixels to default val
cv::Mat img_filled(1000,1000,CV_8U,cv::Scalar(0)); //create image, set encoding and size, init pixels to default val
ros::Publisher pub_costmapelevation;
geometry_msgs::Point pos,cmd_pos;
int count_target_paths = 0;
geometry_msgs::PointStamped mapelevation;
float par_res = 1.0;

float y2r(float y){
  return (img_height_m.rows / 2 - y / par_res);
}

float x2c(float x){
  return (x / par_res + img_height_m.cols/2);
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

bool request_floodfill_slope(geometry_msgs::Point p0,geometry_msgs::Point p1, int lo, int hi){
  int ffillMode = 1;  int connectivity = 8;  int newMaskVal = 255;
  int flags = connectivity + (newMaskVal << 8) +
       (ffillMode == 1 ? cv::FLOODFILL_FIXED_RANGE : 0);
  cv::Rect ccomp;
  img_height_mi.copyTo(img_filled);
  int area  = cv::floodFill(img_filled,cv::Point(x2c(p0.x),y2r(p0.y)),cv::Scalar(150), &ccomp, cv::Scalar(lo),cv::Scalar(hi), flags);
  if(img_filled.at<uchar>(y2r(p1.y),x2c(p1.x)) == 150)
    return true;
  else
    return false;
}

float get_altbetween_points(geometry_msgs::Point p0,geometry_msgs::Point p1){
  int lo = img_height_mi.at<uchar>(y2r(p0.y),x2c(p0.x));
  int hi = 0;
  while(!request_floodfill_slope(p0,p1,lo,hi) && hi < pos.z+10){
    hi += 1;
  }
  return float(hi+lo);
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
	      if(z > img_height_m.at<uchar>(r,c))
	        img_height_m.at<uchar>(r,c) = z;
	    }
		}
	}
}

void get_elevationbaseline_cb(const geometry_msgs::PointStamped::ConstPtr& msg){
	inflate_monoheight(2);
	update_pos_vlp();
	std_msgs::UInt8 costmapelevation_msg;
	costmapelevation_msg.data = int(round(get_altbetween_points(pos,msg->point)));
	pub_costmapelevation.publish(costmapelevation_msg);
}
void get_elevationpathbaseline_cb(const nav_msgs::Path::ConstPtr& msg){
	inflate_monoheight(2);
	update_pos_vlp();
	std_msgs::UInt8 costmapelevation_msg;
	if(msg->poses.size() >= 2)
		costmapelevation_msg.data = int(round(get_altbetween_points(msg->poses[0].pose.position,msg->poses[msg->poses.size()-1].pose.position)));
	else if(msg->poses.size() == 1)
		costmapelevation_msg.data = int(round(get_altbetween_points(pos,msg->poses[0].pose.position)));
	else
		costmapelevation_msg.data = get_zmax_grid_m(y2r(pos.y),x2c(pos.x),2);
	pub_costmapelevation.publish(costmapelevation_msg);
}
int main(int argc, char** argv)
{
  ros::init(argc, argv, "tb_costmapelevation_node");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");

	tf2_ros::TransformListener tf2_listener(tfBuffer);

	ros::Subscriber s0   = nh.subscribe("/tb_costmap/get_elevation_to_point",10,get_elevationbaseline_cb);
	ros::Subscriber s2   = nh.subscribe("/tb_costmap/get_elevation_in_path",10,get_elevationpathbaseline_cb);
	ros::Subscriber s1   = nh.subscribe("/assembled_cloud2",10,pc2_cb);
	pub_costmapelevation = nh.advertise<std_msgs::UInt8>("/tb_costmap/elevation",100);

	ros::spin();
  return 0;
}
