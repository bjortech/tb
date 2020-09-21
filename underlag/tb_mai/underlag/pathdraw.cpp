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
bool got_map;
nav_msgs::Path path;

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
void visited_cb(const nav_msgs::Path::ConstPtr& msg){
  path = *msg;
}
int main(int argc, char** argv)
{
  ros::init(argc, argv, "tb_pathdrawer_node");
  ros::NodeHandle nh;

  cv::Mat mapimg(1000,1000,CV_8UC3,cv::Scalar(0, 0, 0)); //create image, set encoding and size, init pixels to default val
  //cv::Mat mapimg(1000,1000,CV_8U,cv::Scalar(0)); //create image, set encoding and size, init pixels to default val

  ros::Subscriber s = nh.subscribe("/tb_nav/path_visited",1,visited_cb);
  ros::Publisher pub = nh.advertise<sensor_msgs::Image>("/image", 1);
  ros::Rate rate(1);
  int count = 0;
  ros::Time last_check;
  int zmin,zmax,zrng;
  zmin = 5;
  zmax = 8;

  while (nh.ok())
  {
    if((ros::Time::now() - last_check).toSec() > 5){
      last_check = ros::Time::now();
      zrng = zmax - zmin;
      for(int i = 0; i < path.poses.size(); i++){
        int x = path.poses[i].pose.position.x;
        int y = path.poses[i].pose.position.y;
        int z = path.poses[i].pose.position.z;
        if(z < zmin)zmin = z;
        if(z > zmax)zmax = z;
        int zrel = z - zmin;
        int zpx = 255 * (zrng - zrel) / zrng;
        int r = y2r(y,1000,1); int c = x2c(x,1000,1);

        if(path.poses[i].pose.orientation.y == 0.7071)
          mapimg.at<cv::Vec3b>(r,c)[1] = zpx;
        else{
          mapimg.at<cv::Vec3b>(r,c)[2] += 10;
          if(mapimg.at<cv::Vec3b>(r,c)[0] < z)
            mapimg.at<cv::Vec3b>(r,c)[0] = zpx;
        }
      }
    cv::Mat gray;
    cv::cvtColor( mapimg, gray, cv::COLOR_BGR2GRAY );

    cv::Mat canny_output;
    std::vector<std::vector<cv::Point> > contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::blur( gray, gray, cv::Size(4,4) );

    // detect edges using canny
    cv::Canny(gray,canny_output,50,150,3);
    // find contours
    cv::findContours(canny_output,contours,hierarchy,cv::RETR_TREE,cv::CHAIN_APPROX_SIMPLE,cv::Point(0,0));

    // get the moments
    std::vector<cv::Moments> mu(contours.size());
    for( int i = 0; i<contours.size(); i++ )
    {
      mu[i] = cv::moments( contours[i], false );
    }
  // get the centroid of figures.
    std::vector<cv::Point2f> mc(contours.size());
    for( int i = 0; i<contours.size(); i++)
    {
      mc[i] = cv::Point2f( mu[i].m10/mu[i].m00 , mu[i].m01/mu[i].m00 );
    }
    // draw contours
    cv::Mat drawing(canny_output.size(), CV_8UC3, cv::Scalar(255,255,255));
    for( int i = 0; i<contours.size(); i++ )
    {
      cv::Scalar color = cv::Scalar(167,151,0); // B G R values
      cv::drawContours(drawing, contours, i, color, 2, 8, hierarchy, 0, cv::Point());
      cv::circle(drawing,mc[i], 4, color, -1, 8, 0 );
    }
    //cv::namedWindow( "Contours",cv::WINDOW_AUTOSIZE );
    //cv::imshow( "Contours", drawing );
    cv::imwrite( "/home/nuc/Contours.png", drawing );

    cv_bridge::CvImage cv_image;
    sensor_msgs::Image ros_image;
    cv_image.image = mapimg;
    cv_image.encoding = "rgb8";
    cv_image.toImageMsg(ros_image);
    pub.publish(ros_image);
      //  cv_image_mono.image = mapimg_mono;
      //  cv_image_mono.encoding = "mono8";
    }
    rate.sleep();
    ros::spinOnce();
  }
  return 0;
}
