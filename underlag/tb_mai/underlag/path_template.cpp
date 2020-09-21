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

tf2_ros::Buffer tfBuffer;
int koff,ioff;
void koff_cb(const std_msgs::UInt8::ConstPtr& msg){
  koff = msg->data;
}
void ioff_cb(const std_msgs::UInt8::ConstPtr& msg){
  ioff = msg->data;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "tb_pathsphere_node");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");

  tf2_ros::TransformListener tf2_listener(tfBuffer);
  ros::Subscriber ss7 = nh.subscribe("/koff",1,koff_cb);
  ros::Subscriber s = nh.subscribe("/ioff",1,ioff_cb);

  ros::Publisher path_pub = nh.advertise<nav_msgs::Path>("/path", 1);
  nav_msgs::Path path;

  ros::Rate rate(1);
  float sphereradius = 50;
  int len_ray = 10;
  while (nh.ok())
  {
  /*  geometry_msgs::TransformStamped transformStamped;
    try{
      transformStamped = tfBuffer.lookupTransform("map","base_stabilized",
                               ros::Time(0));
    }
    catch (tf2::TransformException &ex) {
      ROS_WARN("%s",ex.what());
      ros::Duration(1.0).sleep();
    }*/
    path.poses.resize(0);
    int num_rays = 16;
    float rads_pr_i = 2*M_PI / num_rays;
    float xpos = 0;
    float ypos = 0;
    float zpos = 0;
    for(int k = koff; k < num_rays/2-koff+1; k++){
      for(int i  = ioff; i < num_rays-ioff+1; i++){
        Eigen::Vector3f pnt1_vec(xpos,ypos,zpos);
        Eigen::Vector3f pnt2_vec(xpos+sin(k*rads_pr_i)*cos(i*rads_pr_i),sin(k*rads_pr_i)*sin(i*rads_pr_i)+ypos,zpos+cos(k*rads_pr_i));
        Eigen::Vector3f cur_vec = pnt1_vec;
        Eigen::Vector3f stride_vec;
        stride_vec = (pnt2_vec - pnt1_vec).normalized();

        geometry_msgs::PoseStamped ps;
        ps.header.frame_id = "map";
        path.poses.push_back(ps);
        ps.pose.orientation.w = 1;
        for(int l = 2; l < len_ray; l++){
          cur_vec = pnt1_vec + stride_vec * l;
          ps.pose.position.x = cur_vec.x();
          ps.pose.position.y = cur_vec.y();
          ps.pose.position.z = cur_vec.z();
          path.poses.push_back(ps);
          ROS_INFO("path: %i",path.poses.size());
        }
      }
    }
    path.header.stamp = ros::Time::now();
    path.header.frame_id = "map";
    path_pub.publish(path);

    rate.sleep();
    ros::spinOnce();
  }
  return 0;
}
