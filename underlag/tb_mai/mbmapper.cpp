#include <ros/ros.h>
#include <octomap_msgs/conversions.h>
#include <octomap_msgs/Octomap.h>
#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <octomap/OcTreeBase.h>
#include <octomap/octomap_types.h>
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
tf2_ros::Buffer tfBuffer;
bool got_map;
double par_maprad;
std::string par_workdir;
int elevation;

void octomap_callback(const octomap_msgs::Octomap& msg){
  abs_octree=octomap_msgs::fullMsgToMap(msg);
  octree.reset(dynamic_cast<octomap::OcTree*>(abs_octree));
  got_map = true;
}
int main(int argc,char** argv){
  ros::init(argc,argv,"tb_mbmapper_node");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");
  private_nh.param("map_sidelength",      par_maprad, 50.0);
  private_nh.getParam("workdir_path",     par_workdir);

  tf2_ros::TransformListener tf2_listener(tfBuffer);


  ros::Subscriber s5  = nh.subscribe("/octomap_full",1,octomap_callback);
  ros::Publisher pub_map_updates = nh.advertise<map_msgs::OccupancyGridUpdate>("/map_updates",100);
  ros::Publisher pub_map  = nh.advertise<nav_msgs::OccupancyGrid>("/map_static",100);
  ros::Rate rate(10.0);

  geometry_msgs::TransformStamped transformStamped;
  float zmin_rel = 10;
  float zmax_rel = 4;
  int count = 0;
  while(ros::ok()){

    count++;

    try{
      transformStamped = tfBuffer.lookupTransform("map","base_stabilized",
                               ros::Time(0));
    }
    catch (tf2::TransformException &ex) {
      ROS_WARN("%s",ex.what());
      ros::Duration(1.0).sleep();
    }
    int xpos = int(round(transformStamped.transform.translation.x));
    int ypos = int(round(transformStamped.transform.translation.y));
    int zpos = int(round(transformStamped.transform.translation.z));
    //zpos = elevation;
    if(got_map){
      geometry_msgs::Point bbmin_octree,bbmax_octree,bbmin_custom,bbmax_custom;
      octree.get()->getMetricMin(bbmin_octree.x,bbmin_octree.y,bbmin_octree.z);
      octree.get()->getMetricMax(bbmax_octree.x,bbmax_octree.y,bbmax_octree.z);

      bbmin_custom.x = xpos-par_maprad;
      bbmin_custom.y = ypos-par_maprad;

      bbmax_custom.x = xpos+par_maprad;
      bbmax_custom.y = ypos+par_maprad;

      float z0 = zpos - 2;
      float z1 = zpos + 1;
      float zmin_touse = fmax(z0,bbmin_octree.z);
      float zmax_touse = fmin(z1,bbmax_octree.z);

      if(z0 > bbmax_octree.z)
        z0 = bbmax_octree.z-2;

      if(z1 > bbmax_octree.z)
        z1 = bbmax_octree.z;

      if(z1 - z0 < 3)
        z0 = z1 - 2;

      if(z0 < bbmin_octree.z)
        z0 = bbmin_octree.z;

      if(z1 < z0 + 2)
        z1 = z0 +2;

      octomap::point3d boundary_min(fmax(bbmin_custom.x,bbmin_octree.x),fmax(bbmin_custom.y,bbmin_octree.y),zmin_touse);
      octomap::point3d boundary_max(fmin(bbmax_custom.x,bbmax_octree.x),fmin(bbmax_custom.y,bbmax_octree.y),zmax_touse);
//      ROS_INFO("ProxyPath Updating EDTO from min %.0f %.0f %.0f to max %.0f %.0f %.0f",boundary_min.x(),boundary_min.y(),boundary_min.z(),boundary_max.x(),boundary_max.y(),boundary_max.z());
      octomap::OcTreeKey minKey, maxKey, curKey;
      if (!octree.get()->coordToKeyChecked(boundary_min, minKey))
      {
          ROS_ERROR("Could not create OcTree key at %f %f %f", boundary_min.x(), boundary_min.y(), boundary_min.z());
      }
      if (!octree.get()->coordToKeyChecked(boundary_max, maxKey))
      {
          ROS_ERROR("Could not create OcTree key at %f %f %f", boundary_max.x(), boundary_max.y(), boundary_max.z());
      }
        nav_msgs::OccupancyGrid map_static,map;

      map_msgs::OccupancyGridUpdate map_update;
      unsigned i, j;

      map.info.width = 1000;
      map.info.height = 1000;
    //  map_update.width  = maxKey[0] - minKey[0] + 1;
      //map_update.height = maxKey[1] - minKey[1] + 1;
      map_update.width = 1000;
      map_update.height = 1000;

      map.info.resolution = octree.get()->getResolution();

      octomap::point3d octoorigin =   octree.get()->keyToCoord(minKey, octree.get()->getTreeDepth());
      map.info.origin.position.x = octoorigin.x() - octree.get()->getResolution() * 0.5;
      map.info.origin.position.y = octoorigin.y() - octree.get()->getResolution() * 0.5;

      map.info.origin.position.x = -500;
      map.info.origin.position.y = -500;
      map_update.x = 0;
      map_update.y = 0;

      map.info.origin.orientation.x = 0.;
      map.info.origin.orientation.y = 0.;
      map.info.origin.orientation.z = 0.;
      map.info.origin.orientation.w = 1.;

      map_update.header.frame_id = "map";
      map.header.frame_id = "map";

      map.data.resize(map.info.width * map.info.height, -1);
      map_update.data.resize(map_update.width * map_update.height, -1);

      //init with unknown
      for(std::vector<int8_t>::iterator it = map.data.begin(); it != map.data.end(); ++it) {
        *it = -1;
      }


      int missing_x = octoorigin.x() - octree.get()->getResolution() * 0.5 + 500;
      int missing_y = octoorigin.y() - octree.get()->getResolution() * 0.5 + 500;

      for (curKey[1] = minKey[1], j = 0; curKey[1] <= maxKey[1]; ++curKey[1], ++j)
      {
          for (curKey[0] = minKey[0], i = 0; curKey[0] <= maxKey[0]; ++curKey[0], ++i)
          {
              for (curKey[2] = minKey[2]; curKey[2] <= maxKey[2]; ++curKey[2])
              { //iterate over height
                  octomap::OcTreeNode* node = octree.get()->search(curKey);
                  if (node)
                  {
                      octomap::point3d pnt = octree.get()->keyToCoord(curKey);
                      bool occupied = octree.get()->isNodeOccupied(node);
                      //ROS_INFO("j: %i, i: %i, map[%i] xyz %.0f %.0f %.0f",
                      //j,i,(map.info.width *j + i),pnt.x(),pnt.y(),pnt.z());

                      if(occupied) {
                        map.data[map.info.width * (j+missing_y) + missing_x + i] = 100;
                    //   map_update.data[map_update.width * j + i] = 100;
                        map_update.data[map_update.width * (j+missing_y) + missing_x + i] = 100;
                        break;
                      }
                      else {
                        map.data[map.info.width * (j+missing_y) + missing_x + i] = 0;
                        map_update.data[map_update.width * (j+missing_y) + missing_x + i] = 0;
                      //  map_update.data[map_update.width * j + i] = 0;

                      //  map.data[map.info.width * j + i] = 0;
                        //map_static.data[map_static.info.width * j + i] = 0;
                      //  map_update.data[map_update.width * j + i] = 0;
                        //map_update_static.data[map_update_static.width * j + i] = 0;
                      }
                  }
              }
          }
      }
      pub_map.publish(map);
      pub_map_updates.publish(map_update);
    }
    rate.sleep();
    ros::spinOnce();
  }
  return 0;
}
