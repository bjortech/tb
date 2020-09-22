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
cv::Mat img(1000,1000,CV_8UC3,cv::Scalar(0, 0, 0)); //create image, set encoding and size, init pixels to default val
cv::Mat img_blank(1000,1000,CV_8UC3,cv::Scalar(0, 0, 0)); //create image, set encoding and size, init pixels to default val
cv::Mat img_height(1000,1000,CV_8U,cv::Scalar(0)); //create image, set encoding and size, init pixels to default val
cv::Mat img_filled(1000,1000,CV_8U,cv::Scalar(0)); //create image, set encoding and size, init pixels to default val
cv::Mat img_paths(1000,1000,CV_8U,cv::Scalar(0)); //create image, set encoding and size, init pixels to default val
cv::Mat imb_blank_mono(1000,1000,CV_8U,cv::Scalar(0)); //create image, set encoding and size, init pixels to default val
tb_msgsrv::Paths paths_clusters;
tb_msgsrv::Polygons paths_bbpolys;
///////////********CTRL***********//////////
ros::Time process_start;
std::string active_process;
///********FRONTIER*************////////
int count_target_paths= 0;
std::vector<int> blacklist;
geometry_msgs::PolygonStamped poly_side,poly_roi2d,poly_roi;

ros::Publisher pub_get_side,pub_path_to_cluster;
nav_msgs::Path path_getting_processed,path_side,path_starsquare,path_side_full;
ros::Time request_time;
ros::Time path_sent_time;
int side_size;
float dt_side;
int count_colorshift = 0;
int last_i = 0;
float extra_length = 5.0;
bool path_sent,path_clusters_received,path_bbpolys_received;

nav_msgs::Path path_master;
double par_z0,par_z1,par_r;
bool par_only_manual;
geometry_msgs::PolygonStamped poly_master;
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
void start_process(std::string name){
  float dt = (ros::Time::now() - process_start).toSec();
  if(active_process != "")
    ROS_INFO("Process: %s took %.4f sec",active_process.c_str(),dt);
  active_process = name;
  process_start  = ros::Time::now();
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
  return (img_height.rows / 2 - y);
}
float x2c(float x){
  return (x + img_height.cols/2);
}
int r2y(float r){
  return int((img_height.rows / 2 - r));
}
int c2x(float c){
  return int((c - img_height.cols / 2));
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
    img.at<cv::Vec3b>( y2r(pnt.y),x2c(pnt.x) )[0] = color[0];
    img.at<cv::Vec3b>( y2r(pnt.y),x2c(pnt.x) )[1] = color[1];
    img.at<cv::Vec3b>( y2r(pnt.y),x2c(pnt.x) )[2] = color[2];
		geometry_msgs::Point pyaw;
		pyaw.x = pnt.x + 3 * cos(yaw);
		pyaw.y = pnt.y + 2 * sin(yaw);
		cv::line (img,  pnt2cv(pnt), pnt2cv(pyaw),color,1,cv::LINE_8,0);
	}
}
int get_random_i(int nummax){
  srand (time(NULL));
  return rand() % nummax;
}
cv::Scalar get_shifting_color(){
  cv::Scalar color;
  color[count_colorshift] = get_random_i(255);
	count_colorshift++;
	if(count_colorshift >= 3)
		count_colorshift = 0;
  return color;
}
///********DRAW*************////////
///********DRAW*************////////
////////////////**********CLUSTERS**************///////////////////////
////////////////**********CLUSTERS**************///////////////////////


std::vector<int> getinpath_indexes_inrad(nav_msgs::Path pathin,int i_to_check,float radius){
  std::vector<int> vec_out;
  if(pathin.poses.size() == 0){
    ROS_INFO("PathAdmin: pathin is empty");
    return vec_out;
  }
  float yaw0 = tf::getYaw(pathin.poses[i_to_check].pose.orientation);
  for(int i = 0; i < pathin.poses.size(); i++){
    float dist = get_dst3d(pathin.poses[i].pose.position,pathin.poses[i_to_check].pose.position);
    float dyaw = get_shortest(tf::getYaw(pathin.poses[i].pose.orientation),yaw0);
    float dz   = 0;//pathin.poses[i].pose.position.z-pathin.poses[i_to_check].pose.position.z;
    if(dyaw < 0)
      dyaw *= -1;

    if(dist <= radius && dist > 0 && dyaw < M_PI/4 && dz == 0)
      vec_out.push_back(i);
  }
  return vec_out;
}

std::vector<std::vector<int>> getinpath_neighbours(nav_msgs::Path pathin,float radius){
	std::vector<std::vector<int>> neighbours_at_index;
  if(pathin.poses.size() == 0){
    ROS_INFO("PathAdmin: pathin is empty");
    return neighbours_at_index;
  }
  for(int i = 0; i < pathin.poses.size(); i++){
		neighbours_at_index.push_back(getinpath_indexes_inrad(pathin,i,radius));
  //  ROS_INFO("neighbours_at_index[%i]: %i",i,neighbours_at_index[i].size());
  }
  return neighbours_at_index;
}

bool in_vec(std::vector<int> vec,int k){
	if(vec.size() == 0)
		return false;
	//	ROS_INFO("in vec: %i",k);
	for(int i = 0; i < vec.size(); i++){
		if(vec[i] == k)
			return true;
	}
	return false;
}

std::vector<int> add_neighbours_index(std::vector<std::vector<int>> neighbours_at_indexes,std::vector<int> neighbours_in_cluster,std::vector<int> indexes_to_add){
  std::vector<int> new_neighbours;
  for(int k = 0; k < indexes_to_add.size(); k++){
    if(!in_vec(neighbours_in_cluster,indexes_to_add[k])
    && !in_vec(new_neighbours,       indexes_to_add[k]))
      new_neighbours.push_back(indexes_to_add[k]);
    for(int i = 0; i < neighbours_at_indexes[indexes_to_add[k]].size(); i++){
      if(!in_vec(neighbours_in_cluster,neighbours_at_indexes[indexes_to_add[k]][i])
      && !in_vec(new_neighbours,       neighbours_at_indexes[indexes_to_add[k]][i])){
        new_neighbours.push_back(neighbours_at_indexes[indexes_to_add[k]][i]);
      }
    }
  }
//  if(new_neighbours.size() > 2)
  //  ROS_INFO("new neighbours: count %i 0: %i N: %i",new_neighbours.size(),new_neighbours[0],new_neighbours[new_neighbours.size()-1]);
  return new_neighbours;
}

std::vector<int> get_neighbour_cluster(nav_msgs::Path pathin,float radius,int start_index){
  std::vector<std::vector<int>> neighbours_at_index;
  neighbours_at_index = getinpath_neighbours(pathin,radius);
  std::vector<int> neighbours_in_cluster;
  std::vector<int> indexes_to_add;
  indexes_to_add.push_back(start_index);
  while(indexes_to_add.size() > 0){
    for(int i = 0; i < indexes_to_add.size(); i++){
      neighbours_in_cluster.push_back(indexes_to_add[i]);
    }
    indexes_to_add = add_neighbours_index(neighbours_at_index,neighbours_in_cluster,indexes_to_add);
  }
  return neighbours_in_cluster;
}

std::vector<int> update_neighbours_clustered(std::vector<int> neighbours_clustered,std::vector<int> neighbours_in_cluster){
  for(int i = 0; i < neighbours_in_cluster.size(); i++){
    neighbours_clustered.push_back(neighbours_in_cluster[i]);
  }
  return neighbours_clustered;
}
std::vector<int> get_neighbours_not_clustered(std::vector<int> neighbours_clustered,int path_size){
  std::vector<int> not_clustered;
  for(int i = 0; i < path_size; i++){
    if(!in_vec(neighbours_clustered,i))
      not_clustered.push_back(i);
  }
  return not_clustered;
}

std::vector<std::vector<int>> get_neighbour_clusters(nav_msgs::Path pathin,float radius){
  std::vector<int> neighbours_not_clustered;
  std::vector<int> neighbours_in_cluster;
  std::vector<int> neighbours_clustered;
  std::vector<std::vector<int>> neighbour_clusters;
  while(neighbours_clustered.size() < pathin.poses.size()){
    neighbours_not_clustered = get_neighbours_not_clustered(neighbours_clustered,pathin.poses.size());
    neighbours_in_cluster    = get_neighbour_cluster(pathin,radius,neighbours_not_clustered[0]);
    neighbours_clustered     = update_neighbours_clustered(neighbours_clustered,neighbours_in_cluster);
    neighbour_clusters.push_back(neighbours_in_cluster);
  //  ROS_INFO("Neighbours cluster: %i / %i, neighbours_in_cluster: %i, neighbour_clusters: %i",neighbours_clustered.size(),neighbours_not_clustered.size(),neighbours_in_cluster.size(),neighbour_clusters.size());
  }
  return neighbour_clusters;
}
std::vector<nav_msgs::Path> paths_from_clusters(nav_msgs::Path pathin,std::vector<std::vector<int>> clusters_in){
//  ROS_INFO("Clusters_in: %i, pathposes_in: %i",clusters_in.size(),pathin.poses.size());
  std::vector<nav_msgs::Path> path_clusters;
  for(int i = 0; i < clusters_in.size(); i++){
    if(clusters_in[i].size() > 1){
      nav_msgs::Path path_cluster;
      path_cluster.header = pathin.header;
      for(int k = 0; k < clusters_in[i].size(); k++){
        path_cluster.poses.push_back(pathin.poses[clusters_in[i][k]]);
      }
  //    ROS_INFO("Cluster: %i path_size: %i",i,path_cluster.poses.size());
      path_clusters.push_back(path_cluster);
    }
  }
  return path_clusters;
}
////////////////**********CLUSTERS**************///////////////////////

int getinpath_neighbours(nav_msgs::Path pathin,int i0,float radius){
  int count = 0;
  for(int i = 0; i < pathin.poses.size(); i++){
    if(get_dst2d(pathin.poses[i].pose.position,pathin.poses[i0].pose.position) <= radius)
      count++;
  }
  return count;
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

bool in_poly(geometry_msgs::PolygonStamped polyin, float x, float y){
  int cross = 0;
	geometry_msgs::Point point;
	point.x = x;
	point.y = y;
  for(int i = 0,j = polyin.polygon.points.size()-1; i < polyin.polygon.points.size(); j=i++){//  for (int i = 0, j = vertices_.size() - 1; i < vertices_.size(); j = i++) {
    if ( ((polyin.polygon.points[i].y > point.y) != (polyin.polygon.points[j].y > point.y))
           && (point.x < (polyin.polygon.points[j].x - polyin.polygon.points[i].x) * (point.y - polyin.polygon.points[i].y) /
            (polyin.polygon.points[j].y - polyin.polygon.points[i].y) + polyin.polygon.points[i].x) )
    {
      cross++;
    }
  }
  return bool(cross % 2);
}
geometry_msgs::Point get_poly_centroidarea(geometry_msgs::PolygonStamped polyin){
    geometry_msgs::Point centroid;
    double signedArea = 0.0;
    double x0 = 0.0; // Current vertex X
    double y0 = 0.0; // Current vertex Y
    double x1 = 0.0; // Next vertex X
    double y1 = 0.0; // Next vertex Y
    double a = 0.0;  // Partial signed area

    // For all vertices except last
    int i=0;

    if(polyin.polygon.points.size() < 2){
      return centroid;
    }
    for (i=0; i<polyin.polygon.points.size()-1; ++i)
    {
        x0 = polyin.polygon.points[i].x;
        y0 = polyin.polygon.points[i].y;
        x1 = polyin.polygon.points[i+1].x;
        y1 = polyin.polygon.points[i+1].y;
        a = x0*y1 - x1*y0;
        signedArea += a;
        centroid.x += (x0 + x1)*a;
        centroid.y += (y0 + y1)*a;
    }

    // Do last vertex separately to avoid performing an expensive
    // modulus operation in each iteration.
    x0 = polyin.polygon.points[i].x;
    y0 = polyin.polygon.points[i].y;
    x1 = polyin.polygon.points[0].x;
    y1 = polyin.polygon.points[0].y;
    a = x0*y1 - x1*y0;
    signedArea += a;
    centroid.x += (x0 + x1)*a;
    centroid.y += (y0 + y1)*a;

    signedArea *= 0.5;
    centroid.x /= (6.0*signedArea);
    centroid.y /= (6.0*signedArea);
    centroid.z = signedArea;

    return centroid;
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
		if(dst_point_in_path_lim(path_side,pathin.poses[i].pose.position,cutoff))
			pathout.poses.push_back(pathin.poses[i]);
	}
	return pathout;
}

void path_side_cb(const nav_msgs::Path::ConstPtr& msg){
	nav_msgs::Path path_side_new = get_new_path(path_side,*msg,1.0);
	side_size = msg->poses.size();
	dt_side = (ros::Time::now() - request_time).toSec();
	ROS_INFO("dt_side: %.3f side_size: %i ",dt_side,side_size);
	if(path_side_new.poses.size() > 0){
		for(int i = 0; i < path_side_new.poses.size(); i++){
			path_side.poses.push_back(path_side_new.poses[i]);
		}
	}
	ROS_INFO("SIDE IN: %i, new: %i, final: %i",msg->poses.size(),path_side_new.poses.size(),path_side.poses.size());
}

geometry_msgs::PolygonStamped create_poly2d(geometry_msgs::Point pin, float rad){
	geometry_msgs::PolygonStamped poly;
	poly.header = hdr();
	poly.polygon.points.resize(5);
	poly.polygon.points[0].x = round(pin.x + rad);
	poly.polygon.points[0].y = round(pin.y + rad);
	poly.polygon.points[1].x = round(pin.x - rad);
	poly.polygon.points[1].y = round(pin.y + rad);
	poly.polygon.points[2].x = round(pin.x - rad);
	poly.polygon.points[2].y = round(pin.y - rad);
	poly.polygon.points[3].x = round(pin.x + rad);
	poly.polygon.points[3].y = round(pin.y - rad);
	poly.polygon.points[4]   = poly.polygon.points[0];
	return poly;
}
void create_poly_volume(geometry_msgs::Point pin, float sides,float z0,float z1){
	poly_roi2d = create_poly2d(pin,sides/2);
	poly_roi = poly_roi2d;
	for(int i = 0; i < poly_roi.polygon.points.size(); i++){
		poly_roi.polygon.points[i].z = z0;
	}
	poly_roi.polygon.points.push_back(poly_roi.polygon.points[0]);
	poly_roi.polygon.points.push_back(poly_roi.polygon.points[1]);
	poly_roi.polygon.points.push_back(poly_roi.polygon.points[2]);
	poly_roi.polygon.points.push_back(poly_roi.polygon.points[3]);
	poly_roi.polygon.points.push_back(poly_roi.polygon.points[4]);

	for(int i = 5; i < poly_roi.polygon.points.size(); i++){
		poly_roi.polygon.points[i].z = z1;
	}
	for(int i = 0; i < poly_roi.polygon.points.size(); i++){
		if(poly_roi.polygon.points[i].z > z1)
			poly_roi.polygon.points[i].z = z1;
		if(poly_roi.polygon.points[i].z < z0)
			poly_roi.polygon.points[i].z = z0;
	}
}

geometry_msgs::Point get_random_pnt(nav_msgs::Path pathin){
	std::vector<int> possible_i;
	for(int i = 0; i < pathin.poses.size(); i++){
		if(!in_vec(blacklist,i))
			possible_i.push_back(i);
	}
	int iout = get_random_i(possible_i.size());
	int iact = possible_i[iout];
	blacklist.push_back(iact);
	return pathin.poses[iact].pose.position;
}
nav_msgs::Path create_gridpath(float area_sidelength,float radlen_xy){
  float centroid_sides = 2*radlen_xy;
  int num_grids = area_sidelength / centroid_sides;
  nav_msgs::Path pathout;
  pathout.header = hdr();
  int i = 0;
  for (int y = 0; y < num_grids; y++)
  {
    for (int x = 0; x < num_grids; x++)
    {
     geometry_msgs::PoseStamped ps;
     ps.pose.position.x = float(area_sidelength*-0.5 + x * centroid_sides+centroid_sides*0.5);
     ps.pose.position.y = float(area_sidelength*-0.5 + y * centroid_sides+centroid_sides*0.5);
     ps.pose.orientation.w = 1.0;
     ps.header = hdr();
     pathout.poses.push_back(ps);
    }
  }
  return pathout;
}

geometry_msgs::PolygonStamped get_master_poly(tb_msgsrv::Polygons polysin){
	geometry_msgs::PolygonStamped polysout;
	polysout.header = hdr();
	for(int i = 0; i < polysin.polygons.size(); i++){
		for(int k = 0; k < polysin.polygons[i].polygon.points.size(); k++){
			polysout.polygon.points.push_back(polysin.polygons[i].polygon.points[k]);
		}
	}
	return polysout;
}
nav_msgs::Path get_master_path(tb_msgsrv::Paths pathsin){
	nav_msgs::Path pathout;
	pathout.header = hdr();
	for(int i = 0; i < pathsin.paths.size(); i++){
		for(int k = 0; k < pathsin.paths[i].poses.size(); k++){
			pathout.poses.push_back(pathsin.paths[i].poses[k]);
		}
	}
	return pathout;
}
void path_clusters_cb(const tb_msgsrv::Paths::ConstPtr& msg){
	paths_clusters = *msg;
	float dt = (ros::Time::now() - path_sent_time).toSec();
	path_clusters_received = true;
	path_master = get_master_path(*msg);
	ROS_INFO("CLUSTERS(num: %i total: %i) received after %.3f seconds",paths_clusters.paths.size(),path_master.poses.size(),dt);
}
void path_bbpolys_cb(const tb_msgsrv::Polygons::ConstPtr& msg){
	paths_bbpolys = *msg;
	float dt = (ros::Time::now() - path_sent_time).toSec();
	path_bbpolys_received = true;
	get_master_poly(*msg);
	poly_master = get_master_poly(*msg);
	ROS_INFO("BBPOLYS(num: %i total: %i) received after %.3f seconds",paths_bbpolys.polygons.size(),poly_master.polygon.points.size(),dt);
}
void send_path_to_processing(nav_msgs::Path pathin){
	path_getting_processed = pathin;
	path_sent_time = ros::Time::now();
	path_sent = true;
	pub_path_to_cluster.publish(pathin);
}
void get_next_request(bool random_i){
	if(!par_only_manual){
		geometry_msgs::Point p;
		if(random_i)
			p = get_random_pnt(path_starsquare);
		else
			p = path_starsquare.poses[last_i+1].pose.position;
		last_i++;
		p.z = 15;
		float z0 = 3;//get_random_i(0);
		float z1 = 30;// z0 + get_random_i(30);
		float r  = 75;//50+get_random_i(100);
		create_poly_volume(p,r,z0,z1);
		ROS_INFO("NEXT POINT: %.0f %.0f %.0f z0: %.0f z1: %.0f r: %.0f",p.x,p.y,p.z,z0,z1,r);
		request_time = ros::Time::now();
		dt_side = 0;
		side_size = 0;
		pub_get_side.publish(poly_roi);
	}
}
void manual_roi_cb(const geometry_msgs::Point::ConstPtr& msg){
	geometry_msgs::Point p;
	p = *msg;
	create_poly_volume(p,par_r,p.z+par_z0,p.z+par_z1);
	ROS_INFO("NEXT POINT: %.0f %.0f %.0f z0: %.0f z1: %.0f r: %.0f",p.x,p.y,p.z,p.z+par_z0,p.z+par_z1,par_r);
	request_time = ros::Time::now();
	dt_side = 0;
	side_size = 0;
	pub_get_side.publish(poly_roi);
}
void manual_par_cb(const geometry_msgs::Point::ConstPtr& msg){
	par_z0 = msg->x;
	par_z1 = msg->y;
	par_r  = msg->z;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "tb_abmap_pathclustering_side_node");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");
	private_nh.param("par_z0",   par_z0, -5.0);
	private_nh.param("par_z1",   par_z1, 3.0);
	private_nh.param("par_r",    par_r, 30.0);
	private_nh.param("only_manual",   par_only_manual, true);

	pub_path_to_cluster = nh.advertise<nav_msgs::Path>("/tb_process/get_path_side_clusters",10);
	ros::Subscriber os1 = nh.subscribe("/tb_side_manual/pnt",1,manual_roi_cb);
	ros::Subscriber os2 = nh.subscribe("/tb_side_manual/par",1,manual_par_cb);

	ros::Subscriber os3 = nh.subscribe("/tb_process/path_side_clusters",1,path_clusters_cb);
	ros::Subscriber os4 = nh.subscribe("/tb_process/paths_side_bbpolys",1,path_bbpolys_cb);
	ros::Subscriber os = nh.subscribe("/tb_edto/side",1,path_side_cb);
	pub_get_side	  	 = nh.advertise<geometry_msgs::PolygonStamped>("/tb_edto/get_side",10);

	ros::Publisher pub_path_master = nh.advertise<nav_msgs::Path>("/tb_vis/side_path_master",10);
	ros::Publisher pub_poly_master = nh.advertise<geometry_msgs::PolygonStamped>("/tb_vis/side_poly_master",10);
	ros::Publisher pub_path_master_segments = nh.advertise<nav_msgs::Path>("/tb_vis/side_pathsegments",10);
	ros::Publisher pub_poly_master_segments = nh.advertise<geometry_msgs::PolygonStamped>("/tb_vis/side_polysegments",10);


  ros::Rate rate(1.0);
  ros::Time last_radialupdate = ros::Time::now();
	path_starsquare = create_gridpath(200,10);
	bool first = true;
	bool use_random = false;
	int count_segment = 0;
  while(ros::ok()){
    rate.sleep();
    ros::spinOnce();
		if(first){
			first = false;
			get_next_request(use_random);
		}
		if(side_size > 500 && !path_sent){
			send_path_to_processing(path_side);
		}
		else if(path_sent){
			if(path_clusters_received && path_bbpolys_received){
				path_sent = false;
				ROS_INFO("PRocess received!",paths_clusters.paths.size(),paths_bbpolys.polygons.size());
				img_blank.copyTo(img);
				start_process("draw");
				int tot = 0;
				path_clusters_received = false;
				path_bbpolys_received = false;
				for(int i = 0; i < paths_clusters.paths.size(); i++){
					cv::Scalar color = get_shifting_color();
					tot += paths_clusters.paths[i].poses.size();
					int s1 = paths_clusters.paths[i].poses.size();
					int s2 = paths_bbpolys.polygons[i].polygon.points.size();
					ROS_INFO("s1: %i s2 %i",s1,s2);

					draw_path(paths_clusters.paths[i],color);
					draw_poly(paths_bbpolys.polygons[i],color);
				//	poly_master.
				}
				count_target_paths++;
				cv::imwrite("/home/nuc/brain/clusters/down"+std::to_string(count_target_paths)+".png",img);
				start_process("");
				get_next_request(use_random);
			}
		}
		if(count_segment < paths_clusters.paths.size()){
			pub_path_master_segments.publish(paths_clusters.paths[count_segment]);
			pub_poly_master_segments.publish(paths_bbpolys.polygons[count_segment]);
			count_segment++;
		}
		else{
			count_segment = 0;
		}

		pub_path_master.publish(path_master);
		pub_poly_master.publish(poly_master);
	}
	return 0;
}
//
