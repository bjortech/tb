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
#include <tb_msgsrv/PathsStamped.h>
#include <tb_msgsrv/Polygons.h>
#include <tb_msgsrv/PolygonsStamped.h>
tb_msgsrv::PathsStamped paths_clusters;
tb_msgsrv::PolygonsStamped paths_bbpolys,polys_cleared,polys_obstacles,polys_unknown;
tb_msgsrv::PathsStamped paths_cstm,paths_side,paths_down;
tb_msgsrv::PolygonsStamped bbpolys_cstm,bbpolys_side,bbpolys_down;
///////////********CTRL***********//////////
ros::Time process_start,state_internal_change,state_target_change;
std::string active_process;
///********FRONTIER*************////////
geometry_msgs::PolygonStamped poly_roi2d,poly_roi,poly_master,poly_cleared;

ros::Publisher pub_cluster_path,pub_get_side,pub_path_to_cluster,pub_get_down,pub_get_poly,pub_virtual_pose,pub_get_sphere;
nav_msgs::Path path_visited,path_down,path_side,path_starsquare;
float dt_side;
int last_i = 0;
bool path_sent;
geometry_msgs::PoseStamped pose_virtual;
geometry_msgs::PointStamped polygon_centroid;
geometry_msgs::Point poly_midpoint;
std::vector<geometry_msgs::PolygonStamped> polys_side_sent;
std::vector<geometry_msgs::PolygonStamped> polys_down_sent;
std::vector<geometry_msgs::PolygonStamped> polys_poly_sent;
std::string state_internal = "idle";
std::string state_target   = "idle";
geometry_msgs::PointStamped target_active;
std::vector<int> blacklist;
geometry_msgs::PointStamped vision_midpoint;
float z0 = 5.0;
float z1 = 10.0;
float poly_cleared_area,poly_elevation,poly_radius;
double par_maprad;
///********FRONTIER*************////////
std_msgs::Header hdr(){
  std_msgs::Header header;
  header.frame_id = "map";
  header.stamp    = ros::Time::now();
  return header;
}
void set_state_target(std::string newstate){
  if(newstate != state_target){
    float dt = (ros::Time::now() - state_target_change).toSec();
    ROS_INFO("MAINSTATE: targetState: %s -> %s (%.3f seconds in state)",state_target.c_str(),newstate.c_str(),dt);
    state_target_change = ros::Time::now();
    state_target  			 = newstate;
  }
}
void set_state_internal(std::string newstate){
  if(newstate != state_internal){
    float dt = (ros::Time::now() - state_internal_change).toSec();
    ROS_INFO("MAINSTATE: InternalState: %s -> %s (%.3f seconds in state)",state_internal.c_str(),newstate.c_str(),dt);
    state_internal_change = ros::Time::now();
    state_internal  			 = newstate;
  }
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
float get_dst2d32(geometry_msgs::Point32 p1, geometry_msgs::Point32 p2){
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

bool in_vec(std::vector<int> vec,int k){
	if(vec.size() == 0)
		return false;
	//	//ROS_INFO("in vec: %i",k);
	for(int i = 0; i < vec.size(); i++){
		if(vec[i] == k)
			return true;
	}
	return false;
}
int get_random_i(int nummax){
  srand (time(NULL));
  return rand() % nummax;
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
nav_msgs::Path constrain_path_bbpoly(nav_msgs::Path pathin,geometry_msgs::PolygonStamped poly_bb,bool get_in_poly){
  nav_msgs::Path path_inpoly,path_outpoly;
  path_outpoly.header = path_inpoly.header = hdr();
  if(pathin.poses.size() == 0 || poly_bb.polygon.points.size() == 0)
    return pathin;
  for(int i = 0; i < pathin.poses.size(); i++){
    if(in_poly(poly_bb,pathin.poses[i].pose.position.x,pathin.poses[i].pose.position.y))
      path_inpoly.poses.push_back(pathin.poses[i]);
		else
			path_outpoly.poses.push_back(pathin.poses[i]);
  }
	if(get_in_poly)
  	return path_inpoly;
	else
		return path_outpoly;
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

void path_down_cb(const nav_msgs::Path::ConstPtr& msg){
	nav_msgs::Path path_down_new = get_new_path(path_down,*msg,1.0);
	if(path_down_new.poses.size() > 0){
		for(int i = 0; i < path_down_new.poses.size(); i++){
			path_down.poses.push_back(path_down_new.poses[i]);
		}
	}
	ROS_INFO("down IN: %i, new: %i, final: %i",msg->poses.size(),path_down_new.poses.size(),path_down.poses.size());
}

void path_side_cb(const nav_msgs::Path::ConstPtr& msg){
	nav_msgs::Path path_side_new = get_new_path(path_side,*msg,1.0);
	if(path_side_new.poses.size() > 0){
		for(int i = 0; i < path_side_new.poses.size(); i++){
			path_side.poses.push_back(path_side_new.poses[i]);
		}
	}
	ROS_INFO("SIDE IN: %i, new: %i, final: %i",msg->poses.size(),path_side_new.poses.size(),path_side.poses.size());
}
void pathvstd_cb(const nav_msgs::Path::ConstPtr& msg){
	path_visited = *msg;
}

geometry_msgs::PolygonStamped create_poly2d_from_maxvals(float xmn,float ymn,float xmx,float ymx){
	ROS_INFO("Creating poly2d: %.0f %.0f -> %.0f %.0f",xmn,ymn,xmx,ymx);
	geometry_msgs::PolygonStamped poly;
	poly.header = hdr();
	poly.polygon.points.resize(5);
	poly.polygon.points[0].x = poly.polygon.points[3].x = xmx;//round(pin.x + rad);
	poly.polygon.points[1].y = poly.polygon.points[0].y = ymx;//round(pin.y + rad);
	poly.polygon.points[2].x = poly.polygon.points[1].x = xmn;//round(pin.x - rad);
	poly.polygon.points[2].y = poly.polygon.points[3].y = ymn;//round(pin.y - rad);
	poly.polygon.points[4]   = poly.polygon.points[0];
	return poly;
}
void create_poly_volume(geometry_msgs::PolygonStamped polyin,float z0,float z1){
	ROS_INFO("Creating poly3d: z0->z1: %.0f -> %.0f",z0,z1);
	float dz = z1-z0;
	poly_elevation = (z0 + z1)/2;
	if(z0 < 2.0){
		z0 = poly_elevation - 1.5;
		z1 = poly_elevation + 1.5;
	}
	poly_roi2d = polyin;
	poly_roi = poly_roi2d;
	for(int i = 0; i < polyin.polygon.points.size(); i++){
		poly_roi.polygon.points[i].z = z0;
		poly_roi.polygon.points.push_back(poly_roi.polygon.points[i]);
		poly_roi.polygon.points[poly_roi.polygon.points.size()-1].z = z1;
	}
}
geometry_msgs::PolygonStamped create_poly2d(geometry_msgs::Point pin, float rad){
	ROS_INFO("Creating poly2d: %.0f %.0f %.0f rad: %.0f",pin.x,pin.y,pin.z,rad);

	geometry_msgs::PolygonStamped poly;
	poly.header = hdr();
	poly.polygon.points.resize(5);
	poly.polygon.points[0].x = poly.polygon.points[3].x = round(pin.x + rad);
	poly.polygon.points[1].y = poly.polygon.points[0].y = round(pin.y + rad);
	poly.polygon.points[2].x = poly.polygon.points[1].x = round(pin.x - rad);
	poly.polygon.points[2].y = poly.polygon.points[3].y = round(pin.y - rad);
	poly.polygon.points[4]   = poly.polygon.points[0];
	return poly;
}

nav_msgs::Path offset_path(nav_msgs::Path pathin,geometry_msgs::Point offset_pnt){
	ROS_INFO("Offset path: %.0f %.0f %.0f",offset_pnt.x,offset_pnt.y,offset_pnt.z);
	for(int i = 0; i < pathin.poses.size(); i++){
		pathin.poses[i].pose.position.x += offset_pnt.x;
		pathin.poses[i].pose.position.x += offset_pnt.y;
		pathin.poses[i].pose.position.x += offset_pnt.z;
	}
}
nav_msgs::Path create_gridpath(float area_sidelength,float radlen_xy){
	ROS_INFO("Create gridpath: %.0f %.0f",area_sidelength,radlen_xy);
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
nav_msgs::Path create_gridpath_aroundpnt(geometry_msgs::Point midpoint,float radius, float radlen_xy){
	return offset_path(create_gridpath(radius*2,radlen_xy),midpoint);
}
void get_cstm(nav_msgs::Path pathin){
	ROS_INFO("get_cstm");
	paths_cstm.paths.resize(0);
	bbpolys_cstm.polygons.resize(0);
}
void get_side(geometry_msgs::PolygonStamped polyin){
	ROS_INFO("get_side");
	paths_side.paths.resize(0);
	bbpolys_side.polygons.resize(0);
	polys_side_sent.push_back(polyin);
	pub_get_side.publish(polyin);
}
void get_down(geometry_msgs::PolygonStamped polyin){
	ROS_INFO("get_down");
	paths_down.paths.resize(0);
	bbpolys_down.polygons.resize(0);
	polys_down_sent.push_back(polyin);
	pub_get_down.publish(polyin);
}
void get_poly(geometry_msgs::PolygonStamped polyin){
	ROS_INFO("get_poly");
	polys_cleared.polygons.resize(0);
	polys_poly_sent.push_back(polyin);
	pub_get_poly.publish(polyin);
}

tb_msgsrv::PathsStamped remove_paths_poses_visited(tb_msgsrv::PathsStamped paths_to_check, float cutoff){
	tb_msgsrv::PathsStamped pathsout;
	pathsout.paths.resize(paths_to_check.paths.size());
	ROS_INFO("Remove points visited");
	std::vector<geometry_msgs::Point> pointsout;
	for(int i = 0; i < paths_to_check.paths.size(); i++){
		for(int k = 0; k < paths_to_check.paths[i].poses.size(); k++){
			if(dst_point_in_path_lim(path_visited,paths_to_check.paths[i].poses[k].pose.position,cutoff))
				pathsout.paths[i].poses.push_back(paths_to_check.paths[i].poses[k]);
		}
	}
	ROS_INFO("REMOVING VISITED POINTS: poses_visited: %i points_in: %i points_out: %i",path_visited.poses.size(),paths_to_check.paths.size(),pathsout.paths.size());
	return pathsout;
}
void analyze_poly(geometry_msgs::PolygonStamped poly){
	ROS_INFO("analyze_poly");
	geometry_msgs::Point midpoint;
	midpoint 	 = poly_midpoint;
	midpoint.z = poly.polygon.points[0].z;
	geometry_msgs::Point cen = get_poly_centroidarea(poly);
	float area = cen.z;
}

nav_msgs::Path at_edge(nav_msgs::Path pathin, geometry_msgs::PolygonStamped poly,float edge_lim){
	nav_msgs::Path e,n,w,s,d,u;
	if(poly.polygon.points.size() > 7){
		float xmx = poly.polygon.points[3].x;
		float ymx = poly.polygon.points[0].y;
		float xmn = poly.polygon.points[1].x;
		float ymn = poly.polygon.points[3].y;
		float zmn = poly.polygon.points[0].z;
		float zmx = poly.polygon.points[6].z;
		for(int i = 0; i < pathin.poses.size(); i++){
			geometry_msgs::Point pnt = pathin.poses[i].pose.position;
			if(abs(pnt.x - xmn) < edge_lim)
				w.poses.push_back(pathin.poses[i]);
			if(abs(pnt.x - xmx) < edge_lim)
				e.poses.push_back(pathin.poses[i]);
			if(abs(pnt.y - ymx) < edge_lim)
				n.poses.push_back(pathin.poses[i]);
			if(abs(pnt.y - ymn) < edge_lim)
				s.poses.push_back(pathin.poses[i]);
			if(abs(pnt.z - zmx) < edge_lim)
				u.poses.push_back(pathin.poses[i]);
			if(abs(pnt.z - zmn) < edge_lim)
				d.poses.push_back(pathin.poses[i]);
		}
	}
	ROS_INFO("AT EDGES: east: %i north: %i west: %i south: %i, up: %i down: %i",e.poses.size(),n.poses.size(),w.poses.size(),s.poses.size(),u.poses.size(),d.poses.size());
	if(e.poses.size() > w.poses.size() && e.poses.size() > n.poses.size() && e.poses.size() > s.poses.size() && e.poses.size() > u.poses.size() && e.poses.size() > d.poses.size())
		return e;
	else if(w.poses.size() > e.poses.size() && w.poses.size() > n.poses.size() && w.poses.size() > s.poses.size() && w.poses.size() > u.poses.size() && w.poses.size() > d.poses.size())
		return w;
	else if(s.poses.size() > e.poses.size() && s.poses.size() > n.poses.size() && s.poses.size() > w.poses.size() && s.poses.size() > u.poses.size() && s.poses.size() > d.poses.size())
		return s;
	else if(n.poses.size() > e.poses.size() && n.poses.size() > s.poses.size() && n.poses.size() > w.poses.size() && n.poses.size() > u.poses.size() && n.poses.size() > d.poses.size())
		return n;
	else if(u.poses.size() > e.poses.size() && u.poses.size() > n.poses.size() && u.poses.size() > w.poses.size() && u.poses.size() > e.poses.size() && u.poses.size() > d.poses.size())
		return u;
	else if(d.poses.size() > e.poses.size() && d.poses.size() > n.poses.size() && d.poses.size() > w.poses.size() && d.poses.size() > e.poses.size() && d.poses.size() > u.poses.size())
		return d;
	else{
		set_state_internal("idle");
		set_state_target("idle");
		return e;
	}
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
geometry_msgs::Point halfway_point(geometry_msgs::Point target, geometry_msgs::Point actual){
	geometry_msgs::Point pout;
	pout.x = (target.x + actual.x)/2;
	pout.y = (target.y + actual.y)/2;
	pout.z = (target.z + actual.z)/2;
	return pout;
}
void reset_paths(){
	z0 = 0;
	z1 = 5;
	create_poly_volume(create_poly2d(vision_midpoint.point,80),10,20);
	pub_virtual_pose.publish(pose_virtual);
	get_poly(poly_roi);
}
void set_virtual_pose(geometry_msgs::Point new_point){
	float hdng = get_hdng(new_point,pose_virtual.pose.position);
	pose_virtual.pose.position = new_point;
	pose_virtual.pose.orientation = tf::createQuaternionMsgFromYaw(hdng);
	pose_virtual.header = hdr();
	vision_midpoint.point.x = pose_virtual.pose.position.x + par_maprad/2 * cos(hdng);
	vision_midpoint.point.y = pose_virtual.pose.position.y + par_maprad/2 * sin(hdng);
	vision_midpoint.point.z = new_point.z;
	vision_midpoint.header  = hdr();
	ROS_INFO("Setting virtual pose and resetting paths: %.0f %.0f %.0f",new_point.x,new_point.y,new_point.z);
	reset_paths();
}

void analyze_data(tb_msgsrv::PathsStamped paths){
	ROS_INFO("analyze_data - %i clusters",paths.paths.size());
	nav_msgs::Path path_to_pursue;
	paths = remove_paths_poses_visited(paths,5.0);
	for(int i = 0; i < paths.paths.size(); i++){
		nav_msgs::Path path_at_edge = at_edge(paths.paths[i],poly_roi,2.5);
		if(path_at_edge.poses.size() > path_to_pursue.poses.size())
			path_to_pursue = path_at_edge;
	}
	if(path_to_pursue.poses.size() > 0){
		set_virtual_pose(get_ave_pnt(path_to_pursue));
	}
}

void paths_clusters_cb(const tb_msgsrv::PathsStamped::ConstPtr& msg){
	ROS_INFO("paths clusters received");
	if(msg->header.frame_id == "cstm"){
		paths_cstm = *msg;
	}
	if(msg->header.frame_id == "side"){
		paths_side = *msg;
		analyze_data(paths_side);
	}
	if(msg->header.frame_id == "down"){
		paths_down = *msg;
		analyze_data(paths_down);
	}
}
void create_poly(geometry_msgs::Point midpoint,float radius,float z0, float z1){
	ROS_INFO("Create poly: %.0f %.0f %.0f rad: %.0f z0: %.0f z1: %.0f",midpoint.x,midpoint.y,midpoint.z,radius,z0,z1);
	poly_midpoint = midpoint;
	poly_radius   = radius;
	create_poly_volume(create_poly2d(midpoint,radius),z0,z1);
}

void polys_cleared_cb(const tb_msgsrv::PolygonsStamped::ConstPtr& msg){
	ROS_INFO("Polys cleared cb: %i polys",msg->polygons.size());
	polys_cleared     = *msg;
}
void polys_obstacles_cb(const tb_msgsrv::PolygonsStamped::ConstPtr& msg){
	ROS_INFO("Polys obstacles cb: %i polys",msg->polygons.size());
	polys_obstacles    = *msg;
}
void polys_unknown_cb(const tb_msgsrv::PolygonsStamped::ConstPtr& msg){
	ROS_INFO("Polys unknown  cb: %i polys",msg->polygons.size());
	polys_unknown    = *msg;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "tb_edto_client_node");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");
	private_nh.param("map_radius",   par_maprad, 15.0);
	pose_virtual.header  =hdr();
	pose_virtual.pose.orientation.w = 1.0;
	pose_virtual.pose.position.z = 15.0;
	pub_path_to_cluster = nh.advertise<nav_msgs::Path>("/tb_process/get_path_clusters",10);

	ros::Subscriber os2 = nh.subscribe("/tb_process/paths_clusters",1,paths_clusters_cb);
	ros::Subscriber o1s = nh.subscribe("/tb_edto/side",1,path_side_cb);
	ros::Subscriber o2s = nh.subscribe("/tb_edto/down",1,path_down_cb);
	ros::Subscriber o3s = nh.subscribe("/tb_edto/polys_cleared",1,polys_cleared_cb);
	ros::Subscriber o4s = nh.subscribe("/tb_edto/polys_obstacles",1,polys_obstacles_cb);
	ros::Subscriber o5s = nh.subscribe("/tb_edto/polys_unknown",1,polys_unknown_cb);
	ros::Subscriber o7s = nh.subscribe("/tb_world/path_visited",1,pathvstd_cb);

	pub_get_side	   = nh.advertise<geometry_msgs::PolygonStamped>("/tb_edto/get_side",10);
	pub_get_down	   = nh.advertise<geometry_msgs::PolygonStamped>("/tb_edto/get_down",10);
	pub_get_poly 		 = nh.advertise<geometry_msgs::PolygonStamped>("/tb_edto/get_poly",10);
	pub_get_sphere 		 = nh.advertise<geometry_msgs::PolygonStamped>("/tb_edto/get_sphere",10);
	pub_cluster_path = nh.advertise<nav_msgs::Path>("/tb_process/get_path_clusters",10);

	pub_virtual_pose   = nh.advertise<geometry_msgs::PoseStamped>("/tb_autonomy/set_target_pose",10);
  ros::Rate rate(1.0);
	path_starsquare = create_gridpath(200,10);
	ros::Time last_update = ros::Time::now();
	bool first = true;
	reset_paths();
  while(ros::ok()){
    rate.sleep();
    ros::spinOnce();
		//if((paths_down.paths.size() > 0 && paths_side.paths.size() > 0) || (ros::Time::now()-last_update).toSec() > 5.0){
			last_update = ros::Time::now();
			create_poly_volume(create_poly2d(pose_virtual.pose.position,20),z0,z1);
			z0 += 5;
			z1 += 5;
			get_down(poly_roi);
			get_side(poly_roi);
		//}
		if(z1 > 25){
			z0 = 0;
			z1 = 5;
		}
	//	pub_virtual_pose.publish(pose_virtual);
	}
	return 0;
}
//
