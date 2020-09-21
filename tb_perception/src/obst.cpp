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
#include <std_msgs/String.h>
#include <sensor_msgs/Range.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_listener.h>
#include "tf2_ros/message_filter.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include <nav_msgs/Path.h>
#include <std_msgs/UInt8.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Point32.h>
#include <eigen3/Eigen/Core>
#include <geometry_msgs/Vector3Stamped.h>
using namespace octomap;
using namespace std;

bool got_map;
int mainstate,armstate,xmin,ymin,zmin,xmax,ymax,zmax,range_x,range_y,range_z;
double par_verticalmargin,ll40_alt;

ros::Time last_time;
tf2_ros::Buffer tfBuffer;

shared_ptr<DynamicEDTOctomap> edf_ptr;
AbstractOcTree* abs_octree;
shared_ptr<OcTree> octree;
ros::Publisher range_pub;

std_msgs::Float64 xy_distance,z_distance,arm1_cmd,arm2_cmd_pan,arm2_cmd_tilt;
geometry_msgs::Vector3 rpy_sdunk;
geometry_msgs::Vector3Stamped armstates_vector;
geometry_msgs::Point bbmin,bbmax,bbmin_custom,bbmax_custom,bbmin_octree,bbmax_octree,pos;
geometry_msgs::PointStamped xy_obs,z_obs,sdunk_endpoint;
sensor_msgs::Range rangevertical,range_camera;

float get_dst3d(geometry_msgs::Point p1, geometry_msgs::Point p2){
  return(sqrt(pow(p1.x-p2.x,2)+pow(p1.y-p2.y,2)+pow(p1.z-p2.z,2)));
}
float get_dst2d(geometry_msgs::Point p1, geometry_msgs::Point p2){
  return(sqrt(pow(p1.x-p2.x,2)+pow(p1.y-p2.y,2)));
}
void checktf(){
  geometry_msgs::TransformStamped transformStamped;

  try{
    transformStamped = tfBuffer.lookupTransform("map","base_stabilized",
                             ros::Time(0));
  }
  catch (tf2::TransformException &ex) {
    ROS_WARN("%s",ex.what());
    ros::Duration(1.0).sleep();
  }
  pos.x = transformStamped.transform.translation.x;
  pos.y = transformStamped.transform.translation.y;
  pos.z = transformStamped.transform.translation.z;
//	pos_yaw = tf::getYaw(transformStamped.transform.rotation);
}

void octomap_callback(const octomap_msgs::Octomap& msg){
      got_map = true;
      abs_octree=octomap_msgs::fullMsgToMap(msg);
      octree.reset(dynamic_cast<octomap::OcTree*>(abs_octree));
};
bool update_edto(geometry_msgs::Point midpoint,float collision_radius,float maprad,float z0,float z1,bool unknownAsOccupied){
    octree.get()->getMetricMin(bbmin_octree.x,bbmin_octree.y,bbmin_octree.z);
    octree.get()->getMetricMax(bbmax_octree.x,bbmax_octree.y,bbmax_octree.z);

    bbmin_custom.x = midpoint.x-maprad;
    bbmin_custom.y = midpoint.y-maprad;
    bbmin_custom.z = midpoint.z-z0;

    bbmax_custom.x = midpoint.x+maprad;
    bbmax_custom.y = midpoint.y+maprad;
    bbmax_custom.z = midpoint.z+z1;

		bbmin.x = fmax(bbmin_custom.x,bbmin_octree.x);
		bbmin.y = fmax(bbmin_custom.y,bbmin_octree.y);
		bbmin.z = fmax(bbmin_custom.z,bbmin_octree.z);

		bbmax.x = fmin(bbmax_custom.x,bbmax_octree.x);
		bbmax.y = fmin(bbmax_custom.y,bbmax_octree.y);
		bbmax.z = fmin(bbmax_custom.z,bbmax_octree.z);
    octomap::point3d boundary_min(bbmin.x,bbmin.y,bbmin.z);
    octomap::point3d boundary_max(bbmax.x,bbmax.y,bbmax.z);

    edf_ptr.reset (new DynamicEDTOctomap(collision_radius,octree.get(),
            boundary_min,
            boundary_max,
            unknownAsOccupied));
    edf_ptr.get()->update();
    xmin    = int(round(boundary_min.x()))+1;  ymin    = int(round(boundary_min.y()))+1; zmin    = int(round(boundary_min.z()));
    xmax    = int(round(boundary_max.x()))-1;  ymax    = int(round(boundary_max.y()))-1; zmax    = int(round(boundary_max.z()));
    range_x = xmax - xmin;                     range_y = ymax - ymin;                    range_z = zmax - zmin;
    int vol = range_x*range_y*range_z;

    if(vol <= 0){
			ROS_INFO("FAILED update_edto FAILED: xmin: %i, ymin: %i, zmin: %i, xmax: %i, ymax: %i, zmax: %i, range_x: %i, range_y: %i, range_z: %i ",xmin,ymin,zmin,xmax,ymax,zmax,range_x,range_y,range_z);
			return false;
		}
		ROS_INFO("OBS_EDTO: xmin: %i, ymin: %i, zmin: %i, xmax: %i, ymax: %i, zmax: %i, range_x: %i, range_y: %i, range_z: %i ",xmin,ymin,zmin,xmax,ymax,zmax,range_x,range_y,range_z);

  return true;
}
geometry_msgs::PointStamped pnt3d2pnt(point3d pin){
	geometry_msgs::PointStamped pout;
	pout.header.frame_id = "map";
	pout.header.stamp    = ros::Time::now();
  pout.point.x = pin.x();
  pout.point.y = pin.y();
  pout.point.z = pin.z();
	return pout;
}
geometry_msgs::Point eigen2pnt(Eigen::Vector3f pin){
	geometry_msgs::Point pout;
  pout.x = pin.x();
  pout.y = pin.y();
  pout.z = pin.z();
	return pout;
}
float get_dt(std::string n){
	float dt = (ros::Time::now() - last_time).toSec();
	int millisec = dt * 1000;
	last_time = ros::Time::now();
	if(n != ""){
		ROS_INFO("OBST: %s took %.5f microseconds %i millisec",n.c_str(),dt,millisec);
	}
	return dt;
}

geometry_msgs::PointStamped check_pnt(geometry_msgs::Point pnt,float collision_radius){
	geometry_msgs::PointStamped pout;
	point3d closestObst;
	point3d p(pnt.x,pnt.y,pnt.z);
	float dst;
	if(pnt.x > bbmin.x && pnt.x < bbmax.x
	&& pnt.y > bbmin.y && pnt.y < bbmax.y
	&& pnt.z > bbmin.z && pnt.z < bbmax.z){
		edf_ptr.get()->getDistanceAndClosestObstacle(p, dst, closestObst);
		if(dst > 0 && dst < collision_radius)
			pout = pnt3d2pnt(closestObst);
		else if(dst >= collision_radius)
			pout.header.frame_id = "cleared";
		else
			pout.header.frame_id = "unknown";
		}
	else
		pout.header.frame_id = "out_of_bounds";
	return pout;
}
float get_linelength(geometry_msgs::Point p0,geometry_msgs::Point p1,float collision_radius,float cutoff_distance){
	Eigen::Vector3f pnt1_vec(p0.x,p0.y,p0.z);
	Eigen::Vector3f pnt2_vec(p1.x,p1.y,p1.z);
	Eigen::Vector3f cur_vec = pnt1_vec;
	Eigen::Vector3f stride_vec;
	float tot_length = (pnt2_vec - pnt1_vec).norm();
	stride_vec = (pnt2_vec - pnt1_vec).normalized() * 1;
	float cur_ray_len=0;
	float distance = collision_radius-1;
	while(cur_ray_len < tot_length){
		cur_vec = cur_vec + stride_vec;
		cur_ray_len = (cur_vec-pnt1_vec).norm();
		point3d stridep(cur_vec.x(),cur_vec.y(),cur_vec.z());
		distance = edf_ptr.get()->getDistance(stridep);
		if(distance > 0 && distance < collision_radius){
			if(distance < cutoff_distance)
				break;
			else{
				octomap::point3d closestObst;
				edf_ptr.get()->getDistanceAndClosestObstacle(stridep,distance,closestObst);
				sdunk_endpoint = pnt3d2pnt(closestObst);
			}
		}
	}
	return cur_ray_len;
}
float evaluate_fromto(geometry_msgs::Point p0,geometry_msgs::Point p1,float collision_radius,float cutoff_distance){
	geometry_msgs::Point midpoint;
	midpoint.x = (p0.x + p1.x )/2;
	midpoint.y = (p0.y + p1.y )/2;
	midpoint.z = (p0.z + p1.z )/2;
	float z0 	   = fmin(p0.z,p1.z)-2;
	float z1 	   = fmax(p0.z,p1.z)+2;
	float maprad = get_dst3d(p0,p1) + collision_radius;
	update_edto(midpoint,collision_radius,maprad,z0,z1,false);
	float linelength = get_linelength(p0,p1,collision_radius,cutoff_distance);
	return linelength;
}


void check_z(){
	float xy_rad = 10;
	float z_pos = 0;
	float z_neg = pos.z;
	float collision_rad = pos.z;
	if(update_edto(pos,collision_rad,xy_rad,z_pos,z_neg,false)){
		geometry_msgs::Point pin;
		pin.z = fmin(pos.z-1,bbmax_octree.z-1);
		pin.y = pos.y;
		pin.x = pos.x;
		z_obs = check_pnt(pin,collision_rad);
		z_distance.data = pos.z- z_obs.point.z;
	}
}
void check_xy(){
	float z_pos = 1;
	float z_neg = 1;
	float collision_rad = 20;
	float xy_rad = 20;
	if(update_edto(pos,collision_rad,xy_rad,z_pos,z_neg,false)){
		xy_obs = check_pnt(pos,collision_rad);
		if(xy_obs.header.frame_id == "map")
			xy_distance.data = get_dst2d(xy_obs.point,pos);
	}
}

void get_point_in_front(float max_length,float collision_radius,float cutoff_distance){
	try
	{
		geometry_msgs::PointStamped p0,p1;
		p0.header.frame_id		 = "camera_link";
		p0.point.x 			 			 = max_length;
		p1 = tfBuffer.transform(p0, "map");
		range_camera.range 				= evaluate_fromto(p0.point,p1.point,collision_radius,cutoff_distance);
		range_camera.header.stamp = p1.header.stamp;
		range_pub.publish(range_camera);
	}
	catch (tf2::TransformException &ex)
	{
		ROS_WARN("Failure %s\n", ex.what());
	}
}

void set_arm_cmds(){
	if(armstates_vector.header.frame_id != "override" && (xy_distance.data < 10 && xy_distance.data > 0)){
		arm1_cmd.data = 0;
		arm2_cmd_pan.data  = atan2(xy_obs.point.y - pos.y,xy_obs.point.x - pos.x);
		arm2_cmd_tilt.data = asin((xy_obs.point.z - pos.z)/ xy_distance.data);
	}
	else{
		arm1_cmd.data 		 = armstates_vector.vector.x;
		arm2_cmd_pan.data  = armstates_vector.vector.y;
		arm2_cmd_tilt.data = armstates_vector.vector.z;
	}
}

void armstate_cb(const geometry_msgs::Vector3Stamped::ConstPtr& msg){
	armstates_vector = *msg;
}

/*

std::vector<geometry_msgs::Point> getpointsstamped_limits(std::vector<geometry_msgs::PointStamped> pointsin){
	std::vector<geometry_msgs::Point> l0l1;
	l0l1.resize(2);
	if(pointsin.size() == 0){
    ROS_INFO("l0l1: pointsin is empty");
		return l0l1;
  }
  geometry_msgs::Point bbtotmax,bbtotmin;
  bbtotmax.x = bbtotmax.y = bbtotmax.z = -100;
  bbtotmin.x = bbtotmin.y = bbtotmin.z = 100;
  for(int i = 0; i < pointsin.size(); i++){
    if(pointsin[i].point.x > bbtotmax.x)bbtotmax.x = pointsin[i].point.x;
    if(pointsin[i].point.y > bbtotmax.y)bbtotmax.y = pointsin[i].point.y;
    if(pointsin[i].point.z > bbtotmax.z)bbtotmax.z = pointsin[i].point.z;
    if(pointsin[i].point.x < bbtotmin.x)bbtotmin.x = pointsin[i].point.x;
    if(pointsin[i].point.y < bbtotmin.y)bbtotmin.y = pointsin[i].point.y;
    if(pointsin[i].point.z < bbtotmin.z)bbtotmin.z = pointsin[i].point.z;
  }
  float diag = sqrt(pow(bbtotmax.x-bbtotmin.x,2)+pow(bbtotmax.y-bbtotmin.y,2)+pow(bbtotmax.z-bbtotmin.z,2));
  ROS_INFO("l0l1: LIMITS: diagonal: %.2f,max(%.2f %.2f %.2f) min(%.2f %.2f %.2f)",diag,bbtotmax.x,bbtotmax.y,bbtotmax.z,bbtotmin.x,bbtotmin.y,bbtotmin.z);
	l0l1[0] = bbtotmin;
	l0l1[1] = bbtotmax;
  return l0l1;
}
std::vector<geometry_msgs::Point> getpoints_limits(std::vector<geometry_msgs::Point> pointsin){
	std::vector<geometry_msgs::Point> l0l1;
	l0l1.resize(2);
	if(pointsin.size() == 0){
    ROS_INFO("l0l1: pointsin is empty");
		return l0l1;
  }
  geometry_msgs::Point bbtotmax,bbtotmin;
  bbtotmax.x = bbtotmax.y = bbtotmax.z = -100;
  bbtotmin.x = bbtotmin.y = bbtotmin.z = 100;
  for(int i = 0; i < pointsin.size(); i++){
    if(pointsin[i].x > bbtotmax.x)bbtotmax.x = pointsin[i].x;
    if(pointsin[i].y > bbtotmax.y)bbtotmax.y = pointsin[i].y;
    if(pointsin[i].z > bbtotmax.z)bbtotmax.z = pointsin[i].z;
    if(pointsin[i].x < bbtotmin.x)bbtotmin.x = pointsin[i].x;
    if(pointsin[i].y < bbtotmin.y)bbtotmin.y = pointsin[i].y;
    if(pointsin[i].z < bbtotmin.z)bbtotmin.z = pointsin[i].z;
  }
  float diag = sqrt(pow(bbtotmax.x-bbtotmin.x,2)+pow(bbtotmax.y-bbtotmin.y,2)+pow(bbtotmax.z-bbtotmin.z,2));
  ROS_INFO("l0l1: LIMITS: diagonal: %.2f,max(%.2f %.2f %.2f) min(%.2f %.2f %.2f)",diag,bbtotmax.x,bbtotmax.y,bbtotmax.z,bbtotmin.x,bbtotmin.y,bbtotmin.z);
	l0l1[0] = bbtotmin;
	l0l1[1] = bbtotmax;
  return l0l1;
}

geometry_msgs::PointStamped check_xyz(geometry_msgs::Point xyz,float xy_rad,float z_rad){
	if(update_edto(xyz,xy_rad,xy_rad,z_rad,z_rad,false)){
		return check_pnt(xyz,xy_rad);
	}
}
std::vector<geometry_msgs::PointStamped> check_points_xyz(std::vector<geometry_msgs::Point> points_xyz,float collision_rad,float borders_bounds){
	std::vector<geometry_msgs::PointStamped> pointsout_xyz;
	std::vector<geometry_msgs::Point> l0l1;
	l0l1 = getpoints_limits(points_xyz);
	geometry_msgs::Point lmid;
	lmid.x = (l0l1[0].x + l0l1[1].x)/2;
	lmid.y = (l0l1[0].y + l0l1[1].y)/2;
	lmid.z = (l0l1[0].z + l0l1[1].z)/2;
	float xy_rad = fmax(l0l1[1].x - l0l1[0].x,l0l1[1].y - l0l1[0].y) + 5;
	float z_rad  = fmax(l0l1[1].z - l0l1[0].z,5);
	ROS_INFO("L0: %.0f %0f %.0f -> L1 %.0f %0f %.0f Lmid: %.0f %0f %.0f ",
	l0l1[0].x,l0l1[0].y,l0l1[0].z,
	l0l1[1].x,l0l1[1].y,l0l1[1].z,
	lmid.x,lmid.y,lmid.z);
	if(update_edto(lmid,collision_rad,xy_rad,z_rad,z_rad,false)){
		for(int i = 0; i < points_xyz.size(); i++){
			pointsout_xyz.push_back(check_pnt(points_xyz[i],collision_rad));
		}
	}
	return pointsout_xyz;
}


void display_points(std::vector<geometry_msgs::Point> points){
	for(int i = 0; i < points.size(); i++){
		ROS_INFO("Points_to_display[%i]: %.0f %.0f %.0f",i,points[i].x,points[i].y,points[i].z);
	}
}

void rpysdunk(){
  geometry_msgs::TransformStamped tf;

  try{
    tf = tfBuffer.lookupTransform("base_link","camera_link",
                             ros::Time(0));
  }
  catch (tf2::TransformException &ex) {
    ROS_WARN("%s",ex.what());
    ros::Duration(1.0).sleep();
  }
	tf2::Matrix3x3 q(tf2::Quaternion(tf.transform.rotation.x, tf.transform.rotation.y, tf.transform.rotation.z, tf.transform.rotation.w));
	q.getRPY(rpy_sdunk.x,rpy_sdunk.y,rpy_sdunk.z);
}
void test_sdunk_tf(){
	rpysdunk();
	raycast_arm();
	float z_pr_xy = normalized_raycast.z() / sqrt( pow(normalized_raycast.x(),2 )+pow(normalized_raycast.y(),2) );
	ROS_INFO("NORMALIZED_RAYCAST: %.2f %.2f %.2f",normalized_raycast.x(),normalized_raycast.y(),normalized_raycast.z());
	ROS_INFO("NORMALIZED_RAYCAST: heading: %.2f z_pr_xy: %.2f",atan2(normalized_raycast.y(),normalized_raycast.x()),z_pr_xy);
	ROS_INFO("RPYSDUNK: %.2f %.2f %.2f",rpy_sdunk.x,rpy_sdunk.y,rpy_sdunk.z);
	std::vector<geometry_msgs::PointStamped> points_evaluated_singles,points_evaluated_combined;
	float reset_timer = get_dt("");
	std::vector<geometry_msgs::Point> points_on_line;
	points_on_line = get_points_on_line(35,3);
	display_points(points_on_line);
	float d1 = get_dt("get_points_on_line");
	float collision_rad = 10;
	for(int i = 0; i < points_on_line.size(); i++){
		points_evaluated_singles.push_back(check_xyz(points_on_line[i],collision_rad,collision_rad));
	}
	float d2 = get_dt("evaluate_singles");
	points_evaluated_combined = check_points_xyz(points_on_line,collision_rad,5);
	float d3 = get_dt("evaluate_combined");
	for(int i = 0; i < fmin(points_evaluated_combined.size(),points_evaluated_singles.size()); i++){
		ROS_INFO("Points_evaluated[%i]: single [%s] %.0f %.0f %.0f comb [%s] %.0f %.0f %.0f,",
		i,points_evaluated_singles[i].header.frame_id.c_str(),points_evaluated_singles[i].point.x,points_evaluated_singles[i].point.y,points_evaluated_singles[i].point.z,
		points_evaluated_combined[i].header.frame_id.c_str(),points_evaluated_combined[i].point.x,points_evaluated_combined[i].point.y,points_evaluated_combined[i].point.z);
	}
}
*/

void ll40_cb(const sensor_msgs::Range::ConstPtr& msg){
	if(msg->range > 1){
	  if(msg->range < 35){
	    try
	    {
				geometry_msgs::PointStamped p,p_out;
				p.point.z = -msg->range;
				p.header.frame_id = "base_link";
	      p_out = tfBuffer.transform(p, "base_stabilized");
				rangevertical.range = abs(p_out.point.z);
				rangevertical.header.stamp = p_out.header.stamp;
				rangevertical_pub.publish(rangevertical);
	    }
	    catch (tf2::TransformException &ex)
	    {
	      ROS_WARN("Failure %s\n", ex.what());
	    }
	  }
	}
}

int main(int argc,char** argv){
  ros::init(argc,argv,"tb_obst_node");
  ros::NodeHandle nh;
  tf2_ros::TransformListener tf2_listener(tfBuffer);
  ros::NodeHandle private_nh("~");
  private_nh.param("vertical_safetymargin", par_verticalmargin, 5.0);//*2.0);

	range_camera.radiation_type = 1;
	range_camera.field_of_view = 0.01;
	range_camera.min_range = 1;
	range_camera.max_range = max_length;
	rangevertical = range_camera;
	rangevertical.max_range = 30;
	rangevertical.header.frame_id = "ll40";
	range_camera.header.frame_id = "camera_link";

	ros::Subscriber s3 = nh.subscribe("/octomap_full",1,octomap_callback);
	ros::Subscriber s1 = nh.subscribe("/tb_fsm/armstate",1,armstate_cb);
	ros::Subscriber s2 = nh.subscribe("/range_data",1,ll40_cb);

	ros::Publisher pub_sdunk_endpoint    = nh.advertise<geometry_msgs::PointStamped>("/tb_obst/camera_endpoint",10);
	range_pub 													 = nh.advertise<sensor_msgs::Range>("/tb_obst/camera_range", 100);
	rangevertical_pub 								   = nh.advertise<sensor_msgs::Range>("/tb_obst/rangevertical_ll40", 100);

	ros::Publisher xy_obs_pub   = nh.advertise<geometry_msgs::PointStamped>("/tb_obst/xy_obs",10);
  ros::Publisher z_obs_pub    = nh.advertise<geometry_msgs::PointStamped>("/tb_obst/z_obs",10);

  ros::Publisher z_distance_pub    = nh.advertise<std_msgs::Float64>("/tb_obst/z_distance",10);
  ros::Publisher xy_distance_pub   = nh.advertise<std_msgs::Float64>("/tb_obst/xy_distance",10);

  bool everyother;
  ros::Rate rate(2.0);
  while(ros::ok()){
    rate.sleep();
    ros::spinOnce();
    checktf();
    if(got_map){
			float reset_timer = get_dt("");
      check_z();
			float check_z = get_dt("checking z");
			check_xy();
			float check_xy = get_dt("checking xy");
			ROS_INFO("check_z from min %.0f %.0f %.0f %.0f",z_distance.data,z_obs.point.x,z_obs.point.y,z_obs.point.z);
			//if(z_obs.header.frame_id == "map"){
			z_obs_pub.publish(z_obs);
      z_distance_pub.publish(z_distance);
			//}
			ROS_INFO("check_xy from min %.0f %.0f %.0f %.0f",xy_distance.data,xy_obs.point.x,xy_obs.point.y,xy_obs.point.z);
			//if(xy_obs.header.frame_id == "map"){
			xy_obs_pub.publish(xy_obs);
      xy_distance_pub.publish(xy_distance);
			//}
			get_point_in_front(15 5,3);
			set_arm_cmds();
			pub_sdunk_endpoint.publish(sdunk_endpoint);
			pub_tiltctrl.publish(arm2_cmd_tilt);
			pub_panctrl.publish(arm2_cmd_pan);
			pub_vlptiltctrl.publish(arm1_cmd);
		//	test_sdunk_tf();
   }
	}
  return 0;
}
