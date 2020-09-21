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

tf2_ros::Buffer tfBuffer;

std::string inspection_type = "idle";
sensor_msgs::LaserScan scan_lo,scan_mi,scan_hi,scan_up,scan_dn,scan_st;

cv::Scalar c_lo,c_dn,c_mi,c_up,c_h,c_hi,c_sz;
geometry_msgs::PointStamped obs_p,obs_r,obs_l,obs_m,obs_b;

ros::Publisher pub_cmdexplore,pub_cmdmb,pub_target_dash,pub_path_best,pub_get_next_path,pub_cmd,pub_cmdpose;
geometry_msgs::PoseStamped pose_down,pose_side,target,base_pose,target_last,target_final,target_dash;
ros::Time activity_change,last_tilt,path_complete_time;
std_msgs::Float64 arm1_tilt_msg;
geometry_msgs::Vector3 vlp_rpy;
geometry_msgs::Point cmd_pos,poly_cleared_centroid,pos,pnt_midpoint,pnt_ref;
nav_msgs::Odometry odom;
nav_msgs::Path scanzone_path,path_ds_full,raw_full,path_raw_side,path_raw_down,path_target_full,path_clear_vlp,path_down_best_in_poly,path_side_best_in_poly,path_full,path_world_visible,path_down_best,path_side_best,path_vlp,path_obs,path_side_full,path_down_full,path_targets,path_visited,path_cleared_full,path_obstacles_full,path_targets_sent,path_side,path_down;
int mainstate;
int counter = 0;
int blankdraw_counter = 0;
int path_targets_i = 0;
int inspection_count = 0;
double par_vlpmaxtilt,par_vlpmintilt,par_vlptiltinterval,par_takeoffaltitude;
bool path_complete,tiltlimit_reached,tilting,side_empty;
cv::Mat img(1000,1000,CV_8UC3,cv::Scalar(0, 0, 0)); //create image, set encoding and size, init pixels to default val
cv::Mat img_super_down(1000,1000,CV_8UC3,cv::Scalar(0, 0, 0)); //create image, set encoding and size, init pixels to default val
cv::Mat img_super_side(1000,1000,CV_8UC3,cv::Scalar(0, 0, 0)); //create image, set encoding and size, init pixels to default val
cv::Mat img_super(1000,1000,CV_8UC3,cv::Scalar(0, 0, 0)); //create image, set encoding and size, init pixels to default val
cv::Mat img_blank(1000,1000,CV_8UC3,cv::Scalar(0, 0, 0)); //create image, set encoding and size, init pixels to default val
float rad2deg = 180.0/M_PI;
float hdng_cutoff = M_PI/8;
bool override = false;
int count_target_paths = 0;
int targets_down_in_poly,targets_side_in_poly;
int dash_stage = 0;
bool got_one,got_one_raw;
float altmax,dst_target,poly_cleared_centroid_area,pos_yaw,scanrange_actual;
std::vector<int> targets_complete;
tb_msgsrv::Paths paths_active_down,down_in_poly;
tb_msgsrv::Paths paths_active_side,side_in_poly;
geometry_msgs::PolygonStamped poly_cleared,target_final_poly;
tb_msgsrv::Paths paths_down,paths_side;
double par_zclearing,par_scanrange;
geometry_msgs::Point scanpoint_actual2,scanpoint_actual,scanpoint_actual_lo,scanpoint_actual_hi;
float target_angle,setpoint_altitude,setpoint_scanrange;
nav_msgs::Path path_lo,path_lo_mi;
nav_msgs::Path path_mi,path_mi_mi;
nav_msgs::Path path_hi,path_hi_mi;
nav_msgs::Path path_up,path_up_mi;
nav_msgs::Path path_dn,path_dn_mi;
nav_msgs::Path path_st,path_st_mi;
nav_msgs::Path path_up_ri,path_up_le,path_mi_ri,path_mi_le,path_hi_ri,path_hi_le,path_lo_ri,path_lo_le,path_dn_ri,path_dn_le,path_st_ri,path_st_le;
bool use_old;
bool use_raw_for_target = true;
int override_tilt = 0;
int override_alt = 0;
int scanzone_size_pnts;
float scanzone_size_area,scanzone_size_ddst,scanzone_size_dalt,scanzone_dst_min,scanzone_dst_max,scanzone_z_min,scanzone_z_max;
geometry_msgs::Point scanzone_centroid,scanzone_avepnt;
geometry_msgs::Point scanzone_bbmin,scanzone_bbmax;
std_msgs::Header hdr(){
  std_msgs::Header header;
  header.frame_id = "map";
  header.stamp    = ros::Time::now();
  return header;
}
double constrainAngle(double x){
  if(x > M_PI)
    return (x - M_PI*2);
  else if(x < -M_PI)
    return (x + M_PI*2);
  else
    return x;
}
float relval(float min,float max,float val){
  return 255*(val - min) / (max - min);
}
bool sort_dst_pair(const std::tuple<int,float>& a,
               const std::tuple<int,float>& b)
{
    return (std::get<1>(a) < std::get<1>(b));
}
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
cv::Point pnt2cv(geometry_msgs::Point pin){
	int c = x2c(pin.x,img.cols,1);
	int r = y2r(pin.y,img.rows,1);
	return cv::Point(c,r);
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
nav_msgs::Path constrain_path_bbpoly(nav_msgs::Path pathin,geometry_msgs::PolygonStamped poly_bb){
  nav_msgs::Path pathout;
  pathout.header = hdr();
  if(pathin.poses.size() == 0 || poly_bb.polygon.points.size() == 0)
    return pathin;
  for(int i = 0; i < pathin.poses.size(); i++){
    if(in_poly(poly_bb,pathin.poses[i].pose.position.x,pathin.poses[i].pose.position.y)){
      pathout.poses.push_back(pathin.poses[i]);
    }
  }
  return pathout;
}
cv::Scalar get_color(int base_blue,int base_green, int base_red){
	cv::Scalar color;
	color[0] = base_blue;
	color[1] = base_green;
	color[2] = base_red;
	return color;
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
nav_msgs::Path get_new_path(nav_msgs::Path path_old,nav_msgs::Path pathin,float cutoff_dst){
  nav_msgs::Path pathout;
  pathout.header = hdr();
  for(int i = 0; i < pathin.poses.size(); i++){
    if(dst_point_in_path_lim(path_old,pathin.poses[i].pose.position,cutoff_dst))
      pathout.poses.push_back(pathin.poses[i]);
  }
  return pathout;
}

nav_msgs::Path remove_old(nav_msgs::Path path_old,geometry_msgs::Point midpoint,float maxrad){
  nav_msgs::Path pathout;
  for(int i = 0; i < path_old.poses.size(); i++){
    if(get_dst2d(midpoint,path_old.poses[i].pose.position) < maxrad){
      pathout.poses.push_back(path_old.poses[i]);
    }
  }
  return pathout;
}
nav_msgs::Path add_new(nav_msgs::Path path_old,nav_msgs::Path pathin){
  for(int i = 0; i < pathin.poses.size(); i++){
      path_old.poses.push_back(pathin.poses[i]);
  }
  return path_old;
}
bool is_within_incline(geometry_msgs::Point pnt,float maxdelta_pitch){
  float inclination       = get_inclination(pos,pnt);
  float delta_inclination = get_shortest(vlp_rpy.y,inclination);
  float pitch_shortest    = get_shortest(atan2(pnt.z-pos.z,get_dst2d(pnt,pos)),vlp_rpy.y);
  ////ROS_INFO("vlp_rpy: %.2f, inclination: %.2f, max_inclination: %.2f, pitch_shortest: %.2f,delta_inclination %.2f",vlp_rpy.y,inclination,maxdelta_pitch,pitch_shortest,delta_inclination);
  if(delta_inclination < maxdelta_pitch && delta_inclination > -maxdelta_pitch)
    return true;
  else
    return false;
}
int getinpath_neighbours(nav_msgs::Path pathin,int i0,float radius){
  int count = 0;
  for(int i = 0; i < pathin.poses.size(); i++){
    if(get_dst2d(pathin.poses[i].pose.position,pathin.poses[i0].pose.position) <= radius)
      count++;
  }
  return count;
}
nav_msgs::Path sort_path(nav_msgs::Path pathin,std::string sort_by){
  nav_msgs::Path pathout;
  if(pathin.poses.size() <= 1)
    return pathin;
  pathout.header = pathin.header;
  std::vector<std::tuple<int,float>>i_dst;
  for(int i = 0; i < pathin.poses.size(); i++){
		if(sort_by == "dst_2d")
      i_dst.push_back(std::make_tuple(i,get_dst2d(pos,pathin.poses[i].pose.position)));
    else if(sort_by == "inclination")
        i_dst.push_back(std::make_tuple(i,get_shortest(vlp_rpy.y,get_inclination(pos,pathin.poses[i].pose.position))));
    else if(sort_by == "dst_3d_ref")
      i_dst.push_back(std::make_tuple(i,get_dst3d(pnt_ref,pathin.poses[i].pose.position)));
    else if(sort_by == "neighbours")
      i_dst.push_back(std::make_tuple(i,getinpath_neighbours(pathin,i,2)));
    else if(sort_by == "dst_3d")
      i_dst.push_back(std::make_tuple(i,get_dst3d(pos,pathin.poses[i].pose.position)));
		else if(sort_by == "hdng_abs")
			i_dst.push_back(std::make_tuple(i,abs(get_shortest(get_hdng(pathin.poses[i].pose.position,pos),pos_yaw) * rad2deg)));
    else if(sort_by == "hdng")
      i_dst.push_back(std::make_tuple(i,get_shortest(get_hdng(pathin.poses[i].pose.position,pos),pos_yaw)));
    else if(sort_by == "zabs")
  		i_dst.push_back(std::make_tuple(i,abs(pathin.poses[i].pose.position.z - pos.z)));
    else if(sort_by == "z")
      i_dst.push_back(std::make_tuple(i,pathin.poses[i].pose.position.z));
	}
      sort(i_dst.begin(),i_dst.end(),sort_dst_pair);
  for(int i = 0; i < i_dst.size(); i++){
    pathout.poses.push_back(pathin.poses[std::get<0>(i_dst[i])]);
  }
	return pathout;
}
void draw_pose(geometry_msgs::Point pnt,float yaw,float len, cv::Scalar color){
  geometry_msgs::Point pyaw;
  pyaw.x = pnt.x + len * cos(yaw);
  pyaw.y = pnt.y + len * sin(yaw);
  cv::circle(img,pnt2cv(pnt),2,color,1);
  cv::line (img, pnt2cv(pnt), pnt2cv(pyaw),color,1,cv::LINE_8,0);
}

float vec_to_ave(std::vector<float> vec_in){
  float vec_sum = 0;
  for(int i = 0; i < vec_in.size(); i++){
    vec_sum += vec_in[i];
  }
  return vec_sum / vec_in.size();
}
float vec_to_max(std::vector<float> vec_in){
  float vec_max = -12310;
  for(int i = 0; i < vec_in.size(); i++){
    if(vec_in[i] > vec_max)
     vec_max = vec_in[i];
  }
  return vec_max;
}
float vec_to_min(std::vector<float> vec_in){
  float vec_min = 133330;
  for(int i = 0; i < vec_in.size(); i++){
    if(vec_in[i] < vec_min)
     vec_min = vec_in[i];
  }
  return vec_min;
}
void draw_path_at_img(nav_msgs::Path pathin,geometry_msgs::Point p0,
	 bool path_line,bool pose_yawline,bool pose_rectangle,bool pose_circle,bool pose_pnt,
  cv::Scalar color, int pose_size)
  {
	geometry_msgs::Point pyaw,prect0,prect1,pnt;
  if(pathin.poses.size() < 1)
    return;
	if(p0.x == 0 && p0.y == 0)
		p0 = pathin.poses[0].pose.position;

	for(int i = 0; i < pathin.poses.size(); i++){
		pnt = pathin.poses[i].pose.position;
		float yaw = tf::getYaw(pathin.poses[i].pose.orientation);
		if(path_line)
			cv::line (img,  pnt2cv(p0), pnt2cv(pnt),color,1,cv::LINE_8,0);
		p0 = pnt;
		if(pose_yawline){
			pyaw.x = pnt.x + pose_size*2 * cos(yaw);
			pyaw.y = pnt.y + pose_size*2 * sin(yaw);
			cv::line (img,  pnt2cv(pnt), pnt2cv(pyaw),color,1,cv::LINE_8,0);
		}
		if(pose_rectangle){
			prect0.x = pnt.x-pose_size;
			prect0.y = pnt.y-pose_size;
			prect1.x = pnt.x+pose_size*2;
			prect1.y = pnt.y+pose_size*2;
			cv::rectangle(img, pnt2cv(prect0),pnt2cv(prect1),color,1,8,0);
		}
		if(pose_circle){
			cv::circle(img,pnt2cv(pnt),pose_size,color,1);
		}
		if(pose_pnt){
			img.at<cv::Vec3b>( y2r(pnt.y,img.rows,1),x2c(pnt.x,img.cols,1) )[0] = color[0];
			img.at<cv::Vec3b>( y2r(pnt.y,img.rows,1),x2c(pnt.x,img.cols,1) )[1] = color[1];
			img.at<cv::Vec3b>( y2r(pnt.y,img.rows,1),x2c(pnt.x,img.cols,1) )[2] = color[2];
		}
	}
}
float get_zmax(nav_msgs::Path pathin){
  float zmx = 0;
  for(int i = 0; i < pathin.poses.size(); i++){
    if(pathin.poses[i].pose.position.z > zmx){
      zmx = pathin.poses[i].pose.position.z;
    }
  }
  return zmx;
}
float get_zmin(nav_msgs::Path pathin){
  float zmn = 1100;
  for(int i = 0; i < pathin.poses.size(); i++){
    if(pathin.poses[i].pose.position.z < zmn){
      zmn = pathin.poses[i].pose.position.z;
    }
  }
  return zmn;
}
cv::Scalar get_shifting_color(int count,int color_intensity){
  cv::Scalar color;
  if(count == 0)
    color[0] = color_intensity;
  else if(count == 1)
    color[1] = color_intensity;
  else
    color[2] = color_intensity;
  return color;
}



geometry_msgs::Point max_score(std::vector<float> scores){
  geometry_msgs::Point maxmin;
  maxmin.x = -100;
  maxmin.y = 100;
  for(int i = 0; i < scores.size(); i++){
    if(maxmin.x < scores[i]){
      maxmin.x = scores[i];
    }
    if(maxmin.y > scores[i]){
      maxmin.y = scores[i];
    }
  }
  return maxmin;
}
float rel_pnt(geometry_msgs::Point maxmin,float score){
  return (255 * (score - maxmin.y) / (maxmin.x - maxmin.y));
}
float rel_score(std::vector<float> scores, float score){
  geometry_msgs::Point maxmin = max_score(scores);
  return rel_pnt(maxmin,score);
}

cv::Scalar rel_color(std::vector<float> scores, float score, std::string rgb){
  cv::Scalar c;
  int relscore = int(rel_score(scores,score));
  if(rgb == "r")
    c[2] = relscore;
  if(rgb == "g")
    c[1] = relscore;
  if(rgb == "b")
    c[0] = relscore;
  return c;
 }

 std::vector<int> getinpath_indexes_simple(nav_msgs::Path pathin,geometry_msgs::Point pnt_to_check,float radius){
   std::vector<int> vec_out;
   if(pathin.poses.size() == 0){
     //ROS_INFO("PathAdmin: pathin is empty");
     return vec_out;
   }
   for(int i = 0; i < pathin.poses.size(); i++){
     if(get_dst2d(pathin.poses[i].pose.position,pnt_to_check) <= radius)
       vec_out.push_back(i);
   }
   return vec_out;
 }


 std::vector<float> get_zvals(nav_msgs::Path pathin,std::vector<int> indexes){
   std::vector<float> zvals;
   zvals.resize(indexes.size());
    for(int i = 0; i < indexes.size(); i++){
     zvals[i] =  pathin.poses[indexes[i]].pose.position.z;
   }
   return zvals;
 }
 std::vector<float> get_vec_attribute_indexes(nav_msgs::Path pathin,std::vector<float> vec_attribute,std::vector<int> indexes){
   std::vector<float> vals;
   vals.resize(indexes.size());
    for(int i = 0; i < indexes.size(); i++){
     vals[i] = vec_attribute[indexes[i]];
   }
   return vals;
 }
 std::vector<int> getinpath_neighbours_point(nav_msgs::Path pathin,geometry_msgs::Point pnt,float radius){
   int count = 0;
   std::vector<int> vec;
   for(int i = 0; i < pathin.poses.size(); i++){
     if(get_dst2d(pathin.poses[i].pose.position,pnt) <= radius)
       vec.push_back(i);
   }
   return vec;
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
 std::vector<geometry_msgs::Point> getinpath_boundingbox(nav_msgs::Path pathin){
   std::vector<geometry_msgs::Point> bbmnbbmx;

   if(pathin.poses.size() == 0){
     ROS_INFO("GENERICNODE: pathin is empty");
     return bbmnbbmx;
   }
   geometry_msgs::Point bbtotmax,bbtotmin;
   bbtotmax.x = bbtotmax.y = bbtotmax.z = -100;
   bbtotmin.x = bbtotmin.y = bbtotmin.z = 100;
   for(int i = 0; i < pathin.poses.size(); i++){
     if(pathin.poses[i].pose.position.x > bbtotmax.x)bbtotmax.x = pathin.poses[i].pose.position.x;
     if(pathin.poses[i].pose.position.y > bbtotmax.y)bbtotmax.y = pathin.poses[i].pose.position.y;
     if(pathin.poses[i].pose.position.z > bbtotmax.z)bbtotmax.z = pathin.poses[i].pose.position.z;
     if(pathin.poses[i].pose.position.x < bbtotmin.x)bbtotmin.x = pathin.poses[i].pose.position.x;
     if(pathin.poses[i].pose.position.y < bbtotmin.y)bbtotmin.y = pathin.poses[i].pose.position.y;
     if(pathin.poses[i].pose.position.z < bbtotmin.z)bbtotmin.z = pathin.poses[i].pose.position.z;
   }
   bbmnbbmx.push_back(bbtotmin);
   bbmnbbmx.push_back(bbtotmax);

   float diag = sqrt(pow(bbtotmax.x-bbtotmin.x,2)+pow(bbtotmax.y-bbtotmin.y,2)+pow(bbtotmax.z-bbtotmin.z,2));
   ROS_INFO("GENERICNODE: LIMITS: diagonal: %.2f,max(%.2f %.2f %.2f) min(%.2f %.2f %.2f)",diag,bbtotmax.x,bbtotmax.y,bbtotmax.z,bbtotmin.x,bbtotmin.y,bbtotmin.z);
   return bbmnbbmx;
 }
 std::vector<float> get_vec_attribute(nav_msgs::Path pathin,std::string type){
   std::vector<float> vec_out;
   for(int i = 0; i < pathin.poses.size(); i++){
     float val = 0;
     if(type == "inclination_abs")
       val = get_inclination(pos,pathin.poses[i].pose.position);
     else if(type == "dst_2d")
       val = get_dst2d(pos,pathin.poses[i].pose.position);
     else if(type == "neighbours")
       val = getinpath_neighbours(pathin,i,3);
     else if(type == "inclination")
       val = get_inclination(pathin.poses[i].pose.position,pos);
     else if(type == "dst_3d_ref")
       val = get_dst3d(pnt_ref,pathin.poses[i].pose.position);
     else if(type == "dst_3d")
       val = get_dst3d(pos,pathin.poses[i].pose.position);
 		else if(type == "hdng_abs")
 			val = abs(get_shortest(get_hdng(pathin.poses[i].pose.position,pos),vlp_rpy.z) * rad2deg);
    else if(type == "hdng_rel")
      val = get_shortest(get_hdng(pathin.poses[i].pose.position,pos),vlp_rpy.z);
    else if(type == "hdng")
       val = get_hdng(pathin.poses[i].pose.position,pos);
     else if(type == "zabs")
   		val = abs(pathin.poses[i].pose.position.z - pos.z);
     else if(type == "zrel")
     	val = pathin.poses[i].pose.position.z - pos.z;
     else if(type == "z")
       val = pathin.poses[i].pose.position.z;
     vec_out.push_back(val);
   }
   return vec_out;
 }
 float get_elevation_below2(){
   std::vector<int> neighbours = getinpath_neighbours_point(path_full,pos,10);
   nav_msgs::Path path_n;
   std::vector<float> zvals;

   for(int i= 0; i < neighbours.size(); i++){
     zvals.push_back(path_full.poses[neighbours[i]].pose.position.z);
   }
   float z_ave = vec_to_ave(zvals);
   float z_max = vec_to_max(zvals);
   float z_min = vec_to_max(zvals);
   return z_max;
 }
 void draw_background(nav_msgs::Path pathin){
  bool draw_density,draw_z_ave;
  float zmin = get_zmin(pathin);
  float zmax = get_zmax(pathin);
  std::vector<float> vec_z = get_vec_attribute(pathin,"z");
  for(int x = 0; x < 100; x++){
    for(int y = 0; y < 100; y++){
      geometry_msgs::Point p;
      p.x = pos.x - 50 + x;
      p.y = pos.y - 50 + y;
      std::vector<int> neighbours = getinpath_neighbours_point(pathin,p,2);
      std::vector<float> zvals    = get_vec_attribute_indexes(pathin,vec_z,neighbours);

      float z_ave = vec_to_ave(zvals);
      float z_max = vec_to_max(zvals);
      float density = neighbours.size();
      cv::Scalar color;
      color[0] = 100 * neighbours.size() / 50;
      color[1] = 100 * (z_ave - zmin) / (zmax - zmin);
      //ROS_INFO("Neighbours: %i ", neighbours.size());
    //  color[2] = 100 * (z_max - zmin) / (zmax - zmin);
      img.at<cv::Vec3b>( y2r(p.y,img.rows,1),x2c(p.x,img.cols,1) )[0] = color[0];
      img.at<cv::Vec3b>( y2r(p.y,img.rows,1),x2c(p.x,img.cols,1) )[1] = color[1];
      img.at<cv::Vec3b>( y2r(p.y,img.rows,1),x2c(p.x,img.cols,1) )[2] = color[2];
    }
  }
}
void draw_path_by_score(nav_msgs::Path pathin,std::vector<float> score,int primary_color,int secondary_color,int tertiary_color,float color_intensity){
  geometry_msgs::Point maxmin;
  maxmin.x = vec_to_max(score);
  maxmin.y = vec_to_min(score);
	geometry_msgs::Point pyaw,prect0,prect1,pnt;
  if(pathin.poses.size() < 1)
    return;
	for(int i = 0; i < pathin.poses.size(); i++){
		pnt = pathin.poses[i].pose.position;
    cv::Scalar color;
    float rel_score = color_intensity * (score[i] - maxmin.y) / (maxmin.x - maxmin.y);
    if(rel_score > 0.9)
      color[tertiary_color] = 255*rel_score;
    if(rel_score > 0.75)
      color[secondary_color] = 255*rel_score;
    if(rel_score > 0.3)
      color[primary_color] = 255*rel_score;
    if(rel_score < 0.3)
      color[primary_color] = 100*rel_score;

    if(pathin.poses.size() > 25){
      img.at<cv::Vec3b>( y2r(pnt.y,img.rows,1),x2c(pnt.x,img.cols,1) )[0] = color[0];
      img.at<cv::Vec3b>( y2r(pnt.y,img.rows,1),x2c(pnt.x,img.cols,1) )[1] = color[1];
      img.at<cv::Vec3b>( y2r(pnt.y,img.rows,1),x2c(pnt.x,img.cols,1) )[2] = color[2];
    }
    else{
      float yaw = tf::getYaw(pathin.poses[i].pose.orientation);
      pyaw.x = pnt.x + 3 * cos(yaw);
      pyaw.y = pnt.y + 3 * sin(yaw);
      cv::line (img,  pnt2cv(pnt), pnt2cv(pyaw),color,1,cv::LINE_8,0);
      cv::circle(img,pnt2cv(pnt),2,color,1);
    }
	}
}



bool pose_is_side(geometry_msgs::PoseStamped pose){
  if(pose.pose.orientation.y == 0.7071)
    return false;
  else
    return true;
}


void draw_clusterpaths_shiftingcolor(tb_msgsrv::Paths clusters_in,bool path_line,bool pose_yawline,bool pose_rectangle,bool pose_circle,bool pose_pnt,int pose_size,int color_intensity,bool pos_is_p0){
  geometry_msgs::Point p0;
  if(pos_is_p0)
    p0 = pos;
  int count = 0;
  for(int i = 0; i < clusters_in.paths.size(); i++){
    count++;
    if(count > 2)
      count = 0;
    draw_path_at_img(clusters_in.paths[i],p0,path_line,pose_yawline,pose_rectangle,pose_circle,pose_pnt,get_shifting_color(count,color_intensity),1);
  }
}

void draw_vlp_view(){
  img_blank.copyTo(img);
  geometry_msgs::Point pnt_vel,pnt_vel_up,pnt_vel_down,pnt_vel_mid,pnt_path;
  pnt_vel.x = 0;
  pnt_vel.y = pos.z;
  pnt_vel_mid.x  = pnt_vel.x + 30 * cos(vlp_rpy.y);
  pnt_vel_mid.y  = pnt_vel.y + 30 * sin(vlp_rpy.y);
  pnt_vel_up.x   = pnt_vel.x + 30 * cos(vlp_rpy.y+M_PI/12);
  pnt_vel_up.y   = pnt_vel.y + 30 * sin(vlp_rpy.y+M_PI/12);
  pnt_vel_down.x = pnt_vel.x + 30 * cos(vlp_rpy.y-M_PI/12);
  pnt_vel_down.y = pnt_vel.y + 30 * sin(vlp_rpy.y-M_PI/12);
  cv::line(img, pnt2cv(pnt_vel),pnt2cv(pnt_vel_mid),get_color(255,255,255),1,8,0);
  cv::line(img, pnt2cv(pnt_vel),pnt2cv(pnt_vel_up),get_color(0,200,0),1,8,0);
  cv::line(img, pnt2cv(pnt_vel),pnt2cv(pnt_vel_down),get_color(0,0,200),1,8,0);
  cv::circle(img,pnt2cv(pnt_vel_mid),3,get_color(255,255,255),1);
  cv::circle(img,pnt2cv(pnt_vel_up),3,get_color(0,200,0),1);
  cv::circle(img,pnt2cv(pnt_vel_down),3,get_color(0,0,200),1);
  cv::imwrite("/home/nuc/brain/draw/control/"+std::to_string(count_target_paths)+"_vlpview.png",img);
}
void draw_poly(geometry_msgs::PolygonStamped polyin,cv::Scalar color){
  for(int i = 1; i < polyin.polygon.points.size(); i++){
    geometry_msgs::Point p1,p2;
    p1.x = polyin.polygon.points[i-1].x;
    p1.y = polyin.polygon.points[i-1].y;
    p2.x = polyin.polygon.points[i].x;
    p2.y = polyin.polygon.points[i].y;
    cv::line (img, pnt2cv(p1), pnt2cv(p2),color,1,cv::LINE_8,0);
  }
}

void superpathdown_cb(const tb_msgsrv::Paths::ConstPtr& msg){
  paths_down = *msg;
}
void superpathside_cb(const tb_msgsrv::Paths::ConstPtr& msg){
  paths_side = *msg;
}
void superpath_cb(const tb_msgsrv::Paths::ConstPtr& msg){
	///(*msg,"full");
}
void closest_pos_cb(const geometry_msgs::PointStamped::ConstPtr& msg){
  obs_p =*msg;
}
void closest_left_cb(const geometry_msgs::PointStamped::ConstPtr& msg){
  obs_l =*msg;
}
void closest_right_cb(const geometry_msgs::PointStamped::ConstPtr& msg){
  obs_r =*msg;
}
void closest_mid_cb(const geometry_msgs::PointStamped::ConstPtr& msg){
  obs_m =*msg;
}
void closest_blw_cb(const geometry_msgs::PointStamped::ConstPtr& msg){
  obs_b =*msg;
}
void hdngclear_cb(const nav_msgs::Path::ConstPtr& msg){
  path_clear_vlp = *msg;
}


geometry_msgs::PolygonStamped get_polyfinal(float l0,float l1,float delta_hdng){
  geometry_msgs::PolygonStamped poly;
  poly.header.frame_id = "map";
  int num_rays    = 16;
  poly.polygon.points.resize(num_rays*2);
  float rads_pr_i = delta_hdng/num_rays;
  float hdng0     = get_hdng(target.pose.position,target_last.pose.position) - delta_hdng/2;
  for(int i = 0; i < num_rays; i++){
    float a = hdng0 + rads_pr_i * i;
    poly.polygon.points[i].x = target.pose.position.x + l0 * cos(a);
    poly.polygon.points[i].y = target.pose.position.y + l0 * sin(a);
    poly.polygon.points[poly.polygon.points.size()-1-i].x = target.pose.position.x + l1 * cos(a);
    poly.polygon.points[poly.polygon.points.size()-1-i].y = target.pose.position.y + l1 * sin(a);
  }
  return poly;
}
void set_target_pose(geometry_msgs::PoseStamped new_target_pose){
	target_last = target;
  if(new_target_pose.pose.position.x != 0){
  	target.pose.position = new_target_pose.pose.position;
  	target.pose.orientation = tf::createQuaternionMsgFromYaw(get_hdng(new_target_pose.pose.position,target_last.pose.position));
  	cmd_pos = new_target_pose.pose.position;
  	target_final_poly = get_polyfinal(1,30,M_PI);
    new_target_pose = target;
    new_target_pose.pose.position.z = 0;
    new_target_pose.header = hdr();
    //pub_cmdpose.publish(new_target_pose);
    if(!override)
      pub_cmd.publish(new_target_pose.pose.position);
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

geometry_msgs::Point get_ave_hdng(nav_msgs::Path pathin){
  geometry_msgs::Point pnt;
	std::vector<float> hdngs;
	std::vector<float> dst2d;
	std::vector<float> z;
  for(int i = 0; i < pathin.poses.size(); i++){
		hdngs.push_back(get_shortest(get_hdng(pathin.poses[i].pose.position,pos),pos_yaw));
		dst2d.push_back(get_dst2d(pathin.poses[i].pose.position,pos));
		z.push_back(pathin.poses[i].pose.position.z);
  }
	float ave_hdngs = vec_to_ave(hdngs);
	float ave_dst2d = vec_to_ave(dst2d);
	float ave_z = 	vec_to_ave(z);
	geometry_msgs::Point ave_pnt;
	ave_pnt.x = pos.x + ave_dst2d*cos(ave_hdngs);
	ave_pnt.y = pos.y + ave_dst2d*sin(ave_hdngs);
	ave_pnt.z = ave_z;
	ROS_INFO("Pathin[%i],ave_hdngs: %.2f dst. %.0f z: %.0f",pathin.poses.size(),ave_hdngs,ave_dst2d,ave_z);
  return ave_pnt;
}
geometry_msgs::Point summarize_paths(nav_msgs::Path pathin,cv::Scalar color){
	geometry_msgs::Point ave_xyz,ave_ra;
	if(pathin.poses.size() == 0)
		return ave_xyz;
	ave_xyz = get_ave_pnt(pathin);
	ave_ra  = get_ave_hdng(pathin);

	float dstxyz  = get_dst2d(ave_xyz,pos);
	float hdngxyz = get_hdng(ave_xyz,pos);
	ROS_INFO("XYZ_AVE  ave_hdngs: %.2f dst. %.0f z: %.0f",hdngxyz,dstxyz,ave_xyz.z);
	ROS_INFO("XYZ_AVE: [%.0f %.0f %.0f]",ave_xyz.x,ave_xyz.y,ave_xyz.z);
	ROS_INFO("HDNGDST: [%.0f %.0f %.0f]",ave_ra.x,ave_ra.y,ave_ra.z);
	cv::line (img, pnt2cv(pos), pnt2cv(ave_xyz),color,1,cv::LINE_8,0);
	cv::line (img, pnt2cv(pos), pnt2cv(ave_ra),color,1,cv::LINE_8,0);
	return ave_xyz;
}

void draw(std::string type){
	cv::circle(img,pnt2cv(cmd_pos),2,get_color(255,255,200),1);
	cv::circle(img,pnt2cv(pos),2,get_color(255,255,0),1);
	cv::line (img, pnt2cv(pos), pnt2cv(cmd_pos),get_color(0,0,255),1,cv::LINE_8,0);
	draw_poly(target_final_poly,get_color(0,0,255));
	cv::imwrite("/home/nuc/brain/draw/"+std::to_string(count_target_paths)+type+".png",img);
	img_blank.copyTo(img);
}
nav_msgs::Path cutoff_below(nav_msgs::Path pathin,int percentage){
  nav_msgs::Path pathout;
  pathout.header = hdr();
  int percent_length = int(round(pathin.poses.size() * percentage / 100));
  for(int i = 0; i < pathin.poses.size(); i++){
    pathout.poses.push_back(pathin.poses[i]);
  }
  //ROS_INFO("Percent: %i, pose_in: %i, percent_len: %i, path_out: %i",percentage,pathin.poses.size(),percent_length,pathout.poses.size());
  return pathout;
}

void draw_path_by_attribute(nav_msgs::Path pathin,std::string type,bool special){
  img_blank.copyTo(img);
  cv::circle(img,pnt2cv(cmd_pos),2,get_color(255,255,200),1);
  cv::circle(img,pnt2cv(pos),2,get_color(255,255,0),1);
  cv::line (img, pnt2cv(pos), pnt2cv(cmd_pos),get_color(0,0,255),1,cv::LINE_8,0);
  draw_path_by_score(pathin,get_vec_attribute(pathin,type),0,1,1,1.0);
  if(special)
    cv::imwrite("/home/nuc/brain/draw/" + std::to_string(count_target_paths)+"_"+type + "n.png",img);
  else
    cv::imwrite("/home/nuc/brain/draw/" + std::to_string(count_target_paths)+"_"+type + "0.png",img);
}
void draw_pose(geometry_msgs::PoseStamped ps,float rel_val, bool included){
  geometry_msgs::Point pnt,pyaw;
  pnt = ps.pose.position;
  cv::Scalar color;

  if(included){
    color[0] = rel_val * 200;
    color[1] = rel_val * 200;
    color[2] = rel_val * 200;
  }
  else{
    color[1] = rel_val * 100;
  }
  if(ps.pose.orientation.y == 0.7071){
    img.at<cv::Vec3b>( y2r(pnt.y,img.rows,1),x2c(pnt.x,img.cols,1) )[0] = color[0];
    img.at<cv::Vec3b>( y2r(pnt.y,img.rows,1),x2c(pnt.x,img.cols,1) )[1] = color[1];
    img.at<cv::Vec3b>( y2r(pnt.y,img.rows,1),x2c(pnt.x,img.cols,1) )[2] = color[2];
  }
  else{
    float yaw = tf::getYaw(ps.pose.orientation);
    pyaw.x = pnt.x + 3 * cos(yaw);
    pyaw.y = pnt.y + 3 * sin(yaw);
    cv::line (img,  pnt2cv(pnt), pnt2cv(pyaw),color,1,cv::LINE_8,0);
    cv::circle(img,pnt2cv(pnt),1,color,1);
  }
}
nav_msgs::Path cutoff_top(nav_msgs::Path pathin,std::string type,int percent_start,int percent_end){
  nav_msgs::Path pathout;
  pathout.header = hdr();
  pathin                  = sort_path(pathin,type);
  std::vector<float> vals = get_vec_attribute(pathin,type);
  float vmx = vec_to_max(vals);
  float vmn = vec_to_min(vals);
  float ave = vec_to_ave(vals);
  float vd = vmx - vmn;
  std::vector<float> valsout;
  float v0 = float(percent_start/100.0) * (vmx - vmn) + vmn;
  float vN = float(percent_end/100.0) * (vmx - vmn) + vmn;
  for(int i = 0; i < pathin.poses.size(); i++){
    float rel_val = (vals[i] - vmn) / vd;
    if(vals[i] >= v0 && vals[i] <= vN){
      pathout.poses.push_back(pathin.poses[i]);
      valsout.push_back(vals[i]);
    }
  }
  float pp = float(pathout.poses.size()) / float(pathin.poses.size());
  int percent_poses = pp * 100;
  ROS_INFO("Cutoff_percent %s , [min->max: ave] [%.2f->%.2f: %.2f] in->out: [%i->%i](%i percent)",type.c_str(),vmn,vmx,ave,pathin.poses.size(),pathout.poses.size(),percent_poses);
  vmx = vec_to_max(valsout);
  vmn = vec_to_min(valsout);
  ave = vec_to_ave(valsout);
  ROS_INFO("Cutoff_percent %s,  [min->max: ave] [%.2f->%.2f: %.2f] (v0 %.0f-> vN %.0f)",type.c_str(),vmn,vmx,ave,v0,vN);
  return pathout;
}
nav_msgs::Path cutoff_outliers(nav_msgs::Path pathin,std::string type){
  nav_msgs::Path pathout;
  pathout.header = hdr();
  pathin                  = sort_path(pathin,type);
  std::vector<float> vals = get_vec_attribute(pathin,type);
  float vmx = vec_to_max(vals);
  float vmn = vec_to_min(vals);
  float ave = vec_to_ave(vals);
  float vd = vmx - vmn;
  float dv_ave = vd / float(pathin.poses.size());
  bool start_done = false;
  bool end_done = false;
  std::vector<float> valsout;
  std::queue<float> que;


  int que_size = 5;
  float que_ave = 0;
  int last_i_start = 0;
  float que_sum = 0;
  int last_i_end = pathin.poses.size();
  for(int i = 0; i < pathin.poses.size(); i++){
    float pp          = float(i) / float(pathin.poses.size());
    float rel_val     = (vals[i] - vmn) / vd;
    float val = vals[i] - vmn;

    int percent_poses = pp * 100;
    int percent_val   = rel_val * 100;
    float ratio = rel_val/(pp+0.0000001);
    que_sum += ratio;
    que.push(ratio);
    if(que.size() > que_size){
      que_sum -= que.front();
      que_ave = que_sum / que_size;
      que.pop();
    }
    //ROS_INFO("Current: [%i/%i] val: %.2f/%.2f rel_val: %.2f  percent_poses: %i percent_val: %i ratio: %.2f que_ave: %.2f size: %i",i,pathin.poses.size(),val,vd,rel_val,percent_poses,percent_val,ratio,que_ave,que_size);

    if(que_ave > 2 && rel_val < 0.3){
      last_i_start = i;
      ROS_INFO("LAST_I: [%i/%i] val: %.2f/%.2f ratio: %.2f",i,pathin.poses.size(),val,vd,ratio);
    }
    if(que_ave > 2 && rel_val > 0.8 && last_i_end > i){
      last_i_end = i;
      ROS_INFO("LAST_I_END: [%i/%i] val: %.2f/%.2f ratio: %.2f",i,pathin.poses.size(),val,vd,ratio);
    }
  }
  for(int i = last_i_start; i < last_i_end; i++){
      valsout.push_back(vals[i]);
      pathout.poses.push_back(pathin.poses[i]);
  }

  float pp = float(pathout.poses.size()) / float(pathin.poses.size());
  int percent_poses = pp * 100;
  ROS_INFO("CUTOFF_OUTLIERS %s , [min->max: ave] [%.2f->%.2f: %.2f] in->out: [%i->%i](%i percent)",type.c_str(),vmn,vmx,ave,pathin.poses.size(),pathout.poses.size(),percent_poses);
  vmx = vec_to_max(valsout);
  vmn = vec_to_min(valsout);
  ave = vec_to_ave(valsout);
  ROS_INFO("CUTOFF_OUTLIERS %s,  [min->max: ave] [%.2f->%.2f: %.2f]",type.c_str(),vmn,vmx,ave);
  return pathout;
}
nav_msgs::Path cutoff_percentage(nav_msgs::Path pathin,int percentage){
  nav_msgs::Path pathout;
  pathout.header = hdr();
  int percent_length = int(round(pathin.poses.size() * percentage / 100));
  for(int i = percent_length; i < pathin.poses.size(); i++){
    pathout.poses.push_back(pathin.poses[i]);
  }
  //ROS_INFO("Percent: %i, pose_in: %i, percent_len: %i, path_out: %i",percentage,pathin.poses.size(),percent_length,pathout.poses.size());
  return pathout;
}


void set_target(){
	int dfac = path_raw_down.poses.size();
	int sfac = path_raw_side.poses.size() * 3;
	int divident = dfac + sfac*3;

	if(dfac == 0 && sfac == 0){
		ROS_INFO("NO CANDIDATES");
		return;
	}

	geometry_msgs::Point pnt_d = summarize_paths(path_raw_down,get_color(200,0,0));
	geometry_msgs::Point pnt_s = summarize_paths(path_raw_side,get_color(0,200,0));

	ROS_INFO("Path_raw_down[%i],z0: %.0f z1: %.0f",path_raw_down.poses.size(),path_raw_down.poses[0].pose.position.z,path_raw_down.poses[path_raw_down.poses.size()-1].pose.position.z);
	ROS_INFO("path_raw_side[%i],z0: %.0f z1: %.0f",path_raw_side.poses.size(),path_raw_side.poses[0].pose.position.z,path_raw_side.poses[path_raw_side.poses.size()-1].pose.position.z);
	nav_msgs::Path path_raw;
	path_raw = path_raw_down;
	for(int i = 0; i < path_raw_side.poses.size(); i++){
		path_raw.poses.push_back(path_raw_side.poses[i]);
	}
	path_raw_down = cutoff_percentage(sort_path(path_raw_down,"z"),30);
	path_raw_side = cutoff_percentage(sort_path(path_raw_side,"z"),30);
	path_raw = cutoff_percentage(sort_path(path_raw,"z"),30);
	geometry_msgs::Point pnt   = summarize_paths(path_raw,get_color(200,200,0));

	ROS_INFO("Path_raw[%i],z0: %.0f z1: %.0f",path_raw.poses.size(),path_raw.poses[0].pose.position.z,path_raw.poses[path_raw.poses.size()-1].pose.position.z);
	ROS_INFO("Path_raw_down[%i],z0: %.0f z1: %.0f",path_raw_down.poses.size(),path_raw_down.poses[0].pose.position.z,path_raw_down.poses[path_raw_down.poses.size()-1].pose.position.z);
	ROS_INFO("path_raw_side[%i],z0: %.0f z1: %.0f",path_raw_side.poses.size(),path_raw_side.poses[0].pose.position.z,path_raw_side.poses[path_raw_side.poses.size()-1].pose.position.z);
	pnt_d.x = pnt_d.x - pos.x;
	pnt_d.y = pnt_d.y - pos.y;
	pnt_d.z = pnt_d.z - pos.z;
	pnt_s.x = pnt_s.x - pos.x;
	pnt_s.y = pnt_s.y - pos.y;
	pnt_s.z = pnt_s.z - pos.z;
	ROS_INFO("DOWN[%i] XYZ: [%.0f %.0f %.0f]",path_raw_down.poses.size(),pnt_d.x,pnt_d.y,pnt_d.z);
	ROS_INFO("SIDE[%i] XYZ: [%.0f %.0f %.0f]",path_raw_side.poses.size(),pnt_s.x,pnt_s.y,pnt_s.z);
	ROS_INFO("SIDE[%i] XYZ: [%.0f %.0f %.0f]",path_raw.poses.size(),pnt.x,pnt.y,pnt.z);
	float x = (pnt_d.x * dfac + pnt_s.x * sfac) / divident;
	float y = (pnt_d.y * dfac + pnt_s.y * sfac) / divident;
	float z = (pnt_d.z * dfac + pnt_s.z * sfac) / divident;
	geometry_msgs::Point pout;
	pout.x = x + pos.x;
	pout.y = y + pos.y;
	pout.z = z + pos.z;
	ROS_INFO("OUT ->   XYZ: [%.0f %.0f %.0f]",x,y,z);
	ROS_INFO("OUT ->   XYZ: [%.0f %.0f %.0f]",pout.x,pout.y,pout.z);
	geometry_msgs::PoseStamped ps;
	ps.pose.position = pnt;
	set_target_pose(ps);
	draw("targetsraw");
}

nav_msgs::Path scatter_path(nav_msgs::Path pathin, float poses_spacing){
  nav_msgs::Path pathout;
  pathout.header = hdr();
  for(int i = 0; i < pathin.poses.size(); i++){
    if(dst_point_in_path_lim(pathout,pathin.poses[i].pose.position,poses_spacing)){
      pathout.poses.push_back(pathin.poses[i]);
    }
  }
  return pathout;
}


void draw_new(nav_msgs::Path pathin){
  img_blank.copyTo(img);

  path_ds_full = remove_old(path_ds_full,pos,50);
  nav_msgs::Path path_new     = get_new_path(path_ds_full,pathin,2);
  path_ds_full = add_new(path_ds_full,path_new);

  path_new = cutoff_top(path_new,"z",60,100);
  geometry_msgs::Point pnt_z = summarize_paths(path_new,get_color(200,0,0));
  ROS_INFO("DOWN[%i] XYZ: [%.0f %.0f %.0f]",path_new.poses.size(),pnt_z.x,pnt_z.y,pnt_z.z);

  path_new = cutoff_top(path_new,"neighbours",60,100);
  geometry_msgs::Point pnt_n = summarize_paths(path_new,get_color(0,200,0));
	ROS_INFO("SIDE[%i] XYZ: [%.0f %.0f %.0f]",path_new.poses.size(),pnt_n.x,pnt_n.y,pnt_n.z);

  path_new = cutoff_top(path_new,"hdng_abs",60,100);
  geometry_msgs::Point pnt_h = summarize_paths(path_new,get_color(0,0,200));
  ROS_INFO("SIDE[%i] XYZ: [%.0f %.0f %.0f]",path_new.poses.size(),pnt_h.x,pnt_h.y,pnt_h.z);

  cv::imwrite("/home/nuc/brain/draw/"+std::to_string(count_target_paths)+"neigbours_removed.png",img);

/*
  pathin = cutoff_top(pathin,"hdng_abs",60,100);
  //nav_msgs::Path path_ds_inc = sort_path(path_ds_full,"inclination");
  draw_path_by_attribute(path_new,"inclination",false);
  draw_path_by_attribute(path_new,"inclination_abs",false);
  draw_path_by_attribute(path_new,"hdng",false);
  draw_path_by_attribute(path_new,"hdng_abs",false);
*/


//  std::vector<float> vec = get_vec_attribute(path_ds_full,"inclination_abs");
  draw_poly(poly_cleared,get_color(255,255,0));
  draw_background(path_ds_full);
  //draw_path(path_ds_full);
//  draw_clusterpaths_shiftingcolor(paths_down,false,false,false,false,true,0,40,false);
  draw_clusterpaths_shiftingcolor(paths_side,true,true,false,true,false,3,100,false);
  cv::circle(img,pnt2cv(cmd_pos),2,get_color(255,255,200),1);
  cv::circle(img,pnt2cv(pos),2,get_color(255,255,0),1);
  cv::line (img, pnt2cv(pos), pnt2cv(cmd_pos),get_color(0,0,255),1,cv::LINE_8,0);

  cv::imwrite("/home/nuc/brain/draw/"+std::to_string(count_target_paths)+"raw_full.png",img);
  geometry_msgs::PoseStamped ps;
  ps.pose.position = pnt_z;
  if(dst_target < 4){
    set_target_pose(ps);
  }
  else{

  }
}

void rawdown_cb(const nav_msgs::Path::ConstPtr& msg){
  draw_path_at_img(*msg,pos,false,false,false,false,true,get_color(40,0,0),1);
  path_raw_down = constrain_path_bbpoly(*msg,target_final_poly);
  draw_path_at_img(path_raw_down,pos,false,false,false,false,true,get_color(100,0,0),1);
  ROS_INFO("SUMMARIZE PATH[%i] (%i poses original)",path_raw_down.poses.size(),msg->poses.size());
  if(got_one_raw){
    got_one_raw = false;
    for(int i = 0; i < msg->poses.size(); i++){
      raw_full.poses.push_back(msg->poses[i]);
    }
    if(use_raw_for_target)
      draw_new(raw_full);
//    set_target();
  }
  else{
    raw_full = *msg;
    got_one_raw = true;
  }
}

void rawside_cb(const nav_msgs::Path::ConstPtr& msg){
  path_raw_side = *msg;
  draw_path_at_img(*msg,pos,false,true,false,true,false,get_color(0,40,0),1);
	path_raw_side = constrain_path_bbpoly(*msg,target_final_poly);
	draw_path_at_img(path_raw_side,pos,false,true,false,true,false,get_color(0,100,0),1);
	ROS_INFO("SUMMARIZE PATH[%i] (%i poses original)",path_raw_side.poses.size(),msg->poses.size());
  if(got_one_raw){
    got_one_raw = false;
    for(int i = 0; i < msg->poses.size(); i++){
      raw_full.poses.push_back(msg->poses[i]);
    }
    if(use_raw_for_target)
      draw_new(raw_full);
  //  set_target();
  }
  else{
    raw_full = *msg;
    got_one_raw = true;
  }
}
void targets_cb(const nav_msgs::Path::ConstPtr& msg){
//  draw_path_at_img(*msg,pos,true,false,true,false,false,get_color(255,255,255),1);
}

void down_best_cb(const nav_msgs::Path::ConstPtr& msg){
  path_down_best = *msg;
  //draw_path_at_img(*msg,pos,true,false,true,false,false,get_color(0,0,0),1);
  if(got_one){
    got_one = false;
  //  cv::imwrite("/home/nuc/brain/draw/"+std::to_string(count_target_paths)+"_targetsbest.png",img);
  }
  else
    got_one = true;
}
void side_best_cb(const nav_msgs::Path::ConstPtr& msg){
  path_side_best = *msg;
  //draw_path_at_img(*msg,pos,true,true,false,true,false,get_color(0,100,0),1);
  if(got_one){
    got_one = false;
  //  cv::imwrite("/home/nuc/brain/draw/"+std::to_string(count_target_paths)+"_targetsbest.png",img);
  }
  else
    got_one = true;
}
void pathvlp_cb(const nav_msgs::Path::ConstPtr& msg){
  path_vlp = *msg;
  if(msg->poses.size() > 0){
    base_pose = msg->poses[0];
	}
}

void lowrateodom_cb(const nav_msgs::Odometry::ConstPtr& msg){
  odom = *msg;
}
void poylcleared_cb(const geometry_msgs::PolygonStamped::ConstPtr& msg){
  poly_cleared = *msg;
//  draw_poly(*msg,get_color(0,0,100));
}
void pntmid_cb(const geometry_msgs::PointStamped::ConstPtr& msg){
//  cv::circle(img,pnt2cv(msg->point),2,get_color(0,0,100),1);
}
float get_slope(geometry_msgs::Point p1, geometry_msgs::Point p2){
  return (p2.z - p1.z) / get_dst2d(p1,p2);
}
float get_a_from_dxy_r(float dxy, float r){
  return acos(dxy/r);
}
float get_dz_from_dxy_a(float dxy, float r){
  return acos(dxy/r);
}
float get_dz_from_r_a(float a, float r){
  return r * sin(a);
}
float get_dxy_from_r_a(float a, float r){
  return r * cos(a);
}
float get_a_from_dz_r(float dz, float r){
  return asin(dz/r);
}
void checktf(){
  geometry_msgs::TransformStamped transformStamped;
  try{
    transformStamped = tfBuffer.lookupTransform("map","velodyne_aligned",
                             ros::Time(0));
  }
  catch (tf2::TransformException &ex) {
    ROS_WARN("%s",ex.what());
    ros::Duration(1.0).sleep();
  }
	 dst_target = get_dst2d(target.pose.position,pos);
	pos.x   = transformStamped.transform.translation.x;
	pos.y   = transformStamped.transform.translation.y;
	pos.z   = transformStamped.transform.translation.z;
	tf2::Matrix3x3 q(tf2::Quaternion(transformStamped.transform.rotation.x, transformStamped.transform.rotation.y, transformStamped.transform.rotation.z, transformStamped.transform.rotation.w));
  q.getRPY(vlp_rpy.x,vlp_rpy.y,vlp_rpy.z);
  vlp_rpy.y *= -1;
  //q.getRPY(vlp_rpy.x,-vlp_rpy.y,vlp_rpy.z);
	pos_yaw = vlp_rpy.z;
}
void checktftime(nav_msgs::Path pathin){
  geometry_msgs::TransformStamped transformStamped;
  try{
    transformStamped = tfBuffer.lookupTransform("map","velodyne_aligned",
                             pathin.header.stamp);
  }
  catch (tf2::TransformException &ex) {
    ROS_WARN("%s",ex.what());
    ros::Duration(1.0).sleep();
  }
	 dst_target = get_dst2d(target.pose.position,pos);
	pos.x   = transformStamped.transform.translation.x;
	pos.y   = transformStamped.transform.translation.y;
	pos.z   = transformStamped.transform.translation.z;
	tf2::Matrix3x3 q(tf2::Quaternion(transformStamped.transform.rotation.x, transformStamped.transform.rotation.y, transformStamped.transform.rotation.z, transformStamped.transform.rotation.w));
  q.getRPY(vlp_rpy.x,vlp_rpy.y,vlp_rpy.z);
  vlp_rpy.y *= -1;
  //	q.getRPY(vlp_rpy.x,-vlp_rpy.y,vlp_rpy.z);
	pos_yaw = vlp_rpy.z;
}
void cmdpos_cb(const geometry_msgs::Point::ConstPtr& msg)
{
	target_last = target;
	target.pose.position = *msg;
	cmd_pos = *msg;
	target_final_poly = get_polyfinal(1,15,M_PI/2);
}
void request_paths(){
  std_msgs::UInt8 get_path;
  pub_get_next_path.publish(get_path);
}
nav_msgs::Path cutoff_abs(nav_msgs::Path pathin,std::string type,float val_0,float val_N){
  if(pathin.poses.size() < 3)
    return pathin;
  nav_msgs::Path pathout;
  pathout.header = hdr();
  if(type == "hdng"){
    for(int i = 0; i < pathin.poses.size(); i++){
      float hdng = get_hdng(pathin.poses[i].pose.position,pos);
      if(hdng >= val_0 && hdng <= val_N){
        pathout.poses.push_back(pathin.poses[i]);
      }
    }
  }
  if(type == "hdng_rel"){
    for(int i = 0; i < pathin.poses.size(); i++){
      float hdng     = get_hdng(pathin.poses[i].pose.position,pos);
      float hdng_rel = get_shortest(hdng,pos_yaw);
      if(hdng_rel >= val_0 && hdng_rel <= val_N){
        pathout.poses.push_back(pathin.poses[i]);
      }
    }
  }
  if(type == "zrel"){
    for(int i = 0; i < pathin.poses.size(); i++){
      if((pathin.poses[i].pose.position.z - pos.z) >= val_0 && (pathin.poses[i].pose.position.z - pos.z) <= val_N){
        pathout.poses.push_back(pathin.poses[i]);
      }
    }
  }
  if(type == "z"){
    for(int i = 0; i < pathin.poses.size(); i++){
      float hdng = get_hdng(pathin.poses[i].pose.position,pos);
      if(hdng >= pathin.poses[i].pose.position.z && pathin.poses[i].pose.position.z <= val_N){
        pathout.poses.push_back(pathin.poses[i]);
      }
    }
  }
  return pathout;
}
nav_msgs::Path cutoff_abs0(nav_msgs::Path pathin,std::string type,float val_0,float val_N){
  if(pathin.poses.size() < 3)
    return pathin;
  nav_msgs::Path pathout;
  pathout.header = hdr();
  pathin                  = sort_path(pathin,type);
  std::vector<float> vals = get_vec_attribute(pathin,type);
  float vmx = vec_to_max(vals);
  float vmn = vec_to_min(vals);
  float ave = vec_to_ave(vals);
  float vd = vmx - vmn;
  std::vector<float> valsout;
  for(int i = 0; i < pathin.poses.size(); i++){
    if(vals[i] >= val_0 && vals[i] <= val_N){
      pathout.poses.push_back(pathin.poses[i]);
      valsout.push_back(vals[i]);
    }
  }
  float pp = float(pathout.poses.size()) / float(pathin.poses.size());
  int percent_poses = pp * 100;
  ROS_INFO("Cutoff_abs %s , [min->max: ave] [%.2f->%.2f: %.2f] in->out: [%i->%i](%i percent)",type.c_str(),vmn,vmx,ave,pathin.poses.size(),pathout.poses.size(),percent_poses);
  vmx = vec_to_max(valsout);
  vmn = vec_to_min(valsout);
  ave = vec_to_ave(valsout);
  ROS_INFO("Cutoff_abs %s,  [min->max: ave] [%.2f->%.2f: %.2f] (v0 %.2f-> vN %.2f)",type.c_str(),vmn,vmx,ave,val_0,val_N);
  return pathout;
}

float get_i_min(sensor_msgs::LaserScan scanin,int i0, int iN){
  float mn = scanin.ranges[i0];
  for(int i = i0; i < iN; i++){
    if(std::isinf(scanin.ranges[i])){
      if(scanin.ranges[i] < mn)
        mn = scanin.ranges[i];
    }
  }
  return mn;
}
sensor_msgs::LaserScan get_ranges(sensor_msgs::LaserScan scanin,int scans_pr_min){
  sensor_msgs::LaserScan scanout;
  scanout = scanin;
  for(int i = scans_pr_min/2; i < scanin.ranges.size()-scans_pr_min/2; i++){
    scanout.ranges[i] = get_i_min(scanin,i-scans_pr_min/2,i+scans_pr_min/2);
  }
  return scanout;
}
//std::deque<int> q{5, 1, 3};
//std::deque<int>::iterator it = std::min_element(q.begin(), q.end());
//std::vector<int>::iterator result = std::min_element(v.begin(), v.end());
//std::cout << *it << std::endl;



geometry_msgs::Point get_maxmin_in_path(nav_msgs::Path pathin,bool get_max,std::string type){
  geometry_msgs::Point pnt;
  float best_val;
  int best_i = 0;
  if(get_max)
    best_val = -99999;
  else
    best_val = 999999;
  for(int i = 0; i < pathin.poses.size(); i++){
    float val = 0;
    if(type == "inclination_abs")
    val = get_inclination(pathin.poses[i].pose.position,pos);
    else if(type == "dst_2d")
      val = get_dst2d(pos,pathin.poses[i].pose.position);
    else if(type == "neighbours")
      val = getinpath_neighbours(pathin,i,3);
    else if(type == "inclination")
      val = get_inclination(pathin.poses[i].pose.position,pos);
    else if(type == "dst_3d_ref")
      val = get_dst3d(pnt_ref,pathin.poses[i].pose.position);
    else if(type == "dst_3d")
      val = get_dst3d(pos,pathin.poses[i].pose.position);
		else if(type == "hdng_abs")
			val = abs(get_shortest(get_hdng(pathin.poses[i].pose.position,pos),vlp_rpy.z) * rad2deg);
    else if(type == "hdng")
      val = get_shortest(get_hdng(pathin.poses[i].pose.position,pos),vlp_rpy.z);
    else if(type == "zabs")
  		val = abs(pathin.poses[i].pose.position.z - pos.z);
    else if(type == "zrel")
    	val = pathin.poses[i].pose.position.z - pos.z;
    else if(type == "z")
      val = pathin.poses[i].pose.position.z;
    else
      ROS_INFO("UNKNOWN");
    if(get_max){
      if(best_val < val){
        best_val = val;
        best_i   = i;
      }
    }
    else{
      if(best_val > val){
        best_val = val;
        best_i = i;
      }
    }
  }
  if(best_i < pathin.poses.size())
    pnt = pathin.poses[best_i].pose.position;
  return pnt;
}





void draw_line(geometry_msgs::Point p0,float a,float r,cv::Scalar color){
  geometry_msgs::Point pyaw;
  pyaw.x = p0.x + r * cos(a);
  pyaw.y = p0.y + r * sin(a);
  cv::line (img, pnt2cv(pos), pnt2cv(pyaw),color,1,cv::LINE_8,0);
}
nav_msgs::Path get_pathfinal(sensor_msgs::LaserScan scanin){
  nav_msgs::Path pathout;
  geometry_msgs::PointStamped pnt,pnt_out;
  pathout.header = hdr();
  geometry_msgs::TransformStamped transformStamped;
//  transformStamped = tfBuffer.lookupTransform("map", scanin.header.frame_id, scanin.header.stamp);
  pnt.header.frame_id = scanin.header.frame_id;
  for(int i = 0; i < scanin.ranges.size(); i++){
    if(std::isinf(scanin.ranges[i])){
    }
    else{
      float a = scanin.angle_min + scanin.angle_increment * i;
      pnt.point.x = scanin.ranges[i] * cos(a);
      pnt.point.y = scanin.ranges[i] * sin(a);
      pnt.header.stamp  = scanin.header.stamp;
      try{
          pnt_out = tfBuffer.transform(pnt, "map");
          geometry_msgs::PoseStamped ps;
          ps.header = hdr();
          ps.pose.position    = pnt_out.point;
          ps.pose.orientation = tf::createQuaternionMsgFromYaw(get_hdng(pnt_out.point,pos));
          pathout.poses.push_back(ps);
      }
      catch (tf2::TransformException &ex) {
            ROS_WARN("%s",ex.what());
            ros::Duration(1.0).sleep();
            continue;
      }
    ///  pnt_out = tfBuffer.transform("map",scanin.header.stamp,pnt,"map",pnt_out);

    }
  }
  return pathout;
}


nav_msgs::Path get_pathfinalclear(sensor_msgs::LaserScan scanin){
  nav_msgs::Path pathout;
  geometry_msgs::PointStamped pnt,pnt_out;
  pathout.header = hdr();
  pnt.header.frame_id = scanin.header.frame_id;
  for(int i = 0; i < scanin.ranges.size(); i++){
    if(std::isinf(scanin.ranges[i])){
      float a = scanin.angle_min + scanin.angle_increment * i;
      pnt.point.x = scanin.range_max * cos(a);
      pnt.point.y = scanin.range_max * sin(a);
      pnt.header.stamp    = ros::Time();
      pnt_out = tfBuffer.transform(pnt, "map");
      geometry_msgs::PoseStamped ps;
      ps.header = hdr();
      ps.pose.position    = pnt_out.point;
      ps.pose.orientation = tf::createQuaternionMsgFromYaw(get_hdng(pnt_out.point,pos));
      pathout.poses.push_back(ps);
    }
    else{

    }
  }
  return pathout;
}
geometry_msgs::PointStamped get_danger(){
  geometry_msgs::PointStamped pnt_out;
  float dst_danger = 100;
  for(int i = 0; i < path_obs.poses.size(); i++){
    float dst = get_dst3d(path_obs.poses[i].pose.position,pos);
    if(path_obs.poses[i].header.frame_id == "closed"){
      pnt_out.header.frame_id = "danger";
      if(dst < dst_danger){
        dst_danger = dst;
        pnt_out.point = path_obs.poses[i].pose.position;
      }
    }
  }
  return pnt_out;
}
geometry_msgs::PointStamped get_safe(){
  geometry_msgs::PointStamped pnt_out;
  float longest_dst = 15;
  for(int i = 0; i < path_obs.poses.size(); i++){
    if(path_obs.poses[i].header.frame_id == "map"){
      if(get_dst3d(pos,path_obs.poses[i].pose.position) > longest_dst){
        pnt_out.point = path_obs.poses[i].pose.position;
        pnt_out.header.frame_id = "safe";
        longest_dst = get_dst2d(pos,path_obs.poses[i].pose.position);
      }
    }
  }
  return pnt_out;
}

float get_elevation_below(){
  std::vector<geometry_msgs::Point> valid_points;
  if(obs_p.header.frame_id == "map")
    valid_points.push_back(obs_p.point);
  if(obs_m.header.frame_id == "map")
    valid_points.push_back(obs_m.point);
  if(obs_b.header.frame_id == "map")
    valid_points.push_back(obs_b.point);
  float highest_point = 0;
  for(int i = 0; i < valid_points.size(); i++){
    float hdng = get_shortest(get_hdng(valid_points[i],pos),pos_yaw);
    float dst  = get_dst2d(valid_points[i],pos);
    if(valid_points[i].z > highest_point)
      highest_point = valid_points[i].z;
    ROS_INFO("Alternative hdng: %.2f dst: %.2f point: %.0f %.0f %.0f",hdng,dst,valid_points[i].x,valid_points[i].y,valid_points[i].z);
  }
  return highest_point;
}
geometry_msgs::Point get_closepnts(){
  std::vector<geometry_msgs::Point> valid_points;
  if(obs_l.header.frame_id == "map")
    valid_points.push_back(obs_l.point);
  if(obs_r.header.frame_id == "map")
    valid_points.push_back(obs_r.point);
  if(obs_p.header.frame_id == "map")
    valid_points.push_back(obs_p.point);
  if(obs_m.header.frame_id == "map")
    valid_points.push_back(obs_m.point);
  if(obs_b.header.frame_id == "map")
    valid_points.push_back(obs_b.point);
  float lowest_dst = 0;
  geometry_msgs::Point best_point;
  if(valid_points.size() > 0)
    best_point = valid_points[0];
  for(int i = 0; i < valid_points.size(); i++){
    float dst  = get_dst2d(valid_points[i],pos);
    if(valid_points[i].z > best_point.z && best_point.z < pos.z)
      best_point = valid_points[i];
    else if(get_dst2d(best_point,pos) > get_dst2d(valid_points[i],pos))
      best_point = valid_points[i];
  }
  return best_point;
}
int percent_clear(nav_msgs::Path pathin){
  float sum_dst = 0;
  float closest_obst_dst = 100;
  float cleared_dst;
  int cleared_cnt = 0;
  for(int i = 0; i < pathin.poses.size(); i++){
    float rel_hdng = get_shortest(get_hdng(path_obs.poses[i].pose.position,pos),pos_yaw);
    float dst      = get_dst3d(path_obs.poses[i].pose.position,pos);
    sum_dst += dst;

    if(pathin.poses[i].header.frame_id == "map"){
      cleared_dst = dst;
      cleared_cnt++;
    }
    if(dst < closest_obst_dst)
      closest_obst_dst = dst;
  }
  float cl_pr_ob = float(cleared_cnt) / float(pathin.poses.size());
  int percent = cl_pr_ob * 100;
  float dst_ave = sum_dst / float(pathin.poses.size());
  float dst_rel = dst_ave / cleared_dst;
  int percent_cleared = dst_rel * 100;
  ROS_INFO("Percent cleared_pr_obs: %i cleared_pr_dst: %i (%i / %i) closest: %.0f, ave: %.0f",percent,percent_cleared,cleared_cnt,pathin.poses.size(),closest_obst_dst,dst_ave);
  return percent_cleared;
}


float calculate_setpoint_altitude(){
  float alt_above_zmax  = pos.z - scanzone_z_max;
  float direct_setpoint_z = fmax(scanzone_avepnt.z,scanzone_centroid.z) + par_zclearing;
  ROS_INFO("ALTCALC: pos_z: %.0f direct_setpoint_z: %.0f scanpoitn_ave_z: %.0f scanpoint_centroid_z: %.0f scanzone_min_max: %.0f->%.0f alt_above_zmax: %.0f",pos.z,direct_setpoint_z,scanzone_avepnt.z,scanzone_centroid.z,scanzone_z_min,scanzone_z_max,alt_above_zmax);
  geometry_msgs::PointStamped danger = get_danger();
  geometry_msgs::PointStamped safe   = get_safe();
  bool no_safe = false;
  bool no_danger  = false;
  bool real_danger = false;
  float dst_safe   = get_dst2d(pos,safe.point);
  if(danger.header.frame_id == "")
    no_danger = true;
  else if(get_dst2d(pos,danger.point) < 15 && danger.point.z > pos.z-3)
    real_danger = true;
  if(safe.header.frame_id == "")
    no_safe = true;
  float temp_change = 0;
  ROS_INFO("ALTCALC: no_safe: %i no_danger: %i real_danger: %i",no_safe,no_danger,real_danger);
  if(real_danger && no_safe){
    temp_change = 3;
  }
  else if(no_safe && !real_danger){
    temp_change = -2;
  }
  else if(!no_safe && real_danger){
    temp_change = 1;
  }
  else{
    temp_change = -1;
  }
  float curr_err = setpoint_altitude - pos.z;
  float new_err  = setpoint_altitude + temp_change -pos.z;
  if(abs(curr_err) < 1)
    return setpoint_altitude+temp_change;
  if(abs(curr_err) > abs(new_err))
    return setpoint_altitude+temp_change;
  else if(abs(curr_err) < 2 && abs(new_err) < 4)
    return setpoint_altitude+temp_change;
  else
    return setpoint_altitude;
}

float calculate_tilt(){
  ROS_INFO("TILTCALC: scanzone[%i],  [d0: %.0f - dN: %.0f,z0: %.0f - zN: %.0f] (ddst: %.0f, dz: %.0f)",scanzone_size_pnts,scanzone_dst_min,scanzone_dst_max,scanzone_z_min,scanzone_z_max,scanzone_size_ddst,scanzone_size_dalt);
  float dst_scanpnt_ave = get_dst3d(pos,scanzone_avepnt);
  float dst_scanpnt_cen = get_dst3d(pos,scanzone_centroid);
  float tilt_ave = get_inclination(scanzone_avepnt,pos);
  float tilt_cen = get_inclination(scanzone_centroid,pos);

  ROS_INFO("TILTCALC:  dst_scanpnt_ave: %:0f dst_scanpnt_cen: %.0f tilt_ave: %.2f tilt_cen: %.2f",dst_scanpnt_ave,dst_scanpnt_cen,tilt_ave,tilt_cen);
  return(M_PI/12);
//  return tilt_ave:;
}
void draw_scanzone(int size){
  geometry_msgs::Point p0,p1,scanzone_avepnt0,scanzone_centroid0,scanzone_avepnt1,scanzone_centroid1;

  scanzone_avepnt0.x = scanzone_avepnt.x-size;
  scanzone_avepnt0.y = scanzone_avepnt.y-size;
  scanzone_avepnt1.x = scanzone_avepnt.x+size*2;
  scanzone_avepnt1.y = scanzone_avepnt.y+size*2;

  scanzone_centroid0.x = scanzone_centroid.x-size;
  scanzone_centroid0.y = scanzone_centroid.y-size;
  scanzone_centroid1.x = scanzone_centroid.x+size*2;
  scanzone_centroid1.y = scanzone_centroid.y+size*2;

  p0.x = pos.x + scanzone_dst_min * cos(pos_yaw);
  p0.y = pos.y + scanzone_dst_min * sin(pos_yaw);

  p1.x = pos.x + scanzone_dst_max * cos(pos_yaw);
  p1.y = pos.y + scanzone_dst_max * sin(pos_yaw);

  cv::circle(img,pnt2cv(scanzone_centroid),size*2,get_color(200,0,0),1);
  cv::circle(img,pnt2cv(scanzone_avepnt),size*2,get_color(0,200,0),1);

  cv::line (img, pnt2cv(p0), pnt2cv(p1),c_sz,1,cv::LINE_8,0);
  cv::line (img, pnt2cv(pos), pnt2cv(scanzone_centroid),c_sz,1,cv::LINE_8,0);
  cv::line (img, pnt2cv(pos), pnt2cv(scanzone_centroid),c_sz,1,cv::LINE_8,0);

  cv::rectangle(img, pnt2cv(scanzone_avepnt1),pnt2cv(scanzone_avepnt1),c_sz,1,8,0);
  cv::rectangle(img, pnt2cv(scanzone_centroid0),pnt2cv(scanzone_centroid0),c_sz,1,8,0);
  cv::rectangle(img, pnt2cv(scanzone_bbmin),pnt2cv(scanzone_bbmax),c_sz,1,8,0);
  calculate_tilt();
}
void drawimg(float hdng_cutoff,float a,std::string name){
  geometry_msgs::Point pyaw,pyaw_mn,pyaw_mx;
  float a1 = constrainAngle(a + hdng_cutoff);
  float a2 = constrainAngle(a - hdng_cutoff);
  float a0 = fmin(a1,a2);
  float aN = fmax(a1,a2);
  draw_line(pos,a0,100,get_color(200,200,200));
  draw_line(pos,a1,100,get_color(200,200,200));
  draw_line(pos,pos_yaw,100,get_color(100,100,0));

  ROS_INFO("vlp_rpy_Z: %.2f, hdng_cutoff: %.2f, a0: %.2f a1: %.2f",vlp_rpy.z,hdng_cutoff,a0,aN);
  draw_path_at_img(path_visited,pos,false,false,false,false,true,get_color(0,50,0),1);

  draw_poly(poly_cleared,get_color(200,200,200));
  cv::line (img, pnt2cv(pos), pnt2cv(pyaw),get_color(100,100,100),1,cv::LINE_8,0);
  cv::line (img, pnt2cv(pos), pnt2cv(pyaw_mn),get_color(200,0,200),1,cv::LINE_8,0);
  cv::line (img, pnt2cv(pos), pnt2cv(pyaw_mx),get_color(200,0,200),1,cv::LINE_8,0);
  cv::circle(img,pnt2cv(pos),3,get_color(200,200,200),1);
  cv::circle(img,pnt2cv(cmd_pos),3,get_color(0,0,200),1);
  cv::line (img, pnt2cv(pos), pnt2cv(cmd_pos),get_color(0,0,200),1,cv::LINE_8,2);
  draw_scanzone(2);
  cv::imwrite("/home/nuc/brain/scanproc/"+std::to_string(counter)+ name + ".png",img);
  img_blank.copyTo(img);
}

void initialscan(sensor_msgs::LaserScan scanin){
		geometry_msgs::Point current_object;
		std::vector<geometry_msgs::Point>cluster;
    std::vector<float> hdngs;
    std::vector<bool> is_clear;
    std::vector<float> hdng_start_obs;
    std::vector<float> hdng_end_obs;
    std::vector<float> obstacle_clusters_r;
    std::vector<float> obstacle_clusters_z;
    std::vector<float> cleared_clusters_ends;
    std::vector<float> obstacle_clusters_starts;
    std::vector<float> obstacle_clusters_ends;
    std::vector<float> cleared_clusters_starts;

    std::vector<geometry_msgs::Point> obstacle_clusters_center;
    std::vector<std::vector<geometry_msgs::Point>>cleared_clusters;
		std::vector<std::vector<geometry_msgs::Point>>obstacle_clusters;
    bool last_clear = false;
    float zmx = 0;
    bool clear;
    float r_sum = 0;
    int largest_cleared_cluster = 0;
    int largest_obstacle_cluster = 0;

    float cleared_lim = 30;
    if(std::isinf(scanin.ranges[0]))
      last_clear = true;
    if(last_clear)
      cleared_clusters_starts.push_back(scanin.angle_min);
    else
      obstacle_clusters_starts.push_back(scanin.angle_min);
    int count_ten = 0;
    for(int i  = 1; i < scanin.ranges.size()-1; i++){
      geometry_msgs::PointStamped pnt,pnt_out;
      float current_angle = scanin.angle_min + scanin.angle_increment * i;
      hdngs.push_back(current_angle);
      //ROS_INFO("Current angle: %.2f range: %.2f cluster: %i",current_angle,scanin.ranges[i],cluster.size());
      clear = false;
      pnt.point.x = scanin.ranges[i] * cos(current_angle);
      pnt.point.y = scanin.ranges[i] * sin(current_angle);
      pnt.header.frame_id = scanin.header.frame_id;
      pnt.header.stamp    = ros::Time();
      pnt_out = tfBuffer.transform(pnt, "map");
      if(scanin.ranges[i] > scanin.ranges[i+1] && scanin.ranges[i] > scanin.ranges[i-1] && scanin.ranges[i] > cleared_lim){
        obstacle_clusters_center.push_back(pnt_out.point);
        cv::circle(img,pnt2cv(pnt_out.point),1,get_color(200,200,200),1);
      }
      count_ten++;
      if(scanin.ranges[i] > cleared_lim)
        clear = true;
      if(count_ten > 10){
        count_ten = 0;
      if(clear)
        draw_line(pos,current_angle,scanin.ranges[i],get_color(relval(0,50,scanin.ranges[i]),relval(0,50,scanin.ranges[i]),0));
      else
        draw_line(pos,current_angle,scanin.ranges[i],get_color(0,0,relval(0,50,scanin.ranges[i])));
      }

    //  cv::circle(img,pnt2cv(pnt_out.point),1,get_color(200,200,0),1);

        // std::isinf(scanin.ranges[i-1]) && std::isinf(scanin.ranges[i]) && std::isinf(scanin.ranges[1+i]))


    //  cv::circle(img,pnt2cv(pnt_out.point),1,get_color(0,0,relval(0,50,pnt_out.point.z)),1);

      is_clear.push_back(clear);
      if(last_clear && !clear){
        zmx = 0;
        cleared_clusters_ends.push_back(current_angle);
        obstacle_clusters_starts.push_back(current_angle);
        cleared_clusters.push_back(cluster);
        if(cluster.size() > largest_cleared_cluster)
          largest_cleared_cluster = cluster.size();
        cluster.resize(0);
      }
      else if(!last_clear && clear){
        float ranges_ave = r_sum / cluster.size();
        r_sum = 0;
        obstacle_clusters_z.push_back(zmx);
        obstacle_clusters_r.push_back(ranges_ave);
        if(cluster.size() > largest_obstacle_cluster)
          largest_obstacle_cluster = cluster.size();
        obstacle_clusters_ends.push_back(current_angle);
        cleared_clusters_starts.push_back(current_angle);
        obstacle_clusters.push_back(cluster);
        cluster.resize(0);
      }
      else if(last_clear && clear){

      }
      else if(!last_clear && !clear){

      }
      else{
        ROS_INFO("ERROR - impossible combinatio (CLEAR vs OBST)");
      }

      cluster.push_back(pnt_out.point);
      last_clear = clear;
      if(!clear){
        r_sum += scanin.ranges[i];
        if(pnt_out.point.z > zmx)
          zmx = pnt_out.point.z;
      }
    }
  for(int i  = 0; i < cleared_clusters.size(); i++){
    int num  = cleared_clusters[i].size();
    float a0 = cleared_clusters_starts[i];
    float aN = cleared_clusters_ends[i];
    float a  = (aN+a0)/2.0;
    draw_line(pos,a,50,get_color(0,200*num/largest_cleared_cluster,0));
    float ad = aN - a0;
    ROS_INFO("Cluster[%i], size: %i a: %.2f (delta. %.2f rads) (a0 %.2f->%.2f an)",i,num,a,ad,a0,aN);
  }
  for(int i  = 0; i < obstacle_clusters.size(); i++){
    int num  = obstacle_clusters[i].size();
    float a0 = obstacle_clusters_starts[i];
    float aN = obstacle_clusters_ends[i];
    float a  = (aN+a0)/2.0;
    float r  = (aN+a0)/2.0;
    float z  = (aN+a0)/2.0;
    draw_line(pos,a,r,get_color(0,0,200*num/largest_obstacle_cluster));

    ROS_INFO("Obstacles[%i], size: %i a: %.2f, zmax: %.2f dst_ave: %.2f",i,num,a,obstacle_clusters_z[i],obstacle_clusters_r[i]);
  }
}

void initialscan2(sensor_msgs::LaserScan scanin){
		geometry_msgs::Point current_object;
		std::vector<geometry_msgs::Point>cluster;
    std::vector<float> hdngs;
    std::vector<bool> is_clear;
    std::vector<float> hdng_start_obs;
    std::vector<float> hdng_end_obs;
    std::vector<float> obstacle_clusters_r;
    std::vector<float> obstacle_clusters_z;
    std::vector<float> cleared_clusters_ends;
    std::vector<float> obstacle_clusters_starts;
    std::vector<float> obstacle_clusters_ends;
    std::vector<float> cleared_clusters_starts;

    std::vector<geometry_msgs::Point> obstacle_clusters_center;
    std::vector<std::vector<geometry_msgs::Point>>cleared_clusters;
		std::vector<std::vector<geometry_msgs::Point>>obstacle_clusters;
    bool last_clear = false;
    float zmx = 0;
    bool clear;
    float r_sum = 0;
    int largest_cleared_cluster = 0;
    int largest_obstacle_cluster = 0;

    scanin = get_ranges(scanin,15);
    float cleared_lim = 30;
    if(std::isinf(scanin.ranges[0]))
      last_clear = true;
    if(last_clear)
      cleared_clusters_starts.push_back(scanin.angle_min);
    else
      obstacle_clusters_starts.push_back(scanin.angle_min);
    int count_ten = 0;
    for(int i  = 1; i < scanin.ranges.size()-1; i++){
      geometry_msgs::PointStamped pnt,pnt_out;
      float current_angle = scanin.angle_min + scanin.angle_increment * i;
      hdngs.push_back(current_angle);
      //ROS_INFO("Current angle: %.2f range: %.2f cluster: %i",current_angle,scanin.ranges[i],cluster.size());
      clear = false;
      pnt.point.x = scanin.ranges[i] * cos(current_angle);
      pnt.point.y = scanin.ranges[i] * sin(current_angle);
      pnt.header.frame_id = scanin.header.frame_id;
      pnt.header.stamp    = ros::Time();
      pnt_out = tfBuffer.transform(pnt, "map");
      if(scanin.ranges[i] > scanin.ranges[i+1] && scanin.ranges[i] > scanin.ranges[i-1] && scanin.ranges[i] > cleared_lim && scanin.ranges[i] < scanin.range_max){
        obstacle_clusters_center.push_back(pnt_out.point);
        cv::circle(img,pnt2cv(pnt_out.point),1,get_color(200,200,200),1);
      }
      count_ten++;
      if(scanin.ranges[i] > cleared_lim)
        clear = true;
      if(count_ten > 10){
        count_ten = 0;
      if(clear)
        draw_line(pos,current_angle,scanin.ranges[i],get_color(relval(0,50,scanin.ranges[i]),relval(0,50,scanin.ranges[i]),0));
      else
        draw_line(pos,current_angle,scanin.ranges[i],get_color(0,0,relval(0,50,scanin.ranges[i])));
      }

    //  cv::circle(img,pnt2cv(pnt_out.point),1,get_color(200,200,0),1);

        // std::isinf(scanin.ranges[i-1]) && std::isinf(scanin.ranges[i]) && std::isinf(scanin.ranges[1+i]))


    //  cv::circle(img,pnt2cv(pnt_out.point),1,get_color(0,0,relval(0,50,pnt_out.point.z)),1);

      is_clear.push_back(clear);
      if(last_clear && !clear){
        zmx = 0;
        cleared_clusters_ends.push_back(current_angle);
        obstacle_clusters_starts.push_back(current_angle);
        cleared_clusters.push_back(cluster);
        if(cluster.size() > largest_cleared_cluster)
          largest_cleared_cluster = cluster.size();
        cluster.resize(0);
      }
      else if(!last_clear && clear){
        float ranges_ave = r_sum / cluster.size();
        r_sum = 0;
        obstacle_clusters_z.push_back(zmx);
        obstacle_clusters_r.push_back(ranges_ave);
        if(cluster.size() > largest_obstacle_cluster)
          largest_obstacle_cluster = cluster.size();
        obstacle_clusters_ends.push_back(current_angle);
        cleared_clusters_starts.push_back(current_angle);
        obstacle_clusters.push_back(cluster);
        cluster.resize(0);
      }
      else if(last_clear && clear){

      }
      else if(!last_clear && !clear){

      }
      else{
        ROS_INFO("ERROR - impossible combinatio (CLEAR vs OBST)");
      }

      cluster.push_back(pnt_out.point);
      last_clear = clear;
      if(!clear){
        r_sum += scanin.ranges[i];
        if(pnt_out.point.z > zmx)
          zmx = pnt_out.point.z;
      }
    }
  for(int i  = 0; i < cleared_clusters.size(); i++){
    int num  = cleared_clusters[i].size();
    float a0 = cleared_clusters_starts[i];
    float aN = cleared_clusters_ends[i];
    float a  = (aN+a0)/2.0;
    draw_line(pos,a,50,get_color(0,200*num/largest_cleared_cluster,0));
    float ad = aN - a0;
    ROS_INFO("Cluster[%i], size: %i a: %.2f (delta. %.2f rads) (a0 %.2f->%.2f an)",i,num,a,ad,a0,aN);
  }
  for(int i  = 0; i < obstacle_clusters.size(); i++){
    int num  = obstacle_clusters[i].size();
    float a0 = obstacle_clusters_starts[i];
    float aN = obstacle_clusters_ends[i];
    float a  = (aN+a0)/2.0;
    float r  = (aN+a0)/2.0;
    float z  = (aN+a0)/2.0;
    draw_line(pos,a,r,get_color(0,0,200*num/largest_obstacle_cluster));

    ROS_INFO("Obstacles[%i], size: %i a: %.2f, zmax: %.2f dst_ave: %.2f",i,num,a,obstacle_clusters_z[i],obstacle_clusters_r[i]);
  }
}

nav_msgs::Path merge_paths(std::vector<nav_msgs::Path> paths){
  nav_msgs::Path pathout;
  for(int i = 0; i < paths.size(); i++){
    for(int k = 0; k < paths[i].poses.size(); k++){
      pathout.poses.push_back(paths[i].poses[k]);
    }
  }
  pathout.header = hdr();
  return pathout;
}
std::vector<nav_msgs::Path> get_pathvector(nav_msgs::Path p_dn,nav_msgs::Path p_lo,nav_msgs::Path p_mi,nav_msgs::Path p_up,nav_msgs::Path p_hi){
  std::vector<nav_msgs::Path> paths;
  paths.push_back(p_dn);
  paths.push_back(p_lo);
  paths.push_back(p_mi);
  paths.push_back(p_up);
  paths.push_back(p_hi);
  return paths;
}

nav_msgs::Path get_path_aroundpnt(nav_msgs::Path pathin,geometry_msgs::Point midpoint,float dxy){
  nav_msgs::Path pathout;
  float xmn = midpoint.x - dxy;
  float ymn = midpoint.y - dxy;
  float xmx = midpoint.x + dxy;
  float ymx = midpoint.y + dxy;
  pathout.header = hdr();
  for(int i = 0; i < pathin.poses.size(); i++){
    if(pathin.poses[i].pose.position.x < xmx
    && pathin.poses[i].pose.position.y < ymx
    && pathin.poses[i].pose.position.x > xmn
    && pathin.poses[i].pose.position.y > ymn)
      pathout.poses.push_back(path_full.poses[i]);
  }
  return pathout;
}
nav_msgs::Path get_pathfull_aroundline(geometry_msgs::Point p2,geometry_msgs::Point p1,float dxy){
  nav_msgs::Path pathout;
  pathout.header = hdr();

  Eigen::Vector3f cur_vec(p1.x,p1.y,p1.z);
  Eigen::Vector3f pnt1(p1.x,p1.y,p1.z);
  Eigen::Vector3f pnt2(p2.x,p2.y,p2.z);
  float tot_ray_len = (pnt2 - pnt1).norm();
  float cur_ray_len = 0;
  Eigen::Vector3f stride_vec = (pnt2 - pnt1).normalized() * dxy;
  geometry_msgs::Point midpoint;

  while(cur_ray_len < tot_ray_len){
    cur_vec = cur_vec + stride_vec;
    midpoint.x = cur_vec.x();
    midpoint.y = cur_vec.y();
    midpoint.z = cur_vec.z();
    nav_msgs::Path path_segment = get_path_aroundpnt(path_full,midpoint,dxy);
    for(int i = 0; i < path_segment.poses.size(); i++){
      pathout.poses.push_back(path_segment.poses[i]);
    }
    cur_ray_len = (cur_vec - pnt1).norm();
  }
  return pathout;
}

void draw_paths_by_scores(std::vector<nav_msgs::Path> pathsin,std::vector<float> scores){
  float scores_max = vec_to_max(scores);
  float scores_min = vec_to_min(scores);
  for(int i = 0; i < pathsin.size(); i++){
    float rel_score  = (scores[i] - scores_min) / (scores_max-scores_min);
    std::vector<float> vvz = get_vec_attribute(pathsin[i],"z");
    ROS_INFO("REL SCORE: %.2f",rel_score);
    if(rel_score == 1.0){
      draw_path_by_score(pathsin[i],vvz,2,1,0,rel_score);
    }
    else if(rel_score > 0.66){
      draw_path_by_score(pathsin[i],vvz,1,1,1,rel_score);
    }
    else if(rel_score > 0.33){
      draw_path_by_score(pathsin[i],vvz,2,2,2,rel_score);
    }
    else{
      draw_path_by_score(pathsin[i],vvz,0,0,0,rel_score*2);
    }
  }
}

std::vector<float> score_paths_by_attributes(std::vector<nav_msgs::Path> pathsin,std::vector<std::string> vec_attributes,std::vector<std::string> vec_attributes_type,std::vector<float> vec_weights){
  std::vector<float> vec_att_ave;
  std::vector<float> vec_att_max;
  std::vector<float> vec_att_min;
  std::vector<float> scores;
  std::vector<float> path_sizes;
  scores.resize(vec_attributes.size());
  vec_att_ave.resize(vec_attributes.size());
  vec_att_max.resize(vec_attributes.size());
  vec_att_min.resize(vec_attributes.size());
  for(int i = 0; i < pathsin.size(); i++){
    path_sizes.push_back(float(pathsin[i].poses.size()));
  }
  float size_max_all = vec_to_max(path_sizes);
  float size_min_all = vec_to_min(path_sizes);
  float s_r = size_max_all - size_min_all;

  nav_msgs::Path path_combined = merge_paths(pathsin);
  for(int k = 0; k < vec_attributes.size(); k++){
    std::vector<float> vec_att = get_vec_attribute(path_combined,vec_attributes[k]);
    vec_att_ave[k] = vec_to_ave(vec_att);
    vec_att_max[k] = vec_to_max(vec_att);
    vec_att_min[k] = vec_to_min(vec_att);
  }
  for(int i= 0; i < pathsin.size(); i++){
    float tot_score = 0;
    for(int k = 0; k < vec_attributes.size(); k++){
      float s = 0;
      if(vec_attributes[k] == "size")
          s = (path_sizes[i] - size_min_all) / s_r;
      else{
        std::vector<float> vec_att = get_vec_attribute(path_combined,vec_attributes[k]);
        if(vec_attributes_type[k] == "max")
          s   = (vec_to_max(vec_att) - vec_att_min[k]) / (vec_att_max[k] - vec_att_min[k]);
        else if(vec_attributes_type[k] == "ave")
          s   = (vec_to_ave(vec_att) - vec_att_min[k]) / (vec_att_max[k] - vec_att_min[k]);
        else if(vec_attributes_type[k] == "min")
          s   = (vec_to_min(vec_att) - vec_att_min[k]) / (vec_att_max[k] - vec_att_min[k]);
      }
      tot_score += s * vec_weights[k];
    }
    scores[i] = tot_score;
  }
  return scores;
}
void scores_testing(std::vector<nav_msgs::Path> pathsin){
  std::vector<std::string> vec_att;
  std::vector<std::string> vec_att_type;
  std::vector<float> vec_weights;
  vec_att.push_back("inclination");
  vec_att_type.push_back("max");
  vec_weights.push_back(-1.0);
  vec_att.push_back("dst_2d");
  vec_att_type.push_back("max");
  vec_weights.push_back(-2.0);
  vec_att.push_back("dst_2d");
  vec_att_type.push_back("ave");
  vec_weights.push_back(-2.0);
  vec_att.push_back("size");
  vec_att_type.push_back("");
  vec_weights.push_back(-2.0);
std::vector<float> scores = score_paths_by_attributes(pathsin,vec_att,vec_att_type,vec_weights);
}
void look_in_directions(float num_rays, float len, float dxy){
  float rads_pr_i = M_PI / num_rays;
  float min_yaw = constrainAngle(pos_yaw - M_PI/2) + rads_pr_i / 2;
  geometry_msgs::Point midpoint;
  std::vector<float> angles;
  std::vector<float> dangles;
  std::vector<float> path_empty_len;
  std::vector<float> path_vstd_len;
  std::vector<float> zaves;
  std::vector<float> zmaxs;
  std::vector<float> zmins;
  std::vector<float> inc_ave;
  std::vector<float> inc_max;
  std::vector<float> inc_min;
  std::vector<float> path_sizes;
  std::vector<nav_msgs::Path> paths_full_angles;
  std::vector<nav_msgs::Path> paths_vstd_angles;
  paths_full_angles.resize(num_rays);
  paths_vstd_angles.resize(num_rays);
  angles.resize(num_rays);
  dangles.resize(num_rays);
  zaves.resize(num_rays);
  zmaxs.resize(num_rays);
  zmins.resize(num_rays);
  inc_ave.resize(num_rays);
  inc_max.resize(num_rays);
  inc_min.resize(num_rays);
  path_vstd_len.resize(num_rays);
  path_empty_len.resize(num_rays);
  path_sizes.resize(num_rays);
  for(int i = 0; i < num_rays; i++){
    float a_i = constrainAngle(min_yaw + rads_pr_i * i);
    float cur_len = dxy;
    angles[i] = a_i;
    dangles[i] = -M_PI/2 + rads_pr_i * i;
    std::vector<nav_msgs::Path> paths_visited_ang;
    std::vector<nav_msgs::Path> paths_full_ang;
    while(cur_len < len){
      cur_len += dxy;
      midpoint.x = pos.x + cur_len*cos(a_i);
      midpoint.y = pos.y + cur_len*sin(a_i);
      nav_msgs::Path path_visited_pnt  = get_path_aroundpnt(path_visited,midpoint,dxy);
      nav_msgs::Path path_full_pnt     = get_path_aroundpnt(path_full,midpoint,dxy);
      paths_full_ang.push_back(path_full_pnt);
      paths_visited_ang.push_back(path_visited_pnt);

      if(path_full_pnt.poses.size() == 0){
        path_empty_len[i] = cur_len;
        break;
      }
      if(path_visited_pnt.poses.size() > 5){
        path_vstd_len[i] = cur_len;
        break;
      }
    }
    paths_vstd_angles[i] = merge_paths(paths_visited_ang);
    paths_full_angles[i] = merge_paths(paths_full_ang);
    path_sizes[i] = float(paths_vstd_angles[i].poses.size());

  }

  nav_msgs::Path path1 = merge_paths(paths_full_angles);
  std::vector<float> vdd = get_vec_attribute(path1,"dst_2d");
  std::vector<float> vzz = get_vec_attribute(path1,"z");
  std::vector<float> vii = get_vec_attribute(path1,"inclination");
  float zave_all = vec_to_ave(vzz);
  float zmax_all = vec_to_max(vzz);
  float zmin_all = vec_to_min(vzz);
  float inc_ave_all = vec_to_ave(vii);
  float inc_max_all = vec_to_max(vii);
  float inc_min_all = vec_to_min(vii);
  float dst_ave_all = vec_to_ave(vdd);
  float dst_max_all = vec_to_max(vdd);
  float dst_min_all = vec_to_min(vdd);
  float size_max_all = vec_to_max(path_sizes);
  float size_min_all = vec_to_min(path_sizes);

  float d_r = dst_max_all - dst_min_all;
  float z_r = zmax_all - zmin_all;
  float i_r = inc_max_all - inc_min_all;
  float s_r = size_max_all - size_min_all;

  ROS_INFO("size: %i inc_ave %.2f inc_max: %.0f zaves: %.0f, zmaxs: %.2f, zmins: %.2f",path1.poses.size(),inc_ave_all,inc_max_all,inc_min_all,zave_all,zmax_all,zmin_all);

  int best_i = 0;
  float best_score = -1000;
  std::vector<float> tot_scores;
  tot_scores.resize(num_rays);
  for(int i = 0; i < num_rays; i++){
    std::vector<float> vd = get_vec_attribute(paths_full_angles[i],"dst_2d");
    std::vector<float> vz = get_vec_attribute(paths_full_angles[i],"z");
    std::vector<float> vi = get_vec_attribute(paths_full_angles[i],"inclination");
    float zmax_v = vec_to_max(vz);
    float zave_v = vec_to_ave(vz);
    float incm_v = vec_to_max(vi);
    float inca_v = vec_to_ave(vi);
    float dstm_v = vec_to_max(vd);
    float dsta_v = vec_to_ave(vd);

    float zmax_s   = (zmax_v - zmin_all) / z_r;
    float zave_s   = (zave_v - zmin_all) / z_r;
    float incm_s   = -1*(incm_v - inc_min_all) / i_r;
    float inca_s   = -1*(inca_v - inc_min_all) / i_r;
    float dstm_s   = -2*(dstm_v - dst_min_all) / d_r;
    float dsta_s   = -1*(dsta_v - dst_min_all) / d_r;
    float size_s   = -1*(path_sizes[i] - size_min_all) / s_r;
    float hdng_score = -3*abs(dangles[i]) / (M_PI/2);
    tot_scores[i] = zmax_s + zave_s + incm_s + inca_s + dstm_s + dsta_s + hdng_score + size_s;
    if(tot_scores[i] > best_score){
      best_score = tot_scores[i];
      best_i = i;
      ROS_INFO("New Best Score [z: %.2f,%.2f], [inc: %.2f,%.2f],[dst: %.2f,%.2f],[ang: %.2f] ",zmax_v,zave_v,incm_v,inca_v,dstm_v,dsta_v,dangles[i]);
      ROS_INFO("New Best Score [z: %.2f,%.2f], [inc: %.2f,%.2f],[dst: %.2f,%.2f],[size: %.2f][ang: %.2f]",path1.poses.size(),zmax_s,zave_s,incm_s,inca_s,dstm_s,dsta_s,size_s,hdng_score);
    }
  }
  draw_paths_by_scores(paths_full_angles,tot_scores);

  float tot_scores_max = vec_to_max(tot_scores);
  float tot_scores_min = vec_to_min(tot_scores);
//for(int i = 0; i < num_rays; i++){
//    float rel_score  = 255 * (tot_scores[i] - tot_scores_min) / (tot_scores_max-tot_scores_min);
//    draw_path_at_img(paths_full_angles[i],pos,false,false,false,false,true,get_color(0,rel_score,rel_score),1);
//  }
  std::vector<float> vva = get_vec_attribute(paths_full_angles[best_i],"z");
  draw_path_by_score(paths_full_angles[best_i],vva,2,1,0,0.2);
  geometry_msgs::PoseStamped ps;
  ROS_INFO("NEW POSE: %.2f ",angles[best_i]);
  ps.pose.position.x = pos.x + 10*cos(angles[best_i]);
  ps.pose.position.y = pos.y + 10*sin(angles[best_i]);
  cv::circle(img,pnt2cv(ps.pose.position),1,get_color(0,0,255),1);
  cv::line(img,pnt2cv(pos),pnt2cv(ps.pose.position),get_color(0,0,255),1,cv::LINE_8,0);
  drawimg(M_PI,pos_yaw,"fullpath_scores"+ std::to_string(count_target_paths) );
  set_target_pose(ps);

}

void update_path_full(){
  img_blank.copyTo(img);
  nav_msgs::Path path_scanned = merge_paths(get_pathvector(path_hi,path_lo,path_up,path_dn,path_mi));
  path_scanned = scatter_path(path_scanned,1);
  nav_msgs::Path path_full_inrad  = remove_old(path_full,pos,50);
  nav_msgs::Path path_scanned_new = get_new_path(path_full_inrad,path_scanned,1);
  path_full = add_new(path_full,path_scanned_new);
  //draw_path_at_img(path_full,pos,false,false,false,false,true,get_color(40,40,0),1);
//  draw_path_at_img(path_full_inrad,pos,false,false,false,false,true,get_color(80,80,0),1);
  //draw_path_at_img(path_scanned_new,pos,false,false,false,false,true,get_color(40,0,200),1);
  std::vector<float> v1 = get_vec_attribute(path_full,"z");
  std::vector<float> v2 = get_vec_attribute(path_full,"z");
  count_target_paths++;

  draw_path_by_score(path_full,v1,0,1,2,0.6);
  draw_path_by_score(path_scanned_new,v2,0,1,0,0.7);
  drawimg(M_PI,pos_yaw,"fullpath"+ std::to_string(count_target_paths) );
}

geometry_msgs::PointStamped analyze_merged_pathsegment0(nav_msgs::Path pathin){
  std::vector<float> v1 = get_vec_attribute(pathin,"dst_2d");
  std::vector<float> v2 = get_vec_attribute(pathin,"z");
  std::vector<float> v3 = get_vec_attribute(pathin,"neighbours");
  float ave_dst = vec_to_ave(v1);
  float ave_z   = vec_to_ave(v2);
  float ave_n   = vec_to_ave(v3);
  float mx_dst = vec_to_max(v1);
  float mx_z   = vec_to_max(v2);
  float mx_n   = vec_to_max(v3);
  float mn_dst = vec_to_min(v1);
  float mn_z   = vec_to_min(v2);
  float mn_n   = vec_to_min(v3);
  float dz   = mx_z - mn_z;
  float ddst = mx_dst - mn_dst;
  geometry_msgs::PointStamped pave;
  pave.point = get_ave_pnt(pathin);
  pave.point.z = mx_z;
  ROS_INFO("analyze_merged_pathsegment dst: [%.2f->%.2f: %.2f] z: [%.2f->%.2f: %.2f] neighbours: [%.2f->%.2f: %.2f] tot: [%i] ",mn_dst,mx_dst,ave_dst,mn_z,mx_z,ave_z,mn_n,mx_n,ave_n,pathin.poses.size());
  if(mx_n > 2 && ddst < dz*3)
    pave.header.frame_id = "wall";
  else if(dz < 3){
    pave.header.frame_id = "plane";
  }
  else{
    pave.header.frame_id = "mess";
  }
  return pave;
}

float get_rel_score(std::vector<float> scores, float score){
  geometry_msgs::Point minmax = max_score(scores);
  return (score - minmax.y) / (minmax.x - minmax.y);
}

cv::Scalar get_rel_color(std::vector<float> scores, float score, std::string rgb){
  cv::Scalar c;
  float relscore = get_rel_score(scores,score);
  if(rgb == "r")
    c[2] = relscore;
  if(rgb == "g")
    c[1] = relscore;
  if(rgb == "b")
    c[0] = relscore;
  return c;
 }

void draw_path_by_attribute(nav_msgs::Path pathin,std::string type){
 geometry_msgs::Point maxmin;
 std::vector<float> vals = get_vec_attribute(pathin,type);
 maxmin.x = vec_to_max(vals);
 maxmin.y = vec_to_min(vals);
 geometry_msgs::Point pyaw,prect0,prect1,pnt;

 if(pathin.poses.size() < 1)
   return;
 for(int i = 0; i < pathin.poses.size(); i++){
   pnt = pathin.poses[i].pose.position;
   cv::Scalar color;
   float rel_score = (vals[i] - maxmin.y) / (maxmin.x - maxmin.y);
   if(type == "z" && vals[i] > pos.z-1)
     color[2] = rel_score*255;
   else
     color[1] = rel_score*255;
   img.at<cv::Vec3b>( y2r(pnt.y,img.rows,1),x2c(pnt.x,img.cols,1) )[0] = color[0];
   img.at<cv::Vec3b>( y2r(pnt.y,img.rows,1),x2c(pnt.x,img.cols,1) )[1] = color[1];
   img.at<cv::Vec3b>( y2r(pnt.y,img.rows,1),x2c(pnt.x,img.cols,1) )[2] = color[2];
 }
}

geometry_msgs::Point path2point(nav_msgs::Path pathin){
  geometry_msgs::Point ave_pnt = get_ave_pnt(pathin);
  ave_pnt.z = get_zmax(pathin);
  return ave_pnt;
}
void draw_point_by_z(geometry_msgs::Point pnt,float zmax,float zmin,bool circle){
  cv::Scalar color;
  float rel_score = (pnt.z - zmin) / (zmax - zmin);
  if(pnt.z > pos.z-1)
    color[0] = rel_score*255;
  else
    color[0] = rel_score*255;
  if(circle)
    cv::circle(img,pnt2cv(pnt),1,color,1);
  else{
    img.at<cv::Vec3b>( y2r(pnt.y,img.rows,1),x2c(pnt.x,img.cols,1) )[0] = color[0];
    img.at<cv::Vec3b>( y2r(pnt.y,img.rows,1),x2c(pnt.x,img.cols,1) )[1] = color[1];
    img.at<cv::Vec3b>( y2r(pnt.y,img.rows,1),x2c(pnt.x,img.cols,1) )[2] = color[2];
  }
}
void eval_area_simple(nav_msgs::Path pathin,float hdng_cutoff){
  int num_is = 64;
  float tot_rads = hdng_cutoff * 2;
  float rads_pr_i = tot_rads / num_is;
  float aN;
  float a0 = -hdng_cutoff;
  std::vector<float> vals = get_vec_attribute(pathin,"z");
  float zmx = vec_to_max(vals);
  float zmn = vec_to_min(vals);
  geometry_msgs::PolygonStamped poly;
  poly.polygon.points.resize(num_is*2+2);
  poly.header = hdr();
  for(int i = 0; i < num_is; i++){
    aN = a0 + rads_pr_i;
    nav_msgs::Path path_hdng = cutoff_abs(pathin,"hdng",a0,aN);
    a0 = aN;

    geometry_msgs::Point dst_max_pnt = get_maxmin_in_path(path_hdng,true,"dst_2d");
    geometry_msgs::Point dst_min_pnt = get_maxmin_in_path(path_hdng,false,"dst_2d");
    geometry_msgs::Point z_max_pnt   = get_maxmin_in_path(path_hdng,true,"z");
    geometry_msgs::Point z_min_pnt   = get_maxmin_in_path(path_hdng,false,"z");
    float dst_maxmin = get_dst2d(dst_max_pnt,dst_min_pnt);
    float dst_max    = get_dst2d(dst_max_pnt,pos);
    float dst_min    = get_dst2d(dst_min_pnt,pos);
    draw_point_by_z(dst_max_pnt,zmx,zmn,true);
    draw_point_by_z(dst_min_pnt,zmx,zmn,false);
    draw_point_by_z(z_max_pnt,zmx,zmn,false);
    draw_point_by_z(z_min_pnt,zmx,zmn,false);
    ROS_INFO("analyze_merged_pathsegment: hdng: %.2f z: [%.2f->%.2f: %.2f] dst: [%.2f->%.2f: %.2f] tot: [%i] ",aN,z_min_pnt.z,z_max_pnt.z,z_max_pnt.z-z_min_pnt.z,dst_min,dst_max,dst_maxmin,pathin.poses.size());
    poly.polygon.points[i].x = dst_max_pnt.x;
    poly.polygon.points[i].y = dst_max_pnt.y;
    poly.polygon.points[i].z = dst_max_pnt.z;
    poly.polygon.points[poly.polygon.points.size()-1-i].x = dst_min_pnt.x;
    poly.polygon.points[poly.polygon.points.size()-1-i].y = dst_min_pnt.y;
    poly.polygon.points[poly.polygon.points.size()-1-i].z = dst_min_pnt.z;
  }
  //draw_poly(poly,get_color(100,100,0));
}
void draw_basepaths(nav_msgs::Path p_dn,nav_msgs::Path p_lo,nav_msgs::Path p_mi,nav_msgs::Path p_up,nav_msgs::Path p_hi){
  draw_path_at_img(p_dn,pos,false,false,false,false,true,c_dn,1);
  draw_path_at_img(p_lo,pos,false,false,false,false,true,c_lo,1);
  draw_path_at_img(p_mi,pos,false,false,false,false,true,c_mi,1);
  draw_path_at_img(p_up,pos,false,false,false,false,true,c_up,1);
  draw_path_at_img(p_hi,pos,false,false,false,false,true,c_hi,1);
}
void get_above_pos(nav_msgs::Path p_dn,nav_msgs::Path p_lo,nav_msgs::Path p_mi,nav_msgs::Path p_up,nav_msgs::Path p_hi){
  nav_msgs::Path path_all   = merge_paths(get_pathvector(p_dn,p_lo,p_mi,p_up,p_hi));
  nav_msgs::Path path_above_z = cutoff_abs(path_all,"zrel",-1,100);
  nav_msgs::Path path_below_z = cutoff_abs(path_all,"zrel",-100,-1);
  draw_path_at_img(path_above_z,pos,false,false,false,false,true,get_color(0,0,200),1);
  draw_path_at_img(path_below_z,pos,false,false,false,false,true,get_color(0,200,0),1);
  //drawimg(M_PI/3,pos_yaw,"above_pos");
}
void get_hi(nav_msgs::Path p_hi,nav_msgs::Path p_hic,float hdng_cutoff){
  p_hi         = cutoff_abs(p_hi,"hdng_rel",-M_PI/3,M_PI/3);
  p_hic        = cutoff_abs(p_hic,"hdng_rel",-M_PI/3,M_PI/3);
  nav_msgs::Path path_above_z = cutoff_abs(p_hi,"zrel",-1,100);
  nav_msgs::Path p_hicabove_z = cutoff_abs(p_hic,"zrel",-1,100);
  draw_path_at_img(path_above_z,pos,false,false,false,false,true,get_color(0,0,200),1);
  draw_path_at_img(p_hicabove_z,pos,false,false,false,false,true,get_color(200,0,200),1);
  //drawimg(M_PI/3,pos_yaw,"above_z");
}
void get_zabove(nav_msgs::Path pathin,float z0,float zN,float hdng_cutoff){
  nav_msgs::Path path_above_z = cutoff_abs(pathin,"zrel",5,-6);
  draw_path_at_img(path_above_z,pos,false,false,false,false,true,get_color(0,0,200),1);
  //drawimg(hdng_cutoff,pos_yaw,"above_z");
}
geometry_msgs::PolygonStamped create_bbpoly(float dmin,float dmax,float z,float a0,float an){
  geometry_msgs::PolygonStamped poly;
  poly.polygon.points.resize(5);
  poly.header = hdr();
  poly.polygon.points[0].x = pos.x + dmin * cos(a0)+pos_yaw;
  poly.polygon.points[0].y = pos.y + dmin * sin(a0+pos_yaw);
  poly.polygon.points[0].z = z;

  poly.polygon.points[1].x = pos.x + dmin * cos(an+pos_yaw);
  poly.polygon.points[1].y = pos.y + dmin * sin(an+pos_yaw);
  poly.polygon.points[1].z = z;

  poly.polygon.points[2].x = pos.x + dmax * cos(an+pos_yaw);
  poly.polygon.points[2].y = pos.y + dmax * sin(an+pos_yaw);
  poly.polygon.points[2].z = z;

  poly.polygon.points[3].x = pos.x + dmax * cos(a0+pos_yaw);
  poly.polygon.points[3].y = pos.y + dmax * sin(a0+pos_yaw);
  poly.polygon.points[3].z = z;

  poly.polygon.points[4] = poly.polygon.points[0];
  return poly;
}
void work_front_scan(nav_msgs::Path pathin,float a,float hdng_cutoff){
  float a1 = constrainAngle(a + hdng_cutoff);
  float a2 = constrainAngle(a - hdng_cutoff);
  float a0 = fmin(a1,a2);
  float an = fmax(a1,a2);
  std::vector<float> vals_z   = get_vec_attribute(pathin,"z");
  draw_path_by_score(pathin,vals_z,0,1,0,0.2);
  nav_msgs::Path path_all_front = cutoff_abs(pathin,"hdng_rel",a0,an);

                     vals_z   = get_vec_attribute(path_all_front,"z");
  std::vector<float> vals_dst = get_vec_attribute(path_all_front,"dst_2d");

  float mx_z   = vec_to_max(vals_z);
  float mn_z   = vec_to_min(vals_z);

  float mx_dst = vec_to_max(vals_dst);
  float mn_dst = vec_to_min(vals_dst);


  geometry_msgs::PolygonStamped poly_bottom = create_bbpoly(mn_dst,mx_dst,mn_z,a0,an);
  geometry_msgs::PolygonStamped poly_top    = create_bbpoly(mn_dst,mx_dst,mx_z,a0,an);

  geometry_msgs::Point poly_centroid = get_poly_centroidarea(poly_bottom);
  float poly_area   = abs(poly_centroid.z);
  float poly_volume = poly_area * (mx_z - mn_z);

  poly_centroid.z = (mx_z + mn_z)/2.0;

  ROS_INFO("POLY FRONT: dst2d: %.0f -> %.0f a0aN: %.2f -> %.2f z0zN: %.0f -> %.0f",mn_dst,mx_dst,a0,an,mn_z,mx_z);
  ROS_INFO("POLY FRONT: dst2d:     %.0f     a0aN:     %.2f          z0zN: %.0f",mx_dst-mn_dst,get_shortest(an,a0),mx_z-mn_z);
  ROS_INFO("POLY FRONT: poly: area:  %.0f volume: %.0f centroid: [%.0f %.0f %.0f]",poly_area,poly_volume,poly_centroid.x,poly_centroid.y,poly_centroid.z);
  draw_path_by_score(path_all_front,vals_z,1,2,0,1.0);
  draw_poly(poly_bottom,get_color(200,0,200));
  cv::circle(img,pnt2cv(scanpoint_actual),3,get_color(0,200,200),1);
  cv::circle(img,pnt2cv(scanpoint_actual_lo),3,get_color(0,200,200),1);
  cv::circle(img,pnt2cv(scanpoint_actual_hi),3,get_color(0,200,200),1);
  ROS_INFO("scanpoint_actual:    %.0f %.0f %.0f",scanpoint_actual.x,scanpoint_actual.y,scanpoint_actual.z);
  ROS_INFO("scanpoint_actual_lo: %.0f %.0f %.0f",scanpoint_actual_lo.x,scanpoint_actual_lo.y,scanpoint_actual_lo.z);
  ROS_INFO("scanpoint_actual_hi: %.0f %.0f %.0f",scanpoint_actual_hi.x,scanpoint_actual_hi.y,scanpoint_actual_hi.z);
  std::string s ="scan_" + std::to_string(count_target_paths) + "_a0_"+std::to_string(a0)+"_an_"+std::to_string(an);
  drawimg(hdng_cutoff,a,s);
}

void pathhdngcleared_cb(const nav_msgs::Path::ConstPtr& msg){
  path_obs = *msg;
}
void pathcleared_cb(const nav_msgs::Path::ConstPtr& msg){
  img_blank.copyTo(img);
  nav_msgs::Path path_ll  = cutoff_abs(*msg,"hdng_rel",-M_PI,-M_PI/4);
  nav_msgs::Path path_l   = cutoff_abs(*msg,"hdng_rel",-M_PI/4,-M_PI/12);
  nav_msgs::Path path_m   = cutoff_abs(*msg,"hdng_rel",-M_PI/12,M_PI/12);
  nav_msgs::Path path_r   = cutoff_abs(*msg,"hdng_rel",M_PI/12,M_PI/4);
  nav_msgs::Path path_rr  = cutoff_abs(*msg,"hdng_rel",M_PI/4,M_PI);
  geometry_msgs::Point pnt_ll = get_ave_pnt(path_ll);
  geometry_msgs::Point pnt_l  = get_ave_pnt(path_l);
  geometry_msgs::Point pnt_m  = get_ave_pnt(path_m);
  geometry_msgs::Point pnt_r  = get_ave_pnt(path_r);
  geometry_msgs::Point pnt_rr = get_ave_pnt(path_rr);
  draw_path_at_img(path_ll,pos,false,false,false,false,true,get_color(200,200,0),1);
  draw_path_at_img(path_l,pos,false,false,false,false,true,get_color(200,0,200),1);
  draw_path_at_img(path_m,pos,false,false,false,false,true,get_color(200,100,200),1);
  draw_path_at_img(path_r,pos,false,false,false,false,true,get_color(200,0,200),1);
  draw_path_at_img(path_rr,pos,false,false,false,false,true,get_color(200,200,0),1);
  drawimg(M_PI/4,pos_yaw,"targets");
  float fac1 = 1.5;
  float fac2 = 2.5;
  int size_rr = path_rr.poses.size(); int size_ll = path_ll.poses.size();
  int size_r = path_r.poses.size() * fac1; int size_l = path_l.poses.size() * fac1;
  int size_m = path_m.poses.size() * fac2;
  ROS_INFO("Size  ll: %i l: %i m: %i r: %i rr: %i",path_ll.poses.size(),path_l.poses.size(),path_m.poses.size(),path_r.poses.size(),path_rr.poses.size());
  ROS_INFO("Score ll: %i l: %i m: %i r: %i rr: %i",size_ll,size_l,size_m,size_r,size_rr);
  geometry_msgs::Point pnt;
  if(size_rr > size_ll && size_rr > size_l && size_rr > size_m && size_rr > size_r)
    pnt = get_ave_pnt(path_rr);
  else if(size_ll > size_rr && size_ll > size_l && size_ll > size_m && size_ll > size_r)
    pnt = get_ave_pnt(path_rr);
  else if(size_l > size_rr && size_l > size_r && size_l > size_m && size_l > size_ll)
    pnt = get_ave_pnt(path_rr);
  else if(size_r > size_rr && size_r > size_l && size_r > size_m && size_r > size_ll)
    pnt = get_ave_pnt(path_rr);
  else
    pnt  = get_ave_pnt(path_m);
  ROS_INFO("Pnt: %.0f %.0f %.0f",pnt.x,pnt.y,pnt.z);
  if(std::isnan(pnt.x) || std::isnan(pnt.y) || std::isnan(pnt.z)){
    ROS_INFO("NANWARN");
  }
  else{
    geometry_msgs::PoseStamped ps;
  	ps.pose.position = pnt;
  	//set_target_pose(ps);
  }
}
void check(){
  float hdng_cutoff = M_PI/4;
  counter++;
  ros::Time t0 = ros::Time::now();
 //sensor_msgs::LaserScan scan_comb = merge_scans(scan_dn,scan_mi,scan_up);
/*
  nav_msgs::Path path_hic    = get_pathfinalclear(scan_hi);
  nav_msgs::Path path_all    = merge_paths(get_pathvector(path_lo,path_mi,path_hi,path_up,path_dn));
  nav_msgs::Path path_all_mi = cutoff_abs(path_all,"hdng_rel",-hdng_cutoff,hdng_cutoff);
  nav_msgs::Path path_lo_mi  = cutoff_abs(path_lo,"hdng_rel",-hdng_cutoff,hdng_cutoff);
  nav_msgs::Path path_hi_mi  = cutoff_abs(path_hi,"hdng",-hdng_cutoff,hdng_cutoff);
  std::vector<float> vals = get_vec_attribute(path_all_mi,"z");
  count_target_paths++;
  scanpoint_actual    = get_ave_pnt(path_all);
  scanrange_actual    = get_dst3d(pos,scanpoint_actual);
  scanpoint_actual_lo = get_ave_pnt(path_hi_mi);
  scanpoint_actual_hi = get_ave_pnt(path_lo_mi);

  float front_cutoff = M_PI/12;
//  work_front_scan(path_all,vlp_rpy.z,front_cutoff);
  ROS_INFO("Path RAW: dn: %i lo: %i mi: %i hi: %i up: %i",path_dn.poses.size(),path_lo.poses.size(),path_mi.poses.size(),path_up.poses.size(),path_hi.poses.size());
*/


  // target_angle      = get_inclination(pos,z_max);
//   setpoint_altitude = z_max.z + par_zclearing;
//   scanrange_actual  = get_dst3d(scanpoint_actual,pos);
//   float dst_zpnt = get_dst3d(pos,z_max);
//   if(dst_zpnt <  10)
//     setpoint_altitude = pos.z + 5;

  //eval_area_simple(path_all,hdng_cutoff);
  //draw_path_by_attribute(path_all_mi,"inclination");
  //drawimg(hdng_cutoff,"inclination");
  //draw_basepaths(path_lo,path_mi,path_hi,path_up,path_dn);



//  drawimg(hdng_cutoff,"smax");
//  initialscan(scan_hi);
//  drawimg(hdng_cutoff,"initscan1");
//  initialscan2(scan_hi);
//  drawimg(hdng_cutoff,"initscan2");

  //get_hi(path_hi,path_hic,hdng_cutoff);

//  drawimg(hdng_cutoff,"base");

//  drawimg(hdng_cutoff,"all_z");
//get_above_pos(path_lo,path_mi,path_hi,path_up,path_dn);

}

void update_front(){
  nav_msgs::Path path_all = merge_paths(get_pathvector(path_hi_mi,path_lo_mi,path_up_mi,path_dn_mi,path_mi_mi));
  scanpoint_actual        = get_ave_pnt(path_all);
}
geometry_msgs::PolygonStamped path_to_bb_poly(nav_msgs::Path pathin,bool use_data){
  pathin = cutoff_outliers(pathin,"z");
  pathin = cutoff_outliers(pathin,"dst_2d");
  std::vector<float> vals_dst  = get_vec_attribute(pathin,"dst_2d");
  std::vector<float> vals_z    = get_vec_attribute(pathin,"z");
  std::vector<float> vals_hdng = get_vec_attribute(pathin,"hdng_rel");

  float av_dst = vec_to_ave(vals_dst);
  float mn_dst = vec_to_min(vals_dst);
  float mx_dst = vec_to_max(vals_dst);
  float av_z = vec_to_ave(vals_z);
  float mn_z = vec_to_min(vals_z);
  ROS_INFO("Mn_z: %.0f",mn_z);

  float mx_z = vec_to_max(vals_z);
  float av_hdng = vec_to_ave(vals_hdng);
  float mn_hdng = vec_to_min(vals_hdng);
  float mx_hdng = vec_to_max(vals_hdng);

  geometry_msgs::PolygonStamped poly_bottom = create_bbpoly(mn_dst,mx_dst,mn_z,mn_hdng,mx_hdng);
  geometry_msgs::PolygonStamped poly_top    = create_bbpoly(mn_dst,mx_dst,mx_z,mn_hdng,mx_hdng);
  ROS_INFO("POLY FRONT: dst2d: %.0f -> %.0f a0aN: %.2f -> %.2f z0zN: %.0f -> %.0f",mn_dst,mx_dst,mn_hdng,mx_hdng,mn_z,mx_z);
  ROS_INFO("POLY FRONT: dst2d:     %.0f     a0aN:     %.2f          z0zN: %.0f",mx_dst-mn_dst,get_shortest(mn_hdng,mx_hdng),mx_z-mn_z);

  geometry_msgs::Point poly_centroid = get_poly_centroidarea(poly_bottom);
  float poly_area   = abs(poly_centroid.z);
  float poly_volume = poly_area * (mx_z - mn_z);
  poly_centroid.z = (mx_z + mn_z)/2.0;
  ROS_INFO("POLY FRONT: poly: area:  %.0f volume: %.0f centroid: [%.0f %.0f %.0f]",poly_area,poly_volume,poly_centroid.x,poly_centroid.y,poly_centroid.z);
  if(use_data){
    std::vector<geometry_msgs::Point> bbvec = getinpath_boundingbox(pathin);
    scanzone_bbmin = bbvec[0];
    scanzone_bbmax = bbvec[1];
    scanzone_size_pnts = pathin.poses.size();
    scanzone_size_area = poly_area;
    scanzone_size_ddst = mx_dst-mn_dst;
    scanzone_size_dalt = mx_z-mn_z;
    scanzone_avepnt   = get_ave_pnt(pathin);
    scanzone_centroid = poly_centroid;
    scanzone_dst_min = mn_dst;
    scanzone_dst_max = mx_dst;
    scanzone_z_min = mn_z;
    scanzone_z_max = mx_z;
    ROS_INFO("SCANZONE: pnts %i, mm: %.0f, ddst: %.0f, zdst: %.0f pnt_ave: [%.0f %.0f %.0f] pnt_cen: [%.0f %.0f %.0f]",scanzone_size_pnts,scanzone_size_ddst,scanzone_size_ddst,scanzone_size_dalt,scanzone_avepnt.x,scanzone_avepnt.y,scanzone_avepnt.z,scanzone_centroid.x,scanzone_centroid.y,scanzone_centroid.z);
  }
  return poly_bottom;
}


void rayranges_cb(const sensor_msgs::LaserScan::ConstPtr& msg){
  path_mi = get_pathfinal(*msg);
  int half_size = msg->ranges.size()/2;
  int best_dst =1111;
  float best_rng = 1111;
  float best_ang = 0;
  for(int i = 0;i < msg->ranges.size(); i++){
    int dst_from_mid = abs(i - half_size);
    if(dst_from_mid < best_dst && msg->ranges[i] < best_dst*1.3){
      best_ang = msg->angle_min + msg->angle_increment * i;
      best_dst = dst_from_mid;
      best_rng = msg->ranges[i];
      ROS_INFO("BEST DST NEW: %.0f @ %.2f rads, %i dst from mid",best_rng,best_ang,dst_from_mid);
    }
  }
  geometry_msgs::PointStamped pnt;
  pnt.point.x = best_rng * cos(best_ang);
  pnt.point.y = best_rng * sin(best_ang);
  pnt.header.stamp   = ros::Time(0);
  pnt.header.frame_id = msg->header.frame_id;
  pnt = tfBuffer.transform(pnt, "map");
  scanpoint_actual2 = pnt.point;
  //nav_msgs::Path path_forw cutoff_abs(path_mi,"hdng_rel",-M_PI/24,M_PI/24);
  path_mi_mi = cutoff_abs(path_mi,"hdng_rel",-hdng_cutoff,hdng_cutoff);
  path_mi_ri = cutoff_abs(path_mi,"hdng_rel",-hdng_cutoff*2,-hdng_cutoff);
  path_mi_le = cutoff_abs(path_mi,"hdng_rel",hdng_cutoff,hdng_cutoff*2);
}
void rayranges_hi_cb(const sensor_msgs::LaserScan::ConstPtr& msg){
  path_hi = get_pathfinal(*msg);
  path_hi_mi = cutoff_abs(path_hi,"hdng_rel",-hdng_cutoff,hdng_cutoff);
  path_hi_ri = cutoff_abs(path_hi,"hdng_rel",-hdng_cutoff*2,-hdng_cutoff);
  path_hi_le = cutoff_abs(path_hi,"hdng_rel",hdng_cutoff,hdng_cutoff*2);
}
void rayranges_lo_cb(const sensor_msgs::LaserScan::ConstPtr& msg){
  path_lo = get_pathfinal(*msg);
  path_lo_mi = cutoff_abs(path_lo,"hdng_rel",-hdng_cutoff,hdng_cutoff);
  path_lo_ri = cutoff_abs(path_lo,"hdng_rel",-hdng_cutoff*2,-hdng_cutoff);
  path_lo_le = cutoff_abs(path_lo,"hdng_rel",hdng_cutoff,hdng_cutoff*2);
}
void scan_up_cb(const sensor_msgs::LaserScan::ConstPtr& msg){
  path_up = get_pathfinal(*msg);
  path_up_mi = cutoff_abs(path_up,"hdng_rel",-hdng_cutoff,hdng_cutoff);
  path_up_ri = cutoff_abs(path_up,"hdng_rel",-hdng_cutoff*2,-hdng_cutoff);
  path_up_le = cutoff_abs(path_up,"hdng_rel",hdng_cutoff,hdng_cutoff*2);
}
void scan_dn_cb(const sensor_msgs::LaserScan::ConstPtr& msg){
  path_dn = get_pathfinal(*msg);
  path_dn_mi = cutoff_abs(path_dn,"hdng_rel",-hdng_cutoff,hdng_cutoff);
  path_dn_ri = cutoff_abs(path_dn,"hdng_rel",-hdng_cutoff*2,-hdng_cutoff);
  path_dn_le = cutoff_abs(path_dn,"hdng_rel",hdng_cutoff,hdng_cutoff*2);
}
void scan_stab_cb(const sensor_msgs::LaserScan::ConstPtr& msg){
  path_st = get_pathfinal(*msg);
  path_st_mi = cutoff_abs(path_st,"hdng_rel",-hdng_cutoff,hdng_cutoff);
  path_st_ri = cutoff_abs(path_st,"hdng_rel",-hdng_cutoff*2,-hdng_cutoff);
  path_st_le = cutoff_abs(path_st,"hdng_rel",hdng_cutoff,hdng_cutoff*2);
  update_front();
}

void define_colors(){
  c_lo[0] = 200;
  c_lo[1] = 0;
  c_lo[2] = 0;

  c_dn[0] = 100;
  c_dn[1] = 100;
  c_dn[2] = 0;

  c_mi[0] = 0;
  c_mi[1] = 50;
  c_mi[2] = 50;

  c_up[0] =  0;
  c_up[1] =  100;
  c_up[2] =  100;

  c_hi[0] = 0;
  c_hi[1] = 0;
  c_hi[2] = 200;

  c_sz[0] = 135;
  c_sz[1] = 125;
  c_sz[2] = 255;
}
void altmax_cb(const std_msgs::Float64::ConstPtr& msg){
  altmax = msg->data;
}
void do_analyze(nav_msgs::Path pathin){
  pathin = sort_path(pathin,"dst_2d");
  std::vector<float> vals_dst  = get_vec_attribute(pathin,"dst_2d");
  std::vector<float> vals_z    = get_vec_attribute(pathin,"z");
  std::vector<float> vals_incl = get_vec_attribute(pathin,"inclination");

  if(pathin.poses.size() == 0)
    return;
  float mn_dst = get_dst2d(pathin.poses[0].pose.position,pos);
  float mx_dst = get_dst2d(pathin.poses[pathin.poses.size()-1].pose.position,pos);

  std::vector<float> dst_interval_inclmax;
  std::vector<float> dst_interval_inclmin;
  std::vector<float> dst_interval_inclave;

  std::vector<float> dst_interval_zmax;
  std::vector<float> dst_interval_zmin;
  std::vector<float> dst_interval_zave;

  std::vector<float> dst_interval_dst0;
  std::vector<float> dst_interval_dstN;
  std::vector<int> dst_interval_count;
  int interval_meters = 5;
  int num_intervals = (mx_dst - mn_dst) / interval_meters;

  float sum_z = 0;
  float sum_incl = 0;
  int interval_count = 0;
  int interval_i = 0;
  dst_interval_count.resize(num_intervals);

  dst_interval_dst0.resize(num_intervals);
  dst_interval_dstN.resize(num_intervals);

  dst_interval_zmax.resize(num_intervals);
  dst_interval_zmin.resize(num_intervals);
  dst_interval_zave.resize(num_intervals);

  dst_interval_inclmax.resize(num_intervals);
  dst_interval_inclmin.resize(num_intervals);
  dst_interval_inclave.resize(num_intervals);

  dst_interval_dst0[0] = mn_dst;
  dst_interval_dstN[0] = mn_dst+interval_meters;

  for(int i= 0; i < num_intervals; i++){
    int s = 0;
    if(i > 0){
      dst_interval_dst0[i] = dst_interval_dstN[i-1];
      dst_interval_dstN[i] = dst_interval_dstN[i-1] + interval_meters;
    ROS_INFO("interval[%i] dst2d %.0f -> %.0f",i,dst_interval_dst0[i],dst_interval_dstN[i]);
    }
    dst_interval_zmin[i] = 100;
    dst_interval_inclmin[i] = 100;
  }
  ROS_INFO("Throug inti");
  nav_msgs::Path path_temp;
  int cnt = 0;
  for(int i = 0; i < pathin.poses.size(); i++){
    if(vals_dst[i] > dst_interval_dstN[interval_i]){
      dst_interval_count[interval_i] = interval_count;
      ROS_INFO("INFIRST [%i] int[%i / %i]: size: %i dst_2d[%.0f->%.0f] z[%.0f->%.0f] zave: %.0f incl: %.2f->%.2f ave: %.2f",i,interval_i,interval_count,dst_interval_count[interval_i],dst_interval_dst0[interval_i],dst_interval_dstN[interval_i],dst_interval_zmin[interval_i],dst_interval_zmax[interval_i],dst_interval_zave[interval_i],dst_interval_inclmin[interval_i],dst_interval_inclmax[interval_i]);
      int s = 0;
      dst_interval_zave[interval_i]  = sum_z / fmax(1,interval_count);
      dst_interval_inclave[interval_i] = sum_incl / fmax(1,interval_count);
      interval_count = 0;
      sum_z = 0;
      sum_incl = 0;
      interval_i++;
      cnt++;
      geometry_msgs::Point pnt;
      draw_path_at_img(path_temp,pos,false,false,false,false,true,get_color(100,0,0),1);
//  draw_path_at_img(,pos,false,false,false,false,false,true,get_shifting_color(cnt,int(dst_interval_zave[interval_i] * 10)),1);
      path_temp.poses.resize(0);
      if(cnt == 2)
        cnt = 0;
    }
    path_temp.poses.push_back(pathin.poses[i]);
    interval_count++;
    sum_z += vals_z[i];
    sum_incl += vals_incl[i];
    if(vals_incl[i] < dst_interval_inclmin[interval_i])
      dst_interval_inclmin[interval_i] = vals_incl[i];
    if(vals_incl[i] > dst_interval_inclmax[interval_i])
      dst_interval_inclmax[interval_i] = vals_incl[i];
    if(vals_z[i] < dst_interval_zmin[interval_i])
      dst_interval_zmin[interval_i] = vals_z[i];
    if(vals_z[i] > dst_interval_zmax[interval_i])
      dst_interval_zmax[interval_i] = vals_z[i];
  }
  float inclchange_sum = 0;
  float dincl_sum = 0;
  float zrngechange_sum = 0;
  float zchangemax_sum = 0;
  float zchangeave_sum = 0;
  float zrnge_sum = 0;
  for(int i = 0; i < dst_interval_zmax.size(); i++){
    zrnge_sum += (dst_interval_zmax[i] - dst_interval_zmin[i]);
    if(i > 0){
      zrngechange_sum += (dst_interval_zmax[i] - dst_interval_zmax[i-1]);
      zchangemax_sum += (dst_interval_zmax[i] - dst_interval_zmax[i-1]);
      zchangeave_sum += (dst_interval_zave[i] - dst_interval_zave[i-1]);
      inclchange_sum += (dst_interval_inclave[i] - dst_interval_inclave[i-1]);
      dincl_sum += (dst_interval_inclmax[i] - dst_interval_inclmin[i-1]);
    }
    ROS_INFO("interval[%i]: size: %i dst_2d[%.0f->%.0f] z[%.0f->%.0f] zave: %.0f incl: %.2f->%.2f ave: %.2f",i,dst_interval_count[i],dst_interval_dst0[i],dst_interval_dstN[i],dst_interval_zmin[i],dst_interval_zmax[i],dst_interval_zave[i],dst_interval_inclmin[i],dst_interval_inclmax[i]);
  }
  float inclchange_ave = inclchange_sum / dst_interval_zmax.size();
  float dincl_ave = dincl_sum / dst_interval_zmax.size();
  float zrngechange_ave = zrngechange_sum / dst_interval_zmax.size();
  float zchangemax_ave = zchangemax_sum / dst_interval_zmax.size();
  float zchangeave_ave = zchangeave_sum / dst_interval_zmax.size();
  float zrnge_ave = zrnge_sum / dst_interval_zmax.size();
  ROS_INFO("inclchange_ave: %.2f dincl_ave: %.2f zrngechange_ave: %.2f zchangemax_ave: %.2f zchangeave_ave: %.2f zrnge_ave: %.2f",inclchange_ave,dincl_ave,zrngechange_ave,zchangemax_ave,zchangeave_ave,zrnge_ave);
  std::string s ="analyzer" + std::to_string(count_target_paths);
  drawimg(hdng_cutoff,pos_yaw,s);
}


void check_front(){
  ros::Time t0 = ros::Time::now();
  float front_cutoff = M_PI/12;
  nav_msgs::Path path_all_mi = merge_paths(get_pathvector(path_mi_mi,path_hi_mi,path_lo_mi,path_up_mi,path_dn_mi));
  nav_msgs::Path path_all_ri = merge_paths(get_pathvector(path_mi_ri,path_hi_ri,path_lo_ri,path_up_ri,path_dn_ri));
  nav_msgs::Path path_all_le = merge_paths(get_pathvector(path_mi_le,path_hi_le,path_lo_le,path_up_le,path_dn_le));
  ROS_INFO("***************************************************************************************************************");
  path_all_mi = cutoff_outliers(path_all_mi,"z");
  path_all_ri = cutoff_outliers(path_all_ri,"z");
  path_all_le = cutoff_outliers(path_all_le,"z");
  std::vector<nav_msgs::Path> paths;
  paths.push_back(path_all_mi);
  paths.push_back(path_all_ri);
  paths.push_back(path_all_le);
  scanzone_path = merge_paths(paths);
  ROS_INFO("CHECK FRONT: GOT PATHVECTORS: %.i, %i, %i",path_all_mi.poses.size(),path_all_ri.poses.size(),path_all_le.poses.size());
//  do_analyze(path_all_mi);
//  do_analyze(path_all_ri);
//  do_analyze(path_all_le);
  if(path_all_mi.poses.size() > 10 && path_all_ri.poses.size() > 10 && path_all_le.poses.size() > 10){
    std::vector<float> vals_mi = get_vec_attribute(path_all_mi,"z");
    std::vector<float> vals_ri = get_vec_attribute(path_all_ri,"z");
    std::vector<float> vals_le = get_vec_attribute(path_all_le,"z");

    draw_path_by_score(path_all_mi,vals_mi,0,2,0,1.0);
    draw_path_by_score(path_all_ri,vals_ri,0,1,0,1.0);
    draw_path_by_score(path_all_le,vals_le,0,1,0,1.0);

    geometry_msgs::PolygonStamped  poly_all_mi = path_to_bb_poly(path_all_mi,true);
    geometry_msgs::PolygonStamped  poly_all_ri = path_to_bb_poly(path_all_ri,false);
    geometry_msgs::PolygonStamped  poly_all_le = path_to_bb_poly(path_all_le,false);

    draw_poly(poly_all_mi,get_color(200,0,200));
    draw_poly(poly_all_ri,get_color(0,200,200));
    draw_poly(poly_all_le,get_color(0,200,200));

    scanpoint_actual = get_ave_pnt(path_all_mi);
    scanrange_actual = get_dst3d(pos,scanpoint_actual);
    count_target_paths++;
  }
  cv::circle(img,pnt2cv(scanpoint_actual),3,get_color(0,200,200),1);
  cv::circle(img,pnt2cv(scanpoint_actual2),3,get_color(200,200,200),1);
  cv::circle(img,pnt2cv(scanpoint_actual_lo),3,get_color(0,200,200),1);
  cv::circle(img,pnt2cv(scanpoint_actual_hi),3,get_color(0,200,200),1);

  ROS_INFO("scanpoint_actual:    %.0f %.0f %.0f",scanpoint_actual.x,scanpoint_actual.y,scanpoint_actual.z);
  ROS_INFO("scanpoint_actual_lo: %.0f %.0f %.0f",scanpoint_actual_lo.x,scanpoint_actual_lo.y,scanpoint_actual_lo.z);
  ROS_INFO("scanpoint_actual_hi: %.0f %.0f %.0f",scanpoint_actual_hi.x,scanpoint_actual_hi.y,scanpoint_actual_hi.z);
  std::string s ="points_front"+std::to_string(count_target_paths);

  drawimg(hdng_cutoff,pos_yaw,s);
  float dt_last_check = (ros::Time::now()-t0).toSec();
  ROS_INFO("CHECK COMPLETE IN %.4f",dt_last_check);
  ROS_INFO("***************************************************************************************************************");
}
void pathvstd_cb(const nav_msgs::Path::ConstPtr& msg){
  path_visited = *msg;
}
nav_msgs::Path create_path(){
  nav_msgs::Path pathout;
  pathout.header = hdr();
	float area_sidelength = 50;
	float total_sidelength = 1000;
	int num_grids = total_sidelength / area_sidelength-1;
  geometry_msgs::PoseStamped pose;
  pose.pose.position.z    = 10;
  pose.pose.orientation.w = 1;
  pose.header = hdr();
  float len = 50;
  for(int i = 0; i < 10; i++){
    for(int k = 0; k < i; k++){
			pose.pose.position.x += pow(-1,i) * len;
      //ROS_INFO("Pnt[%i],x: %.0f y: %.0f",pathout.poses.size(),pose.pose.position.x,pose.pose.position.y);
      pathout.poses.push_back(pose);
    }
    for(int l = 0; l < i; l++){
      pose.pose.position.y += pow(-1,i) * len;
    //  ROS_INFO("Pnt[%i],x: %.0f y: %.0f",pathout.poses.size(),pose.pose.position.x,pose.pose.position.y);
      pathout.poses.push_back(pose);
    }
  }
	return pathout;
}
void override_cb(const std_msgs::String::ConstPtr& msg){
  override = true;
  if(msg->data == "tilt_up"){
    override_tilt++;
  }
  if(msg->data == "tilt_dn"){
    override_tilt--;
  }
  if(msg->data == "alt_up"){
    override_alt++;
  }
  if(msg->data == ""){
    override_alt = 10;
  }
  if(msg->data == "alt_dn"){
    override_alt--;
  }
}
float set_tilt_scanpoint(geometry_msgs::Point target_scanpoint){
  return get_inclination(target_scanpoint,pos);
}
float set_tilt_terrain(geometry_msgs::Point target_terrain){
  return get_inclination(target_terrain,pos);
}
int main(int argc, char** argv)
{
  ros::init(argc, argv, "tb_behavior_node");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");
	private_nh.param("par_vlpmaxtilt",par_vlpmaxtilt, M_PI/14);
	private_nh.param("par_vlpmintilt",par_vlpmintilt, -M_PI/14);
	private_nh.param("par_vlptiltinterval",par_vlptiltinterval, M_PI/10);
	private_nh.param("takeoff_altlvl",par_takeoffaltitude, 5.0);
  private_nh.param("setpoint_scanrange", par_scanrange, 25.0);//*2.0);
  private_nh.param("setpoint_zclearing", par_zclearing, 5.0);//*2.0);
	tf2_ros::TransformListener tf2_listener(tfBuffer);
  ros::Subscriber s0 = nh.subscribe("/tb_path/cleared_poly",1,poylcleared_cb);
  ros::Subscriber s01 = nh.subscribe("/tb_path/cleared",1,pathcleared_cb);
  pub_cmd      				= nh.advertise<geometry_msgs::Point>("/cmd_pos",10);
  pub_cmdpose       	= nh.advertise<geometry_msgs::PoseStamped>("/tb_cmd/posemb",10);
  ros::Publisher pub_scanpoint = nh.advertise<geometry_msgs::PointStamped>("/tb_path/scanpoint",10);
  ros::Publisher pub_scanpoint2	= nh.advertise<geometry_msgs::PointStamped>("/tb_path/scanpoint2",10);
  pub_get_next_path	 	= nh.advertise<std_msgs::UInt8>("/tb_path/request", 100);
  ros::Subscriber s5 = nh.subscribe("/tb_path/visited",10,pathvstd_cb);
  ros::Subscriber ss5 = nh.subscribe("/tb_path/hdng_clear",10,pathhdngcleared_cb);

  ros::Subscriber oc       = nh.subscribe("/override",10,override_cb);
  ros::Subscriber as1       = nh.subscribe("/velodyne_scan",10,rayranges_cb);
  ros::Subscriber a1        = nh.subscribe("/scan_down",10,scan_dn_cb);
  ros::Subscriber a3        = nh.subscribe("/scan_up",10,scan_up_cb);
  ros::Subscriber a6        = nh.subscribe("/scan_stabilized",10,scan_stab_cb);
  ros::Subscriber a2        = nh.subscribe("/scan_tilt_up",10,rayranges_hi_cb);
  ros::Subscriber as4       = nh.subscribe("/scan_tilt_down",10,rayranges_lo_cb);
	//ros::Subscriber sss = nh.subscribe("/cmd_pos",            100,&cmdpos_cb);
  ros::Subscriber z1        = nh.subscribe("/tb_path/altmax",10,altmax_cb);

  ros::Subscriber s3 = nh.subscribe("/tb_path/superpath_side",10,superpathside_cb);
  ros::Subscriber s4 = nh.subscribe("/tb_path/superpath_down",10,superpathdown_cb);
  ros::Subscriber as3 = nh.subscribe("/tb_path/raw_down",10,rawdown_cb);
  ros::Subscriber as5 = nh.subscribe("/tb_path/raw_side",10,rawside_cb);
  ros::Subscriber as13 = nh.subscribe("/tb_behav/targets",10,targets_cb);
  //ros::Subscriber as15 = nh.subscribe("/tb_behav/down_best",10,down_best_cb);
  //ros::Subscriber as1b4 = nh.subscribe("/tb_behav/side_best",10,side_best_cb);
  ros::Subscriber asb4 = nh.subscribe("/tb_obs/closest_pos",10,closest_pos_cb);
  ros::Subscriber asbb4 = nh.subscribe("/tb_obs/closest_left",10,closest_left_cb);
  ros::Subscriber assaa4 = nh.subscribe("/tb_obs/closest_right",10,closest_right_cb);
  ros::Subscriber assa4 = nh.subscribe("/tb_obs/closest_mid",10,closest_mid_cb);
  ros::Subscriber avssa4 = nh.subscribe("/tb_obs/closest_blw",10,closest_blw_cb);
	ros::Subscriber s1124 = nh.subscribe("/tb_draw/cluster",10,superpath_cb);
	ros::Subscriber s114 = nh.subscribe("/tb_draw/cluster",10,superpath_cb);
  ros::Publisher pub_altcmd	 = nh.advertise<std_msgs::Float64>("/tb_cmd/alt_target", 100);
  ros::Publisher pub_tiltvlp = nh.advertise<std_msgs::Float64>("/tb_cmd/arm1_tilt", 10);
  ros::Publisher pub_scanzone_path = nh.advertise<nav_msgs::Path>("/tb_scanzone_path", 10);
	ros::Rate rate(5.0);
  ros::Time last_check;
  ros::Time start= ros::Time::now();

  define_colors();
  bool ros_inform = false;
  setpoint_altitude = 15;
  float dt;
  float z_err,s_err,t_err;
  float z_i,s_i,t_i;
  float z_d,s_d,t_d;
  float pz_P,ps_P,pt_P;
  float pz_D,ps_D,pt_D;
  float pz_I,ps_I,pt_I;
  pz_P = ps_P = pt_P = 1;
  pz_D = ps_D = pt_D = 0.3;
  pz_I = ps_I = pt_I = 0.01;

  float z_err_last,s_err_last,t_err_last;

  ros::Time time_last = ros::Time::now();

  std_msgs::Float64 arm1_tilt_msg;
  std_msgs::Float64 target_alt_msg;
  float setpoint_tilt = 0;
  double syaw;
  while(ros::ok()){
    dt= (ros::Time::now() - time_last).toSec();
    time_last = ros::Time::now();
    rate.sleep();
    ros::spinOnce();
    update_path_full();
    look_in_directions(32,50,10);
    geometry_msgs::PointStamped scanpoint2_msg,scanpoint_msg;
    scanpoint_msg.header = hdr();
    scanpoint2_msg.header = hdr();
    scanpoint_msg.point  = scanpoint_actual;
    scanpoint2_msg.point = scanpoint_actual2;
    pub_scanpoint.publish(scanpoint_msg);
    pub_scanpoint2.publish(scanpoint2_msg);
    if((ros::Time::now()-last_check).toSec() > 1.0){
      check_front();
      last_check = ros::Time::now();
    }
    checktf();

    float inc_cmd     = get_inclination(cmd_pos,pos);
    float inc_scanpnt = get_inclination(scanpoint_actual,pos);
    float velxy = sqrt(pow(odom.twist.twist.linear.x,2)+pow(odom.twist.twist.linear.y,2));
    bool use_cmd = true;
    float inc_used = inc_cmd;
    if(inc_scanpnt > inc_cmd){
      inc_used = inc_scanpnt;
      use_cmd = false;
    }
//    float max_obsz = fmax(obs_p.z,obs_m.z);
  //  ROS_INFO("Altitude target inc_cmd:  %.0f xyz: %.0f %.0f %.0f",inc_cmd,cmd_pos.x,cmd_pos.y,cmd_pos.z);
  //  ROS_INFO("Altitude scapnt inc_pnt:  %.0f xyz: %.0f %.0f %.0f",inc_scanpnt,scanpoint_actual.x,scanpoint_actual.y,scanpoint_actual.z);
    bool use_altitude_pnt_directly = true;
    float vz          = velxy * inc_used;
  //  if(use_altitude_pnt_directly){
  //    setpoint_altitude = fmax(cmd_pos.z,scanpoint_actual.z + par_zclearing);
  //  }
  //  else
//      setpoint_altitude += vz * dt;
  //  ROS_INFO("Altitude scapnt inc_used: %.0f vxy: %.2f vz: %.2f z: %.2f",inc_used,velxy,vz,setpoint_altitude);
//    if(max_obsz > setpoint_altitude)
  //    setpoint_altitude = max_obsz + 3;
    //ROS_INFO("Target angle %.2f from %.2f clearing %.2f scantarget",target_angle,par_zclearing,par_scanrange);

    scanzone_path.header = hdr();
    pub_scanzone_path.publish(scanzone_path);

    nav_msgs::Path path_obs_mi = cutoff_abs(path_obs,"hdng_rel",-hdng_cutoff,hdng_cutoff);
    int percent_cleared = percent_clear(path_obs_mi);
    geometry_msgs::PointStamped closest_danger = get_danger();

    geometry_msgs::Point closest_obs   = get_closepnts();
    float decision_elevation  = get_elevation_below2();
    ROS_INFO("Closest danger: %.0f %.0f %.0f obs: %.0f %.0f %.0f decision_elevation: %.0f",closest_danger.point.x,closest_danger.point.y,closest_danger.point.z,closest_obs.x,closest_obs.y,closest_obs.z,decision_elevation);
    setpoint_altitude = decision_elevation + par_zclearing;
    if(setpoint_tilt > M_PI/8)
      setpoint_tilt = 0;
    ROS_INFO("SETPOINTS: altitude: %.0f scanrange: %.0f target_angle: %.2f setpoint_tilt: %.2f",setpoint_altitude,par_scanrange,target_angle,setpoint_tilt);
    //if(percent_cleared < 60)
    target_angle = calculate_tilt();

    if((ros::Time::now() - start).toSec() < 5){
      setpoint_altitude =15;
      // calculate_setpoint_altitude();
    }
    else{
      if(percent_cleared > 80)
        t_err = 0.5;
      else
        t_err = -0.5;
      setpoint_altitude = decision_elevation + par_zclearing;
    }
    target_angle = get_a_from_dz_r(par_zclearing,par_scanrange);
    t_err = set_tilt_scanpoint(scanpoint_actual) - vlp_rpy.y;
    z_err = setpoint_altitude - pos.z;
    s_err = par_scanrange - scanrange_actual;
    //t_err = setpoint_tilt - vlp_rpy.y;
    //t_err =
    if(std::isnan(dt))
      dt = 0.1;
    if(std::isnan(z_err))
      z_err = 0;
    if(std::isnan(s_err))
      s_err = 0;
    if(std::isnan(t_err))
      t_err = 0;
    if(std::isnan(dt))
      dt = 0.1;
    if(std::isnan(z_i))
      z_i = 0;
    if(std::isnan(s_i))
      s_i = 0;
    if(std::isnan(t_i))
      t_i = 0;
    if(std::isnan(z_d))
      z_d = 0;
    if(std::isnan(s_d))
      s_d = 0;
    if(std::isnan(t_d))
      t_d = 0;
    if(z_i > 5.0 || z_i < -5.0)
      z_i = 0;
    if(s_i > 5.0 || s_i < -5.0)
      s_i = 0;
    if(t_i > 5.0 || t_i < -5.0)
      t_i = 0;
    if(z_d > 5.0 || z_d < -5.0)
      z_d = 0;
    if(s_d > 5.0 || s_d < -5.0)
      s_d = 0;
    if(t_d > 5.0 || t_d < -5.0)
      t_d = 0;
    z_i += z_err*dt; s_i += s_err*dt;  t_i += t_err*dt;

    z_d = (z_err - z_err_last) / dt;
    s_d = (s_err - s_err_last) / dt;
    t_d = (t_err - t_err_last) / dt;

    z_err_last = z_err;
    s_err_last = s_err;
    t_err_last = t_err;

    float cmd_z   = (z_err * pz_P      + z_d * pz_D     + z_i * pz_I);
    float cmd_s   = (s_err * ps_P      + s_d * pz_D     + s_i * ps_I);
    float cmd_t   = (t_err * pt_P      + t_d * pz_D     + t_i * pt_I);
    if(scanrange_actual < 10){
      if(cmd_z < 0.5)
        cmd_z = 0.5;
    }
    arm1_tilt_msg.data  -= (cmd_t*dt);
    target_alt_msg.data += (cmd_z*dt);
  //  setpoint_tilt -= (cmd_s * dt);
    if(target_alt_msg.data > altmax + 5){
      target_alt_msg.data = altmax+5;
    }
    if(ros_inform){
      ROS_INFO("z_err: %.2f (%.2f - %.2f)",z_err,setpoint_altitude,pos.z);
      ROS_INFO("s_err: %.2f (%.2f - %.2f)",s_err,par_scanrange,scanrange_actual);
      ROS_INFO("t_err: %.2f (%.2f - %.2f)",t_err,target_angle,vlp_rpy.y);
      ROS_INFO("z_cmd:  %.2f d: %.2f, i %.2f, cmd: %.2f final_cmd: %.2f",z_err,z_d,z_i,cmd_z,target_alt_msg.data);
      ROS_INFO("scan:   %.2f d: %.2f, i %.2f, cmd: %.2f ",s_err,s_d,s_i,cmd_s);
      ROS_INFO("tilt:   %.2f d: %.2f, i %.2f, cmd: %.2f final_cmd: %.2f",t_err,t_d,t_i,cmd_t,arm1_tilt_msg.data);
    }
    if(override){
      arm1_tilt_msg.data = override_tilt * M_PI/24;
      target_alt_msg.data = override_alt * 2.0;
      ROS_INFO("OVERRIDE: alt: %.2f tilt: %.2f",target_alt_msg.data,arm1_tilt_msg.data);
    }
    if(std::isnan(target_alt_msg.data)){
      ROS_INFO("NANWARN target alt");
      target_alt_msg.data = 15;
      pub_altcmd.publish(target_alt_msg);
    }
    else
      pub_altcmd.publish(target_alt_msg);
    if(std::isnan(arm1_tilt_msg.data))
      arm1_tilt_msg.data = target_angle;
    else
      pub_tiltvlp.publish(arm1_tilt_msg);


  }
  return 0;
}
