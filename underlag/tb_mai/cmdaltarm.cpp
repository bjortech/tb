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

geometry_msgs::Vector3 vlp_rpy;
nav_msgs::Odometry odom;
double par_zclearing,par_scanrange;
const float deg2rad = M_PI/180.0;
const float rad2deg = 180.0/M_PI;
const float earth = 6378137.0;
geometry_msgs::Point cmd_pos,pos,scanpoint_actual;
cv::Mat img(1000,1000,CV_8UC3,cv::Scalar(0, 0, 0));
cv::Mat img_blank(1000,1000,CV_8UC3,cv::Scalar(0, 0, 0));
float scanrange_actual;
std_msgs::Header hdr(){
  std_msgs::Header header;
  header.frame_id = "map";
  header.stamp    = ros::Time::now();
  return header;
}
float get_dst3d(geometry_msgs::Point p1, geometry_msgs::Point p2){
  return(sqrt(pow(p1.x-p2.x,2)+pow(p1.y-p2.y,2)+pow(p1.z-p2.z,2)));
}
float get_dst2d(geometry_msgs::Point p1, geometry_msgs::Point p2){
  return(sqrt(pow(p1.x-p2.x,2)+pow(p1.y-p2.y,2)));
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
double constrainAngle(double x){
  if(x > M_PI)
    return (x - M_PI*2);
  else if(x < -M_PI)
    return (x + M_PI*2);
  else
    return x;
}
geometry_msgs::Point max_score(std::vector<float> scores){
  geometry_msgs::Point minmax;
  minmax.x = -100;
  minmax.y = 100;
  for(int i = 0; i < scores.size(); i++){
    if(minmax.x < scores[i]){
      minmax.x = scores[i];
    }
    if(minmax.y > scores[i]){
      minmax.y = scores[i];
    }
  }
  return minmax;
}
float rel_score(std::vector<float> scores, float score){
  geometry_msgs::Point minmax = max_score(scores);
  return (score - minmax.y) / (minmax.x - minmax.y);
}

cv::Scalar rel_color(std::vector<float> scores, float score, std::string rgb){
  cv::Scalar c;
  float relscore = rel_score(scores,score);
  if(rgb == "r")
    c[2] = relscore;
  if(rgb == "g")
    c[1] = relscore;
  if(rgb == "b")
    c[0] = relscore;
  return c;
 }

void draw_poly(geometry_msgs::PolygonStamped polyin,cv::Scalar color){
  geometry_msgs::Point p1,p2;

  for(int i = 1; i < polyin.polygon.points.size(); i++){
    p1.x = polyin.polygon.points[i-1].x;
    p1.y = polyin.polygon.points[i-1].y;
    p2.x = polyin.polygon.points[i].x;
    p2.y = polyin.polygon.points[i].y;
    cv::line (img, pnt2cv(p1), pnt2cv(p2),color,1,cv::LINE_8,0);
  }
  p1.x = polyin.polygon.points[0].x;
  p1.y = polyin.polygon.points[0].y;
  p2.x = polyin.polygon.points[polyin.polygon.points.size()-1].x;
  p2.y = polyin.polygon.points[polyin.polygon.points.size()-1].y;
  cv::line (img, pnt2cv(p1), pnt2cv(p2),color,1,cv::LINE_8,0);
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

geometry_msgs::PointStamped transformpoint(geometry_msgs::PointStamped pin,std::string frame_out){
  geometry_msgs::PointStamped pout;
  pin.header.stamp = ros::Time();
  try
  {
    pout = tfBuffer.transform(pin, frame_out);
  }
  catch (tf2::TransformException &ex)
  {
        ROS_WARN("Failure %s\n", ex.what());
  }
  return pout;
}

void cmdpos_cb(const geometry_msgs::Point::ConstPtr& msg)
{
  cmd_pos = *msg;
}
void lowrateodom_cb(const nav_msgs::Odometry::ConstPtr& msg){
  odom = *msg;
}
geometry_msgs::Point get_ave_pnt32(geometry_msgs::PolygonStamped polyin){
  geometry_msgs::Point pnt;
  for(int i = 1; i < polyin.polygon.points.size()-1; i++){
    pnt.x += polyin.polygon.points[i].x;
    pnt.y += polyin.polygon.points[i].y;
    pnt.z += polyin.polygon.points[i].z;
  }
  pnt.x /= polyin.polygon.points.size();
  pnt.y /= polyin.polygon.points.size();
  pnt.z /= polyin.polygon.points.size();
  ROS_INFO("AVEPNT: [%i polygonnts] - %.0f %.0f %.0f ",polyin.polygon.points.size(),pnt.x,pnt.y,pnt.z);
  return pnt;
}
float get_zmax32(geometry_msgs::PolygonStamped polyin){
  float zmx = 0;
  for(int i = 1; i < polyin.polygon.points.size()-1; i++){
    if(polyin.polygon.points[i].z > zmx){
      zmx = polyin.polygon.points[i].z;
    }
  }
  return zmx;
}
geometry_msgs::PolygonStamped get_polyfinal(std::vector<float> vec_r, std::vector<float> vec_a){
  geometry_msgs::PolygonStamped poly;
  geometry_msgs::PointStamped pnt,pnt_out;
  poly.header = hdr();
  poly.polygon.points.resize(vec_r.size() + 2);
  pnt.header.frame_id = "velodyne_aligned";
  for(int i = 0; i < vec_r.size(); i++){
    pnt.point.x = vec_r[i] * cos(vec_a[i]);
    pnt.point.y = vec_r[i] * sin(vec_a[i]);
    pnt.header.stamp    = ros::Time();
    pnt_out = tfBuffer.transform(pnt, "map");
    //ROS_INFO("PNT: [%.0f %.0f %.0f] -> [%.0f %.0f %.0f]",pnt.point.x,pnt.point.y,pnt.point.z,pnt_out.point.x,pnt_out.point.y,pnt_out.point.z);
    poly.polygon.points[i+1].x = pnt_out.point.x;
    poly.polygon.points[i+1].y = pnt_out.point.y;
    poly.polygon.points[i+1].z = pnt_out.point.z;
  }
  pnt.point.x = 0;
  pnt.point.y = 0;
  pnt_out = tfBuffer.transform(pnt, "map");
  poly.polygon.points[0].x = pnt_out.point.x;
  poly.polygon.points[0].y = pnt_out.point.y;
  poly.polygon.points[0].z = pnt_out.point.z;
  poly.polygon.points[poly.polygon.points.size()-1] = poly.polygon.points[0];
  return poly;
}
cv::Scalar get_color(int base_blue,int base_green, int base_red){
	cv::Scalar color;
	color[0] = base_blue;
	color[1] = base_green;
	color[2] = base_red;
	return color;
}
void rayranges_cb(const sensor_msgs::LaserScan::ConstPtr& msg){
  ROS_INFO("Rayranges CALLBACK");

  int cnt;
  img_blank.copyTo(img);
  geometry_msgs::PolygonStamped polyleft,polymid,polyright,polyfull,polyfull_max;
  geometry_msgs::PolygonStamped poly,polyleft_max,polymid_max,polyright_max;
  geometry_msgs::Point32 p0,pn,pm;

  float rel_len_sum;
  float r_sum_left = 0;
  float r_sum_mid = 0;
  float r_sum_right = 0;
  float angle_max = msg->angle_max;
  float angle_mid = angle_max / 2;
  poly.polygon.points.resize(msg->ranges.size());
  bool i_inactive;
  int start_i,end_i;
  end_i = 0;
  start_i = 0;

  std::vector<float> l_ranges;
  std::vector<float> l_hdngs;
  std::vector<float> m_ranges;
  std::vector<float> m_hdngs;
  std::vector<float> r_ranges;
  std::vector<float> r_hdngs;
  std::vector<float> ranges;
  std::vector<float> hdngs;
  std::vector<float> ranges_tot;
  std::vector<float> hdngs_tot;
  std::vector<geometry_msgs::PointStamped> points;
  std::vector<geometry_msgs::PointStamped> points_transformed;
  int veci_1 = round((msg->ranges.size())/3);
  ROS_INFO("veci_1. %i of %i",veci_1,msg->ranges.size());
  if(veci_1 == 0)
    return;
  for(int i = 0; i < msg->ranges.size(); i++){
    float a = msg->angle_min + msg->angle_increment * i;
    float r = msg->ranges[i];
    ranges.push_back(r);
    hdngs.push_back(a);
    ranges_tot.push_back(r);
    hdngs_tot.push_back(a);
    if(i == veci_1){
      l_ranges = ranges;
      l_hdngs = hdngs;
      ranges.resize(0);
      hdngs.resize(0);
      ranges.push_back(r);
      hdngs.push_back(a);
    }
    if(i == veci_1*2){
      m_ranges = ranges;
      m_hdngs = hdngs;
      ranges.resize(0);
      hdngs.resize(0);
      ranges.push_back(r);
      hdngs.push_back(a);
    }
  }
  r_ranges = ranges;
  r_hdngs  = hdngs;

  float amid = hdngs_tot[hdngs_tot.size()/2];
  geometry_msgs::Point startpoint;
  float da_l = l_hdngs[l_hdngs.size()-1] - l_hdngs[0];
  float da_m = m_hdngs[m_hdngs.size()-1] - m_hdngs[0];
  float da_r = r_hdngs[r_hdngs.size()-1] - r_hdngs[0];
  float da   = hdngs_tot[hdngs.size()-1] - hdngs_tot[0];

  polyleft     = get_polyfinal(l_ranges,l_hdngs);
  polymid      = get_polyfinal(m_ranges,m_hdngs);
  polyright    = get_polyfinal(r_ranges,r_hdngs);
  polyfull_max = get_polyfinal(ranges_tot,hdngs_tot);

  geometry_msgs::Point cen_l = get_poly_centroidarea(polyleft);
  geometry_msgs::Point cen_m = get_poly_centroidarea(polymid);
  geometry_msgs::Point cen_r = get_poly_centroidarea(polyright);
  geometry_msgs::Point cen_a = get_poly_centroidarea(polyfull_max);

  float l_ave = vec_to_ave(l_ranges);
  float l_max = vec_to_max(l_ranges);
  float l_min = vec_to_min(l_ranges);
  float m_ave = vec_to_ave(m_ranges);
  float m_max = vec_to_max(m_ranges);
  float m_min = vec_to_min(m_ranges);
  float r_ave = vec_to_ave(r_ranges);
  float r_max = vec_to_max(r_ranges);
  float r_min = vec_to_min(r_ranges);

  float l_area = abs(cen_l.z);
  float m_area = abs(cen_m.z);
  float r_area = abs(cen_r.z);
  float a_area = abs(cen_a.z);

	geometry_msgs::Point pnt_l = get_ave_pnt32(polyleft);
	geometry_msgs::Point pnt_m = get_ave_pnt32(polymid);
	geometry_msgs::Point pnt_r = get_ave_pnt32(polyright);

  float dst_pnt_l = get_dst3d(pnt_l,pos);
  float dst_pnt_m = get_dst3d(pnt_m,pos);
  float dst_pnt_r = get_dst3d(pnt_r,pos);

  float zmax_l = get_zmax32(polyleft);
  float zmax_m = get_zmax32(polymid);
  float zmax_r = get_zmax32(polyright);

  scanrange_actual = m_ave;
  scanpoint_actual = pnt_m;
  ROS_INFO("Left-angle:  %i - %.2f - %.2f rads_da: %.3f",l_hdngs.size(),l_hdngs[0],l_hdngs[l_hdngs.size()-1],da_l);
  ROS_INFO("Mid-angle:   %i - %.2f - %.2f rads_da: %.3f",m_hdngs.size(),m_hdngs[0],m_hdngs[m_hdngs.size()-1],da_m);
  ROS_INFO("Right-angle: %i - %.2f - %.2f rads_da: %.3f",r_hdngs.size(),r_hdngs[0],r_hdngs[r_hdngs.size()-1],da_r);

  ROS_INFO("Left-range:  %.0f->%.0f ave: %.0f area: %.0f (%.0f tot)",l_min,l_max,l_ave,l_area,a_area);
  ROS_INFO("Mid-range:   %.0f->%.0f ave: %.0f area: %.0f (%.0f tot)",m_min,m_max,m_ave,m_area,a_area);
  ROS_INFO("Right-range: %.0f->%.0f ave: %.0f area: %.0f (%.0f tot)",r_min,r_max,r_ave,r_area,a_area);

  ROS_INFO("PNT_AVE L: [%.0f %.0f %.0f] DST L: [%.0f] ZMAX L: [%.0f]",pnt_l.x,pnt_l.y,pnt_l.z,dst_pnt_l,zmax_l);
  ROS_INFO("PNT_AVE M: [%.0f %.0f %.0f] DST M: [%.0f] ZMAX M: [%.0f]",pnt_m.x,pnt_m.y,pnt_m.z,dst_pnt_m,zmax_m);
  ROS_INFO("PNT_AVE R: [%.0f %.0f %.0f] DST R: [%.0f] ZMAX R: [%.0f]",pnt_r.x,pnt_r.y,pnt_r.z,dst_pnt_r,zmax_r);

  ROS_INFO("SCANRANGE: [%.2f] SCANPOINT USED: [%.0f %.0f %.0f] DST M: [%.0f] ZMAX M: [%.0f]",pnt_m.x,pnt_m.y,pnt_m.z,dst_pnt_m,zmax_m);

  draw_poly(polyfull_max,get_color(0,100,0));
  draw_poly(polyleft,get_color(100,0,0));
  draw_poly(polymid,get_color(0,100,0));
  draw_poly(polyright,get_color(100,0,0));
  geometry_msgs::Point pyaw;
  pyaw.x = pos.x + scanrange_actual * cos(vlp_rpy.z);
  pyaw.y = pos.y + scanrange_actual * sin(vlp_rpy.z);
  cv::circle(img,pnt2cv(scanpoint_actual),3,get_color(0,0,255),1);
  cv::circle(img,pnt2cv(pos),3,get_color(200,200,200),1);
  cv::line (img, pnt2cv(pos), pnt2cv(pyaw),get_color(200,200,200),1,cv::LINE_8,0);
  cv::line (img, pnt2cv(pos), pnt2cv(scanpoint_actual),get_color(0,0,255),1,cv::LINE_8,0);
  cv::imwrite("/home/nuc/brain/"+std::to_string(scanrange_actual)+"clear.png",img);

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

	pos.x   = transformStamped.transform.translation.x;
	pos.y   = transformStamped.transform.translation.y;
	pos.z   = transformStamped.transform.translation.z;
	tf2::Matrix3x3 q(tf2::Quaternion(transformStamped.transform.rotation.x, transformStamped.transform.rotation.y, transformStamped.transform.rotation.z, transformStamped.transform.rotation.w));
	q.getRPY(vlp_rpy.x,vlp_rpy.y,vlp_rpy.z);
}
float get_slope(geometry_msgs::Point p1, geometry_msgs::Point p2){
  return (p2.z - p1.z) / get_dst2d(p1,p2);
}
float get_inclination(geometry_msgs::Point p1, geometry_msgs::Point p2){
  return atan2(p2.z - p1.z,get_dst2d(p1,p2));
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
float get_tilt_error(){
  float target_angle = get_a_from_dz_r(par_zclearing,par_scanrange);
  float error_angle  = target_angle - vlp_rpy.y;
  return error_angle;
}
int main(int argc, char **argv){
    ros::init(argc, argv, "tb_cmdaltarm_node");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");
    private_nh.param("setpoint_scanrange", par_scanrange, 25.0);//*2.0);
    private_nh.param("setpoint_zclearing", par_zclearing, 5.0);//*2.0);

    ros::Rate rate(50);

    tf2_ros::TransformListener tf2_listener(tfBuffer);

    tf2_ros::TransformBroadcaster tf_b;
    ros::Subscriber as4        = nh.subscribe("/tb_path/ranges2_cleared",10,rayranges_cb);
    ros::Subscriber sss        = nh.subscribe("/cmd_pos",               100,&cmdpos_cb);

    ros::Publisher pub_altcmd	 = nh.advertise<std_msgs::Float64>("/tb_cmd/alt_target", 100);
  	ros::Publisher pub_tiltvlp = nh.advertise<std_msgs::Float64>("/tb_cmd/arm1_tilt", 10);
    cmd_pos.z = 15;
    float dt;
    float setpoint_altitude;
    float setpoint_scanrange;
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

    double syaw;
    while(ros::ok()){
      dt= (ros::Time::now() - time_last).toSec();
      time_last = ros::Time::now();
      rate.sleep();
      ros::spinOnce();

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

      ROS_INFO("Altitude target inc_cmd:  %.0f xyz: %.0f %.0f %.0f",inc_cmd,cmd_pos.x,cmd_pos.y,cmd_pos.z);
      ROS_INFO("Altitude scapnt inc_pnt:  %.0f xyz: %.0f %.0f %.0f",inc_scanpnt,scanpoint_actual.x,scanpoint_actual.y,scanpoint_actual.z);
      bool use_altitude_pnt_directly = true;
      float vz          = velxy * inc_used;
      if(use_altitude_pnt_directly){
          setpoint_altitude = fmax(cmd_pos.z,scanpoint_actual.z + par_zclearing);
      }
      else
        setpoint_altitude += vz * dt;
      ROS_INFO("Altitude scapnt inc_used: %.0f vxy: %.2f vz: %.2f z: %.2f",inc_used,velxy,vz,setpoint_altitude);

      float target_angle = get_a_from_dz_r(par_zclearing,par_scanrange);
      ROS_INFO("Target angle %.2f from %.2f clearing %.2f scantarget",target_angle,par_zclearing,par_scanrange);
      z_err = setpoint_altitude - pos.z;
      s_err = par_scanrange - scanrange_actual;
      t_err = target_angle - vlp_rpy.y;
      ROS_INFO("z_err: %.2f (%.2f - %.2f)",z_err,setpoint_altitude,pos.z);
      ROS_INFO("s_err: %.2f (%.2f - %.2f)",s_err,par_scanrange,scanrange_actual);
      ROS_INFO("t_err: %.2f (%.2f - %.2f)",t_err,target_angle,vlp_rpy.y);
      if(std::isnan(t_err))
        t_err = 0;
      if(abs(z_i) > 5)
        z_i = 0;
      if(abs(s_i) > 5)
        s_i += 0;
      if(abs(t_i) > 5)
        t_i = 0;
      if(abs(z_d) > 5)
        z_d = 0;
      if(abs(s_d) > 5)
        s_d = 0;
      if(abs(t_d) > 5)
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

      arm1_tilt_msg.data  += (cmd_t*dt);
      target_alt_msg.data += (cmd_z*dt);

      ROS_INFO("z_cmd:  %.2f d: %.2f, i %.2f, cmd: %.2f final_cmd: %.2f",z_err,z_d,z_i,cmd_z,target_alt_msg.data);
      ROS_INFO("scan:   %.2f d: %.2f, i %.2f, cmd: %.2f ",s_err,s_d,s_i,cmd_s);
      ROS_INFO("tilt:   %.2f d: %.2f, i %.2f, cmd: %.2f final_cmd: %.2f",t_err,t_d,t_i,cmd_t,arm1_tilt_msg.data);
      if(std::isnan(target_alt_msg.data))
        ROS_INFO("NANWARN target alt");
      else
        pub_altcmd.publish(target_alt_msg);
      if(std::isnan(arm1_tilt_msg.data))
        ROS_INFO("NANWARN target tilt");
      else
        pub_tiltvlp.publish(arm1_tilt_msg);

    }
    return 0;
}
