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


  using namespace octomap;
  using namespace std;

  tf2_ros::Buffer tfBuffer;
  ros::Publisher pub_path_request,pub_path_visited,pub_idle_time,pub_target_approved,targetpoly_pub,target_pub,targetalt_pub;

  bool bag_published,par_live,got_map,got_hi,got_mid,got_down;
  int n_sectors,buildingstate,missionstate,mainstate,xmin,ymin,zmin,xmax,ymax,zmax,range_x,range_y,range_z,zmin_global,zmax_global;
  double par_target_mindst,pos_yaw,par_n_sectors,par_maprad,par_patience,par_takeoffaltitude,par_zjump,closest_obstacle_dist;

  std_msgs::UInt8 state_msg,altlvl_msg;
  std_msgs::Float64 alt_target,idle_time;

  geometry_msgs::Point bbmin_custom,bbmax_custom,bbmin_octree,bbmax_octree,pos;
  geometry_msgs::PointStamped building_centroid,closest_obstacle;
  geometry_msgs::PoseStamped last_pose,target;
  geometry_msgs::PolygonStamped poly_heading,pl,pm,ph;
  nav_msgs::Path path_visited,path_full,path;
  geometry_msgs::Point plo,pmi,phi;

  std::vector<int> z_lvls;
  std::vector<geometry_msgs::PoseStamped> targetcmds_sent;

  shared_ptr<DynamicEDTOctomap> edf_ptr;
  AbstractOcTree* abs_octree;
  shared_ptr<OcTree> octree;
  geometry_msgs::Vector3Stamped path_request;

  sensor_msgs::LaserScan slo,smi,shi;
  const float rad2deg = 180.0/M_PI;

  bool sort_dst_pair(const std::tuple<int,float>& a,const std::tuple<int,float>& b){
      return (std::get<1>(a) < std::get<1>(b));
  }
  std_msgs::Header hdr(){
    std_msgs::Header header;
    header.frame_id = "map";
    header.stamp    = ros::Time::now();
    return header;
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
  double constrainAngle(double x){
    if(x > M_PI)
      return (x - M_PI*2);
    else if(x < -M_PI)
      return (x + M_PI*2);
    else
      return x;
  }
  float get_dst3d(geometry_msgs::Point p1, geometry_msgs::Point p2){
    return(sqrt(pow(p1.x-p2.x,2)+pow(p1.y-p2.y,2)+pow(p1.z-p2.z,2)));
  }
  float get_dst2d(geometry_msgs::Point p1, geometry_msgs::Point p2){
    return(sqrt(pow(p1.x-p2.x,2)+pow(p1.y-p2.y,2)));
  }
  float get_hdng(geometry_msgs::Point p1,geometry_msgs::Point p0){
    float dx = p1.x - p0.x;
    float dy = p1.y - p0.y;
    return atan2(dy,dx);
  }
  int get_zn(float z){
    for(int i = 0; i < z_lvls.size()-1; i++){
      if(z_lvls[i] < z && z_lvls[i+1] > z)
      return i;
    }
    return 0;
  }

  double get_shortest(double target_hdng,double actual_hdng){
    double a = target_hdng - actual_hdng;
    if(a > M_PI)a -= M_PI*2;
    else if(a < -M_PI)a += M_PI*2;
    return a;
  }
  void octomap_callback(const octomap_msgs::Octomap& msg){
    abs_octree=octomap_msgs::fullMsgToMap(msg);
    octree.reset(dynamic_cast<octomap::OcTree*>(abs_octree));
    got_map = true;
  }

  float dst_visited_area(float x,float y, float z){
    float res,dst;
    geometry_msgs::Point pin;
    pin.x = x; pin.y = y; pin.z = z;
    res = 1000;
    if(path_visited.poses.size() == 0)
      return res;
    for(int i = 0; i < path_visited.poses.size(); i++){
      if(abs(path_visited.poses[i].pose.position.z - pin.z) < 2){
        dst = get_dst3d(path_visited.poses[i].pose.position,pin);
        if(dst < res)
          res = dst;
      }
    }
    return res;
  }

  int get_target(nav_msgs::Path pathin,float dst_min){
    if(pathin.poses.size() == 0)
      return 0;
    std::vector<std::tuple<int,float>>dst_tar;
    std::vector<std::tuple<int,float>>dst_cen;
    std::vector<std::tuple<int,float>>dst_tot;

    for(int i = 1; i < pathin.poses.size(); i++){
      float dst_tar_val = get_dst3d(pathin.poses[i].pose.position,target.pose.position);
      float dst_cen_val = get_dst3d(pathin.poses[i].pose.position,building_centroid.point);
      float dst_tot_val = dst_tar_val+dst_cen_val;
      ROS_INFO("Pathin[%i]: dst_target: %.2f, dst_centroid: %.2f, dst_tot: %.2f",i,dst_tar_val,dst_cen_val,dst_tot_val);
      if(dst_tar_val > dst_min){
        dst_tar.push_back(std::make_tuple(i,dst_tar_val));
        dst_cen.push_back(std::make_tuple(i,dst_cen_val));
        dst_tot.push_back(std::make_tuple(i,dst_tot_val));
      }
      else
        ROS_INFO("Pathin[%i]: DISCARDED",i);
    }
    sort(dst_tar.begin(),dst_tar.end(),sort_dst_pair);
    sort(dst_cen.begin(),dst_cen.end(),sort_dst_pair);
    sort(dst_tot.begin(),dst_tot.end(),sort_dst_pair);
    for(int i = 1; i < dst_tar.size(); i++){
      int tar = std::get<0>(dst_tar[i]);
      int cen = std::get<0>(dst_cen[i]);
      int tot = std::get<0>(dst_tot[i]);
      float res_tar = std::get<1>(dst_tar[i]);
      float res_cen = std::get<1>(dst_cen[i]);
      float res_tot = std::get<1>(dst_tot[i]);
      ROS_INFO("Rank[%i]: Index: %i, %i, %i, Meters: %.2f %.2f %.2f",i,tar,cen,tot,res_tar,res_cen,res_tot);
    }
    int  res_i    = std::get<0>(dst_tot[0]);
    float res_tar = std::get<1>(dst_tar[0]);
    float res_cen = std::get<1>(dst_cen[0]);
    float res_tot = std::get<1>(dst_tot[0]);
    ROS_INFO("Result: Index %i with total of %.2f meters (cen: %.2f tar: %.2f)",res_i,res_tar,res_cen,res_tot);
    return res_i;
  }
float closest_in_path(nav_msgs::Path pathin,geometry_msgs::Point pin,float dst_min){
  float res,dst;
  res = 1000;
  int res_i,dzabs;
  if(pathin.poses.size() == 0)
    return res;
  for(int i = 1; i < pathin.poses.size(); i++){
    dst = get_dst3d(pathin.poses[i].pose.position,pin);
    int dz = abs(pathin.poses[i].pose.position.z - last_pose.pose.position.z);
    if(dz < 2 && dst < res && dst > dst_min || dz < dzabs && dst < 3*res && dst > dst_min || res == 1000 && dst > dst_min){
      res = dst;
      res_i = i;
    }
  }
  return res_i;
}
void update_target(geometry_msgs::PoseStamped ps){
  ROS_INFO("MAIN: TARGET: hdng[%.2f]: %.2f %.2f %.2f ->  %.2f %.2f %.2f",get_hdng(target.pose.position,ps.pose.position),target.pose.position.x,target.pose.position.y,target.pose.position.z,ps.pose.position.x,ps.pose.position.y,ps.pose.position.z);
  target = ps;
  targetcmds_sent.push_back(ps);
  pub_target_approved.publish(ps);
}
void aquire_target_from_path(nav_msgs::Path pathin){
  int i,i_new;
  i = closest_in_path(pathin,target.pose.position,par_target_mindst);
  i_new = get_target(pathin,par_target_mindst);
  ROS_INFO("BLD: *************RESULT**************");
  ROS_INFO("BLD: closest_i: %i,i_new: %i",i,i_new);
  if(i_new < pathin.poses.size())
    update_target(pathin.poses[i_new]);
}

  void create_path(float area_sidelength,float area_alt,float centroid_sidelength){
    int num_centroids = int(round(area_sidelength / centroid_sidelength));
    for(int y = 0; y < num_centroids; y++){
      for(int x = 0; x < num_centroids; x++){
        geometry_msgs::PoseStamped pose;
        pose.header.frame_id      = "map";
        pose.header.stamp         = ros::Time::now();
        pose.pose.position.x      = pow(-1,y) * (x * centroid_sidelength - area_sidelength / 2 + centroid_sidelength/2);
        pose.pose.position.y      = -1 * (y * centroid_sidelength - area_sidelength / 2 + centroid_sidelength/2);
        pose.pose.position.z      = area_alt;
        pose.pose.orientation.w   = 1;
        path_full.poses.push_back(pose);
      }
      ROS_INFO("PERCEPTION:  path full %i",path_full.poses.size());
    }
  }

  bool update_edto(geometry_msgs::Point midpoint,float collision_radius,float maprad,float z0,float z1,bool unknownAsOccupied){
      octree.get()->getMetricMin(bbmin_octree.x,bbmin_octree.y,bbmin_octree.z);
      octree.get()->getMetricMax(bbmax_octree.x,bbmax_octree.y,bbmax_octree.z);
      zmin_global = int(round(bbmin_octree.z));
      zmax_global = int(round(bbmax_octree.z));

      z0 = fmin(z0,bbmax_octree.z-2);
      z1 = fmin(z1,bbmax_octree.z);
      bbmin_custom.x = midpoint.x-maprad;
      bbmin_custom.y = midpoint.y-maprad;
      bbmin_custom.z = z0;

      bbmax_custom.x = midpoint.x+maprad;
      bbmax_custom.y = midpoint.y+maprad;
      bbmax_custom.z = z1;
      float zmin_touse = fmax(bbmin_custom.z,bbmin_octree.z);
      float zmax_touse = fmin(bbmax_custom.z,bbmax_octree.z);

      if(zmax_touse < zmin_touse){
        zmax_touse = fmax(bbmin_custom.z,bbmin_octree.z);
        zmin_touse = fmin(bbmax_custom.z,bbmax_octree.z);
      }

      octomap::point3d boundary_min(fmax(bbmin_custom.x,bbmin_octree.x),fmax(bbmin_custom.y,bbmin_octree.y),zmin_touse);
      octomap::point3d boundary_max(fmin(bbmax_custom.x,bbmax_octree.x),fmin(bbmax_custom.y,bbmax_octree.y),zmax_touse);

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
  			ROS_INFO("PERCEPTION:  FAILED update_edto FAILED: xmin: %i, ymin: %i, zmin: %i, xmax: %i, ymax: %i, zmax: %i, range_x: %i, range_y: %i, range_z: %i ",xmin,ymin,zmin,xmax,ymax,zmax,range_x,range_y,range_z);
  			return false;
  		}
    //  ROS_INFO("PERCEPTION:  update_edto[%i points in  sec]: xmin: %i, ymin: %i, zmin: %i, xmax: %i, ymax: %i, zmax: %i, range_x: %i, range_y: %i, range_z: %i ",vol,xmin,ymin,zmin,xmax,ymax,zmax,range_x,range_y,range_z);
    return true;
  }

  geometry_msgs::PolygonStamped get_poly_surround(float collision_radius,float mapradused,int num_rays){
    geometry_msgs::PolygonStamped polygon;
    polygon.header.frame_id = "map";
    polygon.header.stamp = ros::Time::now();
    update_edto(last_pose.pose.position,collision_radius,mapradused,last_pose.pose.position.z+1,last_pose.pose.position.z-1,false);
    octomap::point3d p(last_pose.pose.position.x,last_pose.pose.position.y,last_pose.pose.position.z);
    octomap::point3d closestObst;
    float d;
    edf_ptr.get()->getDistanceAndClosestObstacle(p,d,closestObst);
    if(d < collision_radius && d > 0){
      closest_obstacle.point.x = closestObst.x();
      closest_obstacle.point.y = closestObst.y();
      closest_obstacle.point.z = closestObst.z();
      closest_obstacle_dist    = d;
    }
    else
      closest_obstacle_dist = 100;
    float rads_pr_i = 2*M_PI / num_rays;
    polygon.polygon.points.resize(num_rays);
    for(int i  = 0; i < num_rays; i++){
      Eigen::Vector3f pnt1_vec(last_pose.pose.position.x,last_pose.pose.position.y,last_pose.pose.position.z);
      Eigen::Vector3f pnt2_vec(last_pose.pose.position.x+mapradused*cos(i*rads_pr_i),mapradused*sin(i*rads_pr_i)+last_pose.pose.position.y,last_pose.pose.position.z);
      Eigen::Vector3f cur_vec = pnt1_vec;
      float tot_length = (pnt2_vec - pnt1_vec).norm();
      Eigen::Vector3f stride_vec;
      stride_vec = (pnt2_vec - pnt1_vec).normalized() * 1;
      bool visited_clear = true;
      float cur_ray_len=0;
      float distance = collision_radius-1;
      float next_check = collision_radius;
      while(distance > 2 && cur_ray_len < tot_length){
        cur_vec = cur_vec + stride_vec;
        cur_ray_len = (cur_vec-pnt1_vec).norm();
        point3d stridep(cur_vec.x(),cur_vec.y(),cur_vec.z());
        distance = edf_ptr.get()->getDistance(stridep);
        if(cur_ray_len >= next_check){
         next_check = cur_ray_len + dst_visited_area(cur_vec.x(),cur_vec.y(),cur_vec.z());
         if(dst_visited_area(cur_vec.x(),cur_vec.y(),cur_vec.z()) < 2)
           break;
       }
      }
      polygon.polygon.points[i].x = cur_vec.x();
      polygon.polygon.points[i].y = cur_vec.y();
      polygon.polygon.points[i].z = cur_vec.z();
    }
    return polygon;
  }

  geometry_msgs::PolygonStamped get_poly_heading(float hdng_delta,int num_points){
    geometry_msgs::PolygonStamped polygon;
    polygon.header.frame_id = "map";
    polygon.header.stamp = ros::Time::now();
    poly_heading.polygon.points.resize(num_points+1);
    poly_heading.polygon.points[0].x = last_pose.pose.position.x;
    poly_heading.polygon.points[0].y = last_pose.pose.position.y;
    poly_heading.polygon.points[0].z = last_pose.pose.position.z;
    float rads_pr_i = hdng_delta /num_points;
    geometry_msgs::PointStamped p,pout;
    p.header.frame_id = "base_stabilized";
    p.header.stamp = ros::Time();
    for(int i = 1; i < num_points; i++){
      float a = -hdng_delta/2 + rads_pr_i * i;
      p.point.x = 50*cos(a);
      p.point.y = 50*sin(a);
      p.point.z = last_pose.pose.position.z;
      pout = transformpoint(p,"map");
      poly_heading.polygon.points[i].x = pout.point.x;
      poly_heading.polygon.points[i].y = pout.point.y;
      poly_heading.polygon.points[i].z = pout.point.z;
    }
    poly_heading.polygon.points[num_points].x = last_pose.pose.position.x;
    poly_heading.polygon.points[num_points].y = last_pose.pose.position.y;
    poly_heading.polygon.points[num_points].z = last_pose.pose.position.z;
    return polygon;
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
    pos_yaw = tf::getYaw(transformStamped.transform.rotation);

    if(sqrt(pow(transformStamped.transform.translation.x - last_pose.pose.position.x,2)+ pow(transformStamped.transform.translation.y - last_pose.pose.position.y,2)+
      pow(transformStamped.transform.translation.z - last_pose.pose.position.z,2)) >= 1.00){

      plo = pmi = phi = last_pose.pose.position;
      plo.z = last_pose.pose.position.z - 1;
      phi.z = last_pose.pose.position.z + 1;
      last_pose.pose.position.x  = transformStamped.transform.translation.x;
      last_pose.pose.position.y  = transformStamped.transform.translation.y;
      last_pose.pose.position.z  = transformStamped.transform.translation.z;
      last_pose.pose.orientation = transformStamped.transform.rotation;
      last_pose.header           = transformStamped.header;
      path_visited.poses.push_back(last_pose);
      path_visited.header.stamp  = ros::Time::now();
      pub_path_visited.publish(path_visited);
    }
  /*  else{
       idle_time.data = (ros::Time::now() - path_visited.header.stamp).toSec();
       if(idle_time.data > 1)
        pub_idle_time.publish(idle_time);
    }*/
  }
  void floorcompletionpose_cb(const nav_msgs::Path::ConstPtr& msg){
  //  for(int i = 0; i < msg->poses.size(); i++){

//    }
    //aquire_target_from_path(*msg);
  }
  void test_transform_cb(const geometry_msgs::Point::ConstPtr& msg){
    geometry_msgs::PointStamped pin,pout,pout2;
    pin.header.frame_id = "map";
    pin.header.stamp    = ros::Time();
    pin.point = *msg;
    pout = transformpoint(pin,"base_stabilized");
    float hdng0 = atan2(pin.point.y,pin.point.x);
    float hdng = atan2(pout.point.y,pout.point.x);
    float hdngd = get_shortest(hdng,hdng0);
    ROS_INFO("PERCEPTION: Transformed point: %.0f %.0f %.0f in map to %.0f %.0f %.0f in base_stabilized (hdng(d: %.3f): %.3f -> %.3f)",pin.point.x,pin.point.y,pin.point.z,pout.point.x,pout.point.y,pout.point.z,hdngd,hdng0,hdng);
    pout2 = transformpoint(pout,"map");
    ROS_INFO("PERCEPTION: Transformed point back to map: %.0f %.0f %.0f",pout2.point.x,pout2.point.y,pout2.point.z);
    if(pout2.point.x == pin.point.x && pout2.point.y == pin.point.y && pout2.point.z == pin.point.z)
      ROS_INFO("PERCEPTION: SUCCESS!!!!!!!!!!!");
    else
      ROS_INFO("PERCEPTION: FAILURE!!!!!!!!!!!");
  }
  void buildingstate_cb(const std_msgs::UInt8::ConstPtr& msg){
    buildingstate = msg->data;
  }
  void state_cb(const std_msgs::UInt8::ConstPtr& msg){
    mainstate = msg->data;
  }
  void missionstate_cb(const std_msgs::UInt8::ConstPtr& msg){
    if(msg->data == 1 && missionstate != 1){
      ROS_INFO("PERCEPTION: Missionstate set from %i -> %i, will look for buildings!",missionstate,msg->data);
      missionstate = msg->data;
    }
  }
  void zlvl_cb(const std_msgs::UInt8::ConstPtr& msg){
    if(msg->data < z_lvls.size())
      target.pose.position.z = z_lvls[msg->data];
  }
  void path_cb(const nav_msgs::Path::ConstPtr& msg){
    path = *msg;
    aquire_target_from_path(*msg);
  }

  geometry_msgs::PolygonStamped merge_polygons(geometry_msgs::PolygonStamped polyin,geometry_msgs::PolygonStamped polyin2,geometry_msgs::PolygonStamped polyin3){
    geometry_msgs::PolygonStamped polyout;
    polyout.header = hdr();
    int s1 = polyin.polygon.points.size();
    int s2 = polyin2.polygon.points.size();
    int s3 = polyin3.polygon.points.size();
    polyout.polygon.points.resize(s1+s2+s3);
    for(int i = 0; i < s1; i++){
      polyout.polygon.points[i] = polyin.polygon.points[i];
    }
    for(int i = 0; i < s2; i++){
      polyout.polygon.points[s1+i] = polyin2.polygon.points[i];
    }
    for(int i = 0; i < s3; i++){
      polyout.polygon.points[s2+i] = polyin3.polygon.points[i];
    }
    return polyout;
  }
  std::vector<  geometry_msgs::PolygonStamped > compare_scans(){
    ROS_INFO("PERCEPTION: Comparing Scans");
    std::vector<int> status_is;
    std::vector<int> streak_start;
    std::vector<int> streak_stop;
    std::vector<int> degs;
    std::vector<float> rads;
    std::vector<  geometry_msgs::PolygonStamped > buildings;
    int degrees;
    bool streak;
    int evens   = 0;
    int clears  = 0;
    int unevens = 0;
    int last_streak_range;
    int maxrange =  int(round(slo.range_max));
    for(int i = 0; i < slo.ranges.size();i++){
      int  sl = int(round(slo.ranges[i]));
      int  sm = int(round(smi.ranges[i]));
      int  sh = int(round(shi.ranges[i]));
      float a =	constrainAngle(slo.angle_increment * i + slo.angle_min + pos_yaw);
      int deg = (a + M_PI)*rad2deg;
      degs.push_back(deg);
      rads.push_back(a);

      ROS_INFO("i: %i a: %.3f degs: %i rlo: %i rmi: %i rhi: %i",i,a,deg,sl,sm,sh);
      if(sl == maxrange && sh == maxrange && sm == maxrange){
        status_is.push_back(100);
        clears++;
        if(streak){
          streak = false;
          streak_stop.push_back(i);
          ROS_INFO("Streak ends at %i with last_streak_range: %i",i,last_streak_range);
        }
        streak = false;
      }
      else if(sl == sm && sm == sh){
        if(!streak){
          ROS_INFO("Streak start at %i",sm);
          streak = true;
          last_streak_range = sm;
          streak_start.push_back(i);
        }
        else{
          streak++;
          last_streak_range = sm;
        }
      }
    }
  /*    else if(sm == sl && sh > sm){
        if(streak_roof){
          streak = false;
          streak_stop.push_back(i);
          ROS_INFO("Streak ends after %i ranges at %i with last_streak_range: %i",i,last_streak_range);
        }
      }

      }
    }*/

    for(int i = 0; i < streak_start.size(); i++){
      int nums = streak_stop[i] - streak_start[i];
      geometry_msgs::PolygonStamped building;
      building.polygon.points.resize(nums*2);
      building.header = hdr();
      for(int k = streak_start[i]; k < streak_stop[i]; k++){
        building.polygon.points[k].x = status_is[k] * cos(rads[k]);
        building.polygon.points[k].y = status_is[k] * sin(rads[k]);
        building.polygon.points[k].z = last_pose.pose.position.z;
        building.polygon.points[nums-k].x = status_is[k]+1 * cos(rads[k]);
        building.polygon.points[nums-k].y = status_is[k]+1 * sin(rads[k]);
        building.polygon.points[k].z = last_pose.pose.position.z;
        ROS_INFO("Streak#%i,%i: deg: %i rnge: %i",i,k,degs[k],status_is[k]);
      }
      buildings.push_back(building);
    }
    return buildings;
  }

  geometry_msgs::PolygonStamped scan2poly(sensor_msgs::LaserScan scan,geometry_msgs::Point midpoint){
    ROS_INFO("PERCEPTION: Scan2Poly");
    geometry_msgs::PolygonStamped polyout;
    polyout.header.frame_id = "map";
    polyout.polygon.points.resize(scan.ranges.size());
    for(int i = 0; i < scan.ranges.size();i++){
      geometry_msgs::Point32 p0,p1,ps;
      float r = fmin(scan.ranges[i],scan.range_max);
      float a =	constrainAngle(scan.angle_increment * i + scan.angle_min + pos_yaw);
      polyout.polygon.points[i].x = round(midpoint.x) + r * cos(a);
      polyout.polygon.points[i].y = round(midpoint.y) + r * sin(a);
      polyout.polygon.points[i].z = midpoint.z;
    }
    return polyout;
  }
  void scan_dwn_cb(const sensor_msgs::LaserScan::ConstPtr& scan){
    ROS_INFO("PERCEPTION: scans_cb");
    slo.header.frame_id = "base_stabilized_down";
    slo.angle_min = scan->angle_min;
    slo.angle_max = scan->angle_max;
    slo.range_min = scan->range_min;
    slo.range_max = scan->range_max;
    slo.angle_increment = (scan->angle_max - scan->angle_min)/n_sectors;
    slo.ranges.resize(int(n_sectors));
    double is_pr_k = scan->ranges.size()/n_sectors;
    float scans_pr_sec = scan->ranges.size()/par_n_sectors;
    for(int k = 0; k < n_sectors; k++){
      smi.ranges.push_back(scan->range_max);
      for(int i = 0; i < scans_pr_sec; i++){
        if(scan->ranges[i] < smi.ranges[k])
          smi.ranges[k] = scan->ranges[i];
      }
    }
    slo.header.stamp = ros::Time::now();
    pl = scan2poly(slo,plo);
  }
  void scan_stabilized_cb(const sensor_msgs::LaserScan::ConstPtr& scan){
    ROS_INFO("PERCEPTION: scans_cb");
    smi.header.frame_id = "base_stabilized";
    smi.angle_min = scan->angle_min;
    smi.angle_max = scan->angle_max;
    smi.range_min = scan->range_min;
    smi.range_max = scan->range_max;
    smi.angle_increment = (scan->angle_max - scan->angle_min)/n_sectors;
    smi.ranges.resize(n_sectors);

    double is_pr_k = scan->ranges.size()/n_sectors;
    float scans_pr_sec = scan->ranges.size()/par_n_sectors;
    for(int k = 0; k < n_sectors; k++){
      smi.ranges.push_back(scan->range_max);
      for(int i = 0; i < scans_pr_sec; i++){
        if(scan->ranges[i] < smi.ranges[k])
          smi.ranges[k] = scan->ranges[i];
      }
    }
    smi.header.stamp = ros::Time::now();
    pm = scan2poly(smi,pmi);
  }
  void scan_up_cb(const sensor_msgs::LaserScan::ConstPtr& scan){
    ROS_INFO("PERCEPTION: scans_cb");
    shi.header.frame_id = "base_stabilized_up";
    shi.angle_min = scan->angle_min;
    shi.angle_max = scan->angle_max;
    shi.range_min = scan->range_min;
    shi.range_max = scan->range_max;
    shi.angle_increment = (scan->angle_max - scan->angle_min)/n_sectors;
    float scans_pr_sec = scan->ranges.size()/par_n_sectors;
    int k = 0;
    for(int k = 0; k < n_sectors; k++){
      shi.ranges.push_back(scan->range_max);
      for(int i = 0; i < scans_pr_sec; i++){
        if(scan->ranges[i] < shi.ranges[k])
          shi.ranges[k] = scan->ranges[i];
      }
    }
    shi.header.stamp = ros::Time::now();
    ph = scan2poly(shi,phi);
  }
  void init_stuff(){
    n_sectors = int(round(par_n_sectors));
    closest_obstacle.header.frame_id = path_full.header.frame_id = path_visited.header.frame_id = last_pose.header.frame_id = "map";
    target.pose.orientation.w = last_pose.pose.orientation.w = 1;
    path_visited.poses.push_back(last_pose);
    for(int i = 0; i < 40; i++){
      z_lvls.push_back(par_zjump*i);
    }
    ROS_INFO("PERCEPTION: STUFF init");
  }
  int main(int argc, char** argv)
  {
    ros::init(argc, argv, "tb_fsmperception_node");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");
    private_nh.param("par_zjump",   par_zjump, 3.0);//*2.0);
    private_nh.param("map_sidelength",par_maprad, 300.0);
    private_nh.param("n_sectors",par_n_sectors, 36.0);
    private_nh.param("par_target_mindst",   par_target_mindst, 3.0);//*2.0);

    tf2_ros::TransformListener tf2_listener(tfBuffer);
    float centroid_sides = 20;
    init_stuff();

    pub_path_visited    = nh.advertise<nav_msgs::Path>("/tb_nav/path_visited",10);
  //  pub_idle_time       = nh.advertise<std_msgs::Float64>("/tb_nav/idle_time",10);
    pub_target_approved = nh.advertise<geometry_msgs::PoseStamped>("/tb_cmd/approved",100);
  //
    ros::Publisher path_full_pub= nh.advertise<nav_msgs::Path>("/tb_nav/full_path",10);
    ros::Publisher polysafe_pub = nh.advertise<geometry_msgs::PolygonStamped>("/tb_obs/poly_safe",100);
    ros::Publisher polyhdng_pub = nh.advertise<geometry_msgs::PolygonStamped>("/tb_obs/poly_hdng",100);
    ros::Publisher pub_bld = nh.advertise<geometry_msgs::PolygonStamped>("/tb_obs/poly_building",100);
    ros::Publisher pub_closest_obstacle = nh.advertise<geometry_msgs::PointStamped>("/tb_obs/closest",100);
    ros::Publisher pub_centroid_initial = nh.advertise<geometry_msgs::PointStamped>("/tb_bld/centroid_initial",100);
    ros::Publisher pub_scanmergepoly = nh.advertise<geometry_msgs::PolygonStamped>("/tb_obs/scanmergepoly",100);

    ros::Subscriber s1  = nh.subscribe("/octomap_full",1,octomap_callback);
    ros::Subscriber s3  = nh.subscribe("/test_pointtransform",1,test_transform_cb);
    ros::Subscriber s33  = nh.subscribe("/tb_path",1,&path_cb);

    ros::Subscriber s4 = nh.subscribe("/tb_bld/building_state",1,buildingstate_cb);
    ros::Subscriber s5 = nh.subscribe("/tb_fsm/main_state",100,&state_cb);
    ros::Subscriber s7 = nh.subscribe("/tb_fsm/mission_state",100,&missionstate_cb);
    ros::Subscriber s8 = nh.subscribe("/tb_fsm/altlvl",100,&zlvl_cb);
    ros::Subscriber s9 = nh.subscribe("/tb_bld/path_floor",100,&floorcompletionpose_cb);
  //  ros::Subscriber s10 = nh.subscribe("/tb_path",100,&path_cb);
    ros::Subscriber s11 = nh.subscribe("/scan_dwn",10, scan_dwn_cb);
    ros::Subscriber s12 = nh.subscribe("/scan_stabilized",10, scan_stabilized_cb);
    ros::Subscriber s13 = nh.subscribe("/scan_up",10, scan_up_cb);

    //ros::Publisher cmdarm_pub = nh.advertise<std_msgs::Float64>("/tb_cmd/arm_pitch",10);
    std_msgs::Float64 cmdarm_msg;

    ros::Rate rate(2.0);
    ros::Time start = ros::Time::now();
    bool invoke_published,relative_not_abs_zn,use_scans;
    use_scans = true;
    ros::Time last_global_plan,last_info;
    int buildings_sent = 0;
      std::vector<geometry_msgs::PolygonStamped> buildings;
    while(ros::ok()){
      if(got_map){
        checktf();
        if(path_full.poses.size() == 0)
          create_path(par_maprad,par_takeoffaltitude,centroid_sides);
        ROS_INFO("PERCEPTION: PolyHdng");
        polyhdng_pub.publish(get_poly_heading(M_PI/2,16));
        polysafe_pub.publish(get_poly_surround(10,50,36));
        ROS_INFO("PERCEPTION: PolySafe");

        pub_closest_obstacle.publish(closest_obstacle);
        path_full_pub.publish(path_full);

          ROS_INFO("PERCEPTION: UseScans");
        pub_scanmergepoly.publish(merge_polygons(pl,pm,ph));
          ROS_INFO("PERCEPTION: buildings");

        //  buildings = compare_scans();
        if(buildings.size() > 0 && buildings_sent < buildings.size()){
          pub_bld.publish(buildings[buildings_sent]);
          buildings_sent++;
        }
        else
          buildings_sent = 0;


    /*    if(path_request.header.frame_id == ""){
            count++;
          if(count == 4){
            count = 0;
            request_path("update",buildingstate,2,15,100);
          }
          else if(mainstate == 1 && missionstate == 1){
            request_path("building",buildingstate,0,0,50);
          }
        }
        else if((ros::Time::now() - path_request.header.stamp).toSec() > 2.0){
          float dt = (ros::Time::now() - path_request.header.stamp).toSec();
          ROS_INFO("PERCEPTION: Path Request waited for %.3f s,requesting update",dt);
          request_path("update",buildingstate,2,15,100);

        }
        else{
          count = 0;
        }*/
      }
      rate.sleep();
      ros::spinOnce();
    }
    return 0;
  }
