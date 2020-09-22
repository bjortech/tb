












float cupd_weight=5;
float cnew_weight=5;
float emp1_weight=5;
float emp5_weight=3;
float zmax_weight=-4;
float numz_weight=-2;
float vstd_weight=-5;
float hdng_weight=-5;
float dstp_weight=2;
float dst0_weight=-5;
cv::Mat img_update(1000,1000,CV_8UC3,cv::Scalar(0, 0, 0)); //create image, set encoding and size, init pixels to default val

cv::Mat img_new(1000,1000,CV_8UC3,cv::Scalar(0, 0, 0)); //create image, set encoding and size, init pixels to default val



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


nav_msgs::Path get_path_within_m(nav_msgs::Path pathin,float meters){
  nav_msgs::Path pathout;
  pathout.header = hdr();
  if(path_visited.poses.size() > 0)
    pathout.poses.push_back(path_visited.poses[path_visited.poses.size()-1]);
  for(int i = 0; i < pathin.poses.size(); i++){
    if(meters > get_dst2d(pathin.poses[i].pose.position,pos))
      pathout.poses.push_back(pathin.poses[i]);
  }
  return pathout;
}
geometry_msgs::Quaternion get_quat(geometry_msgs::Point to, geometry_msgs::Point from){
  float incl = atan2(to.z - from.z,get_dst2d(to,from));
  float hdng = get_hdng(to,from);
  return tf::createQuaternionMsgFromRollPitchYaw(0,-incl,hdng);
}

void get_updated_mbmap(){
  map_msgs::OccupancyGridUpdate update;
  geometry_msgs::Point poly_centroid;
  float poly_area;
  nav_msgs::Path path_global_inoply;
  last_mbmidpnt.point = perfalt_pnt.point;
  float maprad_area = pow(par_maprad*2,2);
  float zrnge = state_octomap_zmax - state_octomap_zmin;
  float zint = zrnge / 5;

  float zmn = fmax(state_octomap_zmin,fmin(state_octomap_zmax-10,perfalt_pnt.point.z));
  float zmx = fmin(zmn+20,state_octomap_zmax);
  ROS_INFO("Zmn-mx: %.0f->%.0f (minmax: %.0f->%.0f)",zmn,zmx,state_octomap_zmin,state_octomap_zmax);
  float zrange = zmx-zmn;
  for(int i = 0; i < 5; i++){
    last_mbmidpnt.point.z = zmn + i * zrange/5.0;
    update             = update_mbmap(last_mbmidpnt.point,par_maprad);
    poly_centroid      = get_poly_centroidarea(poly_mb);
    path_global_inoply = constrain_path_bbpoly(path_global_plan,poly_mb);
    poly_area          = abs(poly_centroid.z);
    ROS_INFO("POLY FRONT: poly: area:  %.0f/%.0f pathglobal: %i, z: %.0f",poly_area,maprad_area,path_global_inoply.poses.size(),last_mbmidpnt.point.z);
  }
  pub_map_updates.publish(update);
}

nav_msgs::Path update_edto(nav_msgs::Path pathin,float collision_radius){
  geometry_msgs::Point bbmin_custom,bbmax_custom,bbmin_octree,bbmax_octree,bbmin,bbmax;
  pathin = scatter_path(get_path_within_m(pathin,20),2);
  std::vector<geometry_msgs::Point> bbmnmx = getinpath_boundingbox(pathin);
  bbmin_custom = bbmnmx[0];
  bbmax_custom = bbmnmx[1];
  bbmin_custom.z -= 5.0;
  bbmax_custom.z += 5.0;
  bbmin_custom.x -= 5.0;
  bbmax_custom.x += 5.0;
  bbmin_custom.y -= 5.0;
  bbmax_custom.y += 5.0;
  octree.get()->getMetricMin(bbmin_octree.x,bbmin_octree.y,bbmin_octree.z);
  octree.get()->getMetricMax(bbmax_octree.x,bbmax_octree.y,bbmax_octree.z);

  if(bbmin_custom.z >= bbmax_octree.z)
    return pathin;
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
          false));
  edf_ptr.get()->update();
  closest_obstacle.header = hdr();
  closest_obstacle.point.x = 0;
  closest_obstacle.point.y = 0;
  closest_obstacle.point.z = 0;
  for(int i = 0; i < pathin.poses.size(); i++){
    geometry_msgs::Point pnt = pathin.poses[i].pose.position;
    if(pnt.x < bbmax.x && pnt.x > bbmin.x
    && pnt.y < bbmax.y && pnt.y > bbmin.y){
      point3d closestObst;
      point3d p(pnt.x,pnt.y,pnt.z);
      float dst = collision_radius;
      edf_ptr.get()->getDistanceAndClosestObstacle(p, dst, closestObst);

      if(i == 0){
        if(dst < collision_radius){
          closest_obstacle.point.x = closestObst.x();
          closest_obstacle.point.y = closestObst.y();
          closest_obstacle.point.z = closestObst.z();
          closest_obstacle_dst = dst;
          //ROS_INFO("Closest obstacle: %.0f %.0f %.0f dst: %.0f",closest_obstacle.point.x,closest_obstacle.point.y,closest_obstacle.point.z,closest_obstacle_dst);
        }
      }
      else if(dst < min_dst){
        while(dst < min_dst){
          p.z() += 1.0;
          edf_ptr.get()->getDistanceAndClosestObstacle(p, dst, closestObst);
        }
      }
      else if(dst > max_dst){
        while(dst >= max_dst){
          p.z() -= 1;
          edf_ptr.get()->getDistanceAndClosestObstacle(p, dst, closestObst);
        }
      }
      if(closestObst.x() =! 0){
        geometry_msgs::Point pnt_co;
        pnt_co.x = closestObst.x();
        pnt_co.y = closestObst.y();
        pnt_co.z = closestObst.z();
        pathin.poses[i].pose.orientation = get_quat(pnt_co,pathin.poses[i].pose.position);
      }
      //ROS_INFO("Pathin: %i moved from Z: %.0f to %.0f",i,pnt.z,p.z());
      pathin.poses[i].pose.position.z = p.z();
    }
  }
  return pathin;
}
std::vector<geometry_msgs::Point> get_points_to_block(geometry_msgs::Point p0,float hdng,float block_a0,float block_an){
  std::vector<geometry_msgs::Point> vec_pnts;
  geometry_msgs::Point pnt;
  p0.x -= cos(hdng) * 5;
  p0.y -= sin(hdng) * 5;
  int len = 10;
  for(int i = 0; i < len; i++){
    pnt.x = p0.x + i * cos(hdng+block_a0);
    pnt.y = p0.y + i * sin(hdng+block_a0);
    pnt.x -= 1;
    vec_pnts.push_back(pnt);
    pnt.x += 2;
    vec_pnts.push_back(pnt);
    pnt.y += 1;
    vec_pnts.push_back(pnt);
    pnt.y -= 2;
    vec_pnts.push_back(pnt);
  }
  for(int i = 0; i < len; i++){
    pnt.x = p0.x + i * cos(hdng+block_an);
    pnt.y = p0.y + i * sin(hdng+block_an);
    pnt.x -= 1;
    vec_pnts.push_back(pnt);
    pnt.x += 2;
    vec_pnts.push_back(pnt);
    pnt.y += 1;
    vec_pnts.push_back(pnt);
    pnt.y -= 2;
    vec_pnts.push_back(pnt);
    vec_pnts.push_back(pnt);
  }
  return vec_pnts;
}

void get_polyclear(float maprad,float collision_radius){
  int num_is = 32;
  poly_mb.polygon.points.resize(num_is);
  poly_mb.header = hdr();
  for(int i = 0; i < num_is; i++){
    float a = -M_PI + i * (2*M_PI/num_is);
    Eigen::Vector3f pnt1_vec(perfalt_pnt.point.x,perfalt_pnt.point.y,perfalt_pnt.point.z);
    Eigen::Vector3f pnt2_vec(maprad*cos(a)+perfalt_pnt.point.x,perfalt_pnt.point.y + maprad*sin(a),perfalt_pnt.point.z);
    Eigen::Vector3f stride_vec,cur_vec,cur_vec_poly;
    stride_vec = (pnt2_vec - pnt1_vec).normalized() * 1;
    float distance = collision_radius;
    float cur_ray_len = 0;
    cur_vec = pnt1_vec;
    while(cur_ray_len < maprad){
      cur_vec     = cur_vec + stride_vec;
      cur_ray_len = (cur_vec - pnt1_vec).norm();
      point3d p(cur_vec.x(),cur_vec.y(),cur_vec.z());
      if(edf_ptr.get()->getDistance(p) >= collision_radius){
        poly_mb.polygon.points[i].z = p.z();
        poly_mb.polygon.points[i].x = p.x();
        poly_mb.polygon.points[i].y = p.y();
      }
    }
  }
}


map_msgs::OccupancyGridUpdate update_mbmap(geometry_msgs::Point midpoint,float maprad){
  map_msgs::OccupancyGridUpdate update;
  if(!got_map)
    return update;
  geometry_msgs::Point bbmin_octree,bbmax_octree,bbmin_custom,bbmax_custom;
  bbmin_custom.x = midpoint.x-par_maprad;
  bbmin_custom.y = midpoint.y-par_maprad;
  bbmax_custom.x = midpoint.x+par_maprad;
  bbmax_custom.y = midpoint.y+par_maprad;
  octree.get()->getMetricMin(bbmin_octree.x,bbmin_octree.y,bbmin_octree.z);
  octree.get()->getMetricMax(bbmax_octree.x,bbmax_octree.y,bbmax_octree.z);
  state_octomap_zmax = bbmax_octree.z;
  state_octomap_zmin = bbmin_octree.z;

    //ROS_INFO("Mbmap: %.0f %.0f %.0f maprad  %.0fhdng  %.0f block_a0 %.0f block_an %.0f ",midpoint.x,midpoint.y,midpoint.z,maprad,hdng,block_a0,block_an);
  bool outside_octree = false;
  float z0,z1,z;
  if(midpoint.z > bbmax_octree.z+2)
    outside_octree = true;
  else if(midpoint.z >= bbmax_octree.z-2)
    z = int(round(bbmax_octree.z - 1));
  else
    z = midpoint.z;
  z0 = midpoint.z - 1;
  z1 = midpoint.z + 1;
  int xmin = -400;  int ymin = -400;  int xmax = +400;  int ymax = +400;  int range_x = 800;  int range_y = 800;
  if(!outside_octree){
    octomap::point3d boundary_min(fmax(bbmin_custom.x,bbmin_octree.x),fmax(bbmin_custom.y,bbmin_octree.y),z0);
    octomap::point3d boundary_max(fmin(bbmax_custom.x,bbmax_octree.x),fmin(bbmax_custom.y,bbmax_octree.y),z1);
    octomap::OcTreeKey minKey, maxKey, curKey;
    if (!octree.get()->coordToKeyChecked(boundary_min, minKey))
      ROS_ERROR("Could not create OcTree key at %f %f %f", boundary_min.x(), boundary_min.y(), boundary_min.z());
    if (!octree.get()->coordToKeyChecked(boundary_max, maxKey))
      ROS_ERROR("Could not create OcTree key at %f %f %f", boundary_max.x(), boundary_max.y(), boundary_max.z());
    xmin    = int(round(boundary_min.x()));  ymin    = int(round(boundary_min.y()));
    xmax    = int(round(boundary_max.x()));  ymax    = int(round(boundary_max.y()));
   range_x = xmax - xmin;                   range_y = ymax - ymin;


    edf_ptr.reset (new DynamicEDTOctomap(3,octree.get(),
        boundary_min,
        boundary_max,false));
    edf_ptr.get()->update();
    get_polyclear(maprad,3);
  }

  update.header.stamp = ros::Time::now();
  update.header.frame_id = "map";
  update.x = 500+xmin;
  update.y = 500+ymin;
  update.width = range_x;
  update.height =range_y;
  update.data.resize(update.width * update.height);

  unsigned int i = 0;
  if(!outside_octree){
    for (int y = ymin; y < ymax; y++){
      for (int x = xmin; x < xmax; x++){
        octomap::point3d p(x,y,z);
        if(edf_ptr.get()->getDistance(p) < 3)
            update.data[i++] = 100;
          else
            update.data[i++] = 0;
      }
    }
  }
  else{
    for (int y = ymin; y < ymax; y++){
      for (int x = xmin; x < xmax; x++){
        update.data[i++] = 0;
      }
    }
  }/*
  //ROS_INFO("Update: X,Y: %i,%i widt x height: %i x %i",update.x,update.y,update.width,update.height);
  std::vector<geometry_msgs::Point> points_to_block = get_points_to_block(midpoint,hdng,block_a0,block_an);
  //ROS_INFO("Points to block: %i",points_to_block.size());
  for(int i = 0; i < points_to_block.size(); i++){
    int yi = int(points_to_block[i].y+500) - (update.y);
    int xi = int(points_to_block[i].x+500) - (update.x);
    if(yi < range_y && xi < range_x && yi > 0 && xi > 0){
      int ii = update.width * yi + xi;
      update.data[ii] = 0;
    }
  }*/
  last_mbmidpnt.point = midpoint;
  last_mbmidpnt.header =hdr();
  return update;
}




if(state_forwpah == "resetting"){
  state_forwpah = "filling";
  float tarhead = get_hdng(perfalt_pnt.point,pos_pnt.point);
  send_fixed_setpoint(pos_pnt.point.x,pos_pnt.point.y,fmax(pos_pnt.point.z,perfalt_pnt.point.z),get_inclination_target(),tarhead,true);
  geometry_msgs::Point reset_pnt = get_resetpnt(5);
  reset_odom(reset_pnt.x,reset_pnt.y,reset_pnt.z,0,"pos");
}

void odom_cb(const nav_msgs::Odometry::ConstPtr& msg){
  if((ros::Time::now() - ps_forwsim.header.stamp).toSec() > 0.2){
    ps_forwsim.pose.position = msg->pose.pose.position;
    ps_forwsim.pose.orientation = msg->pose.pose.orientation;
    ps_forwsim.header = hdr();
    ps_forwsim.header.frame_id = state_perfalt;
    ps_forwsim.pose.position.z = perfalt_pnt.point.z + par_zclearing;
    zvals_forwsim.push_back(int(ps_forwsim.pose.position.z));
    path_forwsim.poses.push_back(ps_forwsim);
    if(state_forwpah == "active"){
      zvals_forwsim.erase(zvals_forwsim.begin());
      path_forwsim.poses.erase(path_forwsim.poses.begin());
      std::deque<int>::iterator it = std::max(zvals_forwsim.begin(), zvals_forwsim.begin()+10);
      path_forwsim.poses[0].pose.position.z = float(*it);
      pose_setpoint = path_forwsim.poses[0];
      pub_setpoint.publish(pose_setpoint);
    }
  }
}
void reset_odom(float in_x,float in_y,float in_yaw,float in_vx,std::string special){
  nav_msgs::Odometry odom_reset;
  odom_reset.pose.pose.position.x = in_x;
  odom_reset.pose.pose.position.y = in_y;
  odom_reset.twist.twist.linear.x = in_vx;
  odom_reset.twist.twist.linear.y = 0;
  odom_reset.pose.pose.orientation = tf::createQuaternionMsgFromYaw(in_yaw);
  odom_reset.header.stamp = ros::Time::now();
  if(special == "")
    odom_reset.header.frame_id = "base_perfect";
  if(special == "pos"){
    odom_reset.pose.pose.position.x = pos.x;
    odom_reset.pose.pose.position.y = pos.y;
    odom_reset.twist.twist.linear.x = 0;
    odom_reset.pose.pose.orientation = tf::createQuaternionMsgFromYaw(vlp_rpy.z);
  }
  zvals_forwsim.resize(0);
  path_forwsim.poses.resize(0);
  pub_setodom.publish(odom_reset);
}
geometry_msgs::Point get_resetpnt(float min_dst_pos){
  geometry_msgs::Point reset_pnt;
  float max_dst_pos = 15;
  float reset_yaw = vlp_rpy.z;
  if(path_global_plan.poses.size() > 11){
    int closest_pnt_i = 0;
    float closest_pnt_dst = 10000;
    for(int i = 0; i < path_global_plan.poses.size()-10; i++){
      float dst = get_dst2d(path_global_plan.poses[i].pose.position,pos);
      if(dst < closest_pnt_dst && dst > min_dst_pos){
        closest_pnt_dst = dst;
        closest_pnt_i = i;
      }
      if(dst > max_dst_pos)
        break;
    }
    reset_pnt = path_global_plan.poses[closest_pnt_i].pose.position;
    reset_yaw = get_hdng(path_global_plan.poses[closest_pnt_i+5].pose.position,reset_pnt);
  }
  else{
    reset_yaw = get_hdng(targetmb.pose.position,pos);
    reset_pnt.x = pos.x + min_dst_pos * cos(reset_yaw);
    reset_pnt.y = pos.y + min_dst_pos * sin(reset_yaw);
  }
  if(get_dst2d(reset_pnt,pos) > 5){

  }
  ROS_INFO("Reset pnt: %.0f %.0f yaw: %0f",reset_pnt.x,reset_pnt.y,reset_yaw);
  reset_pnt.z = reset_yaw;
  return reset_pnt;
}
void update_state_forwpath(){
  if(path_forwsim.poses.size() > 2){
    float dn = get_dst2d(path_forwsim.poses[path_forwsim.poses.size()-1].pose.position,path_forwsim.poses[0].pose.position);
    float dt = (path_forwsim.poses[path_forwsim.poses.size()-1].header.stamp - path_forwsim.poses[0].header.stamp).toSec();
    float m_pr_s = (dn)/dt;
    int cnt_edto = 0;
    int cnt_nodata = 0;
    int cnt_grid = 0;
    for(int i = 0; i < path_forwsim.poses.size(); i++){
      std::string h = path_forwsim.poses[i].header.frame_id;
      float dst = get_dst2d(path_forwsim.poses[i].pose.position,pos);
      if(dst > 5){
        if(h == "got_edto")
          cnt_edto++;
        if(h == "no_data")
          cnt_nodata++;
        if(h == "got_grid")
          cnt_grid++;
      }
    }
    ROS_INFO("FOrwpath: %.0f m / %.4f sec - %.2f m/s, %i/%i edto/grid %i nodata",dn,dt,m_pr_s,cnt_edto,cnt_grid,cnt_nodata);
    if(cnt_nodata > 0 || m_pr_s < 0.5)
      state_forwpah = "resetting";
    else if(dn < 3 || dt < 2.0)
      state_forwpah = "filling";
    else{
      state_forwpah = "active";
    }
  }
  else
    state_forwpah = "filling";
}


void draw_globalplan_elevbb(nav_msgs::Path global_plan_in_elevbb){
  geometry_msgs::Point bbmin_custom,bbmax_custom,midpoint,bbmapradmin,bbmapradmax;
  bbmapradmin.x = perfalt_pnt.point.x - par_maprad;
  bbmapradmin.y = perfalt_pnt.point.y - par_maprad;
  bbmapradmax.x = perfalt_pnt.point.x + par_maprad;
  bbmapradmax.y = perfalt_pnt.point.y + par_maprad;
  std::vector<geometry_msgs::Point> bbmnmx = getinpath_boundingbox(global_plan_in_elevbb);
  bbmin_custom = bbmnmx[0];
  bbmax_custom = bbmnmx[1];
  midpoint.x = (bbmin_custom.x+bbmax_custom.x)/2;
  midpoint.y = (bbmin_custom.y+bbmax_custom.y)/2;
  img_blank.copyTo(img);
  cv::circle(img,pnt2cv(pos),2,get_color(200,200,200),1);
  cv::circle(img,pnt2cv(perfalt_pnt),2,get_color(85,200,200),1);
  cv::circle(img,pnt2cv(midpoint),2,get_color(100,100,75),1);
  draw_grid_within(bbmin_custom,bbmax_custom,radlen_xy);
  cv::rectangle(img, pnt2cv(bbmapradmin),pnt2cv(bbmapradmax),get_color(100,100,75),1,8,0);
  cv::rectangle(img, pnt2cv(bbmin_custom),pnt2cv(bbmax_custom),get_color(100,100,75),1,8,0);
  draw_path(*msg,get_color(100,0,0),0);
  draw_path(global_plan_in_elevbb,get_color(200,0,0),1);
  cv::Mat img_new2;
  midpoint.x += 100;
  midpoint.y += 100;
  putText(img,std::to_string(zmax), pnt2cv(midpoint),
      cv::FONT_HERSHEY_COMPLEX_SMALL, 0.8, get_color(15,25,100), 1, CV_AA);
  cv::resize(img, img_new2, cv::Size(), 2.0, 2.0);
  cv::imwrite("/home/nuc/brain/fullpath/"+std::to_string(count_target_paths)+"global_plan_in_elevbb.png",img_new2);
}

void request_pathelevation_cb(const nav_msgs::Path::ConstPtr& msg){
  if(got_map){
    nav_msgs::Path path_elevated      = get_path_elevation_covered(*msg);
    //ROS_INFO("Total %i poses elevated from %i total",path_elevated.poses.size(),msg->poses.size());
    nav_msgs::Path path_elevated_edto = update_edto(path_elevated,20);
    //ROS_INFO("Total %i poses elevated from %i total",path_elevated_edto.poses.size(),path_elevated.poses.size());
    pub_path_elevated.publish(setincltest(path_elevated,path_elevated_edto));
  }
}

int get_zmin_path(nav_msgs::Path pathin){
  float zmn = 1100;
  for(int i = 0; i < pathin.poses.size(); i++){
    if(pathin.poses[i].pose.position.z < zmn){
      zmn = pathin.poses[i].pose.position.z;
    }
  }
  return int(zmn);
}
int get_zmin_grid(geometry_msgs::Point midpoint, int radlen_xy){
  int c1 = x2c(midpoint.x-radlen_xy);
  int c0 = x2c(midpoint.x+radlen_xy);
  int r1 = y2r(midpoint.y-radlen_xy);
  int r0 = y2r(midpoint.y+radlen_xy);
  int zmn = 0;
  for(int c = fmin(c1,c0); c < fmax(c1,c0); c++){
    for(int r = fmin(r1,r0); r < fmax(r1,r0); r++){
      if(img_height.at<cv::Vec3b>(r,c)[2] > zmn)
        zmn = img_height.at<cv::Vec3b>(r,c)[2];
    }
  }
  return zmn;
}
int get_area_visits(geometry_msgs::Point midpoint, int radlen_xy,bool zmax_not_size){
  nav_msgs::Path vstd = get_path_aroundpnt(path_visited,midpoint,radlen_xy);
  if(zmax_not_size)
    return get_zmax_path(vstd);
  else
    return vstd.poses.size();
}
int get_count_new(geometry_msgs::Point midpoint, int radlen_xy){
  int c1 = x2c(midpoint.x-radlen_xy);
  int c0 = x2c(midpoint.x+radlen_xy);
  int r1 = y2r(midpoint.y-radlen_xy);
  int r0 = y2r(midpoint.y+radlen_xy);
  int pnts = 0;
  for(int c = fmin(c1,c0); c < fmax(c1,c0); c++){
    for(int r = fmin(r1,r0); r < fmax(r1,r0); r++){
      if(img_new.at<cv::Vec3b>(r,c)[1] > 0)
        pnts++;
    }
  }
  return pnts;
}
nav_msgs::Path get_path_aroundpnt(nav_msgs::Path pathin,geometry_msgs::Point midpoint,float radlen_xy){
  nav_msgs::Path pathout;
  float xmn = midpoint.x - radlen_xy;
  float ymn = midpoint.y - radlen_xy;
  float xmx = midpoint.x + radlen_xy;
  float ymx = midpoint.y + radlen_xy;
  pathout.header = hdr();

  for(int i = 0; i < pathin.poses.size(); i++){
    if(pathin.poses[i].pose.position.x < xmx
    && pathin.poses[i].pose.position.y < ymx
    && pathin.poses[i].pose.position.x > xmn
    && pathin.poses[i].pose.position.y > ymn)
      pathout.poses.push_back(pathin.poses[i]);
  }
  return pathout;
}
nav_msgs::Path create_gridpath(float area_sidelength,float radlen_xy){
  float centroid_sides = 2*radlen_xy;
  int num_grids = area_sidelength / centroid_sides;
  nav_msgs::Path pathout;
  pathout.header = hdr();
  pathout.poses.resize(num_grids*num_grids);
  //ROS_INFO("Area side: %.2f radlen: %.2f num_grids: %i centroid_sides: %.2f",area_sidelength,radlen_xy,num_grids,centroid_sides);
  int i = 0;
  for (int y = 0; y < num_grids; y++)
  {
    for (int x = 0; x < num_grids; x++)
    {
     geometry_msgs::Point pnt;
     pathout.poses[i].pose.position.x = float(area_sidelength*-0.5 + x * centroid_sides);
     pathout.poses[i].pose.position.y = float(area_sidelength*-0.5 + y * centroid_sides);
     pathout.poses[i].pose.position.z =0;
     pathout.poses[i].pose.orientation.w = 1.0;
     pathout.poses[i].header = hdr();
     i++;
    }
  }
  return pathout;
}
nav_msgs::Path create_linepath(geometry_msgs::Point p0, float line_length,float line_heading,float interval_meters){
  nav_msgs::Path pathout;
  pathout.header = hdr();
  int num_intervals = line_length / interval_meters;
  for(int i = 0; i < num_intervals+1; i++){
    geometry_msgs::PoseStamped ps;
    ps.header = hdr();
    ps.pose.position.x = p0.x + interval_meters * i * cos(line_heading);
    ps.pose.position.y = p0.y + interval_meters * i * sin(line_heading);
    ps.pose.orientation = tf::createQuaternionMsgFromYaw(line_heading);
    pathout.poses.push_back(ps);
  }
 return pathout;
}
std::vector<nav_msgs::Path> create_linepaths(float delta_a,float length,float interval_meters,int num_is){
  std::vector<nav_msgs::Path> linepaths;
  linepaths.resize(num_is);
  float a0 = pos_yaw - delta_a;
  float aN = pos_yaw + delta_a;
  float rads_pr_i = get_shortest(aN,a0) / num_is;
  for(int i = 0; i < num_is; i++){
    float a_i    = constrainAngle(a0 + rads_pr_i * i);
    linepaths[i] = create_linepath(pos,length,a_i,interval_meters);
  }
  return linepaths;
}

std::vector<int> vec_to_min_max_ave_int(std::vector<int> vec_in){
  std::vector<int> min_max_ave;
  min_max_ave.push_back(1431);
  min_max_ave.push_back(-1431);
  min_max_ave.push_back(0);
  if(vec_in.size() == 0){
    min_max_ave[0] = 0;
    min_max_ave[1] = 0;
    return min_max_ave;
  }
  int vec_sum = 0;
  for(int i = 0; i < vec_in.size(); i++){
    vec_sum += vec_in[i];
    if(vec_in[i] > min_max_ave[1])
     min_max_ave[1] = vec_in[i];
    if(vec_in[i] < min_max_ave[0])
     min_max_ave[0] = vec_in[i];
  }
  min_max_ave[2] = vec_sum / vec_in.size();
  return min_max_ave;
}
std::vector<geometry_msgs::Point> bbrc_to_bbxy(std::vector<int> bbmnmnx_rc){
  std::vector<geometry_msgs::Point> bbmnmx_xy;
  bbmnmx_xy.resize(2);
  bbmnmx_xy[0].x = c2x(bbmnmnx_rc[1]);
  bbmnmx_xy[0].y = r2y(bbmnmnx_rc[2]);
  bbmnmx_xy[1].x = c2x(bbmnmnx_rc[3]);
  bbmnmx_xy[1].y = r2y(bbmnmnx_rc[0]);
  return bbmnmx_xy;
}
std::vector<int> get_bbmnmnx(bool get_height){
  std::vector<int> bbvec; //rmn_cmn_rmx_cmx
  bbvec.resize(4);
  bbvec[0] = img_height.rows;
  bbvec[1] = img_height.cols;
  bbvec[2] = 0;
  bbvec[3] = 0;
  for(int r = 0; r < img_height.rows; r++){
    for(int c = 0; c < img_height.cols; c++){
      int val = img_update_copy.at<cv::Vec3b>(r,c)[1];
      if(get_height)
        val = img_height.at<cv::Vec3b>(r,c)[1];
      if(val > 0){
        if(r < bbvec[0])
          bbvec[0] = r;
        if(c < bbvec[1])
          bbvec[1] = c;
        if(c > bbvec[3])
          bbvec[3] = c;
        if(r > bbvec[2])
          bbvec[2] = r;
      }
    }
  }
  return bbvec;
}
void testnew(nav_msgs::Path pathin,std::string name){
  if(pathin.poses.size() == 0)
    return;
  start_process("get_bbs");
  std::vector<int> bb_upd = get_bbmnmnx(false);
  std::vector<int> bb_hgt = get_bbmnmnx(true);

  std::vector<geometry_msgs::Point> bbxy_upd = bbrc_to_bbxy(bb_upd);
  std::vector<geometry_msgs::Point> bbxy_hgt = bbrc_to_bbxy(bb_hgt);
  start_process("get_zmax_zmin_area_vstd");

  //ROS_INFO("BB_upd_ r,c:(%i,%i)->r,c:(%i,%i) height r,c:(%i,%i)->r,c:(%i,%i)", bb_upd[0],bb_upd[1],bb_upd[2],bb_upd[3],bb_hgt[0],bb_hgt[1],bb_hgt[2],bb_hgt[3]);
   //ROS_INFO("BB UPD: XY:(%.0f,%.0f)->XY:(%.0f,%.0f) BB_hgt: XY:(%.0f,%.0f)->XY:(%.0f,%.0f)",bbxy_upd[0].x,bbxy_upd[0].y,bbxy_upd[1].x,bbxy_upd[1].y,  bbxy_hgt[0].x,bbxy_hgt[0].y,bbxy_hgt[1].x,bbxy_hgt[1].y);
  float radlen_xy = 5;
  start_process("draw_rect");
  float dmx_xy = fmax(fmax(abs(bbxy_hgt[0].x),abs(bbxy_hgt[1].x)),fmax(abs(bbxy_hgt[0].y),abs(bbxy_hgt[1].y)));
  geometry_msgs::Point mx_xy,mn_xy;
  mx_xy.x = dmx_xy;mx_xy.y = dmx_xy;
  mn_xy.x = -dmx_xy;mn_xy.y = -dmx_xy;
  cv::rectangle(img, pnt2cv(bbxy_upd[0]),pnt2cv(bbxy_upd[1]),get_color(0,100,100),1,8,0);
  cv::rectangle(img, pnt2cv(bbxy_hgt[0]),pnt2cv(bbxy_hgt[1]),get_color(100,0,100),1,8,0);
  cv::rectangle(img, pnt2cv(mn_xy),pnt2cv(mx_xy),get_color(0,200,200),1,8,0);
  nav_msgs::Path path_ordered = create_gridpath(dmx_xy*2,5);
  //ROS_INFO("GOT PAHFULL: %i",path_ordered.poses.size());

  nav_msgs::Path pathfull_ordered_empty,pathfull_ordered_zmax;
  start_process("push_back");
  for(int i = 0; i < path_ordered.poses.size(); i++){
    float zmax_grid = get_zmax_grid(path_ordered.poses[i].pose.position,radlen_xy);
    if(zmax_grid == 0){
      pathfull_ordered_empty.poses.push_back(path_ordered.poses[i]);
    }
    else{
      pathfull_ordered_zmax.poses.push_back(path_ordered.poses[i]);
      pathfull_ordered_zmax.poses[pathfull_ordered_zmax.poses.size()-1].pose.position.z = zmax_grid;
    }
  }

  start_process("normalize_and_draw");
  std::vector<int> cnew;
  std::vector<int> cupd;
  std::vector<int> emp1;
  std::vector<int> emp5;
  std::vector<int> numz;
  std::vector<int> vstd;
  std::vector<int> hdng;
  std::vector<int> dstp;
  std::vector<int> dst0;
  std::vector<int> zmax;
  geometry_msgs::Point p0;
  //ROS_INFO("Path clear. %i empty %i zmax: %i",pathin.poses.size(),pathfull_ordered_empty.poses.size(),pathfull_ordered_zmax.poses.size());
  for(int i = 0; i < pathin.poses.size(); i++){
    nav_msgs::Path path_cand_empty_10   = get_path_aroundpnt(pathfull_ordered_empty,pathin.poses[i].pose.position,radlen_xy*10);
    nav_msgs::Path path_cand_empty_5    = get_path_aroundpnt(pathfull_ordered_empty,pathin.poses[i].pose.position,radlen_xy*5);
    nav_msgs::Path path_cand_zmax       = get_path_aroundpnt(pathfull_ordered_zmax,pathin.poses[i].pose.position,radlen_xy*2.2);

    int num_cnew = get_count_new(pathin.poses[i].pose.position,radlen_xy);
    int num_cupd = get_count_update(pathin.poses[i].pose.position,radlen_xy);
    int num_vstd = get_area_visits(pathin.poses[i].pose.position,radlen_xy*2.2,false);

    int num_emp1 =  path_cand_empty_10.poses.size();
    int num_emp5 =  path_cand_empty_5.poses.size();
    int num_zmax =  path_cand_zmax.poses.size();
    int vzmax = get_zmax_path(path_cand_zmax);
    int vhdng = abs(get_shortest(get_hdng(pathin.poses[i].pose.position,pos),pos_yaw)*rad2deg);
    int vdstp = get_dst2d(pathin.poses[i].pose.position,pos);
    int vdst0 = get_dst2d(pathin.poses[i].pose.position,p0);
    //ROS_INFO("EVALUATING[%i]: num_cnew: %i num_cupd: %i num_emp1: %i num_emp5: %i num_zmax: %i num_vstd: %i zmax: %i hdng: %i  dstp: %i  dst0: %i",i,num_cnew,num_cupd,num_emp1,num_emp5,num_zmax,num_vstd,vzmax,vhdng,vdstp,vdst0);
    cnew.push_back(num_cnew);
    cupd.push_back(num_cupd);
    emp1.push_back(num_emp1);
    emp5.push_back(num_emp5);
    numz.push_back(num_zmax);
    vstd.push_back(num_vstd);
    zmax.push_back(vzmax);
    hdng.push_back(vhdng);
    dstp.push_back(vdstp);
    dst0.push_back(vdst0);
  }
  std::vector<int> cnew_mma = vec_to_min_max_ave_int(cnew);
  std::vector<int> cupd_mma = vec_to_min_max_ave_int(cupd);
  std::vector<int> emp1_mma = vec_to_min_max_ave_int(emp1);
  std::vector<int> emp5_mma = vec_to_min_max_ave_int(emp5);
  std::vector<int> numz_mma = vec_to_min_max_ave_int(numz);
  std::vector<int> vstd_mma = vec_to_min_max_ave_int(vstd);
  std::vector<int> zmax_mma = vec_to_min_max_ave_int(zmax);
  std::vector<int> hdng_mma = vec_to_min_max_ave_int(hdng);
  std::vector<int> dstp_mma = vec_to_min_max_ave_int(dstp);
  std::vector<int> dst0_mma = vec_to_min_max_ave_int(dst0);
  int cnew_rnge = cnew_mma[1]-cnew_mma[0];
  int cupd_rnge = cupd_mma[1]-cupd_mma[0];
  int emp1_rnge = emp1_mma[1]-emp1_mma[0];
  int emp5_rnge = emp5_mma[1]-emp5_mma[0];
  int zmax_rnge = zmax_mma[1]-zmax_mma[0];
  int vstd_rnge = vstd_mma[1]-vstd_mma[0];
  int numz_rnge = numz_mma[1]-numz_mma[0];
  int hdng_rnge = hdng_mma[1]-hdng_mma[0];
  int dstp_rnge = dstp_mma[1]-dstp_mma[0];
  int dst0_rnge = dst0_mma[1]-dst0_mma[0];
  //ROS_INFO("EVALUATION RANGES[%i tot]: num_cnew: %i num_cupd: %i num_emp1: %i num_emp5: %i num_zmax: %i num_vstd: %i zmax: %i hdng: %i  dstp: %i  dst0: %i",pathin.poses.size(),cnew_rnge,cupd_rnge,emp1_rnge,emp5_rnge,zmax_rnge,vstd_rnge,zmax_rnge,hdng_rnge,dstp_rnge,dst0_rnge);
  std::vector<float> scores;

  float emp1_rel,emp5_rel,numz_rel,vstd_rel,zmax_rel,hdng_rel,dstp_rel,dst0_rel,cnew_rel,cupd_rel;
  int r = 0; int g = 0; int b = 0;
  for(int i = 0; i < pathin.poses.size(); i++){
    if(cnew_rnge > 0)
      cnew_rel = (float(cnew[i] - cnew_mma[0]))/float(cnew_rnge);
    if(cupd_rnge > 0)
      cupd_rel = (float(cupd[i] - cupd_mma[0]))/float(cupd_rnge);
    if(emp1_rnge > 0)
      emp1_rel = (float(emp1[i] - emp1_mma[0]))/float(emp1_rnge);
    if(emp5_rnge > 0)
      emp5_rel = (float(emp5[i] - emp5_mma[0]))/float(emp5_rnge);
    if(zmax_rnge > 0)
      zmax_rel = (float(zmax[i] - zmax_mma[0]))/float(zmax_rnge);
    if(vstd_rnge > 0)
      vstd_rel = (float(vstd[i] - vstd_mma[0]))/float(vstd_rnge);
    if(numz_rnge > 0)
      numz_rel = (float(numz[i] - numz_mma[0]))/float(numz_rnge);
    if(hdng_rnge > 0)
      hdng_rel = (float(hdng[i] - hdng_mma[0]))/float(hdng_rnge);
    if(dstp_rnge > 0)
      dstp_rel = (float(dstp[i] - dstp_mma[0]))/float(dstp_rnge);
    if(dst0_rnge > 0)
      dst0_rel = (float(dst0[i] - dst0_mma[0]))/float(dst0_rnge);
    //ROS_INFO("RAnges: cnew: %i cupd: %i emp1: %i emp5: %i numz: %i vstd: %i zmax: %i hdng: %i  dstp: %i  dst0: %i",cnew[i],cupd[i],emp1[i],emp5[i],numz[i],vstd[i],zmax[i],hdng[i],dstp[i],dst0[i]);
    //ROS_INFO("Rel_vAL: emp1: %.2f emp5: %.2f numz: %.2f vstd: %.2f zmax: %.2f hdng: %.2f  dstp: %.2f  dst0: %.2f",cnew_rel,cupd_rel,emp1_rel,emp5_rel,numz_rel,vstd_rel,zmax_rel,hdng_rel,dstp_rel,dst0_rel);
    scores.push_back(cupd_rel * cupd_weight+ emp5_rel * emp5_weight+ zmax_rel * zmax_weight+ numz_rel * numz_weight+ vstd_rel * vstd_weight+ hdng_rel * hdng_weight+ dstp_rel * dstp_weight+ dst0_rel * dst0_weight);
  }
  std::vector<float> scores_mma = vec_to_min_max_ave(scores);
  int best_i = 0;
  float best_score = 0;
  std::vector<float> scores_rel;
  scores_rel.resize(scores.size());
  for(int i = 0; i < scores.size(); i++){
    float score_rel = (scores[i] - scores_mma[0]) / (scores_mma[1]-scores_mma[0]);
    scores_rel[i] = score_rel;
    nav_msgs::Path path_cand_empty_10  = get_path_aroundpnt(pathfull_ordered_empty,pathin.poses[i].pose.position,radlen_xy*10);
    nav_msgs::Path path_cand_empty_5   = get_path_aroundpnt(pathfull_ordered_empty,pathin.poses[i].pose.position,radlen_xy*5);
    if(score_rel > 0.9){
      if(score_rel > best_score){
        best_i = i;
        best_score = score_rel;
      }
    //  draw_path(path_cand_empty_10,get_color(200,0,0),0);
    //  draw_path(path_cand_empty_5,get_color(200,0,0),0);
      cv::circle(img,pnt2cv(pathin.poses[i].pose.position),2,get_color(200,0,0),1);
      cv::line (img, pnt2cv(pos), pnt2cv(pathin.poses[i].pose.position),get_color(200,0,0),1,cv::LINE_8,0);
    //  ROS_INFO("WINNER[%i %.2f]: emp1: %i emp5: %i zmax: %i vstd: %i hdng: %i  dstp: %i  dst0: %i",i,score_rel,emp1[i],emp5[i],zmax[i],vstd[i],hdng[i],dstp[i],dst0[i]);
    }
    if(score_rel < 0.1){
    //  draw_path(path_cand_empty_10,get_color(0,0,200),0);
    //  draw_path(path_cand_empty_5,get_color(0,0,200),0);
      cv::circle(img,pnt2cv(pathin.poses[i].pose.position),2,get_color(0,0,200),1);
      cv::line (img, pnt2cv(pos), pnt2cv(pathin.poses[i].pose.position),get_color(0,0,200),1,cv::LINE_8,0);
    //  ROS_INFO("LOSER[%i %.2f]: emp1: %i emp5: %i zmax: %i vstd: %i hdng: %i  dstp: %i  dst0: %i",i,score_rel,emp1[i],emp5[i],zmax[i],vstd[i],hdng[i],dstp[i],dst0[i]);
    }
  }

  start_process("write_img");
  count_target_paths++;
//  draw_path(path_visited,get_color(0,200,0),2);
  //draw_path(path_ordered,get_color(0,0,200),2);
  //draw_path(pathfull,get_color(200,0,0),2);
//  draw_path(pathin,get_color(0,200,0),2);

  draw_path_by_score(pathin,scores_rel,2,2,0,1.0);
  geometry_msgs::PointStamped grid_winner;
  if(best_i > 0 && best_i < pathin.poses.size()){
    grid_winner.point = pathin.poses[best_i].pose.position;
  }
  grid_winner.header = hdr();
	//cv::circle(img,pnt2cv(grid_winner.point),2,get_color(255,255,200),1);
	cv::circle(img,pnt2cv(pos),2,get_color(255,255,0),1);
  cv::line (img, pnt2cv(pos), pnt2cv(grid_winner.point),get_color(255,255,0),1,cv::LINE_8,0);

  cv::imwrite("/home/nuc/brain/fullpath/"+std::to_string(count_target_paths)+name+".png",img);
  pub_winner.publish(grid_winner);
}
void get_grid_cb(const std_msgs::UInt8::ConstPtr& msg){
  cupd_weight=1;
  cnew_weight=1;
  emp1_weight=0;
  emp5_weight=0;
  zmax_weight=-1;
  numz_weight=0;
  vstd_weight=-1;
  hdng_weight=-1;
  dstp_weight=1;
  dst0_weight=-1;
  img_blank.copyTo(img);
  nav_msgs::Path path;
  float dxy = 5;
  int num_rays = 32;
  path  = merge_paths(create_linepaths(M_PI/3,50,dxy,num_rays));
  testnew(path,"comb");
}
void test_cb(const std_msgs::UInt8::ConstPtr& msg){
  float dxy = 5;
  int num_rays = 8;
  nav_msgs::Path path;
  if(msg->data < 1)
    path  = merge_paths(create_linepaths(M_PI/3,50,dxy,num_rays));
  else
    path  = (create_gridpath(150,10));
  cupd_weight=0;
  cnew_weight=0;
  emp1_weight=0;
  emp5_weight=0;
  numz_weight=0;
  zmax_weight=0;
  vstd_weight=0;
  hdng_weight=1;
  dstp_weight=0;
  dst0_weight=0;
  dst0_weight=0;
  img_blank.copyTo(img);
  testnew(path,"hdng");
  hdng_weight=0;
  zmax_weight=-1;
  img_blank.copyTo(img);
  testnew(path,"zmax");
  zmax_weight=0;
  numz_weight=1;
  img_blank.copyTo(img);
  testnew(path,"numz");
  numz_weight=0;
  cupd_weight=1;
  img_blank.copyTo(img);
  testnew(path,"cupd");
  cupd_weight=0;
  cnew_weight=1;
  img_blank.copyTo(img);
  testnew(path,"cnew");
  cnew_weight=0;
  emp1_weight=1;
  img_blank.copyTo(img);
  testnew(path,"emp1");
  emp1_weight=0;
  emp5_weight=1;
  img_blank.copyTo(img);
  testnew(path,"emp5");
  dstp_weight=1;
  img_blank.copyTo(img);
  testnew(path,"dstp");
  dstp_weight=0;
  dst0_weight=1;
  img_blank.copyTo(img);
  testnew(path,"dst0");
  cupd_weight=1;
   cnew_weight=1;
   emp1_weight=0;
   emp5_weight=0;
   zmax_weight=-1;
   numz_weight=0;
   vstd_weight=-1;
   hdng_weight=-1;
   dstp_weight=1;
   dst0_weight=-1;
   img_blank.copyTo(img);
   testnew(path,"comb");
}


pub_winner          = nh.advertise<geometry_msgs::PointStamped>("/tb_world/grid_target",10);
ros::Subscriber as4 = nh.subscribe("/tb_world/test_scores",10,test_cb);
ros::Subscriber as6 = nh.subscribe("/tb_world/get_grid",10,get_grid_cb);
ros::Subscriber bas3 = nh.subscribe("/tb_world/path_to_elevate",10,request_pathelevation_cb);
ros::Subscriber as3 = nh.subscribe("/tb_forwsim/path_raw",10,request_pathelevation_cb);
pub_path_elevated	  = nh.advertise<nav_msgs::Path>("/tb_world/path_elevated",10);
pub_path_global  = nh.advertise<nav_msgs::Path>("/tb_world/path_global_base",10);
