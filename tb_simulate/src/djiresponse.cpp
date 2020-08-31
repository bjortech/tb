#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/Range.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/NavSatFix.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>
#include <std_msgs/UInt8.h>

ros::Publisher pos_pub;
geometry_msgs::PointStamped pos;
sensor_msgs::Imu imu;
double yaw,yaw1,par_latitude,par_longitude,par_altitude,par_vlprange;
ros::Time t_last,time_last;
bool got_imu;
geometry_msgs::Twist twist;
geometry_msgs::Pose orientation_received;
geometry_msgs::Vector3Stamped vel;
geometry_msgs::QuaternionStamped att;
std_msgs::Float32 alt;

double saturate(double val, double max){
    if (abs(val) >= max) val = (val>0) ? max : -1 * max;
    else                 val = val;
    if((std::isnan(val)) || (std::isinf(val))) return 0;
    else                                       return val;
}
double constrainAngle(double x){
  if(x > M_PI)
    return (x - M_PI*2);
  else if(x < -M_PI)
    return (x + M_PI*2);
  else
    return x;
}
void pcbb(const sensor_msgs::Joy::ConstPtr msg)
{
  float dt = (ros::Time::now() - time_last).toSec();
  time_last = ros::Time::now();
  pos.point.x += saturate(msg->axes[0],10) * dt;
  pos.point.y += saturate(msg->axes[1],10) * dt;
  pos.point.z += saturate(msg->axes[2]-pos.point.z,10) * dt;
  yaw = msg->axes[3];
}
void yawcmd_cb(const std_msgs::Float64::ConstPtr& msg){
  yaw = msg->data;
}
int main(int argc, char **argv) {
    ros::init(argc, argv, "tb_simulate_djiresponse_node");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");
    private_nh.param("reference_latitude", par_latitude, 60.366926);//*2.0);
    private_nh.param("reference_longitude",par_longitude, 5.357470);//*2.0);
    private_nh.param("reference_altitude", par_altitude, 0.0);//*2.0);
    time_last = ros::Time::now();
    pos.header.frame_id = "odom";
    vel.header.frame_id = "/local";
    imu.orientation.w = 1;
    att.quaternion.w = 1;

    ros::Subscriber s5b    =  nh.subscribe("/tb_exeq/cmd_joy", 100,pcbb);

    ros::Publisher gph_pub = nh.advertise<std_msgs::UInt8>("/dji_sdk/gps_health", 10);
		ros::Publisher alt_pub = nh.advertise<std_msgs::Float32>("/dji_sdk/height_above_takeoff", 10);
		ros::Publisher imu_pub = nh.advertise<sensor_msgs::Imu>("/dji_sdk/imu", 10);
  	ros::Publisher ref_pub = nh.advertise<sensor_msgs::NavSatFix>("/dji_sdk/local_frame_ref", 10);
  	ros::Publisher vel_pub = nh.advertise<geometry_msgs::Vector3Stamped>("/dji_sdk/velocity", 10);
    ros::Publisher att_pub = nh.advertise<geometry_msgs::QuaternionStamped>("/dji_sdk/attitude", 10);
		pos_pub 							 = nh.advertise<geometry_msgs::PointStamped>("/dji_sdk/local_position", 10);

    std_msgs::UInt8 gph;
		std_msgs::Float32 alt;
		sensor_msgs::NavSatFix ref;
		bool first = true;

    gph.data = 5;
    ref.longitude = par_longitude;
    ref.latitude = par_latitude;
    ref.altitude = par_altitude;

    ros::Time first_timer = ros::Time::now();
    ros::Rate rate(50);
    while(ros::ok()){
      if(first && ((ros::Time::now() - first_timer).toSec() > 3)){
        first = false;
        ref.header.stamp = ros::Time::now();
        ref_pub.publish(ref);
      }
      att.quaternion = imu.orientation = tf::createQuaternionMsgFromYaw(yaw);
      imu.angular_velocity.z = twist.angular.z;
      if(got_imu){
        att.quaternion = imu.orientation = orientation_received.orientation;
      }
      pos.header.stamp = att.header.stamp = imu.header.stamp = ros::Time::now();
      alt.data = pos.point.z;
      alt_pub.publish(alt);
      att_pub.publish(att);
      pos_pub.publish(pos);
      imu_pub.publish(imu);
      gph_pub.publish(gph);
      vel_pub.publish(vel);
      rate.sleep();
      ros::spinOnce();
  }
}
