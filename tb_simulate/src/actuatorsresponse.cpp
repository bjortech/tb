#include <string>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>
#include <std_msgs/Float64.h>

double arm1_til_target,arm2_til_target,arm2_pan_target,arm1_til_vmax,arm2_til_vmax,arm2_pan_vmax;
double arm1_til_limlo,arm1_til_limhi,arm2_til_limlo,arm2_til_limhi,arm2_pan_limlo,arm2_pan_limhi;

void arm1_tilt_cb(const std_msgs::Float64::ConstPtr& msg){
	if(msg->data > arm1_til_limhi)
		arm1_til_target = arm1_til_limhi;
	else if(msg->data < arm1_til_limlo)
		arm1_til_target = arm1_til_limlo;
	else
		arm1_til_target = msg->data;
}

void arm2_tilt_cb(const std_msgs::Float64::ConstPtr& msg){
	if(msg->data > arm2_til_limhi)
		arm2_til_target = arm2_til_limhi;
	else if(msg->data < arm2_til_limlo)
		arm2_til_target = arm2_til_limlo;
	else
		arm2_til_target = msg->data;
}

void arm2_pan_cb(const std_msgs::Float64::ConstPtr& msg){
	if(msg->data > arm2_pan_limhi)
		arm2_pan_target = arm2_pan_limhi;
	else if(msg->data < arm2_pan_limlo)
		arm2_pan_target = arm2_pan_limlo;
	else
		arm2_pan_target = msg->data;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "tb_simulate_actuatorresponse_node");
    ros::NodeHandle nh;
		ros::NodeHandle private_nh("~");
		arm1_til_limlo = -1.5;
		arm1_til_limhi = 1.0;
		arm2_til_limlo = -1.3;
		arm2_til_limhi = 0.5;
		arm2_pan_limlo = -1.3;
		arm2_pan_limhi = 1.3;
		try{
			nh.getParam("/tilt_controller/joint_speed", arm1_til_vmax);
			nh.getParam("/tilt_controller/joint_speed", arm2_til_vmax);
			nh.getParam("/pan_controller/joint_speed", arm2_pan_vmax);
		  ROS_INFO("Successfully Loaded Joint Velocity parameters");
	  }
	  catch(int e)
	  {
			arm1_til_vmax = 1.17;
			arm2_til_vmax = 1.17;
			arm2_pan_vmax = 1.17;
	    ROS_WARN("Parameters are not properly loaded from file, loading defaults");
	  }

    ros::Publisher joint_pub  = nh.advertise<sensor_msgs::JointState>("joint_states", 1);

		ros::Subscriber s1				= nh.subscribe("/tilt_controller/command", 10,arm1_tilt_cb);
		ros::Subscriber s2_tilt  	= nh.subscribe("/tilt2_controller/command", 10,arm2_tilt_cb);
		ros::Subscriber s2_pan		= nh.subscribe("/pan_controller/command", 10,arm2_pan_cb);

    sensor_msgs::JointState joint_state;
		joint_state.name.resize(3);
		joint_state.position.resize(3);
		joint_state.name[0] ="tilt_joint";
		joint_state.name[1] ="tilt_joint2";
		joint_state.name[2] ="pan_joint";
		ros::Rate rate(50);
    while (ros::ok()) {
        //update joint_state
				double dt = (ros::Time::now() - joint_state.header.stamp).toSec();
        joint_state.header.stamp = ros::Time::now();
				float arm1_til_err = arm1_til_target - joint_state.position[0];
				float arm1_til_motion = arm1_til_err * arm1_til_vmax * dt;
				joint_state.position[0] += arm1_til_motion;

				float arm2_til_err = arm2_til_target - joint_state.position[1];
				float arm2_til_motion = arm2_til_err * arm2_til_vmax * dt;
				joint_state.position[1] += arm2_til_motion;

        float arm2_pan_err = arm2_pan_target - joint_state.position[2];
				float arm2_pan_motion = arm2_pan_err * arm2_pan_vmax * dt;
				joint_state.position[2] += arm2_pan_motion;

        joint_pub.publish(joint_state);

				rate.sleep();
				ros::spinOnce();

		}
    return 0;
}
