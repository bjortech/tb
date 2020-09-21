#include <ros/ros.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Empty.h>
#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <frontier_exploration/ExploreTaskAction.h>
#include <frontier_exploration/ExploreTaskActionGoal.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib_msgs/GoalID.h>
#include <std_msgs/UInt8.h>
ros::Publisher result_pub;
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
typedef actionlib::SimpleActionClient<frontier_exploration::ExploreTaskAction> ExploreClient;
std_msgs::Bool success;
std_msgs::Bool failure;
geometry_msgs::PointStamped point;
geometry_msgs::Point compute2DPolygonCentroid(geometry_msgs::PolygonStamped polyin)
{
    geometry_msgs::Point centroid;
    double signedArea = 0.0;
    double x0 = 0.0; // Current vertex X
    double y0 = 0.0; // Current vertex Y
    double x1 = 0.0; // Next vertex X
    double y1 = 0.0; // Next vertex Y
    double a = 0.0;  // Partial signed area

    // For all vertices
    for (int i=0; i<polyin.polygon.points.size(); ++i)
    {
        x0 = polyin.polygon.points[i].x;
        y0 = polyin.polygon.points[i].y;
        x1 = polyin.polygon.points[(i+1) % polyin.polygon.points.size()].x;
        y1 = polyin.polygon.points[(i+1) % polyin.polygon.points.size()].y;
        a = x0*y1 - x1*y0;
        signedArea += a;
        centroid.x += (x0 + x1)*a;
        centroid.y += (y0 + y1)*a;
    }

    signedArea *= 0.5;
    centroid.x /= (6.0*signedArea);
    centroid.y /= (6.0*signedArea);

    return centroid;
}
void targetpoly_cb(const geometry_msgs::PolygonStamped::ConstPtr& msg){
  if(msg->header.frame_id != ""){
    if(msg->polygon.points.size() >= 3){
      ExploreClient exploreClient("explore_server", true);
      while(!exploreClient.waitForServer(ros::Duration(5.0))){
        ROS_INFO("Waiting for the explore action server");
      }
      frontier_exploration::ExploreTaskGoal goal;
      goal.explore_center.header.stamp    = ros::Time::now();
      goal.explore_center.header.frame_id = "map";
      goal.explore_boundary.header = goal.explore_center.header;
    //  goal.explore_center.point.x = (msg->polygon.points[0].x + msg->polygon.points[2].x)/2;
    //  goal.explore_center.point.y = (msg->polygon.points[0].y + msg->polygon.points[2].y)/2;
      goal.explore_center.point     = compute2DPolygonCentroid(*msg);
      goal.explore_boundary.polygon = msg->polygon;
      goal.explore_boundary.polygon.points[0].z = 0;

      ROS_INFO("Sending goal");
      exploreClient.sendGoal(goal);
      exploreClient.waitForResult();

      if(exploreClient.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
        ROS_INFO("You have arrived to the goal position");
        result_pub.publish(success);
      }
      else{
        ROS_INFO("The base failed for some reason");
        result_pub.publish(failure);
      }
    }
    else{
      result_pub.publish(failure);
    }
  }
}

void targetpose_cb(const geometry_msgs::PoseStamped::ConstPtr& pose){
  if(pose->header.frame_id != ""){
    MoveBaseClient ac("move_base", true);

    while(!ac.waitForServer(ros::Duration(5.0))){
      ROS_INFO("Waiting for the move_base action server");
    }

    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose.header.frame_id = pose->header.frame_id;
    goal.target_pose.header.stamp = ros::Time::now();
    point.point.x = pose->pose.position.x;
    point.point.y = pose->pose.position.y;

    goal.target_pose.pose.position.x = pose->pose.position.x;
    goal.target_pose.pose.position.y = pose->pose.position.y;
    goal.target_pose.pose.orientation = pose->pose.orientation;


    ROS_INFO("Sending goal");
    ac.sendGoal(goal);
    ac.waitForResult();

    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
        ROS_INFO("You have arrived to the goal position");
        result_pub.publish(success);
    }
    else{
      ROS_INFO("The base failed for some reason");
      result_pub.publish(failure);
    }
  }
}



int main(int argc, char **argv) {
    ros::init(argc, argv, "tb_cmdmb_node");
    ros::NodeHandle nh;
    success.data = true;
    failure.data = false;
    ros::Subscriber s0    = nh.subscribe("/tb_cmd/posemb", 10,&targetpose_cb);
    ros::Subscriber s1    = nh.subscribe("/tb_cmd/poly_explore", 10,&targetpoly_cb);
    result_pub = nh.advertise<std_msgs::Bool>("/tb_cmdmb/success",10);

    ros::spin();
}
