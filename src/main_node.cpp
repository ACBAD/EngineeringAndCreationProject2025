#include <ros/ros.h>
#include <ros/duration.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <iostream>

geometry_msgs::Pose user_set_start_pose_right = {};
geometry_msgs::Pose user_set_start_pose_left = {};
geometry_msgs::PoseWithCovarianceStamped global_start_pose_right, global_start_pose_left;
int start_pose_signal = 0;

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "main_node");
  ros::NodeHandle node_handle;

  global_start_pose_right.header.frame_id = "map";
  global_start_pose_right.header.stamp = ros::Time::now();
  global_start_pose_right.pose.covariance = {};
  global_start_pose_right.pose.pose = user_set_start_pose_right;

  global_start_pose_left.header.frame_id = "map";
  global_start_pose_left.header.stamp = ros::Time::now();
  global_start_pose_left.pose.covariance = {};
  global_start_pose_left.pose.pose = user_set_start_pose_left;

  ROS_INFO("main start");
  std::cin>>start_pose_signal;
  ros::Publisher initialposePub = node_handle.advertise<geometry_msgs::PoseWithCovarianceStamped>("/initialpose", 2);
  ros::Publisher twistPub = node_handle.advertise<geometry_msgs::Twist>("/cmd_vel", 2);
  if(start_pose_signal == 0) {
    ROS_ERROR("error input");
    return 1;
  }
  if(start_pose_signal > 0) {
    ROS_INFO("choose right pose");
    initialposePub.publish(global_start_pose_right);
  }
  else if(start_pose_signal < 0) {
    ROS_INFO("choose left pose");
    initialposePub.publish(global_start_pose_left);
  }
  ROS_INFO("initialpose sent, rolling for amcl");
  geometry_msgs::Twist init_rolling_twist{};
  init_rolling_twist.angular.z = 99999999;
  twistPub.publish(init_rolling_twist);
  // 弱智ide，明明就能sleep5s
  // ReSharper disable once CppExpressionWithoutSideEffects
  ros::Duration(5.0).sleep();

  ros::spin();
  return 0;
}
