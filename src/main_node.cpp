#include <ros/ros.h>
#include <ros/duration.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <std_msgs/UInt8.h>
#include <iostream>

geometry_msgs::Pose user_set_start_pose_right = {};
geometry_msgs::Pose user_set_start_pose_left = {};
geometry_msgs::Pose user_set_pick_pose_left = {};
geometry_msgs::Pose user_set_pick_pose_right = {};
geometry_msgs::PoseWithCovarianceStamped global_start_pose_right, global_start_pose_left;
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
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
  std::cout<<"input postion set, positive for right, negetive for left\ninput: ";
  std::cin>>start_pose_signal;
  ros::Publisher initialpose_pub = node_handle.advertise<geometry_msgs::PoseWithCovarianceStamped>("/initialpose", 2);
  ros::Publisher twist_pub = node_handle.advertise<geometry_msgs::Twist>("/cmd_vel", 2);
  ros::Publisher cover_pub = node_handle.advertise<std_msgs::UInt8>("/cover_cmd", 2);
  if(start_pose_signal == 0) {
    ROS_ERROR("error input");
    return 1;
  }
  if(start_pose_signal > 0) {
    ROS_INFO("choose right pose");
    initialpose_pub.publish(global_start_pose_right);
  }
  else if(start_pose_signal < 0) {
    ROS_INFO("choose left pose");
    initialpose_pub.publish(global_start_pose_left);
  }
  ROS_INFO("initialpose sent, rolling for amcl");
  geometry_msgs::Twist init_rolling_twist{};
  init_rolling_twist.angular.z = 99999999;
  twist_pub.publish(init_rolling_twist);
  // 弱智ide，明明就能sleep5s
  // ReSharper disable once CppExpressionWithoutSideEffects
  ros::Duration(5.0).sleep();
  MoveBaseClient ac("move_base", true);
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }
  ROS_INFO("action server online");
  move_base_msgs::MoveBaseGoal init_start_goal;
  init_start_goal.target_pose.header.frame_id = "map";
  init_start_goal.target_pose.header.stamp = ros::Time::now();
  if(start_pose_signal > 0)
    init_start_goal.target_pose.pose = global_start_pose_right.pose.pose;
  if(start_pose_signal < 0)
    init_start_goal.target_pose.pose = global_start_pose_left.pose.pose;
  ac.sendGoal(init_start_goal);
  ROS_INFO("ac goal sent");
  ac.waitForResult();
  if(ac.getState() != actionlib::SimpleClientGoalState::SUCCEEDED) {
    ROS_ERROR("init localization failed! rerun!");
    return 0;
  }
  ROS_INFO("init ok");
  move_base_msgs::MoveBaseGoal pick_goal;
  pick_goal.target_pose.header.frame_id = "map";
  if(start_pose_signal > 0)
    pick_goal.target_pose.pose = user_set_pick_pose_right;
  if(start_pose_signal < 0)
    pick_goal.target_pose.pose = user_set_pick_pose_left;
  std::cout<<"waiting for start signal";
  std::cin.get();
  ROS_INFO("ATTACK!");
  // TODO
  ROS_INFO("rolling cover!");
  pick_goal.target_pose.header.stamp = ros::Time::now();
  ac.sendGoal(pick_goal);

  ros::spin();
  return 0;
}
