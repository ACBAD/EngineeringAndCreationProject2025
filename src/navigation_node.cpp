#include <defines.h>
#include <ros/ros.h>
#include <eac_pkg/EacGoal.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Twist.h>
#include <actionlib/client/simple_action_client.h>

typedef actionlib::SimpleClientGoalState goal_state;
int side_color = -1;
UserSetPose poses;

actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> *ac;


bool naviServiceCallback(eac_pkg::EacGoal::Request& request, eac_pkg::EacGoal::Response& response) {
  ROS_INFO("request");
  move_base_msgs::MoveBaseGoal goal;
  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();
  if(request.goal_index == 0) {
    ROS_INFO("custom pose");
    goal.target_pose.pose = request.custom_goal;
  }else goal.target_pose.pose = poses.poses[request.goal_index];
  response.state = true;

  ac->sendGoal(goal);
  ROS_INFO("ac goal sent");
  bool timeout_reach = false;
  if (request.timeout) {
    ros::NodeHandle node_handle;
    auto timeoutReachCallback = [&timeout_reach](ros::TimerEvent event) {timeout_reach = true;};
    ros::Timer navi_timer = node_handle.createTimer(ros::Duration(request.timeout), timeoutReachCallback);
  }
  while (ac->getState() != goal_state::SUCCEEDED ||
    ac->getState() != goal_state::PENDING ||
    !timeout_reach){ros::spinOnce();}
  if(timeout_reach)ac->cancelGoal();

  const goal_state goal_result = ac->getState();
  if(goal_result != goal_state::SUCCEEDED)response.state = false;
  response.extra_msg = static_cast<uint>(goal_result.state_);
  return true;
}

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "navigation_node");

  static ros::NodeHandle node_handle;
  static actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> local_ac("move_base", true);
  if(!ros::param::get("side_color", side_color)) {
    ROS_ERROR("side_color not define, fatal error");
    return 1;
  }
  if(side_color == SIDE_RED)poses.init_red();
  else if(side_color == SIDE_BLUE)poses.init_blue();
  else return 2;
  ac = &local_ac;

  ROS_WARN("First init ok");
  const ros::Publisher twist_pub = node_handle.advertise<geometry_msgs::Twist>("/cmd_vel", 2);
  const ros::Publisher initialpose_pub = node_handle.advertise<geometry_msgs::PoseWithCovarianceStamped>("/initialpose", 2);
  // ReSharper disable once CppExpressionWithoutSideEffects
  ros::Duration(1.0).sleep();
  geometry_msgs::PoseWithCovarianceStamped init_start_pose;
  init_start_pose.header.frame_id = "map";
  init_start_pose.header.stamp = ros::Time::now();
  init_start_pose.pose.covariance = {};
  init_start_pose.pose.pose = poses.poses[START_POSE];
  // 设定初始位置

  initialpose_pub.publish(init_start_pose);
  ROS_INFO("initialpose sent, rolling for amcl");
  geometry_msgs::Twist init_rolling_twist{};
  // 高速旋转初始化amcl
  init_rolling_twist.angular.z = 0.3;

  twist_pub.publish(init_rolling_twist);
  // ReSharper disable once CppExpressionWithoutSideEffects
  ros::Duration(5.0).sleep();
  init_rolling_twist.angular.z = 0;
  twist_pub.publish(init_rolling_twist);
  while(!local_ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }
  ROS_INFO("navigation server online");

  ros::ServiceServer navi_server = node_handle.advertiseService("navigation", naviServiceCallback);
  ros::spin();
  return 0;
}
