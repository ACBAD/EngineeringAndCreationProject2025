#include <defines.h>
#include <ros/ros.h>
#include <eac_pkg/EacGoal.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Twist.h>
#include <actionlib/client/simple_action_client.h>

actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac("move_base", true);
int side_color = -1;
UserSetPose poses;

actionlib::SimpleClientGoalState gotoGoal(const geometry_msgs::Pose& pose_goal) {
  move_base_msgs::MoveBaseGoal goal;
  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();
  goal.target_pose.pose = pose_goal;
  ac.sendGoal(goal);
  ROS_INFO("ac goal sent");
  ac.waitForResult();
  return ac.getState();
}

bool naviServiceCallback(eac_pkg::EacGoal::Request& request, eac_pkg::EacGoal::Response& response) {
  ROS_INFO("request");
  if(request.goal_index == 0) {
    ROS_INFO("custom pose");
    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();
    goal.target_pose.pose = request.custom_goal;
  }
  return true;
}

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "navigation_node");

  if(!ros::param::get("side_color", side_color)) {
    ROS_ERROR("side_color not define, fatal error");
    return 1;
  }
  if(side_color == SIDE_RED)poses.init_red();
  else if(side_color == SIDE_BLUE)poses.init_blue();
  else return 2;

  ros::NodeHandle node_handle;
  geometry_msgs::PoseWithCovarianceStamped init_start_pose;
  init_start_pose.header.frame_id = "map";
  init_start_pose.header.stamp = ros::Time::now();
  init_start_pose.pose.covariance = {};
  init_start_pose.pose.pose = poses.poses[START_POSE];
  // 设定初始位置
  ros::Publisher initialpose_pub = node_handle.advertise<geometry_msgs::PoseWithCovarianceStamped>("/initialpose", 2);
  initialpose_pub.publish(init_start_pose);
  ROS_INFO("initialpose sent, rolling for amcl");
  geometry_msgs::Twist init_rolling_twist{};
  // 高速旋转初始化amcl
  init_rolling_twist.angular.z = 99999999;
  ros::Publisher twist_pub = node_handle.advertise<geometry_msgs::Twist>("/cmd_vel", 2);
  twist_pub.publish(init_rolling_twist);
  // ReSharper disable once CppExpressionWithoutSideEffects
  ros::Duration(5.0).sleep();
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }
  ROS_INFO("navigation server online");

  ros::ServiceServer navi_server = node_handle.advertiseService("navigation", naviServiceCallback);
  ros::spin();
  return 0;
}
