#include <defines.h>
#include <ros/ros.h>
#include <eac_pkg/EacGoal.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Twist.h>
#include <actionlib/client/simple_action_client.h>
#include <std_msgs/UInt8.h>

typedef actionlib::SimpleClientGoalState goal_state;
int side_color = -1;
UserSetPose poses;
actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> *ac;


bool naviServiceCallback(eac_pkg::EacGoal::Request& request, eac_pkg::EacGoal::Response& response) {
  ROS_INFO("Received navigation request with goal index: %d", request.goal_index);
  // 初始化目标
  move_base_msgs::MoveBaseGoal goal;
  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();
  // 选择目标位姿
  if(request.goal_index == 0) {
    ROS_INFO("custom pose");
    goal.target_pose.pose = request.custom_goal;
  }else goal.target_pose.pose = poses.poses[request.goal_index];
  response.state = true;
  // 发送目标
  ac->sendGoal(goal);
  ROS_INFO("Goal sent to move_base");
  // 超时与状态监控
  bool timeout_reach = false;
  const ros::Time start_time = ros::Time::now();
  while (ros::ok() && !timeout_reach && !ac->getState().isDone()) {
    ros::spinOnce();
    // 主动检查超时
    if (request.timeout > 0 && (ros::Time::now() - start_time).toSec() >= request.timeout) {
      timeout_reach = true;
      ROS_WARN("Navigation timeout reached");
    }
    // ReSharper disable once CppExpressionWithoutSideEffects
    ros::Duration(0.1).sleep();
  }
  // 处理超时取消
  if (timeout_reach) {
    ac->cancelGoal();
    ROS_INFO("Goal cancelled due to timeout");
  }
  // 设置响应
  const auto state = ac->getState();
  response.state = (state == actionlib::SimpleClientGoalState::SUCCEEDED);
  response.extra_msg = static_cast<uint>(state.state_);
  ROS_INFO("Navigation finished with state: %s", state.toString().c_str());
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
  init_start_pose.pose.covariance = {0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.06853892326654787};
  init_start_pose.pose.pose = poses.poses[START_POSE];
  // 设定初始位置

  initialpose_pub.publish(init_start_pose);
  ROS_INFO("init pose 1");
  // ReSharper disable once CppExpressionWithoutSideEffects
  ros::Duration(3.0).sleep();
  initialpose_pub.publish(init_start_pose);
  ROS_INFO("init pose 2");
  // ReSharper disable once CppExpressionWithoutSideEffects
  ros::Duration(3.0).sleep();
  initialpose_pub.publish(init_start_pose);
  // ReSharper disable once CppExpressionWithoutSideEffects
  ros::Duration(3.0).sleep();
  initialpose_pub.publish(init_start_pose);
  ROS_INFO("initialpose sent, rolling for amcl");
  // ReSharper disable once CppExpressionWithoutSideEffects
  while(!local_ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }
  ROS_INFO("navigation server online");

  ros::ServiceServer navi_server = node_handle.advertiseService("navigation", naviServiceCallback);
  ros::spin();
  return 0;
}
