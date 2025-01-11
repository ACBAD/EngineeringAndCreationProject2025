#include <ros/ros.h>
#include <ros/duration.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <std_msgs/UInt8.h>
#include <iostream>

enum SideColor {
  SIDE_RED,
  SIDE_BLUE
};

class UserSetPose {
public:
  geometry_msgs::Pose start_pose;
  geometry_msgs::Pose pick_pose;
  geometry_msgs::Pose security_zone;
  UserSetPose(){}
  void init_red() {
    start_pose.position.x = 1.0;
    start_pose.position.y = 2.0;
    start_pose.position.z = 0.5;
    start_pose.orientation.w = 1.0;

    pick_pose.position.x = 3.0;
    pick_pose.position.y = 1.5;
    pick_pose.position.z = 0.0;
    pick_pose.orientation.w = 1.0;

    security_zone.position.x = 0.0;
    security_zone.position.y = 0.0;
    security_zone.position.z = 0.0;
    security_zone.orientation.w = 1.0;
  }
  void init_blue() {
    start_pose.position.x = 1.0;
    start_pose.position.y = 2.0;
    start_pose.position.z = 0.5;
    start_pose.orientation.w = 1.0;

    pick_pose.position.x = 3.0;
    pick_pose.position.y = 1.5;
    pick_pose.position.z = 0.0;
    pick_pose.orientation.w = 1.0;

    security_zone.position.x = 0.0;
    security_zone.position.y = 0.0;
    security_zone.position.z = 0.0;
    security_zone.orientation.w = 1.0;
  }
};

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
int start_pose_signal = 0;
bool cover_state = true;

void updateCoverState(const std_msgs::UInt8& msg) {
  if(msg.data)cover_state = true;
  else cover_state = false;
}

int checkBallsIsAviliale() {
  // TODO checkLogic
  return 0;
}

int schema2() {
  // TODO schema2
  return 0;
}

MoveBaseClient ac("move_base", true);

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

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "main_node");
  ros::NodeHandle node_handle;
  // 此处开始为抽签完成后的调试阶段
  ROS_INFO("main start");
  // 输入选择的位置，红方或者是蓝方
  std::cout<<"input postion, positive for red, negetive for blue\ninput: ";
  std::cin>>start_pose_signal;
  ros::Publisher initialpose_pub = node_handle.advertise<geometry_msgs::PoseWithCovarianceStamped>("/initialpose", 2);
  ros::Publisher twist_pub = node_handle.advertise<geometry_msgs::Twist>("/cmd_vel", 2);
  ros::Publisher cover_pub = node_handle.advertise<std_msgs::UInt8>("/cover_cmd", 2);
  if(start_pose_signal == 0) {
    ROS_ERROR("error input");
    return 1;
  }
  UserSetPose poses;
  if(start_pose_signal > 0) {
    ROS_INFO("choose red pose");
    poses.init_red();
  }
  else if(start_pose_signal < 0) {
    ROS_INFO("choose blue pose");
    poses.init_blue();
  }
  geometry_msgs::PoseWithCovarianceStamped init_start_pose;
  init_start_pose.header.frame_id = "map";
  init_start_pose.header.stamp = ros::Time::now();
  init_start_pose.pose.covariance = {};
  init_start_pose.pose.pose = poses.start_pose;
  // 设定初始位置
  initialpose_pub.publish(init_start_pose);
  ROS_INFO("initialpose sent, rolling for amcl");
  geometry_msgs::Twist init_rolling_twist{};
  // 高速旋转初始化amcl
  init_rolling_twist.angular.z = 99999999;
  twist_pub.publish(init_rolling_twist);
  // ReSharper disable once CppExpressionWithoutSideEffects
  ros::Duration(5.0).sleep();
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }
  ROS_INFO("action server online");
  // 前往设定起点（应该动作幅度不大）
  if(gotoGoal(poses.start_pose) != actionlib::SimpleClientGoalState::SUCCEEDED) {
    ROS_ERROR("init failed, rerun");
    return 1;
  }
  ROS_INFO("init ok");
  // 等待发出开始指令
  std::cout<<"waiting for start signal";
  std::cin.get();

  ROS_INFO("ATTACK!");
  std_msgs::UInt8 cover_angle;
  cover_angle.data = 160;
  // 先让罩子开始转
  cover_pub.publish(cover_angle);
  ROS_INFO("rolling cover!");

  // 同时前往球区前方，如果没过去就判定为失败进入方案2
  if(gotoGoal(poses.pick_pose) != actionlib::SimpleClientGoalState::SUCCEEDED) {
    ROS_WARN("schema 1 FAILED! try schema 2");
    schema2();
    return 0;
  }
  ROS_INFO("reach target location, waiting cover ready");
  cover_state = false;
  ros::Subscriber cover_sub = node_handle.subscribe("/cover_cmd", 2, updateCoverState);
  // 等待罩子就位
  // spinOnce can call updateCoverState
  // ReSharper disable once CppDFALoopConditionNotUpdated
  while (!cover_state) {
    ros::spinOnce();
  }
  ROS_INFO("cover ready, covering!");
  cover_state = false;
  cover_angle.data = 180;
  // 就位后马上罩住
  cover_pub.publish(cover_angle);
  // spinOnce can call updateCoverState
  // ReSharper disable once CppDFALoopConditionNotUpdated
  while (!cover_state) {
    ros::spinOnce();
  }
  ROS_INFO("cover done! go to security zone");
  // 前往安全区，失败就转方案2
  if(gotoGoal(poses.security_zone) != actionlib::SimpleClientGoalState::SUCCEEDED) {
    ROS_WARN("schema 1 FAILED! try schema 2");
    schema2();
    return 0;
  }
  ROS_INFO("reach security zone, check balls state");
  // 在安全区前检查球是否正常，不正常就转方案2
  if(checkBallsIsAviliale()) {
    ROS_WARN("schema 1 FAILED! try schema 2");
    schema2();
    return 0;
  }
  cover_state = false;
  cover_angle.data = 0;
  // 抬起罩子
  cover_pub.publish(cover_angle);
  // spinOnce can call updateCoverState
  // ReSharper disable once CppDFALoopConditionNotUpdated
  while (!cover_state) {
    ros::spinOnce();
  }
  // 赢
  ROS_WARN("WIN");
  ros::spin();
  return 0;
}
