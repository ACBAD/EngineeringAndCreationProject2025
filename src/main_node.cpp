#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Twist.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <std_msgs/UInt8.h>
#include <iostream>
#include <defines.h>
#include <eac_pkg/EacGoal.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
int side_color = 0;
bool cover_state = true;

void updateCoverState(const std_msgs::UInt8& msg) {
  if(msg.data)cover_state = true;
  else cover_state = false;
}

int schema2(const ros::Publisher& schema2_pub) {
  std_msgs::UInt8 msg;
  msg.data = 1;
  schema2_pub.publish(msg);
  return 0;
}

int gotoGoal(ros::ServiceClient& navi_client,
            const PoseNames pose_name,
            const uint8_t timeout = 0,
            const geometry_msgs::Pose& pose = {}) {
  const geometry_msgs::Pose empty_pose;
  if(pose_name == COSTUM && pose == empty_pose) {
    ROS_ERROR("need costum pose data");
    return 1;
  }
  eac_pkg::EacGoal goal_msg;
  goal_msg.request.goal_index = pose_name;
  goal_msg.request.timeout = timeout;
  navi_client.call(goal_msg);
  if(goal_msg.response.state == false)return 3;
  return 0;
}

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "main_node");
  ros::NodeHandle node_handle;
  std_msgs::UInt8 cover_angle;
  // 此处开始为抽签完成后的调试阶段
  ROS_INFO("main start");
  if(!ros::param::get("side_color", side_color)) {
    ROS_ERROR("side_color not define, do you forget start navigation node?");
    return 1;
  }

  const ros::Publisher cover_pub = node_handle.advertise<std_msgs::UInt8>("/cover_cmd", 2);
  const ros::Publisher schema2_pub = node_handle.advertise<std_msgs::UInt8>("/schema2_node", 2);
  ros::ServiceClient navi_client = node_handle.serviceClient<eac_pkg::EacGoal>("navigation");
  const ros::Publisher twist_pub = node_handle.advertise<geometry_msgs::Twist>("/cmd_vel", 2);

  // 重置罩子状态
  cover_angle.data = 195;
  cover_pub.publish(cover_angle);
  // ReSharper disable once CppExpressionWithoutSideEffects
  ros::Duration(1).sleep();
  ROS_INFO("init ok");
  // 等待发出开始指令
  std::cout<<"waiting for start signal";
  std::cin.get();

  ROS_INFO("ATTACK!");

  cover_angle.data = 50;
  // 先让罩子开始转
  cover_pub.publish(cover_angle);
  ROS_INFO("rolling cover!");

  geometry_msgs::Twist go_msg;
  for (double i = 0; i < 0.7 ;i += 0.01) {
    go_msg.linear.x = i;
    twist_pub.publish(go_msg);
    // ReSharper disable once CppExpressionWithoutSideEffects
    ros::Duration(0.01).sleep();
  }
  go_msg.linear.x = 0.7;
  twist_pub.publish(go_msg);
  // ReSharper disable once CppExpressionWithoutSideEffects
  ros::Duration(0.85).sleep();
  go_msg.linear.x = 0;
  twist_pub.publish(go_msg);

  cover_state = false;
  ros::Subscriber cover_sub = node_handle.subscribe("/cover_state", 2, updateCoverState);
  // 等待罩子就位
  // spinOnce can call updateCoverState
  // ReSharper disable once CppDFALoopConditionNotUpdated
  while (!cover_state) {ros::spinOnce();}
  ROS_INFO("cover ready, covering!");
  cover_state = false;
  cover_angle.data = 5;
  // 就位后马上罩住
  cover_pub.publish(cover_angle);
  // spinOnce can call updateCoverState
  // ReSharper disable once CppDFALoopConditionNotUpdated
  while (!cover_state) {ros::spinOnce();}
  ROS_INFO("cover done! go to security zone");
  // 前往安全区，失败就转方案2
  if(gotoGoal(navi_client, SECURITY_ZONE) != 0) {
    ROS_WARN("schema 1 FAILED! try schema 2");
    schema2(schema2_pub);
    return 0;
  }
  ROS_INFO("reach security zone");

  cover_state = false;
  cover_angle.data = 195;
  // 抬起罩子
  cover_pub.publish(cover_angle);
  // spinOnce can call updateCoverState
  // ReSharper disable once CppDFALoopConditionNotUpdated
  while (!cover_state) {ros::spinOnce();}
  // 赢
  schema2(schema2_pub);
  return 0;
}
