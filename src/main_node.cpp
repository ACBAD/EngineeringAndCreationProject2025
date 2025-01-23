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
int start_pose_signal = 0;
bool cover_state = true;

void updateCoverState(const std_msgs::UInt8& msg) {
  if(msg.data)cover_state = true;
  else cover_state = false;
}

bool obj_detected = false;
uint8_t obj_state;
void objAviliableCallback(const std_msgs::UInt8& msg) {
  obj_detected = true;
  obj_state = msg.data;
}

int schema2(const ros::Publisher& schema2_pub) {
  std_msgs::UInt8 msg;
  msg.data = 1;
  schema2_pub.publish(msg);
  return 0;
}

int gotoGoal(ros::ServiceClient navi_client,
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
  // 此处开始为抽签完成后的调试阶段
  ROS_INFO("main start");
  // 输入选择的位置，红方或者是蓝方
  std::cout<<"input postion, positive for red, negetive for blue\ninput: ";
  std::cin>>start_pose_signal;
  ros::param::set("side_color", start_pose_signal);
  const ros::Publisher cover_pub = node_handle.advertise<std_msgs::UInt8>("/cover_cmd", 2);
  const ros::Publisher schema2_pub = node_handle.advertise<std_msgs::UInt8>("/schema2_node", 2);
  ros::ServiceClient navi_client = node_handle.serviceClient<eac_pkg::EacGoal>("navigation");
  if(start_pose_signal == 0) {
    ROS_ERROR("error input");
    return 1;
  }
  UserSetPose poses;
  if(start_pose_signal > 0) {
    ROS_INFO("choose red pose");
    poses.init_red();
    poses.chosen_color = SIDE_RED;
  }
  else if(start_pose_signal < 0) {
    ROS_INFO("choose blue pose");
    poses.init_blue();
    poses.chosen_color = SIDE_BLUE;
  }

  // 前往设定起点（应该动作幅度不大）
  if(gotoGoal(navi_client, START_POSE) != 0) {
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
  if(gotoGoal(navi_client, PICK_POSE) != 0) {
    ROS_WARN("schema 1 FAILED! try schema 2");
    schema2(schema2_pub);
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
  if(gotoGoal(navi_client, SECURITY_ZONE) != 0) {
    ROS_WARN("schema 1 FAILED! try schema 2");
    schema2(schema2_pub);
    return 0;
  }
  ROS_INFO("reach security zone, check balls state");
  // 在安全区前检查球是否正常，不正常就转方案2
  ros::Subscriber obj_aviliable_sub = node_handle.subscribe("/obj_aviliable", 2, objAviliableCallback);
  std_msgs::UInt8 active_aviliable_check;
  active_aviliable_check.data = 2;
  schema2_pub.publish(active_aviliable_check);
  // ReSharper disable once CppDFALoopConditionNotUpdated
  while (!obj_detected) {
    ros::spinOnce();
  }
  if(obj_state == 2 || obj_state == 3 || (static_cast<SideColor>(obj_state)) != poses.chosen_color) {
    ROS_WARN("schema 1 FAILED! try schema 2");
    schema2(schema2_pub);
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
