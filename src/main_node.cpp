#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Twist.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <std_msgs/UInt8.h>
#include <iostream>
#include <defines.h>
#include <eac_pkg/EacGoal.h>
#include <eac_pkg/ObjectInfoArray.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
int side_color = 0;
bool cover_state = true;
eac_pkg::ObjectInfoArray object_infos;

void updateObjectsInfos(const eac_pkg::ObjectInfoArray& msg) {object_infos = msg;}

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
  // 此处开始为抽签完成后的调试阶段
  ROS_INFO("main start");
  if(!ros::param::get("side_color", side_color)) {
    ROS_ERROR("side_color not define, do you forget start navigation node?");
    return 1;
  }

  const ros::Publisher cover_pub = node_handle.advertise<std_msgs::UInt8>("/cover_cmd", 2);
  const ros::Publisher schema2_pub = node_handle.advertise<std_msgs::UInt8>("/schema2_node", 2);
  ros::Subscriber object_info_sub = node_handle.subscribe("/objects_data", 2, updateObjectsInfos);
  ros::ServiceClient navi_client = node_handle.serviceClient<eac_pkg::EacGoal>("navigation");

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
  cover_angle.data = 30;
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
  ROS_INFO("reach security zone, check balls state");

  // 在安全区前检查球是否正常，不正常就转方案2
  while (!checkInfoAviliable(object_infos.stamp)){ros::spinOnce();}
  const bool has_red = std::any_of(object_infos.data.begin(), object_infos.data.end(),
                                     [](const eac_pkg::ObjectInfo& n) {
                                       return n.color == OBJ_RED && n.distance < DISTANCE_TOLERANCE_LIMIT ;
                                     });
  const bool has_blue = std::any_of(object_infos.data.begin(), object_infos.data.end(),
                                    [](const eac_pkg::ObjectInfo& n) {
                                      return n.color == OBJ_BLUE && n.distance < DISTANCE_TOLERANCE_LIMIT;
                                    });
  if((side_color == SIDE_RED && has_blue) || (side_color == SIDE_BLUE && has_red)) {
    ROS_ERROR("wrong object state, use schema 2");
    cover_state = false;
    cover_angle.data = 195;
    // 抬起罩子
    cover_pub.publish(cover_angle);
    // spinOnce can call updateCoverState
    // ReSharper disable once CppDFALoopConditionNotUpdated
    while (!cover_state) {ros::spinOnce();}
    schema2(schema2_pub);
    return 0;
  }

  cover_state = false;
  cover_angle.data = 195;
  // 抬起罩子
  cover_pub.publish(cover_angle);
  // spinOnce can call updateCoverState
  // ReSharper disable once CppDFALoopConditionNotUpdated
  while (!cover_state) {ros::spinOnce();}
  // 赢
  ROS_WARN("WIN");
  ros::spin();
  return 0;
}
