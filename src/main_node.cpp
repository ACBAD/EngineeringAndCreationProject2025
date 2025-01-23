#include <ros/ros.h>
#include <ros/duration.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <std_msgs/UInt8.h>
#include <eac_pkg/ObjectInfoArray.h>
#include <iostream>

// TODO 测量距离容忍极限
#define DISTANCE_TOLERANCE_LIMIT 1.0
// TODO 测量极限转换系数
#define ANGLE_TOLERANCE_LIMIT 1.0

constexpr double pi = 3.1415926535897932384626;

enum SideColor {
  SIDE_RED,
  SIDE_BLUE
};

enum ObjectColor {
  OBJ_RED,
  OBJ_BLUE,
  OBJ_YELLOW,
  OBJ_BLACK
};

struct ObjectsStatus {
  double distance;
  double angle;
  uint8_t color;
  uint8_t shape;
};

bool visual_state = false;
std::vector<ObjectsStatus> objects_status;

class UserSetPose {
public:
  geometry_msgs::Pose start_pose;
  geometry_msgs::Pose pick_pose;
  geometry_msgs::Pose security_zone;
  uint8_t chosen_color = -1;
  UserSetPose(){}
  void init_red() {
    chosen_color = SIDE_RED;
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
    chosen_color = SIDE_BLUE;
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

void objectCallback(const eac_pkg::ObjectInfoArray& msg) {
  const uint8_t count = msg.data.size();
  objects_status.resize(count);
  for (uint8_t obj_index = 0;obj_index < count;obj_index ++) {
    objects_status[obj_index].distance = msg.data[obj_index].distance.data;
    objects_status[obj_index].angle = msg.data[obj_index].angle.data;
    objects_status[obj_index].color = msg.data[obj_index].color.data;
    objects_status[obj_index].shape = msg.data[obj_index].shape.data;
  }
  ROS_DEBUG("objectCallback OK");
  visual_state = true;
}

void updateCoverState(const std_msgs::UInt8& msg) {
  if(msg.data)cover_state = true;
  else cover_state = false;
}

ros::Subscriber object_sub;
int checkObjectssIsAviliale(const ObjectColor aginst_color) {
  if(objects_status.size() == 0) {
    ROS_WARN("visual recongnize failed!!!");
    return 1;
  }
  for(auto itr = objects_status.begin();itr != objects_status.end();++itr) {
    if(itr->color == aginst_color) {
      ROS_WARN("detected aginst color!!!");
      return 2;
    }
  }
  return 0;
}

ros::Publisher twist_pub;
int sendRotateTwist(const double angle = 30, const double palstance = 1) {
  const double radian = angle * (pi / 180.0);
  const double wait_sec = radian / palstance;
  geometry_msgs::Twist rotate_cmd;
  rotate_cmd.angular.z = palstance;
  twist_pub.publish(rotate_cmd);
  rotate_cmd.angular.z = 0;
  // ReSharper disable once CppExpressionWithoutSideEffects
  ros::Duration(wait_sec).sleep();
  twist_pub.publish(rotate_cmd);
  return 0;
}

geometry_msgs::Twist sendStraightTwist(const double distance_velocity, const double velocity = 0) {
  geometry_msgs::Twist straight_cmd;
  straight_cmd.linear.x = velocity == 0 ? distance_velocity : 0.5;
  twist_pub.publish(straight_cmd);
  straight_cmd.linear.x = 0;
  if(velocity == 0)return straight_cmd;
  const double wait_sec = distance_velocity / velocity;
  // ReSharper disable once CppExpressionWithoutSideEffects
  ros::Duration(wait_sec).sleep();
  twist_pub.publish(straight_cmd);
  return straight_cmd;
}

int waitForVisualResult() {
  visual_state = false;
  // ReSharper disable once CppDFALoopConditionNotUpdated
  while (!visual_state) {
    ros::spinOnce();
  }
  return 0;
}

ros::Timer navi_timer;
int schema2(const ros::NodeHandle& node_handle) {
  uint8_t sys_state = 1;
  constexpr char title_msg[] = "schema2 acting: %s";
  while (sys_state != 0) {
    switch (sys_state) {
    case 1: {
      waitForVisualResult();
      if(objects_status.size() > 0) {
        sys_state++;
        break;
      }
      ROS_INFO(title_msg, "rotating, checking objects...");
      sendRotateTwist();
      break;
    }
    case 2: {
      ROS_INFO(title_msg, "object detected, finding nearest...");
      auto nearest_object = std::min_element(objects_status.begin(), objects_status.end(),
        [](const ObjectsStatus& a, const ObjectsStatus& b){return a.distance < b.distance;});
      if(abs(nearest_object->angle) > ANGLE_TOLERANCE_LIMIT)sendRotateTwist(-nearest_object->angle);
      waitForVisualResult();
      nearest_object = std::min_element(objects_status.begin(), objects_status.end(),
        [](const ObjectsStatus& a, const ObjectsStatus& b){return a.distance < b.distance;});
      if(nearest_object == objects_status.end()) {
        ROS_WARN(title_msg, "object lost!!! return to case 1");
        sys_state = 1;
        break;
      }
      if(abs(nearest_object->angle) < ANGLE_TOLERANCE_LIMIT) {
        sys_state++;break;
      }
      ROS_WARN(title_msg, "aligning failed! return to case 2");
      break;
    }
    case 3: {
      ROS_INFO(title_msg, "aligning OK, try to reach object");
      auto nearest_object = std::min_element(objects_status.begin(), objects_status.end(),
        [](const ObjectsStatus& a, const ObjectsStatus& b){return a.distance < b.distance;});
      if(nearest_object->distance < DISTANCE_TOLERANCE_LIMIT) {
        sys_state++;break;
      }
      ROS_INFO(title_msg, "too far, reaching ...");
      sendStraightTwist(0.5);

      break;
    }
    default:{
      ROS_WARN(title_msg, "invalid sys_state, return to 0");
      sys_state = 0;
      break;
    }
    }
  }
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
  twist_pub = node_handle.advertise<geometry_msgs::Twist>("/cmd_vel", 2);
  ros::Publisher cover_pub = node_handle.advertise<std_msgs::UInt8>("/cover_cmd", 2);
  object_sub = node_handle.subscribe("/objects_data", 2, objectCallback);
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
    schema2(node_handle);
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
    schema2(node_handle);
    return 0;
  }
  ROS_INFO("reach security zone, check balls state");
  // 在安全区前检查球是否正常，不正常就转方案2
  if(!checkObjectssIsAviliale(poses.chosen_color == SIDE_RED ? OBJ_RED : OBJ_BLUE)) {
    ROS_WARN("schema 1 FAILED! try schema 2");
    schema2(node_handle);
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
