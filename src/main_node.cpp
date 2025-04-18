#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Twist.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <std_msgs/UInt8.h>
#include <iostream>
#include <defines.h>
#include <eac_pkg/EacGoal.h>
#include <eac_pkg/ZoneInfo.h>

#define ROS_SPINIF(x) while ((x) && ros::ok()) { ros::spinOnce(); }

ros::Publisher twist_pub;
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
int side_color = 0;
bool cover_state = true;
bool clog_state = false;
eac_pkg::ZoneInfo zone_info;

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

void clogCallback(const std_msgs::UInt8& msg) {
  if(msg.data == 'N')clog_state = false;
  else clog_state = true;
}

void zoneCallback(const eac_pkg::ZoneInfo& msg) {zone_info = msg;}

/**
 *
 * @param angle 角度，默认10
 * @param palstance 角速度，默认0.1
 * @return 0
 */
int sendRotateTwist(const double angle = 10, const double palstance = 0.5) {
  const double radian = angle * (pi / 180.0);
  const double wait_sec = abs(radian / palstance);
  geometry_msgs::Twist rotate_cmd;
  rotate_cmd.angular.z = palstance;
  twist_pub.publish(rotate_cmd);
  rotate_cmd.angular.z = 0;
  // ReSharper disable once CppExpressionWithoutSideEffects
  ros::Duration(wait_sec).sleep();
  twist_pub.publish(rotate_cmd);
  return 0;
}

/**
 *
 * @param distance_velocity 当速度为0时，此值为速度，速度非零时此值为前进距离
 * @param velocity 速度，默认为0
 * @return 构造的Twist消息
 */
geometry_msgs::Twist sendStraightTwist(const double distance_velocity, const double velocity = 0) {
  geometry_msgs::Twist straight_cmd;
  straight_cmd.linear.x = velocity == 0 ? distance_velocity : velocity;
  twist_pub.publish(straight_cmd);
  straight_cmd.linear.x = 0;
  if (velocity == 0)return straight_cmd;
  const double wait_sec = distance_velocity / velocity;
  // ReSharper disable once CppExpressionWithoutSideEffects
  ros::Duration(wait_sec).sleep();
  twist_pub.publish(straight_cmd);
  return straight_cmd;
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
  const ros::Publisher schema2_pub = node_handle.advertise<std_msgs::UInt8>("/schema2_cmd", 2);
  ros::ServiceClient navi_client = node_handle.serviceClient<eac_pkg::EacGoal>("navigation");
  twist_pub = node_handle.advertise<geometry_msgs::Twist>("/cmd_vel", 2);
  ros::Subscriber clog_sub = node_handle.subscribe("/clogging_state", 1, clogCallback);
  ros::Subscriber zone_sub = node_handle.subscribe("/zone_data", 1, zoneCallback);

  // 重置罩子状态
  cover_angle.data = 5;
  cover_pub.publish(cover_angle);
  cover_angle.data = 195;
  cover_pub.publish(cover_angle);
  // ReSharper disable once CppExpressionWithoutSideEffects
  ros::Duration(3).sleep();
  ROS_INFO("init ok");
  // 等待发出开始指令
  std::cout<<"waiting for start signal";
  if(const char sig = std::cin.get(); sig == '0') {
    ROS_WARN("user out");
    return 1;
  }
  // ReSharper disable once CppExpressionWithoutSideEffects
  ros::Duration(3.0).sleep();
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
  ros::Duration(0.8).sleep();
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
  // if(gotoGoal(navi_client, SECURITY_ZONE) != 0) {
  //   ROS_WARN("schema 1 FAILED! try schema 2");
  //   schema2(schema2_pub);
  //   return 0;
  // }
  // ROS_INFO("reach security zone");
  int sys_state = 5;
  constexpr char title_msg[] = "main acting: %s";
  while (true) {
    switch (sys_state) {
    case 5: {
      ROS_SPINIF(!checkInfoAviliable(zone_info.stamp));
      if(zone_info.distance == 0 && zone_info.angle == 0) {
        ROS_INFO(title_msg, "can not detect zone, rotating...");
        sendRotateTwist();
        break;
      }
      ROS_DEBUG("Now zone, distance is %f, angle is %f", zone_info.distance, zone_info.angle);
      ROS_INFO(title_msg, "zone detected! next");
      sys_state++;
      break;
    }
    case 6: {
      while (abs(zone_info.angle) > ZONE_ANGLE_LIMIT) {
        auto last_stamp = zone_info.stamp;
        ROS_SPINIF(!checkInfoAviliable(zone_info.stamp));
        ROS_DEBUG("Now stamp is %f, last stamp is %f", zone_info.stamp.toSec(), last_stamp.toSec());
        if(zone_info.angle == 0 && zone_info.distance == 0) {
          ROS_WARN(title_msg, "zone lost! back to last case");
          sys_state--;
          break;
        }
        ROS_DEBUG("Now angle is %f, limit is %f", zone_info.angle, ANGLE_TOLERANCE_LIMIT(zone_info.distance));
        if(abs(zone_info.angle) < ZONE_ANGLE_LIMIT)break;
        sendRotateTwist(zone_info.angle > 0 ? 30 : -30);
        // ReSharper disable once CppExpressionWithoutSideEffects
        ros::Duration(1.0).sleep();
        ROS_DEBUG("rotating ...");
      }
      if(sys_state == 5)break;
      ROS_INFO(title_msg, "aligning ok");
      sys_state++;
      break;
    }
    case 7: {
      ROS_INFO(title_msg, "reaching to zone");
      sendStraightTwist(0.2);
      ROS_INFO(title_msg, "going for zone");
      // 旧逻辑，通过视觉判断是否到达安全区
      // while (zone_info.distance > REACH_ZONE_DISTANCE) {
      //   ROS_SPINIF(!checkInfoAviliable(zone_info.stamp));
      //   if (zone_info.distance < REACH_ZONE_DISTANCE)break;
      // }
      // 新逻辑，通过堵转判断是否到达安全区
      while (!clog_state) {
        ROS_SPINIF(!checkInfoAviliable(zone_info.stamp));
        if (zone_info.distance == 0)break;
      }
      if(zone_info.angle == 0 && zone_info.distance == 0) {
        ROS_WARN("reach failed!");
        sys_state = 5;
        break;
      }
      ROS_INFO(title_msg, "reach zone");
      sendStraightTwist(0);
      sys_state ++;
      break;
    }
    case 8: {
      ROS_INFO(title_msg, "releasing objects...");
      cover_angle.data = 195;
      cover_pub.publish(cover_angle);
      cover_state = false;
      // ReSharper disable once CppDFALoopConditionNotUpdated
      ROS_SPINIF(!cover_state);
      sendStraightTwist(-1, -0.2);
      schema2(schema2_pub);
      return 0;
    }
    default: {
      ROS_WARN("invalid sys_state, return to 0");
      sys_state = 1;
      break;
    }
    }
  }
}
