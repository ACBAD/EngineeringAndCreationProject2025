#include <ros/ros.h>
#include <std_msgs/String.h>
#include <eac_pkg/ObjectInfoArray.h>
#include <eac_pkg/ObjectInfo.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/UInt8.h>
#include <defines.h>
#include <eac_pkg/EacGoal.h>
#include <eac_pkg/ZoneInfo.h>

#define ROS_SPINIF(x) while ((x) && ros::ok()) { ros::spinOnce(); }

uint8_t trigger = 0;
bool cover_state = false;
eac_pkg::ObjectInfoArray object_infos;
eac_pkg::ZoneInfo zone_info;
int side_color;
SideColor agninst_color;
ros::Publisher twist_pub;

/**
 * 视觉识别回调，滤除对方物体
 * @param msg 物体信息列表
 */
void objectCallback(const eac_pkg::ObjectInfoArray& msg) {
  object_infos.stamp = msg.stamp;
  object_infos.data.resize(0);
  std::copy_if(msg.data.begin(), msg.data.end(), std::back_inserter(object_infos.data),
    [](const eac_pkg::ObjectInfo& obj) {return obj.color != agninst_color;});
}

void zoneCallback(const eac_pkg::ZoneInfo& msg) {zone_info = msg;}

enum DetectionTypes {
  OBJECT_DETECT,
  ZONE_DETECT
};
DetectionTypes detection_type = OBJECT_DETECT;

/**
 *
 * @param angle 角度，默认10
 * @param palstance 角速度，默认0.1
 * @return 0
 */
int sendRotateTwist(const double angle = 10, const double palstance = 0.5) {
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

void nodeCallback(const std_msgs::UInt8& msg) {
  trigger = msg.data;
}

void coverStateCallback(const std_msgs::UInt8 msg) {cover_state = msg.data;}

/**
 * 寻找物体列表内的最近物体，基于distance属性
 * @exception std::out_of_range
 * @return 最近物体在object_infos的迭代器
 */
auto findNearestObject() {
  if(object_infos.data.empty())throw std::out_of_range("obj is none");
  return std::min_element(object_infos.data.begin(), object_infos.data.end(),
    [](const eac_pkg::ObjectInfo& a, const eac_pkg::ObjectInfo& b) {return a.distance < b.distance;});
}

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "schema2_node");
  ros::NodeHandle node_handle;
  ros::Subscriber trigger_sub = node_handle.subscribe("/schema2_cmd", 2, nodeCallback);
  ros::Subscriber object_sub = node_handle.subscribe("/objects_data", 1, objectCallback);
  ros::Subscriber zone_sub = node_handle.subscribe("/zone_data", 1, zoneCallback);
  ros::ServiceClient navi_client = node_handle.serviceClient<eac_pkg::EacGoal>("navigation");
  const ros::Publisher cover_pub = node_handle.advertise<std_msgs::UInt8>("/cover_cmd", 2);
  ros::Subscriber cover_sub = node_handle.subscribe("/cover_state", 2, coverStateCallback);
  twist_pub = node_handle.advertise<geometry_msgs::Twist>("/cmd_vel", 2);
  ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug);
  if (!ros::param::get("side_color", side_color)) {
    ROS_ERROR("side_color not set!");
    return 1;
  }
  ROS_INFO("waiting for trigger...");
  // ReSharper disable once CppDFALoopConditionNotUpdated
  ROS_SPINIF(!trigger);
  ROS_WARN("Triggered!!!");
  agninst_color = side_color == SIDE_RED ? SIDE_BLUE : SIDE_RED;
  uint8_t sys_state = 5;
  constexpr char title_msg[] = "schema2 acting: %s";
  while (sys_state != 0 && ros::ok()) {
    switch (sys_state) {
    case 1: {
      ROS_SPINIF(!checkInfoAviliable(object_infos.stamp));
      if (object_infos.data.size() > 0) {
        sys_state++;
        break;
      }
      ROS_INFO(title_msg, "rotating, checking objects...");
      sendRotateTwist();
      // ReSharper disable once CppExpressionWithoutSideEffects
      ros::Duration(0.5).sleep();
      break;

    }
    case 2: {
      ROS_INFO(title_msg, "object detected, aligning nearest...");
      // 获取最近的物体
      auto nearest_object = object_infos.data.begin();

      // 顿挫转圈以将最近物体置于视野中央
      while (abs(nearest_object->angle) > ANGLE_TOLERANCE_LIMIT(nearest_object->distance)) {
        auto last_stamp = object_infos.stamp;
        ROS_SPINIF(!checkInfoAviliable(object_infos.stamp));
        ROS_DEBUG("Now stamp is %f, last stamp is %f", object_infos.stamp.toSec(), last_stamp.toSec());
        try {nearest_object = findNearestObject();}
        catch (const std::out_of_range &e) {
          ROS_WARN(title_msg, e.what());
          sys_state = 1;
          break;
        }
        ROS_DEBUG("Now angle is %f, limit is %f", nearest_object->angle, ANGLE_TOLERANCE_LIMIT(nearest_object->distance));
        if(abs(nearest_object->angle) < ANGLE_TOLERANCE_LIMIT(nearest_object->distance))break;
        sendRotateTwist(nearest_object->angle > 0 ? 30 : -30);
        // ReSharper disable once CppExpressionWithoutSideEffects
        ros::Duration(1.0).sleep();
        ROS_DEBUG("rotating ...");
      }
      ROS_INFO(title_msg, "aligning ok");
      sys_state++;
      break;
    }
    case 3: {
      ROS_INFO(title_msg, "try to reach object");
      sendStraightTwist(0.2);
      ROS_WARN(title_msg, "go straight for object");
      // 定义检查物体是否到达的函数
      auto checkReachObjectState = []() {
        if (checkInfoAviliable(object_infos.stamp)) {
          return std::any_of(object_infos.data.begin(), object_infos.data.end(),
            [](const eac_pkg::ObjectInfo& n) {
              return n.distance < COVERABLE_DISTANCE;
            });}
        return false;
      };
      ROS_WARN(title_msg, "checking if reached");
      ROS_SPINIF(!checkReachObjectState());
      ROS_WARN(title_msg, "object coverable, stop");
      sendStraightTwist(0);
      ROS_SPINIF(!checkInfoAviliable(object_infos.stamp));
      if (checkReachObjectState()) {
        sys_state++;
        break;
      }
      ROS_WARN(title_msg, "reach failed, back to case 1");
      sys_state = 1;
      break;
    }
    case 4: {
      ROS_INFO(title_msg, "try to cover object");
      std_msgs::UInt8 cover_angle;
      cover_angle.data = 10;
      cover_pub.publish(cover_angle);
      cover_state = false;
      // ReSharper disable once CppDFALoopConditionNotUpdated
      ROS_SPINIF(!cover_state);
      ROS_INFO(title_msg, "object coverd! check cover state");
      ROS_SPINIF(!checkInfoAviliable(object_infos.stamp));
      if(object_infos.data.size() == 0) {
        ROS_WARN(title_msg, "cover nothing !");
        sys_state = 1;
        break;
      }
      ROS_INFO(title_msg, "objects ok! back to security zone...");
      sys_state++;
      break;
    }
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
      while (abs(zone_info.angle) > ANGLE_TOLERANCE_LIMIT(zone_info.distance)) {
        auto last_stamp = zone_info.stamp;
        ROS_SPINIF(!checkInfoAviliable(zone_info.stamp));
        ROS_DEBUG("Now stamp is %f, last stamp is %f", zone_info.stamp.toSec(), last_stamp.toSec());
        if(zone_info.angle == 0 && zone_info.distance == 0) {
          ROS_WARN(title_msg, "zone lost! back to last case");
          sys_state--;
          break;
        }
        ROS_DEBUG("Now angle is %f, limit is %f", zone_info.angle, ANGLE_TOLERANCE_LIMIT(zone_info.distance));
        if(abs(zone_info.angle) < ANGLE_TOLERANCE_LIMIT(zone_info.distance))break;
        sendRotateTwist(zone_info.angle > 0 ? 30 : -30);
        // ReSharper disable once CppExpressionWithoutSideEffects
        ros::Duration(1.0).sleep();
        ROS_DEBUG("rotating ...");
      }
      ROS_INFO(title_msg, "aligning ok");
      sys_state++;
      break;
    }
    case 7: {
      ROS_INFO(title_msg, "reaching to zone");
      sendStraightTwist(0.2);
      ROS_INFO(title_msg, "going for zone");
      while (zone_info.distance > REACH_ZONE_DISTANCE) {
        ROS_SPINIF(!checkInfoAviliable(zone_info.stamp));
        if (zone_info.distance < REACH_ZONE_DISTANCE)break;
      }
      if(zone_info.angle == 0 && zone_info.distance == 0) {
        ROS_WARN("reach failed!");
        sys_state = 5;
        break;
      }
      ROS_INFO(title_msg, "reach zone");
      ROS_INFO("Debug OK");
      return 0;
      break;
    }
    case 8: {
      ROS_INFO(title_msg, "releasing objects...");
      std_msgs::UInt8 cover_angle;
      cover_angle.data = 0;
      cover_pub.publish(cover_angle);
      cover_state = false;
      // ReSharper disable once CppDFALoopConditionNotUpdated
      ROS_SPINIF(!cover_state);
      sendStraightTwist(-1, -0.2);
      sys_state = 1;
      break;
    }
    default: {
      ROS_WARN(title_msg, "invalid sys_state, return to 0");
      sys_state = 1;
      break;
    }
    }
  }
  return 0;
}
