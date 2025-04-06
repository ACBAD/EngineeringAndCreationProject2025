#include <ros/ros.h>
#include <std_msgs/String.h>
#include <eac_pkg/ObjectInfoArray.h>
#include <eac_pkg/ObjectInfo.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/UInt8.h>
#include <defines.h>
#include <eac_pkg/EacGoal.h>

#define ROS_SPINFOR(x) while (x && ros::ok()) { ros::spinOnce(); }

uint8_t trigger = 0;
bool cover_state = false;
eac_pkg::ObjectInfoArray object_infos;
int side_color;
SideColor agninst_color;

/**
 * 视觉识别回调，滤除对方物体
 * @param msg 物体信息列表
 */
void objectCallback(const eac_pkg::ObjectInfoArray& msg) {
  object_infos.stamp = msg.stamp;
  std::copy_if(msg.data.begin(), msg.data.end(), std::back_inserter(object_infos.data),
    [](const eac_pkg::ObjectInfo& obj) {return obj.color != agninst_color;});
}

ros::Publisher twist_pub;

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
  straight_cmd.linear.x = velocity == 0 ? distance_velocity : 0.5;
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
  ros::Subscriber object_sub = node_handle.subscribe("/objects_data", 2, objectCallback);
  ros::ServiceClient navi_client = node_handle.serviceClient<eac_pkg::EacGoal>("navigation");
  const ros::Publisher cover_pub = node_handle.advertise<std_msgs::UInt8>("/cover_cmd", 2);
  ros::Subscriber cover_sub = node_handle.subscribe("/cover_state", 2, coverStateCallback);
  twist_pub = node_handle.advertise<geometry_msgs::Twist>("/cmd_vel", 2);
  ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug);
  ROS_INFO("waiting for trigger...");
  // ReSharper disable once CppDFALoopConditionNotUpdated
  ROS_SPINFOR(!trigger);
  ROS_WARN("Triggered!!!");
  if (!ros::param::get("side_color", side_color)) {
    ROS_ERROR("side_color not set!");
    return 1;
  }
  agninst_color = side_color == SIDE_RED ? SIDE_BLUE : SIDE_RED;
  uint8_t sys_state = 1;
  constexpr char title_msg[] = "schema2 acting: %s";
  while (sys_state != 0 && ros::ok()) {
    switch (sys_state) {
    case 1: {
      ROS_SPINFOR(!checkInfoAviliable(object_infos.stamp));
      if (object_infos.data.size() > 0) {
        sys_state++;
        break;
      }
      ROS_INFO(title_msg, "rotating, checking objects...");
      sendRotateTwist();
      // ReSharper disable once CppExpressionWithoutSideEffects
      ros::Duration(1.00).sleep();
      break;

    }
    case 2: {
      ROS_INFO(title_msg, "object detected, aligning nearest...");
      // 获取最近的物体
      auto nearest_object = object_infos.data.begin();

      // 顿挫转圈以将最近物体置于视野中央
      while (abs(nearest_object->angle) > ANGLE_TOLERANCE_LIMIT(nearest_object->distance)) {
        ros::spinOnce();
        if(!checkInfoAviliable(object_infos.stamp))
          continue;
        try {nearest_object = findNearestObject();}
        catch (const std::out_of_range &e) {
          ROS_WARN(title_msg, "object lost!!!");
          sys_state = 1;
          break;
        }
        ROS_DEBUG("Now angle is %f", nearest_object->angle);
        sendRotateTwist(nearest_object->angle > 0 ? 10 : -10);
        sendRotateTwist(0);
        // ReSharper disable once CppExpressionWithoutSideEffects
        ros::Duration(1.0).sleep();
        ROS_DEBUG("rotating ...");
      }
      ROS_INFO(title_msg, "aligning ok");
      // 检查物体是否仍available
      ROS_SPINFOR(!checkInfoAviliable(object_infos.stamp));

      // 获取最新的最近物体信息
      try {nearest_object = findNearestObject();}
      catch (const std::out_of_range &e) {
        ROS_WARN(title_msg, "object lost!!!");
        sys_state = 1;
        break;
      }
      // 如果物体已进入视野中央，进入下一case
      if (abs(nearest_object->angle) < ANGLE_TOLERANCE_LIMIT(nearest_object->distance)) {
        ROS_WARN("Debug OK");
        return 0;
        sys_state++;
        break;
      }
      ROS_WARN(title_msg, "aligning failed! return to case 2");
      break;
    }
    case 3: {
      ROS_INFO(title_msg, "aligning OK, try to reach object");
      // 获取最近物体
      auto nearest_object = object_infos.data.begin();
      try {nearest_object = findNearestObject();}
      catch (const std::out_of_range &e) {
        ROS_WARN(title_msg, "object lost!!!");
        sys_state = 1;
        break;
      }
      sendStraightTwist(0.5);
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
      ROS_SPINFOR(checkReachObjectState());
      ROS_WARN(title_msg, "object coverable, stop");
      sendStraightTwist(0);

      while (checkInfoAviliable(object_infos.stamp)) {
        // ReSharper disable once CppTooWideScope
        ROS_DEBUG("against_color is %d, obj distance is %f, obj color is %d",
          agninst_color, object_infos.data[0].distance, object_infos.data[0].color);
        if (checkReachObjectState()) {
          sys_state++;
          break;
        }
        ROS_WARN(title_msg, "reach failed, back to case 1");
        sys_state = 1;
        break;
      }
      break;
    }
    case 4: {
      ROS_INFO("try to cover object");
      std_msgs::UInt8 cover_angle;
      cover_angle.data = 5;
      cover_pub.publish(cover_angle);
      cover_state = false;
      // ReSharper disable once CppDFALoopConditionNotUpdated
      ROS_SPINFOR(!cover_state);
      ROS_INFO(title_msg, "back to security zone...");
      eac_pkg::EacGoal goal_msg;
      goal_msg.request.goal_index = SECURITY_ZONE;
      goal_msg.request.timeout = 30;
      if(navi_client.call(goal_msg) == false || goal_msg.response.state == false)
        ROS_WARN(title_msg, "error ouccred in reaching security zone");
      sys_state++;
      break;
    }
    case 5: {
      ROS_INFO(title_msg, "releasing objects...");
      std_msgs::UInt8 cover_angle;
      cover_angle.data = 0;
      cover_pub.publish(cover_angle);
      cover_state = false;
      // ReSharper disable once CppDFALoopConditionNotUpdated
      while (!cover_state){ros::spinOnce();}
      sys_state = 0;
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
