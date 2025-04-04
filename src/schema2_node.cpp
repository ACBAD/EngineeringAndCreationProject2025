#include <ros/ros.h>
#include <std_msgs/String.h>
#include <eac_pkg/ObjectInfoArray.h>
#include <eac_pkg/ObjectInfo.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/UInt8.h>
#include <defines.h>
#include <eac_pkg/EacGoal.h>

uint8_t trigger = 0;
bool cover_state = false;
eac_pkg::ObjectInfoArray object_infos;

void objectCallback(const eac_pkg::ObjectInfoArray& msg) {
  object_infos = msg;
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

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "schema2_node");
  ros::NodeHandle node_handle;
  ros::Subscriber trigger_sub = node_handle.subscribe("/schema2_cmd", 2, nodeCallback);
  ros::Subscriber object_sub = node_handle.subscribe("/objects_data", 2, objectCallback);
  ros::ServiceClient navi_client = node_handle.serviceClient<eac_pkg::EacGoal>("navigation");
  const ros::Publisher cover_pub = node_handle.advertise<std_msgs::UInt8>("/cover_cmd", 2);
  ros::Subscriber cover_sub = node_handle.subscribe("/cover_state", 2, coverStateCallback);
  twist_pub = node_handle.advertise<geometry_msgs::Twist>("/cmd_vel", 2);
  ROS_INFO("waiting for trigger...");
  // ReSharper disable once CppDFALoopConditionNotUpdated
  while (!trigger && ros::ok()) {
    ros::spinOnce();
  }
  ROS_WARN("Triggered!!!");
  int side_color;
  if (!ros::param::get("side_color", side_color)) {
    ROS_ERROR("side_color not set!");
    return 1;
  }
  SideColor agninst_color = side_color == SIDE_RED ? SIDE_BLUE : SIDE_RED;
  uint8_t sys_state = 1;
  constexpr char title_msg[] = "schema2 acting: %s";
  while (sys_state != 0 && ros::ok()) {
    switch (sys_state) {
    case 1: {
      while (!checkInfoAviliable(object_infos.stamp) && ros::ok()) { ros::spinOnce(); }
      if (object_infos.data.size() > 0) {
        sys_state++;
        break;
      }
      ROS_INFO(title_msg, "rotating, checking objects...");
      sendRotateTwist();
      break;
    }
    case 2: {
      ROS_INFO(title_msg, "object detected, finding nearest...");
      auto nearest_object = std::min_element(object_infos.data.begin(), object_infos.data.end(),
        [agninst_color](const eac_pkg::ObjectInfo& a, const eac_pkg::ObjectInfo& b) {
          if (a.color == agninst_color)return false;
          if (b.color == agninst_color)return true;
          return a.distance < b.distance;
      });
      if (abs(nearest_object->angle) > ANGLE_TOLERANCE_LIMIT)
        sendRotateTwist(-nearest_object->angle);
      while (!checkInfoAviliable(object_infos.stamp)) { ros::spinOnce(); }
      nearest_object = std::min_element(object_infos.data.begin(), object_infos.data.end(),
        [agninst_color](const eac_pkg::ObjectInfo& a, const eac_pkg::ObjectInfo& b) {
          if (a.color == agninst_color)return false;
          if (b.color == agninst_color)return true;
          return a.distance < b.distance;
      });
      if (nearest_object == object_infos.data.end()) {
        ROS_WARN(title_msg, "object lost!!! return to case 1");
        sys_state = 1;
        break;
       }
      if (abs(nearest_object->angle) < ANGLE_TOLERANCE_LIMIT) {
        sys_state++;
        break;
      }
      ROS_WARN(title_msg, "aligning failed! return to case 2");
      break;
    }
    case 3: {
      ROS_INFO(title_msg, "aligning OK, try to reach object");
      const auto nearest_object = std::min_element(object_infos.data.begin(),
        object_infos.data.end(),
      [agninst_color](const eac_pkg::ObjectInfo& a, const eac_pkg::ObjectInfo& b) {
        if (a.color == agninst_color)return false;
        if (b.color == agninst_color)return true;
        return a.distance < b.distance;
      });
      if (nearest_object->distance < DISTANCE_TOLERANCE_LIMIT) {
        sys_state++;
        break;
      }
      ROS_INFO(title_msg, "too far, reaching ...");
      sendStraightTwist(0.5);
      sys_state++;
      break;
    }
    case 4: {
      ROS_INFO(title_msg, "reaching goal...");
      bool object_reached = false;
      const auto nearest_object = std::min_element(object_infos.data.begin(), object_infos.data.end(),
        [agninst_color](const eac_pkg::ObjectInfo& a,
                       const eac_pkg::ObjectInfo& b) {
          if (a.color == agninst_color)return false;
          if (b.color == agninst_color)return true;
          return a.distance < b.distance;
      });
      ros::Timer reach_timer = node_handle.createTimer(ros::Duration(nearest_object->distance / 0.5),
       [&object_reached](const ros::TimerEvent&) {object_reached = true;});
      auto checkReachObjectState = [agninst_color]() {
        if (checkInfoAviliable(object_infos.stamp)) {
          return std::any_of(object_infos.data.begin(), object_infos.data.end(),
            [agninst_color](const eac_pkg::ObjectInfo& n) {
              return n.color != agninst_color && n.angle < ANGLE_TOLERANCE_LIMIT && n.distance > DISTANCE_TOLERANCE_LIMIT;
           });
        }
        return true;
      };
      while (!object_reached && checkReachObjectState()) { ros::spinOnce(); }
      sendStraightTwist(0);
      while (checkInfoAviliable(object_infos.stamp)) {
        if (std::any_of(object_infos.data.begin(), object_infos.data.end(),
                        [agninst_color](const eac_pkg::ObjectInfo& n) {
                          return n.color != agninst_color && n.distance < DISTANCE_TOLERANCE_LIMIT;
                        })) {
          sys_state++;
          break;
        }
        ROS_WARN("reach failed, back to case 1");
        sys_state = 1;
        break;
      }
      break;
    }
    case 5: {
      std_msgs::UInt8 cover_angle;
      cover_angle.data = 180;
      cover_pub.publish(cover_angle);
      cover_state = false;
      // ReSharper disable once CppDFALoopConditionNotUpdated
      while (!cover_state){ros::spinOnce();}
      ROS_INFO(title_msg, "back to security zone...");
      if (!gotoGoal(navi_client, SECURITY_ZONE)) {
        ROS_WARN(title_msg, "error ouccred in reaching security zone");
        return 2;
      }
      sys_state++;
      break;
    }
    case 6: {
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
      sys_state = 0;
      break;
    }
    }
  }
  return 0;
}
