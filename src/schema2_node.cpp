#include <ros/ros.h>
#include <std_msgs/String.h>
#include <eac_pkg/ObjectInfoArray.h>
#include <eac_pkg/ObjectInfo.h>
#include <geometry_msgs/Twist.h>
#include <defines.h>

// TODO 测量距离容忍极限
#define DISTANCE_TOLERANCE_LIMIT 1.0
// TODO 测量极限转换系数
#define ANGLE_TOLERANCE_LIMIT 1.0

uint8_t trigger = 0;

bool visual_state = false;
std::vector<ObjectState> objects_status;

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

void nodeCallback(const std_msgs::UInt8& msg) {
  trigger = msg.data;
}

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "schema2_node");
  ros::NodeHandle node_handle;
  ros::Subscriber trigger_sub = node_handle.subscribe("/schema2_cmd", 2, nodeCallback);
  ros::Subscriber object_sub = node_handle.subscribe("/objects_data", 2, objectCallback);
  ROS_INFO("waiting for trigger...");
  // ReSharper disable once CppDFALoopConditionNotUpdated
  while (!trigger) {
    ros::spinOnce();
  }
  ROS_WARN("Triggered!!!");

  if(trigger == 2) {
    ROS_INFO("check aviliable");
    waitForVisualResult();
    const bool has_red = std::any_of(objects_status.begin(), objects_status.end(),
                                     [](const ObjectState& n){return n.color == OBJ_RED;});
    const bool has_blue = std::any_of(objects_status.begin(), objects_status.end(),
                                      [](const ObjectState& n){return n.color == OBJ_BLUE;});
    std_msgs::UInt8 obj_check_state;
    obj_check_state.data = 2;
    if(has_red)obj_check_state.data = OBJ_RED;
    if(has_blue)obj_check_state.data = OBJ_BLUE;
    if(has_blue && has_red)obj_check_state.data = 3;
    const ros::Publisher obj_check_pub = node_handle.advertise<std_msgs::UInt8>("/obj_aviliable", 2);
    obj_check_pub.publish(obj_check_state);
  }

  trigger = 0;
  ROS_INFO("waiting for final trigger...");
  // ReSharper disable once CppDFALoopConditionNotUpdated
  while (!trigger) {
    ros::spinOnce();
  }
  ROS_WARN("Final Triggered!!!");

  if (trigger != 1) {
    ROS_ERROR("WORNG TRINGGER!!!");
    return 1;
  }

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
        [](const ObjectState& a, const ObjectState& b){return a.distance < b.distance;});
      if(abs(nearest_object->angle) > ANGLE_TOLERANCE_LIMIT)sendRotateTwist(-nearest_object->angle);
      waitForVisualResult();
      nearest_object = std::min_element(objects_status.begin(), objects_status.end(),
        [](const ObjectState& a, const ObjectState& b){return a.distance < b.distance;});
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
        [](const ObjectState& a, const ObjectState& b){return a.distance < b.distance;});
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
