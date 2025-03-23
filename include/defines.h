//
// Created by xd on 25-1-23.
//

#ifndef DEFINES_H
#define DEFINES_H
#include <geometry_msgs/Pose.h>
constexpr double pi = 3.1415926535897932384626;

#define DISTANCE_TOLERANCE_LIMIT 9999999999999
#define ANGLE_TOLERANCE_LIMIT 199999999999999

enum SideColor {
  SIDE_RED,
  SIDE_BLUE
};

enum PoseNames {
  COSTUM,
  START_POSE,
  PICK_POSE,
  SECURITY_ZONE
};

class UserSetPose {
public:
  geometry_msgs::Pose poses[10];
  uint8_t chosen_color = -1;
  UserSetPose(){}
  void init_red() {
    chosen_color = SIDE_RED;
    poses[START_POSE].position.x = 0.27276426553726196;
    poses[START_POSE].position.y = -0.0033133625984191895;
    poses[START_POSE].orientation.z = -0.015136861496236815;
    poses[START_POSE].orientation.w = 0.9998854311490111;

    poses[PICK_POSE].position.x = 3.0;
    poses[PICK_POSE].position.y = 1.5;
    poses[PICK_POSE].position.z = 0.0;
    poses[PICK_POSE].orientation.w = 1.0;

    poses[SECURITY_ZONE].position.x = 0.0;
    poses[SECURITY_ZONE].position.y = 0.0;
    poses[SECURITY_ZONE].position.z = 0.0;
    poses[SECURITY_ZONE].orientation.w = 1.0;
  }
  void init_blue() {
    chosen_color = SIDE_BLUE;
    poses[START_POSE].position.x = 1.0;
    poses[START_POSE].position.y = 2.0;
    poses[START_POSE].position.z = 0.5;
    poses[START_POSE].orientation.w = 1.0;

    poses[PICK_POSE].position.x = 3.0;
    poses[PICK_POSE].position.y = 1.5;
    poses[PICK_POSE].position.z = 0.0;
    poses[PICK_POSE].orientation.w = 1.0;

    poses[SECURITY_ZONE].position.x = 0.0;
    poses[SECURITY_ZONE].position.y = 0.0;
    poses[SECURITY_ZONE].position.z = 0.0;
    poses[SECURITY_ZONE].orientation.w = 1.0;
  }
};

enum ObjectColor {
  OBJ_RED,
  OBJ_BLUE,
  OBJ_YELLOW,
  OBJ_BLACK
};

inline bool checkInfoAviliable(const ros::Time infoTime, const ros::Duration delta = ros::Duration(0.01)) {
  if(ros::Time::now() - infoTime > delta)return false;
  return true;
}

#endif //DEFINES_H
