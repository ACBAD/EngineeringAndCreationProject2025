//
// Created by xd on 25-1-23.
//

#ifndef DEFINES_H
#define DEFINES_H
#include <geometry_msgs/Pose.h>
constexpr double pi = 3.14159265358979323;

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
    poses[START_POSE].position.x = 0.02624654769897461;
    poses[START_POSE].position.y = -0.8079619407653809;
    poses[START_POSE].orientation.z = 0.010020251335068935;
    poses[START_POSE].orientation.w = 0.9999497960213712;

    poses[PICK_POSE].position.x = 3.0;
    poses[PICK_POSE].position.y = 1.5;
    poses[PICK_POSE].orientation.z = 0.0;
    poses[PICK_POSE].orientation.w = 1.0;

    poses[SECURITY_ZONE].position.x = 0.0;
    poses[SECURITY_ZONE].position.y = 0.0;
    poses[SECURITY_ZONE].orientation.z = 0.0;
    poses[SECURITY_ZONE].orientation.w = 1.0;
  }
  void init_blue() {
    chosen_color = SIDE_BLUE;
    poses[START_POSE].position.x = 0.02624654769897461;
    poses[START_POSE].position.y = -0.8079619407653809;
    poses[START_POSE].orientation.z = 0.010020251335068935;
    poses[START_POSE].orientation.w = 0.9999497960213712;

    poses[PICK_POSE].position.x = 3.0;
    poses[PICK_POSE].position.y = 1.5;
    poses[PICK_POSE].orientation.z = 0.0;
    poses[PICK_POSE].orientation.w = 1.0;

    poses[SECURITY_ZONE].position.x = 0.0;
    poses[SECURITY_ZONE].position.y = 0.0;
    poses[SECURITY_ZONE].orientation.z = 0.0;
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
