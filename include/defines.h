//
// Created by xd on 25-1-23.
//

#ifndef DEFINES_H
#define DEFINES_H
#include <geometry_msgs/Pose.h>
constexpr double pi = 3.14159265358979323;

#define REACH_ZONE_DISTANCE 30
#define COVERABLE_DISTANCE 430
#define DISTANCE_TOLERANCE_LIMIT 9999999999999
#define ANGLE_TOLERANCE_LIMIT(x) -1.904640251876979e-9*x*x*x*x*x+0.000004798811956045067*x*x*x*x-0.004808540901191375*x*x*x+2.3948489957436414*x*x-593.4371614544533*x+58815.573043513534
#define ZONE_ANGLE_LIMIT 160


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
    poses[START_POSE].position.x = 0.9126139879226685;
    poses[START_POSE].position.y = 0.14473581314086914;
    poses[START_POSE].orientation.z = -0.7088159706112025;
    poses[START_POSE].orientation.w = 0.7053934503569613;

    poses[PICK_POSE].position.x = 3.0;
    poses[PICK_POSE].position.y = 1.5;
    poses[PICK_POSE].orientation.z = 0.0;
    poses[PICK_POSE].orientation.w = 1.0;

    poses[SECURITY_ZONE].position.x = -0.02042520046234131;
    poses[SECURITY_ZONE].position.y = -0.7943828105926514;
    poses[SECURITY_ZONE].orientation.z = 0.9999943086827945;
    poses[SECURITY_ZONE].orientation.w = 0.0033738112009850593;
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

    poses[SECURITY_ZONE].position.x = -0.09043943881988525;
    poses[SECURITY_ZONE].position.y = -0.77778160572052;
    poses[SECURITY_ZONE].orientation.z = -0.9999017358670853;
    poses[SECURITY_ZONE].orientation.w = 0.014018509549504503;
  }
};

enum ObjectColor {
  OBJ_RED,
  OBJ_BLUE,
  OBJ_BLACK,
  OBJ_GREEN
};

inline bool checkInfoAvailable(const ros::Time& infoTime,
  const ros::Duration& delta = ros::Duration(0.25)) {
  return (ros::Time::now() - infoTime) <= delta;
}

#endif //DEFINES_H
