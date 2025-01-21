#include<ros/ros.h>
#include<eac_pkg/VrResult.h>
#include<geometry_msgs/Point.h>

// TODO 测量距离系数
#define DISTANCE_SCALE 1.0
// TODO 测量角度偏移系数
#define X_ANGLE_SCALE 1.0
#define Y_ANGLE_SCALE 1.0
// TODO 定义图像宽高
#define IMAGE_HETGHT 640
#define IMAGE_WIDTH 640

void VisualCallback(const eac_pkg::VrResultConstPtr& msg) {
  // TODO 测量球框对角线长度
  constexpr double ball_diagonal = 1.0;
  const uint8_t now_count = msg->count.data;

  // check data validation
  constexpr char invalid_msg[] = "visual data invalid!!! (%s) drop it";
  if(msg->locations.layout.dim[0].size != now_count) {
    ROS_WARN(invalid_msg, "locations size not match count");
    return;}if(msg->colors.layout.dim.size() != now_count) {
    ROS_WARN(invalid_msg, "colors size not match count");
    return;}if(msg->shapes.layout.dim.size() != now_count) {
    ROS_WARN(invalid_msg, "colors size not match count");
    return;
  }

}

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "visual_node");
  ros::NodeHandle node_handle;
  ROS_INFO("visual_node inited");
  ros::Subscriber visual_sub = node_handle.subscribe("visual_data", 2, VisualCallback);
  ros::spin();
}

