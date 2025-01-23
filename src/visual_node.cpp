#include <ros/ros.h>
#include <eac_pkg/VrResult.h>
#include <eac_pkg/ObjectInfo.h>
#include <eac_pkg/ObjectInfoArray.h>

// TODO 测量距离系数
#define DISTANCE_SCALE 1.0
// TODO 测量角度偏移系数
#define ANGLE_SCALE 1.0
// TODO 定义图像宽高
#define IMAGE_HEIGHT 640
#define IMAGE_WIDTH 640

ros::Publisher objects_pub;

void visualCallback(const eac_pkg::VrResultConstPtr& msg) {
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

  // convert visual result to distance and angle
  const uint8_t result_offset = msg->locations.layout.dim[1].stride;
  if(msg->locations.data.size() < now_count) {
    ROS_WARN(invalid_msg, "location data insufficient");
    return;
  }
  eac_pkg::ObjectInfoArray objects;
  objects.data.resize(now_count);
  for (int object_index = 0;object_index < now_count; object_index++) {
    // TODO 测量球框对角线长度
    constexpr double ball_diagonal = 1.0;
    const double ltx_location = msg->locations.data[object_index * result_offset];
    const double lty_location = msg->locations.data[object_index * result_offset + 1];
    const double rbx_location = msg->locations.data[object_index * result_offset + 2];
    const double rby_location = msg->locations.data[object_index * result_offset + 3];
    const uint8_t color = msg->colors.data[object_index];
    const uint8_t shape = msg->shapes.data[object_index];

    // check validation
    auto checkAndWarn = [&](const int value, const int max, const char* name) -> bool {
      if (value < 0 || value > max) {
        ROS_WARN(invalid_msg, name);
        return false; }
      return true;
    };
    if(!checkAndWarn(ltx_location, IMAGE_WIDTH, "ltx invalid"))continue;
    if(!checkAndWarn(rbx_location, IMAGE_WIDTH, "rbx invalid"))continue;
    if(!checkAndWarn(lty_location, IMAGE_HEIGHT, "lty invalid"))continue;
    if(!checkAndWarn(rby_location, IMAGE_HEIGHT, "rby invalid"))continue;

    const double visual_diagonal = sqrt(pow((ltx_location - rbx_location), 2) + pow((lty_location - rby_location), 2));
    const double distance = DISTANCE_SCALE * visual_diagonal / ball_diagonal;
    const double center_x = (ltx_location + rbx_location) / 2;
    const double angle = ANGLE_SCALE * center_x - IMAGE_WIDTH / 2;

    objects.data[object_index].angle = angle;
    objects.data[object_index].distance = distance;
    objects.data[object_index].color = color;
    objects.data[object_index].shape = shape;
    ROS_DEBUG("convert done");
  }
  objects.stamp = ros::Time::now();
  objects_pub.publish(objects);
}


int main(int argc, char* argv[]) {
  ros::init(argc, argv, "visual_node");
  ros::NodeHandle node_handle;
  ROS_INFO("visual_node inited");
  objects_pub = node_handle.advertise<eac_pkg::ObjectInfoArray>("/objects_data", 2);
  ros::Subscriber visual_sub = node_handle.subscribe("/visual_data", 2, visualCallback);
  ros::spin();
}

