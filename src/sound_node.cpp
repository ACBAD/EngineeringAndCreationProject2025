#include <ros/ros.h>
#include <std_msgs/UInt8.h>

void playCallback(const std_msgs::UInt8& msg) {
  if(msg.data == 0) {

  }
}

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "sound_node");
  ros::NodeHandle node_handle;
  ros::Subscriber subscriber = node_handle.subscribe("/play", 2, playCallback);
  ros::spin();
}

