#include <ros/ros.h>
#include <iostream>
#include <std_msgs/UInt8.h>

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "recognize_node");
  ros::NodeHandle node_handle;
  const ros::Publisher publisher = node_handle.advertise<std_msgs::UInt8>("/recognize", 2);
  std_msgs::UInt8 data;
  while (ros::ok()) {
    std::cin>>data.data;
    publisher.publish(data);
    std::cout<<"sent: "<<data.data<<std::endl;
  }
}

