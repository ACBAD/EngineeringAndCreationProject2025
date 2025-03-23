#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "distance_test");
  ros::NodeHandle node_handle;
  const ros::Publisher twist_pub = node_handle.advertise<geometry_msgs::Twist>("/cmd_vel", 2);
  while (ros::ok()) {
    double sleep_time;
    ROS_INFO("Input sleep");
    std::cin>>sleep_time;
    if (sleep_time == 0) {
      ROS_INFO("DONE");
      break;
    }
    ROS_INFO("Sleep for %lf", sleep_time);
    geometry_msgs::Twist go_msg;
    go_msg.linear.x = 0.7;
    twist_pub.publish(go_msg);
    ROS_INFO("GO");
    // ReSharper disable once CppExpressionWithoutSideEffects
    ros::Duration(sleep_time).sleep();
    go_msg.linear.x = 0;
    twist_pub.publish(go_msg);
    ROS_INFO("Stop");
  }
}

