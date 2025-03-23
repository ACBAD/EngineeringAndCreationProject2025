#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "distance_test");
  ros::NodeHandle node_handle;
  const ros::Publisher twist_pub = node_handle.advertise<geometry_msgs::Twist>("/cmd_vel", 2);
  while (ros::ok()) {
    double end_vel, last_time, time_step;
    ROS_INFO("Input end vel");
    std::cin>>end_vel;
    if (end_vel == 0) {
      ROS_INFO("DONE");
      break;
    }
    ROS_INFO("End vel is %lf", end_vel);
    ROS_INFO("Input time step");
    std::cin>>time_step;
    ROS_INFO("Time step is %lf", time_step);
    ROS_INFO("Input last time");
    std::cin>>last_time;
    ROS_INFO("START GO");
    geometry_msgs::Twist go_msg;
    for (double i = 0; i < end_vel ;i += 0.01) {
      go_msg.linear.x = i;
      twist_pub.publish(go_msg);
      // ReSharper disable once CppExpressionWithoutSideEffects
      ros::Duration(time_step).sleep();
    }

    if (last_time > 0) {
      go_msg.linear.x = end_vel;
      twist_pub.publish(go_msg);
      ROS_INFO("GO");
      // ReSharper disable once CppExpressionWithoutSideEffects
      ros::Duration(last_time).sleep();
    }

    go_msg.linear.x = 0;
    twist_pub.publish(go_msg);
    ROS_INFO("Stop");
  }
}

