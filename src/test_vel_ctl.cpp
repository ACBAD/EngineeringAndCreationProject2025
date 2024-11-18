#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "test_vel_vtl");
  ros::NodeHandle node_handle;
  const ros::Publisher publisher = node_handle.advertise<geometry_msgs::Twist>("/cmd_vel", 2);
  while (ros::ok()) {
    int x,r;
    std::cin>>x>>r;
    geometry_msgs::Twist msg;
    msg.linear.x = x;
    msg.linear.y = 0;
    msg.linear.z = 0;
    msg.angular.x = 0;
    msg.angular.y = 0;
    msg.angular.z = r;
    publisher.publish(msg);
    printf("send cmd_vel: X:%d, R:%d\n", x, r);
  }
  return 0;
}

