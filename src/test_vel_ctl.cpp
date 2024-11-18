#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#define MAX_LINEAR_SPEED 1.2
#define MAX_ANG_SPEED 1

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "test_vel_vtl");
  ros::NodeHandle node_handle;
  const ros::Publisher publisher = node_handle.advertise<geometry_msgs::Twist>("/cmd_vel", 2);
  while (ros::ok()) {
    float x,r;
    std::cin>>x>>r;
    geometry_msgs::Twist msg;
    msg.linear.x = x/100 * MAX_LINEAR_SPEED;
    msg.linear.y = 0;
    msg.linear.z = 0;
    msg.angular.x = 0;
    msg.angular.y = 0;
    msg.angular.z = r/100 * MAX_ANG_SPEED;
    publisher.publish(msg);
    printf("send cmd_vel: X:%f, R:%f\n", x, r);
  }
  return 0;
}
