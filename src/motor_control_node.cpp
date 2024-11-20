#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <eac_pkg/motor_data.h>
#include <unistd.h>
#include <termios.h>
#include <poll.h>
#include <fcntl.h>
#include <csignal>
#define SERIAL_PORT "/dev/ttyS3"
#define MAX_LINEAR_SPEED 1.2
#define MAX_ANG_SPEED 1

ros::Publisher odom_pub;
geometry_msgs::Twist global_twist;

ssize_t tread(const int fd, void *buf, const size_t count, const int timeout_ms) {
  pollfd fds{};
  fds.fd = fd;
  fds.events = POLLIN; // 关注是否可读
  // 以毫秒为单位设置超时
  const int ret = poll(&fds, 1, timeout_ms);
  if (ret > 0) {
    if (fds.revents & POLLIN) {
      return read(fd, buf, count);
    }
  } else if (ret == 0)
    return -2;
  ROS_ERROR("Poll Error");
  return -1;
}

void substring(const char *src, const int start, const int length, char* dest) {
  if (start >= 0 && length > 0 && start < strlen(src)) {
    strncpy(dest, src + start, length);
    dest[length] = '\0'; // 手动添加字符串结束符
  } else {
    dest[0] = '\0'; // 输入无效时返回空字符串
  }
}

long long convertNumber(const char* motor_data, const int start) {
  char temp_num_str[20];
  errno = 0;
  char* end;
  substring(motor_data, start, 10, temp_num_str);
  const long long temp_num = std::strtoll(temp_num_str, &end, 10);
  if (errno == ERANGE)
    ROS_ERROR("Error in converting %s : Out of range", motor_data);
  else if (*end != '\0')
    ROS_ERROR("Error in converting %s : Invalid number", motor_data);
  else
    return temp_num;
  return -1;
}

void updateMotorAction(const geometry_msgs::Twist& msg) {
  global_twist = msg;
}

void sendMotorAction() {
  const int x = std::min(static_cast<int>(global_twist.linear.x/MAX_LINEAR_SPEED*100), 100);
  const int r = std::min(static_cast<int>(global_twist.angular.z/MAX_ANG_SPEED*100), 100);
  char ctl_str[100];
  sprintf(ctl_str, "{X%c%03dR%c%03d",
    global_twist.linear.x>=0?'+':'-', x,
    global_twist.angular.z>=0?'+':'-', r
    );
  const int write_sp = open("/dev/ttyS3", O_RDWR | O_NOCTTY);
  if (write_sp < 0) {
    std::cerr<<"Error opening serial port"<<std::endl;
  }
  const ssize_t write_count = write(write_sp, ctl_str, strlen(ctl_str));
  close(write_sp);
  if(write_count == strlen(ctl_str))
    ROS_DEBUG("Send vel: %s, strlen:%lu", ctl_str, strlen(ctl_str));
  else
    ROS_WARN("Failed send vel: %s, success_num is %ld", ctl_str, write_count);
  char motor_data[60];
  const int read_sp = open("/dev/ttyS3", O_RDWR | O_NOCTTY);
  const ssize_t read_count = tread(read_sp, motor_data, 60, 1);
  if(read_count > 0) {
    motor_data[52] = 0;
    ROS_DEBUG("Receive raw motor data: %s", motor_data);
    eac_pkg::motor_data data;
    data.stamp = ros::Time::now();
    data.left_laps_p = convertNumber(motor_data, 3);
    data.left_laps_n = convertNumber(motor_data, 16);
    data.right_laps_n = convertNumber(motor_data, 29);
    data.right_laps_p = convertNumber(motor_data, 42);
    if (data.left_laps_p < 0 || data.left_laps_n < 0 || data.right_laps_n < 0 || data.right_laps_p < 0) {
      ROS_WARN("Converting number Error");
      return;
    }
    odom_pub.publish(data);
  }
  else if(read_count == -2)
    ROS_WARN("Failed Receive motor data: Timeout");
  else
    ROS_WARN("Failed Receive motor data: read_count is %ld", read_count);
  close(read_sp);
}

int main(int argc, char* argv[]) {

  // ROS init
  ros::init(argc, argv, "motor_control_node");
  ros::NodeHandle node_handle;
  odom_pub = node_handle.advertise<eac_pkg::motor_data>("/motor_data", 2);
  ros::Subscriber vel_sub = node_handle.subscribe("/cmd_vel", 2, updateMotorAction);
  ros::Rate rate(20);
  ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);

  global_twist.linear.x = 0;
  global_twist.linear.y = 0;
  global_twist.linear.z = 0;
  global_twist.angular.x = 0;
  global_twist.angular.y = 0;
  global_twist.angular.z = 0;
  while (ros::ok()) {
    ros::spinOnce();
    sendMotorAction();
    rate.sleep();
  }
  return 0;
}
