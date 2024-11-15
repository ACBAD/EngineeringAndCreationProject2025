#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <unistd.h>
#include <termios.h>
#include <fcntl.h>
#include <csignal>
#define SERIAL_PORT "/dev/ttyS3"
#define BAUD_RATE B115200
#define MAX_LINEAR_SPEED 1.2
#define MAX_ANG_SPEED 1

int getSerialDevice() {
  // UART dev init
  termios tty{};
  const int serial_port = open(SERIAL_PORT, O_RDWR | O_NOCTTY);
  if (serial_port < 0) {
    std::cerr<<"Error opening serial port"<<std::endl;
    return -1;
  }
  // Configuration of serial
  memset(&tty, 0, sizeof(tty));
  if (tcgetattr(serial_port, &tty) != 0) {
    std::cerr<<"Error getting serial port attributes"<<std::endl;
    close(serial_port);
    return -1;
  }
  cfsetospeed(&tty, BAUD_RATE); // set correct baud rate
  cfsetispeed(&tty, BAUD_RATE);
  tty.c_cflag |= (CLOCAL | CREAD);
  tty.c_cflag &= ~PARENB;
  tty.c_cflag &= ~CSTOPB;
  tty.c_cflag &= ~CSIZE;
  tty.c_cflag |= CS8;
  // write configration to serial
  if (tcsetattr(serial_port, TCSANOW, &tty) != 0) {
    std::cerr<<"Error setting serial port attributes"<<std::endl;
    close(serial_port);
    return -1;
  }
  return serial_port;
}

int serial_port = -1;

void sendMotorAction(const geometry_msgs::Twist& msg) {
  const int x = std::min(static_cast<int>(msg.linear.x/MAX_LINEAR_SPEED*100), 100);
  const int r = std::min(static_cast<int>(msg.angular.z/MAX_ANG_SPEED*100), 100);
  char ctl_stl[11];
  sprintf(ctl_stl, "X%c%03dR%c%03d",
    msg.linear.x>=0?'+':'-', x,
    msg.angular.z>=0?'+':'-', r
    );
  if(write(serial_port, ctl_stl, 10))
    ROS_INFO("Send vel: %s", ctl_stl);
  else
    ROS_WARN("Failed send vel: %s", ctl_stl);
}

void handleSIGTERM(int sig) {
  std::cout<<"Exit";
  exit(0);
}

int main(int argc, char* argv[]) {
  // SIGTERM handler init
  struct sigaction sa{};
  sa.sa_flags = 0;
  sa.sa_handler = handleSIGTERM;
  if (sigaction(SIGINT, &sa, nullptr) == -1) {
    printf("sig bind err");
    return 1;
  }

  // SerialDevice init
  serial_port = getSerialDevice();
  if(serial_port < 0)
    return 1;

  // ROS init
  ros::init(argc, argv, "motor_control_node");
  ros::NodeHandle node_handle;
  ros::Subscriber vel_sub = node_handle.subscribe("/cmd_vel", 2, sendMotorAction);

  ros::spin();
  return 0;
}
