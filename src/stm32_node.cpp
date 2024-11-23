#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <unistd.h>
#include <termios.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <csignal>
#include <poll.h>
#include <rapidjson/document.h>
#include <rapidjson/prettywriter.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/UInt32.h>
#define SERIAL_PORT "/dev/ttyS3"
#define MAX_LINEAR_SPEED 1.2
#define MAX_ANG_SPEED 1
#define READ_STR_LENGTH 500

ros::Publisher rw_pub;
ros::Publisher lw_pub;
ros::Publisher sound_pub;
geometry_msgs::Twist global_twist;
std_msgs::UInt8 global_rail;

class SerialDevice {
  int serial_port = -1;
public:
  SerialDevice(){
    serial_port = open(SERIAL_PORT, O_RDWR | O_NOCTTY);
    ROS_WARN("Serial Open");
    int size = 8192; // 设置为 8KB
    ioctl(serial_port, TIOCSWINSZ, &size);
    ROS_WARN("Set buffer size: %d", size);
  }
  ~SerialDevice() {
    if(serial_port < 0)
      return;
    close(serial_port);
  }
  rapidjson::Document tread(const int timeout_ms) const {
    if(serial_port < 0) {
      ROS_WARN("Read failed: serial not open");
      return nullptr;
    }
    pollfd fds{};
    fds.fd = serial_port;
    fds.events = POLLIN; // 关注是否可读
    // 以毫秒为单位设置超时
    const int ret = poll(&fds, 1, timeout_ms);
    if(ret <= 0) {
      if (ret == 0)
        ROS_WARN("Read failed: Wait timeout");
      else
        ROS_WARN("Read failed: Poll Error 2");
      return nullptr;
    }
    if (!(fds.revents & POLLIN)) {
      ROS_WARN("Read failed: Poll Error 1");
      return nullptr;
    }
    char buffer[READ_STR_LENGTH];
    const ssize_t read_count = read(serial_port, buffer, READ_STR_LENGTH);
    if(read_count <= 0) {
      ROS_WARN("Read failed: read count is %ld", read_count);
      return nullptr;
    }
    ROS_DEBUG("Read count: %ld", read_count);
    rapidjson::Document ret_d;
    ret_d.SetObject();
    if(ret_d.Parse(buffer).HasParseError()) {
      ROS_INFO("Read failed: Parse failed, try to extract json");
      const char* left_brace = strchr(buffer, '{');
      char* right_brace = strchr(buffer, '}');
      if(left_brace == nullptr)
        ROS_WARN("Read failed: extract failed, { not exist , raw str is %s", buffer);
      else if(right_brace == nullptr)
        ROS_WARN("Read failed: extract failed, } not exist , raw str is %s", buffer);
      else {
        *(right_brace + 1) = 0;
        ret_d.SetObject();
        if(ret_d.Parse(left_brace).HasParseError()) {
          ROS_WARN("Read failed: extract failed, total failed , raw str is %s", buffer);
          return nullptr;
        }
      }
    }
    ROS_DEBUG("Read raw str: %s", buffer);
    return ret_d;
  }
  ssize_t send(const rapidjson::Document& d) const {
    if(serial_port < 0)
      return -10;
    tcflush(serial_port, TCIOFLUSH);
    rapidjson::StringBuffer sb;
    rapidjson::Writer<rapidjson::StringBuffer> writer(sb);
    d.Accept(writer);
    const ssize_t write_count = write(serial_port, sb.GetString(), sb.GetSize());
    if(write_count == sb.GetSize())
      ROS_DEBUG("Send vel: %s", sb.GetString());
    else
      ROS_WARN("Failed send vel: %s, success_num is %ld", sb.GetString(), write_count);
    return write_count;
  }
};

void updateMotorAction(const geometry_msgs::Twist& msg) {
  global_twist = msg;
}

void updateRailLocation(const std_msgs::UInt8& msg) {
  global_rail = msg;
}

void sendAllArgs(const SerialDevice& sd) {
  const int x = std::min(static_cast<int>(global_twist.linear.x/MAX_LINEAR_SPEED*100), 100);
  const int r = std::min(static_cast<int>(global_twist.angular.z/MAX_ANG_SPEED*100), 100);
  rapidjson::Document vel_obj;
  vel_obj.SetObject();
  vel_obj.AddMember("X", x, vel_obj.GetAllocator());
  vel_obj.AddMember("R", r, vel_obj.GetAllocator());
  vel_obj.AddMember("Rail", global_rail.data, vel_obj.GetAllocator());
  const ssize_t write_count = sd.send(vel_obj);
  if(write_count < 0)
    return;
  rapidjson::Document stm32_data = std::move(sd.tread(200));
  if(stm32_data.IsNull())
    return;

  if(!stm32_data.HasMember("R")) {
    ROS_WARN("Decode error: R not exist");
    return;
  }
  if(!stm32_data.HasMember("L")) {
    ROS_WARN("Decode error: L not exist");
    return;
  }
  if(!stm32_data["R"].IsUint64()) {
    ROS_WARN("Decode error: R is not Uint64");
    return;
  }
  if(!stm32_data["L"].IsUint64()) {
    ROS_WARN("Decode error: L is not Uint64");
    return;
  }

  std_msgs::UInt32 rWheel, lWheel;
  rWheel.data = stm32_data["R"].GetUint64();
  lWheel.data = stm32_data["L"].GetUint64();
  rw_pub.publish(rWheel);
  lw_pub.publish(lWheel);
  if(!stm32_data.HasMember("SC")) {
    ROS_WARN("Decode error: SC cmd not exist");
    return;
  }
  if(!stm32_data["SC"].IsUint()) {
    ROS_WARN("Decode error: SC cmd type error");
    return;
  }
  std_msgs::UInt8 sound_cmd;
  sound_cmd.data = stm32_data["SC"].GetUint();
  sound_pub.publish(sound_cmd);
}

int main(int argc, char* argv[]) {

  // ROS init
  ros::init(argc, argv, "stm32_node");
  ros::NodeHandle node_handle;

  rw_pub = node_handle.advertise<std_msgs::UInt32>("/rwheel_ticks", 2);
  lw_pub = node_handle.advertise<std_msgs::UInt32>("/lwheel_ticks", 2);
  sound_pub = node_handle.advertise<std_msgs::UInt8>("/sound_cmd", 2);
  ros::Subscriber vel_sub = node_handle.subscribe("/cmd_vel", 2, updateMotorAction);
  ros::Subscriber rail_sub = node_handle.subscribe("/rail_cmd", 2, updateRailLocation);
  ros::Rate rate(20);
  ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);
  // ReSharper disable once CppTooWideScope
  const SerialDevice serial_device;
  global_twist.linear.x = 0;
  global_twist.linear.y = 0;
  global_twist.linear.z = 0;
  global_twist.angular.x = 0;
  global_twist.angular.y = 0;
  global_twist.angular.z = 0;
  while (ros::ok()) {
    ros::spinOnce();
    sendAllArgs(serial_device);
    rate.sleep();
  }
  return 0;
}
