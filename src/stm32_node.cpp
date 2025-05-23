#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <unistd.h>
#include <termios.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <csignal>
#include <poll.h>
#include <rapidjson/document.h>
#include <rapidjson/writer.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/Int32.h>
#include <stm32_defines.h>

ros::Publisher rw_pub;
ros::Publisher lw_pub;
ros::Publisher cover_pub;
ros::Publisher clog_pub;
geometry_msgs::Twist global_twist;
uint8_t cover_cmd = 195;
double total_right = 0;
double total_left = 0;
bool is_left_clog = false, is_right_clog = false;

class EasyDocument{
  rapidjson::Document d;
public:
  EasyDocument() = delete;
  explicit EasyDocument(rapidjson::Document other): d(std::move(other)) {}
  rapidjson::Document extractDocument(){return std::move(d);}
  template <typename T>
  auto getElementEasier(const char* key) const {

    if(d.IsNull())
      throw std::runtime_error("has parse error, or this is a null value");
    if(d.HasParseError())
      throw std::runtime_error("has parse error");
    if(!d.HasMember(key)){
      char _[100];
      std::sprintf(_, "%s not exist", key);
      throw std::runtime_error(_);
    }
    const auto &dk = d[key];
    auto throwType = [key](const char* type) {
      char _[100];
      std::sprintf(_, "%s is not %s", key, type);
      throw std::runtime_error(_);
    };
    if constexpr (std::is_same_v<std::decay_t<T>, bool>) {
      if(!dk.IsBool())
        throwType("bool");
      return dk.GetBool();
    }
    if constexpr (std::is_same_v<std::decay_t<T>, std::string>) {
      if(!dk.IsString())
        throwType("string");
      return dk.GetString();
    }
    if(!dk.IsNumber())
      throwType("number");
    if constexpr (std::is_same_v<std::decay_t<T>, double>)
      return dk.GetDouble();
    if constexpr (std::is_same_v<std::decay_t<T>, int64_t>) {
      if(!dk.IsInt64())
        throwType("int, but float");
      return dk.GetInt64();
    }
  }
};

class SerialDevice {
  int serial_port = -1;
public:
  SerialDevice(){
    serial_port = open(SERIAL_PORT, O_RDWR | O_NOCTTY);
    ROS_WARN("Serial Open");
    int size = 8192;
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
    rapidjson::Writer writer(sb);
    d.Accept(writer);
    const ssize_t write_count = write(serial_port, sb.GetString(), sb.GetSize());
    if(write_count == sb.GetSize())
      ROS_DEBUG("Send vel: %s", sb.GetString());
    else
      ROS_WARN("Failed send vel: %s, success_num is %ld", sb.GetString(), write_count);
    return write_count;
  }
};

void updateMotorAction(const geometry_msgs::Twist& msg) {global_twist = msg;}

void updateCoverAction(const std_msgs::UInt8& msg) {cover_cmd = msg.data;}

void sendAllArgs(const SerialDevice& sd) {
  const double x = std::min(global_twist.linear.x, MAX_LINEAR_SPEED);
  const double r = std::min(global_twist.angular.z, MAX_LINEAR_SPEED);
  rapidjson::Document cmd_obj;
  cmd_obj.SetObject();
  cmd_obj.AddMember("X", x, cmd_obj.GetAllocator());
  cmd_obj.AddMember("R", r, cmd_obj.GetAllocator());
  cmd_obj.AddMember("cover_cmd", cover_cmd, cmd_obj.GetAllocator());
  if(const ssize_t write_count = sd.send(cmd_obj); write_count < 0)return;
  std_msgs::UInt8 cover_state;
  const EasyDocument stm32_data(std::move(sd.tread(200)));
  try {
    total_right = stm32_data.getElementEasier<double>("R");
    total_left = stm32_data.getElementEasier<double>("L");
    cover_state.data = stm32_data.getElementEasier<bool>("cover_state");
    is_right_clog = stm32_data.getElementEasier<bool>("is_right_clog");
    is_left_clog = stm32_data.getElementEasier<bool>("is_left_clog");
  }catch (std::runtime_error& e) {
    ROS_WARN("Error in parsing: %s", e.what());
    return;
  }
  std_msgs::Int32 R,L;
  R.data = static_cast<int32_t>(total_right);
  L.data = static_cast<int32_t>(total_left);
  rw_pub.publish(R);
  lw_pub.publish(L);
  std_msgs::UInt8 clog_msg;
  clog_msg.data = 'N';
  if(is_left_clog)clog_msg.data = 'L';
  if(is_right_clog)clog_msg.data = 'R';
  if(is_left_clog && is_right_clog)clog_msg.data = 'A';
  clog_pub.publish(clog_msg);
  cover_pub.publish(cover_state);
}

int main(int argc, char* argv[]) {
  // ROS init
  ros::init(argc, argv, "stm32_node");
  ros::NodeHandle node_handle;

  rw_pub = node_handle.advertise<std_msgs::Int32>("/rwheel_ticks", 2);
  lw_pub = node_handle.advertise<std_msgs::Int32>("/lwheel_ticks", 2);
  cover_pub = node_handle.advertise<std_msgs::UInt8>("/cover_state", 2);
  clog_pub = node_handle.advertise<std_msgs::UInt8>("clogging_state", 2);
  ros::Subscriber vel_sub = node_handle.subscribe("/cmd_vel", 2, updateMotorAction);
  ros::Subscriber cover_sub = node_handle.subscribe("/cover_cmd", 2, updateCoverAction);
  ros::Rate rate(30);
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
