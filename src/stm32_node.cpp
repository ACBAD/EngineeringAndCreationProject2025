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
geometry_msgs::Twist global_twist;
std_msgs::UInt8 global_cover;
int32_t total_right = 0;
int32_t total_left = 0;

class EasyDocument{
  rapidjson::Document d;
public:
  EasyDocument() = delete;
  explicit EasyDocument(rapidjson::Document&& other): d(std::move(other)) {}
  rapidjson::Document extractDocument(){
    rapidjson::Document temp;
    temp.Swap(d);
    return temp;
  }
  template <typename T>
  auto getElementEasier(const char* key) const {

    if(d.IsNull())
      throw std::runtime_error("has parse error, or this is a null value");
    if(d.HasParseError())
      throw std::runtime_error("has parse error");

    const auto &dk = d[key];
    if(!d.HasMember(key)){
      char _[100];
      std::sprintf(_, "%s not exist", key);
      throw std::runtime_error(_);
    }

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

void updateMotorAction(const geometry_msgs::Twist& msg) {
  global_twist = msg;
}

void updateCoverAction(const std_msgs::UInt8& msg) {
  global_cover = msg;
}

void sendAllArgs(const SerialDevice& sd) {
  const int x = std::min(static_cast<int>(global_twist.linear.x/MAX_LINEAR_SPEED*100), 100);
  const int r = std::min(static_cast<int>(global_twist.angular.z/MAX_ANG_SPEED*100), 100);
  rapidjson::Document cmd_obj;
  cmd_obj.SetObject();
  cmd_obj.AddMember("X", x, cmd_obj.GetAllocator());
  cmd_obj.AddMember("R", r, cmd_obj.GetAllocator());
  cmd_obj.AddMember("SC", global_cover.data, cmd_obj.GetAllocator());
  const ssize_t write_count = sd.send(cmd_obj);
  if(write_count < 0)
    return;
  std_msgs::UInt8 cover_cmd;
  rapidjson::Document stm32_data = std::move(sd.tread(200));
  // EasyDocument stm32_data(std::move(sd.tread(200)));
  // try {
  //   total_right += stm32_data.getElementEasier<int64_t>("R");
  //   total_left -= stm32_data.getElementEasier<int64_t>("L");
  //   cover_cmd.data = stm32_data.getElementEasier<bool>("cover_state");
  // }catch (std::runtime_error& e) {
  //   ROS_WARN("Error in parsing: %s", e.what());
  // }
  if(!stm32_data.IsNull()) {
    ROS_DEBUG("%lu", stm32_data["cover_cmd"].GetUint64());
  }
  std_msgs::Int32 R,L;
  R.data = total_right;
  L.data = total_left;
  rw_pub.publish(R);
  lw_pub.publish(L);
  cover_pub.publish(cover_cmd);
}

int main(int argc, char* argv[]) {
  // ROS init
  ros::init(argc, argv, "stm32_node");
  ros::NodeHandle node_handle;

  rw_pub = node_handle.advertise<std_msgs::Int32>("/rwheel_ticks", 2);
  lw_pub = node_handle.advertise<std_msgs::Int32>("/lwheel_ticks", 2);
  cover_pub = node_handle.advertise<std_msgs::UInt8>("/cover_state", 2);
  ros::Subscriber vel_sub = node_handle.subscribe("/cmd_vel", 2, updateMotorAction);
  ros::Subscriber cover_sub = node_handle.subscribe("/cover_cmd", 2, updateCoverAction);
  ros::Rate rate(20);
  ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug);
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
