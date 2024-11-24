#include <rapidjson/document.h>
#include <iostream>
#include <rapidjson/stringbuffer.h>

template <typename T>
class JsonConversion;
template <>
class JsonConversion <int64_t> {
  uint64_t value = 0;
public:
  JsonConversion() = default;
  operator uint64_t() const {return value;}
  bool convert(const rapidjson::Document& d, const char* name) {
    if (!d.HasMember(name) || !d[name].IsInt64())
      return false;
    value = d[name].GetInt64();
    return true;
  }
};
template <>
class JsonConversion <double> {
  double value = 0;
public:
  JsonConversion() = default;
  operator double() const {return value;}
  bool convert(const rapidjson::Document& d, const char* name) {
    if (!d.HasMember(name) || !d[name].IsDouble())
      return false;
    value = d[name].GetDouble();
    return true;
  }
};
template <>
class JsonConversion <std::string> {
  std::string value;
public:
  JsonConversion() = default;
  operator std::string() {return value;}
  bool convert(const rapidjson::Document& d, const char* name) {
    if (!d.HasMember(name) || !d[name].IsString())
      return false;
    value = std::move(d[name].GetString());
    return true;
  }
};

rapidjson::Document ret_test(const char* inp) {
  rapidjson::Document temp;
  temp.SetObject();
  rapidjson::Value tinp;
  tinp.SetString(inp, strlen(inp), temp.GetAllocator());
  temp.AddMember("test", tinp, temp.GetAllocator());
  return temp;
}

int main(int argc, char* argv[]) {
  char yuuka[100];
  std::cin>>yuuka;
  rapidjson::Document buf;
  buf.SetObject();
  buf.Parse(yuuka);
  JsonConversion<std::string> jc;
  if(!jc.convert(buf, "yuuka"))
    std::cout<<"error"<<std::endl;
  else
    std::cout<<static_cast<std::string>(jc)<<std::endl;
  return 0;
}

