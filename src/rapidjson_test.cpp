#include <rapidjson/document.h>
#include <iostream>
#include <rapidjson/stringbuffer.h>

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
  const rapidjson::Document buf = std::move(ret_test(yuuka));
  std::cout<<buf["test"].GetString()<<std::endl;
  return 0;
}

