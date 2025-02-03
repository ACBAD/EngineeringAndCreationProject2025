#include <rapidjson/document.h>

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

int main(int argc, char* argv[]) {

  return 0;
}
