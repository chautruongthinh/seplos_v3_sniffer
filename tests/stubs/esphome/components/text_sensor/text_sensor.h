#pragma once
#include <string>
#include <utility>
namespace esphome { namespace text_sensor {
class TextSensor {
 public:
  explicit TextSensor(std::string name) : name_(std::move(name)) {}
  const std::string &get_name() const { return name_; }
  void publish_state(const std::string &value) { state = value; ++publications; }
  std::string state;
  unsigned publications{0};
 private:
  std::string name_;
};
}}
