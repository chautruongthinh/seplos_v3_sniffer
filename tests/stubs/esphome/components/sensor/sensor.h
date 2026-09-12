#pragma once
#include <string>
#include <utility>
namespace esphome { namespace sensor {
class Sensor {
 public:
  explicit Sensor(std::string name) : name_(std::move(name)) {}
  const std::string &get_name() const { return name_; }
  void publish_state(float value) { state = value; ++publications; }
  float state{0};
  unsigned publications{0};
 private:
  std::string name_;
};
}}
