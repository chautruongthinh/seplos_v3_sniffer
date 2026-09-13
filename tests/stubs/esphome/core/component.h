#pragma once
#include <cstdint>
namespace esphome {
extern uint32_t test_millis;
extern uint32_t test_clock_step;
inline uint32_t millis() {
  const uint32_t now = test_millis;
  test_millis += test_clock_step;
  return now;
}
class Component {
 public:
  virtual ~Component() = default;
  virtual void setup() {}
  virtual void loop() {}
  virtual void dump_config() {}
};
}
