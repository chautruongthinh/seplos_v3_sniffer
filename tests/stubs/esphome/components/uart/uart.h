#pragma once
#include <cstdint>
#include <deque>
#include <vector>
namespace esphome { namespace uart {
class UARTDevice {
 public:
  std::deque<uint8_t> input;
  bool fail_read{false};
  int available() { return input.size(); }
  bool read_byte(uint8_t *value) {
    if (fail_read || input.empty()) return false;
    *value = input.front();
    input.pop_front();
    return true;
  }
  void feed(const std::vector<uint8_t> &bytes) {
    input.insert(input.end(), bytes.begin(), bytes.end());
  }
};
}}
