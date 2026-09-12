#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

namespace esphome {
namespace seplos_parser {

// Longest supported response is 57 bytes. Power-of-two indexing avoids
// shifting the retained bytes on every rejected header or CRC.
class FrameBuffer {
 public:
  static constexpr size_t CAPACITY = 64;
  size_t size() const { return size_; }
  uint8_t operator[](size_t index) const { return data_[(head_ + index) & (CAPACITY - 1)]; }
  void push_back(uint8_t value) {
    if (size_ == CAPACITY)
      discard(1);
    data_[(head_ + size_) & (CAPACITY - 1)] = value;
    ++size_;
  }
  void discard(size_t count) {
    if (count > size_)
      count = size_;
    head_ = (head_ + count) & (CAPACITY - 1);
    size_ -= count;
  }

 private:
  std::array<uint8_t, CAPACITY> data_{};
  size_t head_{0};
  size_t size_{0};
};

}  // namespace seplos_parser
}  // namespace esphome
