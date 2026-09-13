#include "esphome/components/seplos_parser/seplos_parser.h"
#include <cassert>
#include <cmath>
#include <iostream>
#include <limits>
#include <random>
#include <vector>

namespace esphome {
uint32_t test_millis = 0;
uint32_t test_clock_step = 0;
}
using esphome::seplos_parser::SeplosParser;
using esphome::seplos_parser::FrameBuffer;
using esphome::test_millis;
using esphome::test_clock_step;

// Independent bitwise oracle (production uses a lookup table).
uint16_t crc(const std::vector<uint8_t> &bytes) {
  uint16_t result = 0xffff;
  for (uint8_t byte : bytes) {
    result ^= byte;
    for (int bit = 0; bit < 8; ++bit)
      result = (result >> 1) ^ ((result & 1) ? 0xa001 : 0);
  }
  return result;
}
void seal(std::vector<uint8_t> &frame) {
  const uint16_t checksum = crc(frame);
  frame.push_back(checksum & 0xff);
  frame.push_back(checksum >> 8);
}
void word(std::vector<uint8_t> &frame, size_t offset, uint16_t value) {
  frame[offset] = value >> 8;
  frame[offset + 1] = value & 0xff;
}
std::vector<uint8_t> packet(uint8_t count, uint8_t address = 1) {
  std::vector<uint8_t> frame(size_t(count) + 3, 0);
  frame[0] = address;
  frame[1] = count == 0x12 ? 1 : 4;
  frame[2] = count;
  if (count == 0x24) {
    word(frame, 3, 5123);
    word(frame, 5, uint16_t(int16_t(-1234)));
    word(frame, 13, 765);
  } else if (count == 0x34) {
    for (int i = 0; i < 16; ++i) word(frame, 3 + i * 2, 3200 + i);
    word(frame, 35, 2982);
    word(frame, 51, 3000);
    word(frame, 53, 3100);
  } else {
    frame[9] = 1;
    frame[10] = 0x80;
    frame[11] = 2;
    frame[18] = 3;
  }
  seal(frame);
  return frame;
}
struct Fixture {
  SeplosParser parser;
  esphome::sensor::Sensor voltage{"bms0 pack_voltage"}, current{"bms0 current"},
      cell{"bms0 cell_16"}, voltage1{"bms1 pack_voltage"}, power{"bms0 power"};
  esphome::text_sensor::TextSensor status{"bms0 system_status"},
      balance{"bms0 active_balancing_cells"};
  explicit Fixture(int interval = 0) {
    test_millis = 0;
    test_clock_step = 0;
    parser.set_bms_count(2);
    parser.set_update_interval(interval);
    for (auto *s : {&voltage, &current, &cell, &voltage1, &power}) parser.register_sensor(s);
    parser.register_text_sensor(&status);
    parser.register_text_sensor(&balance);
    parser.setup();
  }
  void send(const std::vector<uint8_t> &frame) { parser.feed(frame); parser.loop(); }
};
void test_ring_crc() {
  FrameBuffer ring;
  for (int i = 0; i < 64; ++i) ring.push_back(i);
  ring.discard(61);
  for (int i = 64; i < 125; ++i) ring.push_back(i);
  for (int i = 0; i < 64; ++i) assert(ring[i] == 61 + i);
  ring.push_back(125);
  assert(ring.size() == 64 && ring[0] == 62);
  ring.discard(999);
  assert(ring.size() == 0);
  SeplosParser parser;
  std::vector<uint8_t> known{'1','2','3','4','5','6','7','8','9'};
  for (auto byte : known) ring.push_back(byte);
  assert(crc(known) == 0x4b37);
  assert(parser.calculate_modbus_crc(ring, ring.size()) == 0x4b37);
  assert(!parser.is_valid_header() && parser.get_expected_length() == 0);
  assert(!parser.validate_crc(0) && !parser.validate_crc(999));
  parser.process_packet(0);
}
void test_decode_and_burst() {
  Fixture f;
  f.parser.feed(packet(0x24));
  f.parser.feed(packet(0x34));
  f.parser.feed(packet(0x12));
  f.parser.feed(packet(0x24, 2));
  f.parser.loop();
  assert(f.parser.available() == 0);
  assert(f.voltage.publications == 1 && f.cell.publications == 1);
  assert(f.status.publications == 1 && f.voltage1.publications == 1);
  assert(std::fabs(f.voltage.state - 51.23f) < .001f);
  assert(std::fabs(f.current.state + 12.34f) < .001f);
  assert(std::fabs(f.power.state - 51.23f * -12.34f) < .01f);
  assert(std::fabs(f.cell.state - 3.215f) < .0001f);
  assert(f.status.state == "Charge" && f.balance.state == "1, 16");
}
void test_fragmentation_and_noise() {
  for (uint8_t count : {0x24, 0x34, 0x12}) {
    const auto bytes = packet(count);
    for (size_t split = 0; split <= bytes.size(); ++split) {
      Fixture f;
      f.send({bytes.begin(), bytes.begin() + split});
      test_millis = 20;
      f.send({bytes.begin() + split, bytes.end()});
      assert(f.voltage.publications + f.cell.publications + f.status.publications == 1);
    }
  }
  Fixture f;
  auto bad = packet(0x24); bad[8] ^= 0x80;
  f.send(bad);
  assert(f.voltage.publications == 0);
  f.send({0xff, 0, 0x11, 4, 0x24, 1, 3, 0x24});
  f.send(packet(0x24));
  assert(f.voltage.publications == 1);
  // False long-frame header before a complete shorter frame: idle recovery
  // must rescue the valid suffix rather than clear the entire buffer.
  f.send({1, 4, 0x34});
  f.send(packet(0x24));
  test_millis += 100;
  f.parser.loop();
  assert(f.voltage.publications == 2);
  f.send({1, 4, 0x34, 0});
  test_millis += 100;
  f.parser.loop();
  f.send(packet(0x24));
  assert(f.voltage.publications == 3);
}
void test_intervals_and_rollover() {
  Fixture f(3);
  for (uint8_t count : {0x24, 0x34, 0x12}) f.send(packet(count));
  f.send(packet(0x24, 2));
  assert(f.voltage.publications == 1 && f.cell.publications == 1 && f.status.publications == 1);
  assert(f.voltage1.publications == 1);
  test_millis = 2999;
  f.parser.dump_config(); // Must not reset publication timers.
  for (uint8_t count : {0x24, 0x34, 0x12}) f.send(packet(count));
  assert(f.voltage.publications == 1 && f.cell.publications == 1 && f.status.publications == 1);
  test_millis = 3000;
  for (uint8_t count : {0x24, 0x34, 0x12}) f.send(packet(count));
  assert(f.voltage.publications == 2 && f.cell.publications == 2 && f.status.publications == 2);
  Fixture wrap(1);
  test_millis = std::numeric_limits<uint32_t>::max() - 499;
  wrap.send(packet(0x24));
  test_millis = 499;
  wrap.send(packet(0x24));
  assert(wrap.voltage.publications == 1);
  test_millis = 500;
  wrap.send(packet(0x24));
  assert(wrap.voltage.publications == 2);
}
void test_limits_and_read_failure() {
  Fixture f;
  f.send(packet(0x24, 16)); // Valid bus address, not configured here.
  assert(f.voltage.publications == 0);
  f.parser.feed(packet(0x24));
  f.parser.fail_read = true;
  f.parser.loop();
  assert(f.parser.available() == 41);
  f.parser.fail_read = false;
  f.parser.loop();
  assert(f.voltage.publications == 1);
  for (int i = 0; i < 20; ++i) f.parser.feed(packet(0x24));
  const int queued = f.parser.available();
  f.parser.loop();
  assert(queued - f.parser.available() == 256);
  while (f.parser.available()) f.parser.loop();
  assert(f.voltage.publications == 21);
  f.parser.feed(packet(0x24));
  test_clock_step = 1;
  f.parser.loop();
  assert(f.parser.available() > 0); // Time budget also yields.
  test_clock_step = 0;
  while (f.parser.available()) f.parser.loop();
  assert(f.voltage.publications == 22);
}
void test_random_streams() {
  Fixture f;
  std::mt19937 random(42);
  for (int i = 0; i < 10000; ++i) {
    std::vector<uint8_t> noise(random() % 150);
    for (auto &byte : noise) byte = random();
    f.send(noise);
    test_millis += 100;
    f.parser.loop();
    const unsigned before = f.voltage.publications;
    f.send(packet(0x24));
    assert(f.voltage.publications == before + 1);
  }
  // Long repeat stream exercises ring wrap without idle gaps.
  const unsigned before = f.voltage.publications;
  for (int i = 0; i < 100000; ++i) f.send(packet(0x24));
  assert(f.voltage.publications == before + 100000);
}
int main() {
  test_ring_crc();
  test_decode_and_burst();
  test_fragmentation_and_noise();
  test_intervals_and_rollover();
  test_limits_and_read_failure();
  test_random_streams();
  std::cout << "PASS: parser decode, CRC, splits, bursts, noise, recovery, scheduling, rollover, budgets\n";
}
