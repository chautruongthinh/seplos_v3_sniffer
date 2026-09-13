# seplos_v3_sniffer

[www.seplos.com](https://www.seplos.com/)

This package is designed to read the Seplos V3 BMS in a multipack configuration. In the V3 generation, the first BMS acts as a Modbus master while all other BMS act as slaves. In this configuration, it is no longer possible to access the BMS from a third device via Modbus, since two master devices cannot exist in an RS-485 Modbus system. The package passively detects the communication between the devices, which does not disrupt the communication of the individual BMS.

In the YAML, all required data must be inserted/modified.
Every 200 ms, the BMS transmits a new data set. The update interval can be changed (default: 5 seconds).

## ESP8266 stability and performance

The parser uses a fixed 64-byte receive ring (largest supported response: 57
bytes), with no receive-buffer allocation or byte shifting. Each loop reads up
to 256 bytes, checking a 5 ms budget between reads so ESPHome can service other
components. This is cooperative, not a hard deadline: publishing callbacks can
take longer. CRC failures resynchronize one byte at a time; a 100 ms stalled
stream timeout recovers incomplete frames and any complete valid suffix.

`update_interval` is an integer in **seconds**, independently applied to each
pack and each of the three response groups (pack, cells, status). The first valid
response publishes immediately. Numeric decoding and publishing are now skipped
until that group's interval expires, rather than running sensor filters for
every response. `0` publishes every valid response and increases CPU/network
load; it cannot make the BMS transmit faster. The example retains 3 seconds.
`bms_count` must be 1–16. Names must still follow `bms0 pack_voltage`,
`bms1 cell_1`, etc.; changing the `battery_bank` prefixes breaks name binding.

The existing YAML is preserved byte-for-byte: board, pins, logging, UART buffers,
sensor names and filters are not changed by this PR. Existing throttle filters
can stay in place; parser-side throttling avoids decoding intermediate numeric
responses before those filters run. No wiring change is required by the parser.

### Validation before flashing

Run native regression tests on Linux with g++ and Python/PyYAML:

```sh
bash tests/run.sh
python tests/check_yaml.py
```

These compile the actual parser against small ESPHome test doubles, with
AddressSanitizer and UndefinedBehaviorSanitizer. Coverage includes CRC, all split
positions, three response types, multiple packs, noise, incomplete frames,
100,000 repeated valid frames, independent update intervals, clock rollover,
failed reads, and loop work budgets. The stubs do **not** test ESPHome codegen,
Wi-Fi, interrupts, flash/RAM size, or electrical behavior. In a container where
LeakSanitizer cannot inspect processes, use
`ASAN_OPTIONS=detect_leaks=0 bash tests/run.sh`; this disables leak detection only.

To compile a checked-out branch, replace the example's `external_components`
block with the following so the build uses the checked-out code, not remote main:

```yaml
external_components:
  - source:
      type: local
      path: esphome/components
```

Provide your own `secrets.yaml`, then run:

```sh
esphome config seplos_v3_sniffer.yaml
esphome compile seplos_v3_sniffer.yaml
```

Native tests are not an ESP8266 firmware build or a hardware soak test. Before
using this branch unattended, record the ESPHome version and firmware RAM/flash
usage, compare sensor values with the BMS, and run for 24–48 hours with all packs
and Home Assistant connected. Check for resets, growing heap usage, stale cells,
and recovery after Wi-Fi/API interruption. Retain the previous firmware for
rollback. No claim of maximum speed or long-term hardware stability is made
without measurements on the actual board.

![seplos 4x](https://github.com/user-attachments/assets/9d710287-069d-44b6-acda-e96764642a33)

To establish a connection, pins 1/8 (B), 2/7 (A) and 5 (GND) must be connected to the RS485 adapter of the ESP8622/ESP32. Various RS485 to TTL adapters can be used.

During my tests, I found that the 120 Ohm terminator in the adapter is not necessary. There is also no terminator in the original Seplos V3 USB adapter. If only one BMS is to be read, it is necessary to connect pin 6 (B) to pin 5 (GND) so that the master can send data independently.

![pinout](https://github.com/user-attachments/assets/1c8ec271-d20f-4a5d-baf4-87e5a98fc35a)

The following data points are currently read out:
```
pack_voltage
current
remaining_capacity
total_capacity
total_discharge_capacity
soc
soh
cycle_count
average_cell_voltage
average_cell_temp
max_cell_voltage
min_cell_voltage
delta_cell_voltage
max_cell_temp
min_cell_temp
maxdiscurt
maxchgcurt
cell_1_voltage
cell_2_voltage
cell_3_voltage
cell_4_voltage
cell_5_voltage
cell_6_voltage
cell_7_voltage
cell_8_voltage
cell_9_voltage
cell_10_voltage
cell_11_voltage
cell_12_voltage
cell_13_voltage
cell_14_voltage
cell_15_voltage
cell_16_voltage
cell_temp_1
cell_temp_2
cell_temp_3
cell_temp_4
case_temp
power_temp
system_status
active_balancing_cells
cell_temperature_alarms
cell_voltage_alarms
FET_status
active_alarms
active_protections
```
