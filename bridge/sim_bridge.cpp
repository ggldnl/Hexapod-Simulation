// sim_bridge.cpp: a thin C API around the real firmware core, so the Python
// simulation can run the actual C++ control stack in-process (via ctypes).
//
// It wires the same stuff main.cpp does (FrameParser -> router -> robot::Robot),
// but behind a SimHardware that records servo angles for PyBullet instead of
// driving real servos, and lets the sim inject the measured current/voltage.
//
// Build:  ./build.sh   ->   libhexapod_fw.so
// Host-only (SDK-free core); never flashed.

#include <cstdint>
#include <cstring>

#include "core/config.hpp"
#include "communication/protocol.hpp"
#include "communication/hardware.hpp"
#include "core/robot.hpp"
#include "core/router.hpp"

namespace {

// The "hardware" for the sim: capture the servo command, expose settable sensors.
struct SimHardware : hw::Interface {
  float servos[cfg::N_SERVOS] = {};
  bool powered = false;
  float voltage = 7.4f;
  float current = 0.0f;

  void write_servos(const float (&deg)[cfg::N_SERVOS]) override {
    std::memcpy(servos, deg, sizeof servos);
  }
  void read_servos(float (&deg)[cfg::N_SERVOS]) override {
    std::memcpy(deg, servos, sizeof servos);
  }
  void set_power(bool on) override { powered = on; }
  float read_voltage() override { return voltage; }
  float read_current() override { return current; }
  void set_led(uint8_t, uint8_t, uint8_t, uint8_t, float) override {}
  void update_leds() override {}
};

struct Firmware {
  SimHardware hw;
  robot::Robot robot{hw};
  proto::FrameParser parser;
};

} // namespace

extern "C" {

Firmware *fw_new() { return new Firmware(); }
void fw_free(Firmware *f) { delete f; }

// Advance the control loop by dt seconds (== robot.update()).
void fw_update(Firmware *f, float dt) { f->robot.update(dt); }

// Feed `n` wire bytes into the parser; append any reply frames to out[] (up to
// out_cap). Returns the number of reply bytes written.
int fw_feed(Firmware *f, const uint8_t *in, int n, uint8_t *out, int out_cap) {
  int written = 0;
  for (int i = 0; i < n; ++i) {
    if (!f->parser.feed(in[i]))
      continue;
    uint8_t payload[cfg::MAX_PAYLOAD];
    uint8_t reply_op = 0;
    const uint8_t m = router::dispatch(f->robot, f->parser.opcode(),
                                       f->parser.payload(), f->parser.length(),
                                       payload, reply_op);
    if (m > 0) {
      uint8_t frame[cfg::MAX_PAYLOAD + 5];
      const size_t k = proto::encode_frame(reply_op, payload, m, frame);
      for (size_t j = 0; j < k && written < out_cap; ++j)
        out[written++] = frame[j];
    }
  }
  return written;
}

// Copy the latest 18 servo angles (servo-space degrees) into out18.
void fw_servos(Firmware *f, float *out18) {
  std::memcpy(out18, f->hw.servos, sizeof(f->hw.servos));
}

int fw_powered(Firmware *f) { return f->hw.powered ? 1 : 0; }
void fw_set_current(Firmware *f, float a) { f->hw.current = a; }
void fw_set_voltage(Firmware *f, float v) { f->hw.voltage = v; }
int fw_num_servos() { return cfg::N_SERVOS; }

} // extern "C"
