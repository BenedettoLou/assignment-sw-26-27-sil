#ifndef RACEUP_SIL_ECU_API_HPP
#define RACEUP_SIL_ECU_API_HPP

#include <array>
#include <cstddef>
#include <cstdint>

namespace raceup::sil {

enum class Ecu : std::uint8_t {
  Smu,
  Ebs,
};

enum class SensorId : std::uint8_t {
  SmuApps1Raw,
  SmuApps2Raw,
  SmuBrakeRaw,
  EbsValvesPowered,
  EbsAsmsActive,
  EbsWatchdogHealthy,
  Count,
};

enum class OutputId : std::uint8_t {
  SmuThrottlePercent,
  SmuBrakePercent,
  EbsValve1Engaged,
  EbsValve2Engaged,
  EbsAsNodeClosed,
  EbsFaultCode,
  Count,
};

struct Frame {
  std::uint32_t id = 0U;
  std::uint8_t dlc = 0U;
  std::array<std::uint8_t, 8U> data{};
};

namespace can_id {
inline constexpr std::uint32_t SmuSense = 0x120U;
inline constexpr std::uint32_t EbsStatus = 0x220U;
inline constexpr std::uint32_t VehicleStatus = 0x300U;
inline constexpr std::uint32_t BrakePressure = 0x301U;
inline constexpr std::uint32_t EbsCommand = 0x302U;
}  // namespace can_id

inline Frame make_frame(const std::uint32_t id, const std::uint8_t dlc) {
  Frame frame{};
  frame.id = id;
  frame.dlc = dlc;
  return frame;
}

namespace ecu_io {

// These functions are implemented by the POSIX SIL. The supplied ECU logic
// must remain independent of the simulator implementation.
std::uint16_t read_sensor(SensorId id);
bool try_read(Ecu ecu, Frame& frame);
void write(Ecu ecu, const Frame& frame);
void write_output(Ecu ecu, OutputId id, std::uint16_t value);

}  // namespace ecu_io

namespace smu {
void reset();
void run_cycle();
}  // namespace smu

namespace ebs {
void reset();
void run_cycle();
}  // namespace ebs

}  // namespace raceup::sil

#endif  // RACEUP_SIL_ECU_API_HPP
