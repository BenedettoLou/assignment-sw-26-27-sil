#include "ecu_api.hpp"

#include <algorithm>
#include <cstdint>

namespace raceup::sil::smu {
namespace {

constexpr std::uint16_t kApps1Zero = 4000U;
constexpr std::uint16_t kApps1Full = 3000U;
constexpr std::uint16_t kApps2Zero = 3500U;
constexpr std::uint16_t kApps2Full = 2500U;
constexpr std::uint16_t kBrakeZero = 4000U;
constexpr std::uint16_t kBrakeFull = 1000U;
constexpr std::uint16_t kAppsShortThresholdRaw = 50U;
constexpr std::uint16_t kAppsPlausibilityThresholdPercent = 10U;

enum class Fault : std::uint8_t {
  None = 0U,
  AppsTracksShorted = 1U,
  AppsImplausible = 2U,
};

std::uint16_t normalize_inverted(const std::uint16_t raw,
                                 const std::uint16_t zero,
                                 const std::uint16_t full) {
  if (zero == full) {
    return 0U;
  }
  if (raw >= zero) {
    return 0U;
  }
  if (raw <= full) {
    return 100U;
  }

  const auto numerator = static_cast<std::uint32_t>(zero - raw) * 100U;
  const auto denominator = static_cast<std::uint32_t>(zero - full);
  return static_cast<std::uint16_t>(numerator / denominator);
}

std::uint16_t absolute_difference(const std::uint16_t first,
                                  const std::uint16_t second) {
  return first > second ? static_cast<std::uint16_t>(first - second)
                        : static_cast<std::uint16_t>(second - first);
}

}  // namespace

void reset() {}

void run_cycle() {
  const std::uint16_t apps1_raw =
      ecu_io::read_sensor(SensorId::SmuApps1Raw);
  const std::uint16_t apps2_raw =
      ecu_io::read_sensor(SensorId::SmuApps2Raw);
  const std::uint16_t brake_raw =
      ecu_io::read_sensor(SensorId::SmuBrakeRaw);

  const std::uint16_t apps1 =
      normalize_inverted(apps1_raw, kApps1Zero, kApps1Full);
  const std::uint16_t apps2 =
      normalize_inverted(apps2_raw, kApps2Zero, kApps2Full);
  const std::uint16_t brake =
      normalize_inverted(brake_raw, kBrakeZero, kBrakeFull);

  Fault fault = Fault::None;
  if (absolute_difference(apps1_raw, apps2_raw) < kAppsShortThresholdRaw) {
    fault = Fault::AppsTracksShorted;
  } else if (absolute_difference(apps1, apps2) >
             kAppsPlausibilityThresholdPercent) {
    fault = Fault::AppsImplausible;
  }

  const std::uint16_t throttle = fault == Fault::None ? apps1 : 0U;

  ecu_io::write_output(Ecu::Smu, OutputId::SmuThrottlePercent, throttle);
  ecu_io::write_output(Ecu::Smu, OutputId::SmuBrakePercent, brake);

  Frame frame = make_frame(can_id::SmuSense, 3U);
  frame.data[0] = static_cast<std::uint8_t>(throttle);
  frame.data[1] = static_cast<std::uint8_t>(brake);
  frame.data[2] = static_cast<std::uint8_t>(fault);
  ecu_io::write(Ecu::Smu, frame);
}

}  // namespace raceup::sil::smu
