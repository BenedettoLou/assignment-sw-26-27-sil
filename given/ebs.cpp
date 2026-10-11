#include "ecu_api.hpp"

#include <cstdint>

namespace raceup::sil::ebs {
namespace {

constexpr std::uint16_t kStaleCycles = 25U;
constexpr std::uint16_t kMinPneumaticBar = 7U;
constexpr std::uint16_t kMaxPneumaticBar = 10U;
constexpr std::uint16_t kMinHydraulicBar = 15U;
constexpr std::uint16_t kMaxHydraulicBar = 99U;
constexpr std::uint16_t kCoherencyFactor = 18U;
constexpr std::uint16_t kSpeedStationaryDecimetersPerSecond = 5U;

enum class Fault : std::uint8_t {
  None = 0U,
  Communication = 1U,
  AsmsInactive = 2U,
  PneumaticPressure = 3U,
  BrakePressure = 4U,
  Watchdog = 5U,
  BrakeRequest = 6U,
};

struct State {
  bool manual_mission = false;
  bool ready_to_drive = false;
  std::uint16_t speed = 0U;
  std::uint16_t front_brake = 0U;
  std::uint16_t rear_brake = 0U;
  std::uint16_t pneumatic_left = 0U;
  std::uint16_t pneumatic_right = 0U;
  bool brake_request = false;
  bool mission_finished = false;
  std::uint16_t vehicle_age = kStaleCycles;
  std::uint16_t pressure_age = kStaleCycles;
  std::uint16_t command_age = kStaleCycles;
};

State state{};

bool in_range(const std::uint16_t value, const std::uint16_t minimum,
              const std::uint16_t maximum) {
  return value > minimum && value < maximum;
}

bool pressure_coherent(const std::uint16_t hydraulic,
                       const std::uint16_t pneumatic) {
  return static_cast<std::uint32_t>(hydraulic) * 10U >
         static_cast<std::uint32_t>(pneumatic) * kCoherencyFactor;
}

void update_age(std::uint16_t& age) {
  if (age < kStaleCycles) {
    ++age;
  }
}

void read_inputs() {
  Frame frame{};
  while (ecu_io::try_read(Ecu::Ebs, frame)) {
    if (frame.id == can_id::VehicleStatus && frame.dlc >= 3U) {
      state.manual_mission = frame.data[0] != 0U;
      state.ready_to_drive = frame.data[1] != 0U;
      state.speed = frame.data[2];
      state.vehicle_age = 0U;
    } else if (frame.id == can_id::BrakePressure && frame.dlc >= 4U) {
      state.front_brake = frame.data[0];
      state.rear_brake = frame.data[1];
      state.pneumatic_left = frame.data[2];
      state.pneumatic_right = frame.data[3];
      state.pressure_age = 0U;
    } else if (frame.id == can_id::EbsCommand && frame.dlc >= 2U) {
      state.brake_request = frame.data[0] != 0U;
      state.mission_finished = frame.data[1] != 0U;
      state.command_age = 0U;
    }
  }
}

}  // namespace

void reset() { state = State{}; }

void run_cycle() {
  read_inputs();

  const bool valves_powered =
      ecu_io::read_sensor(SensorId::EbsValvesPowered) != 0U;
  const bool asms_active =
      ecu_io::read_sensor(SensorId::EbsAsmsActive) != 0U;
  const bool watchdog_healthy =
      ecu_io::read_sensor(SensorId::EbsWatchdogHealthy) != 0U;

  Fault fault = Fault::None;
  if (state.vehicle_age >= kStaleCycles || state.pressure_age >= kStaleCycles ||
      state.command_age >= kStaleCycles) {
    fault = Fault::Communication;
  } else if (!asms_active) {
    fault = Fault::AsmsInactive;
  } else if (!in_range(state.pneumatic_left, kMinPneumaticBar,
                       kMaxPneumaticBar) ||
             !in_range(state.pneumatic_right, kMinPneumaticBar,
                       kMaxPneumaticBar)) {
    fault = Fault::PneumaticPressure;
  } else if (!in_range(state.front_brake, kMinHydraulicBar,
                       kMaxHydraulicBar) ||
             !in_range(state.rear_brake, kMinHydraulicBar,
                       kMaxHydraulicBar) ||
             !pressure_coherent(state.front_brake, state.pneumatic_left) ||
             !pressure_coherent(state.rear_brake, state.pneumatic_right)) {
    fault = Fault::BrakePressure;
  } else if (!watchdog_healthy || !valves_powered) {
    fault = Fault::Watchdog;
  } else if (state.brake_request || state.mission_finished) {
    fault = Fault::BrakeRequest;
  } else if (!state.ready_to_drive) {
    // Manual mission does not bypass the ready-to-drive requirement.
    fault = Fault::Communication;
  }

  const bool safe_to_close = fault == Fault::None &&
                             state.speed <= kSpeedStationaryDecimetersPerSecond;
  const bool as_node_closed = safe_to_close;
  const bool brakes_engaged = !safe_to_close;

  ecu_io::write_output(Ecu::Ebs, OutputId::EbsValve1Engaged,
                       brakes_engaged ? 1U : 0U);
  ecu_io::write_output(Ecu::Ebs, OutputId::EbsValve2Engaged,
                       brakes_engaged ? 1U : 0U);
  ecu_io::write_output(Ecu::Ebs, OutputId::EbsAsNodeClosed,
                       as_node_closed ? 1U : 0U);
  ecu_io::write_output(Ecu::Ebs, OutputId::EbsFaultCode,
                       static_cast<std::uint16_t>(fault));

  Frame status = make_frame(can_id::EbsStatus, 5U);
  status.data[0] = static_cast<std::uint8_t>(as_node_closed);
  status.data[1] = static_cast<std::uint8_t>(brakes_engaged);
  status.data[2] = static_cast<std::uint8_t>(brakes_engaged);
  status.data[3] = static_cast<std::uint8_t>(fault);
  status.data[4] = static_cast<std::uint8_t>(state.speed);
  ecu_io::write(Ecu::Ebs, status);

  update_age(state.vehicle_age);
  update_age(state.pressure_age);
  update_age(state.command_age);
}

}  // namespace raceup::sil::ebs
