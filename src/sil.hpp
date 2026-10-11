#ifndef RACEUP_SIL_HPP
#define RACEUP_SIL_HPP

#include "ecu_api.hpp"

#include <chrono>
#include <cstdint>
#include <memory>

namespace raceup::sil {

class SilSystem {
 public:
  SilSystem();
  SilSystem(const SilSystem&) = delete;
  SilSystem& operator=(const SilSystem&) = delete;
  ~SilSystem();

  bool start();
  void stop();
  bool running() const;

  void set_sensor(SensorId id, std::uint16_t value);
  std::uint16_t output(OutputId id) const;

  // Inject an external frame into the EBS input side of the simulated bus.
  void send_frame(const Frame& frame);

  // Read a frame emitted by either ECU. Returns false on timeout.
  bool receive_frame(Frame& frame,
                     std::chrono::milliseconds timeout =
                         std::chrono::milliseconds(100));

 private:
  // Platform callbacks used by given/ecu_api.hpp. They are intentionally
  // hidden from the test-facing API; the implementation in sil.cpp adapts
  // the global ECU callbacks to this simulator instance.
  friend std::uint16_t ecu_io::read_sensor(SensorId id);
  friend bool ecu_io::try_read(Ecu ecu, Frame& frame);
  friend void ecu_io::write(Ecu ecu, const Frame& frame);
  friend void ecu_io::write_output(Ecu ecu, OutputId id,
                                   std::uint16_t value);

  std::uint16_t read_sensor(SensorId id) const;
  bool try_read(Ecu ecu, Frame& frame);
  void write(Ecu ecu, const Frame& frame);
  void write_output(Ecu ecu, OutputId id, std::uint16_t value);

  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace raceup::sil

#endif  // RACEUP_SIL_HPP
