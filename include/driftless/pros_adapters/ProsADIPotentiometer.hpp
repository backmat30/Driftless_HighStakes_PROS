#ifndef __PROS_ADI_POTENTIOMETER_HPP__
#define __PROS_ADI_POTENTIOMETER_HPP__

#include <cmath>
#include <memory>

#include "driftless/io/IPotentiometer.hpp"
#include "pros/adi.hpp"
#include "pros/rtos.hpp"

namespace driftless {
namespace pros_adapters {
class ProsADIPotentiometer : public driftless::io::IPotentiometer {
 private:
  // converts decidegrees to radians
  static constexpr double DECIDEGREES_TO_RADIANS{M_PI / 1800.0};

  // max value output by the potentiometer
  static constexpr double MAX_VALUE{4095.0 * M_PI / 1800.0};

  // the potentiometer being adapted
  std::unique_ptr<pros::adi::AnalogIn> m_potentiometer{};

  // whether the potentiometer is reversed or not
  bool m_reversed{};

  // the position offset
  double position_offset{};

 public:
  // construct a new pros potentiometer adapter
  ProsADIPotentiometer(std::unique_ptr<pros::adi::AnalogIn>& potentiometer,
                       bool reversed);

  // initialize the potentiometer
  void init() override;

  // calibrates the potentiometer
  void calibrate() override;

  // gets the angle from the potentiometer
  double getAngle() override;
};
}  // namespace pros_adapters
}  // namespace driftless
#endif