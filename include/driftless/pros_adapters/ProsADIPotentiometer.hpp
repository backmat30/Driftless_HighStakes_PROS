#ifndef __PROS_ADI_POTENTIOMETER_HPP__
#define __PROS_ADI_POTENTIOMETER_HPP__

#include <cmath>
#include <memory>

#include "driftless/io/IPotentiometer.hpp"
#include "pros/adi.hpp"
#include "pros/rtos.hpp"

/// @brief The namespace for driftless library code
/// @author Matthew Backman
namespace driftless {

/// @brief The namespace for PROS adapters
/// @author Matthew Backman
namespace pros_adapters {

/// @brief Adapter class for the PROS ADI potentiometer
/// @author Matthew Backman
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
  /// @brief Constructor
  /// @param potentiometer __std::unique_ptr<pros::adi::AnalogIn>&__ The
  /// potentiometer to adapt
  /// @param reversed __bool__ Whether the potentiometer is reversed
  ProsADIPotentiometer(std::unique_ptr<pros::adi::AnalogIn>& potentiometer,
                       bool reversed);

  /// @brief Initializes the potentiometer
  void init() override;

  /// @brief Calibrates the potentiometer
  void calibrate() override;

  /// @brief Gets the angle from the potentiometer
  /// @return __double__ The angle value
  double getAngle() override;
};
}  // namespace pros_adapters
}  // namespace driftless
#endif