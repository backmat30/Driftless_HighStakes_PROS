#ifndef __DIRECT_INTAKE_HPP__
#define __DIRECT_INTAKE_HPP__

#include <memory>

#include "driftless/hal/MotorGroup.hpp"
#include "driftless/robot/subsystems/intake/IIntake.hpp"

/// @brief The namespace for driftless library code
/// @author Matthew Backman
namespace driftless {

/// @brief The namespace for robot code
/// @author Matthew Backman
namespace robot {

/// @brief The namespace for subsystems code
/// @author Matthew Backman
namespace subsystems {

/// @brief The namespace for the intake subsystem
/// @author Matthew Backman
namespace intake {

/// @brief Class for controlling the intake mechanism directly using motors
/// @author Matthew Backman
class DirectIntake : public IIntake {
 private:
  // group of motors used to run the intake
  driftless::hal::MotorGroup m_motors{};

 public:
  /// @brief Initializes the intake
  void init() override;

  /// @brief Runs the intake
  void run() override;

  /// @brief Sets the voltage of the intake motors
  /// @param voltage __double__ The voltage to set
  void setVoltage(double voltage) override;

  /// @brief Sets the motors
  /// @param motors __driftless::hal::MotorGroup&__ The motors to set
  void setMotors(driftless::hal::MotorGroup& motors);
};
}  // namespace intake
}  // namespace subsystems
}  // namespace robot
}  // namespace driftless
#endif