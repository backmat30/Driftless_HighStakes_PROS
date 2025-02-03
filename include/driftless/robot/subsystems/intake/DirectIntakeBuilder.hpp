#ifndef __DIRECT_INTAKE_BUILDER_HPP__
#define __DIRECT_INTAKE_BUILDER_HPP__

#include "driftless/robot/subsystems/intake/DirectIntake.hpp"

/// @brief The namespace for driftless library code
/// @author Matthew Backman
namespace driftless {

/// @brief The namespace for robot code
/// @author Matthew Backman
namespace robot {

/// @brief The namespace for subsystems code
/// @author Matthew Backman
namespace subsystems {

  /// @brief The namespace for intake subsystem code
/// @author Matthew Backman
namespace intake {

/// @brief Builder class for creating DirectIntake objects
/// @author Matthew Backman
class DirectIntakeBuilder {
 private:
  // the motors used for the intake
  driftless::hal::MotorGroup m_motors{};

 public:
  /// @brief Adds motors to the builder
  /// @param motor __std::unique_ptr<driftless::io::IMotor>&__ The motor to add
  /// @return __DirectIntakeBuilder*__ The builder instance
  DirectIntakeBuilder* withMotor(std::unique_ptr<driftless::io::IMotor>& motor);

  /// @brief Builds a new DirectIntake object
  /// @return __std::unique_ptr<DirectIntake>__ The built DirectIntake object
  std::unique_ptr<DirectIntake> build();
};
}  // namespace intake
}  // namespace subsystems
}  // namespace robot
}  // namespace driftless
#endif