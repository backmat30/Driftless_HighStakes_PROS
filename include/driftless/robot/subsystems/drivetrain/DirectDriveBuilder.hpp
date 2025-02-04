#ifndef __DIRECT_DRIVE_BUILDER_HPP__
#define __DIRECT_DRIVE_BUILDER_HPP__

#include <memory>

#include "DirectDrive.hpp"
#include "driftless/hal/MotorGroup.hpp"
#include "driftless/io/IMotor.hpp"
#include "driftless/robot/subsystems/drivetrain/IDrivetrain.hpp"

/// @brief The namespace for driftless library code
/// @author Matthew Backman
namespace driftless {

/// @brief The namespace for robot code
/// @author Matthew Backman
namespace robot {

/// @brief The namespace for subsystems code
/// @author Matthew Backman
namespace subsystems {

/// @brief The namespace for the drivetrain subsystem code
/// @author Matthew Backman
namespace drivetrain {

/// @brief Builder class for creating DirectDrive objects
/// @author Matthew Backman
class DirectDriveBuilder {
 private:
  hal::MotorGroup m_left_motors{};

  hal::MotorGroup m_right_motors{};

  double m_velocity_to_voltage{1.0};

  double m_gear_ratio{};

  double m_wheel_radius{};

  double m_drive_radius{};

 public:
  /// @brief Adds a left motor to the builder
  /// @param motor __std::unique_ptr<io::IMotor>&__ The motor being added
  /// @return __DirectDriveBuilder*__ Pointer to the current builder
  DirectDriveBuilder* withLeftMotor(std::unique_ptr<io::IMotor>& motor);

  /// @brief Adds a right motor to the builder
  /// @param motor __std::unique_ptr<io::IMotor>&__ The motor being added
  /// @return __DirectDriveBuilder*__ Pointer to the current builder
  DirectDriveBuilder* withRightMotor(std::unique_ptr<io::IMotor>& motor);

  /// @brief Adds a velocity to voltage conversion to the builder
  /// @param velocity_to_voltage _double_ The conversion factor being added
  /// @return __DirectDriveBuilder*__ Pointer to the current builder
  DirectDriveBuilder* withVelocityToVoltage(double velocity_to_voltage);

  /// @brief Adds a wheel radius to the builder
  /// @param wheel_radius __double__ The wheel radius
  /// @return __DirectDriveBuilder*__ Pointer to the current builder
  DirectDriveBuilder* withWheelRadius(double wheel_radius);

  /// @brief Adds a drive radius to the builder
  /// @param drive_radius __double__ The drive radius
  /// @return __DirectDriveBuilder*__ Pointer to the current builder
  DirectDriveBuilder* withDriveRadius(double drive_radius);

  /// @brief Builds a new DirectDrive object
  /// @return __std::unique_ptr<IDrivetrain>__ Pointer to a new DirectDrive
  /// object
  std::unique_ptr<IDrivetrain> build();
};
}  // namespace drivetrain
}  // namespace subsystems
}  // namespace robot
}  // namespace driftless
#endif