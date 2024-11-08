#ifndef __DIRECT_DRIVE_BUILDER_HPP__
#define __DIRECT_DRIVE_BUILDER_HPP__

#include <memory>

#include "DirectDrive.hpp"
#include "driftless/hal/MotorGroup.hpp"
#include "driftless/io/IMotor.hpp"
#include "driftless/robot/subsystems/drivetrain/IDrivetrain.hpp"

namespace driftless {
namespace robot {
namespace subsystems {
namespace drivetrain {
class DirectDriveBuilder {
 private:
  hal::MotorGroup m_left_motors{};
  hal::MotorGroup m_right_motors{};

  double m_velocity_to_voltage{1.0};
  double m_gear_ratio{};
  double m_wheel_radius{};
  double m_drive_radius{};

 public:
  // Adds a designated motor to the left side of the robot
  DirectDriveBuilder* withLeftMotor(std::unique_ptr<io::IMotor>& motor);

  // Adds a designated motor to the right side of the robot
  DirectDriveBuilder* withRightMotor(std::unique_ptr<io::IMotor>& motor);

  // Designates the ratio between robot velocity and motor voltage
  DirectDriveBuilder* withVelocityToVoltage(double velocity_to_voltage);

  // Designates the radius of the drive wheels
  DirectDriveBuilder* withWheelRadius(double wheel_radius);

  // Designates the radius of the robot
  DirectDriveBuilder* withDriveRadius(double drive_radius);

  // final call in the chain, uses the newly defined data to construct a drive
  // train
  std::unique_ptr<IDrivetrain> build();
};
}  // namespace drivetrain
}  // namespace subsystems
}  // namespace robot
}  // namespace driftless
#endif