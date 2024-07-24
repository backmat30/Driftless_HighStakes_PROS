#ifndef __DIRECT_DRIVE_BUILDER_HPP__
#define __DIRECT_DRIVE_BUILDER_HPP__

#include "DirectDrive.hpp"
#include "pvegas/hal/MotorGroup.hpp"
#include "pvegas/pros_adapters/ProsV5Motor.hpp"
#include "pvegas/robot/subsystems/drivetrain/IDrivetrain.hpp"
#include <memory>


namespace pvegas {
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
  DirectDriveBuilder* withLeftMotor(std::unique_ptr<pros_adapters::ProsV5Motor>& motor);

  DirectDriveBuilder* withRightMotor(std::unique_ptr<pros_adapters::ProsV5Motor>& motor);

  DirectDriveBuilder* withVelocityToVoltage(double velocity_to_voltage);

  DirectDriveBuilder* withWheelRadius(double wheel_radius);

  DirectDriveBuilder* withDriveRadius(double drive_radius);

  std::unique_ptr<IDrivetrain> build();
};
} // namespace drivetrain
} // namespace subsystems
} // namespace robot
} // namespace pvegas
#endif