#ifndef __DIRECT_DRIVE_HPP__
#define __DIRECT_DRIVE_HPP__

#include "driftless/hal/MotorGroup.hpp"
#include "driftless/robot/subsystems/drivetrain/IDriveTrain.hpp"

namespace driftless {
namespace robot {
namespace subsystems {
namespace drivetrain {
class directDrive : public IDrivetrain {
 private:
  hal::MotorGroup m_left_motors{};
  hal::MotorGroup m_right_motors{};

  double m_velocity_to_voltage{1.0};

  double m_gear_ratio{};

  double m_wheel_radius{};

  double m_drive_radius{};

 public:
  void init() override;

  void run() override;

  void setVelocity(Velocity velocity) override;

  void setVoltage(double left_voltage, double right_voltage) override;

  void setLeftMotors(hal::MotorGroup& left_motors);

  void setRightMotors(hal::MotorGroup& right_motors);

  void setVelocityToVoltage(double velocity_to_voltage);

  void setGearRatio(double gear_ratio);

  void setWheelRadius(double wheel_radius);

  void setDriveRadius(double drive_radius);

  Velocity getVelocity() override;

  double getDriveRadius() override;
};
}  // namespace drivetrain
}  // namespace subsystems
}  // namespace robot
}  // namespace driftless
#endif