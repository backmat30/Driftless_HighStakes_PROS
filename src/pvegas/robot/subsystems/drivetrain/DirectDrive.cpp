#include "pvegas/robot/subsystems/drivetrain/DirectDrive.hpp"

namespace pvegas {
namespace robot {
namespace subsystems {
namespace drivetrain {
void directDrive::init() {
  m_left_motors.init();
  m_right_motors.init();
}

void directDrive::run() {}

void directDrive::setVelocity(Velocity velocity) {
  m_left_motors.setVoltage(velocity.left_velocity * m_velocity_to_voltage);
  m_right_motors.setVoltage(velocity.right_velocity * m_velocity_to_voltage);
}

void directDrive::setVoltage(double left_voltage, double right_voltage) {
  m_left_motors.setVoltage(left_voltage);
  m_right_motors.setVoltage(right_voltage);
}

void directDrive::setLeftMotors(hal::MotorGroup& left_motors) {
  m_left_motors = left_motors;
}

void directDrive::setRightMotors(hal::MotorGroup& right_motors) {
  m_right_motors = right_motors;
}

void directDrive::setVelocityToVoltage(double velocity_to_voltage) {
  m_velocity_to_voltage = velocity_to_voltage;
}

void directDrive::setGearRatio(double gear_ratio) { m_gear_ratio = gear_ratio; }

void directDrive::setWheelRadius(double wheel_radius) {
  m_wheel_radius = wheel_radius;
}

void directDrive::setDriveRadius(double drive_radius) {
  m_drive_radius = drive_radius;
}

Velocity directDrive::getVelocity() {
  Velocity velocity{
      m_left_motors.getAngularVelocity() * m_wheel_radius / m_gear_ratio,
      m_right_motors.getAngularVelocity() * m_wheel_radius / m_gear_ratio};
  return velocity;
}

double directDrive::getDriveRadius() { return m_drive_radius; }

}  // namespace drivetrain
}  // namespace subsystems
}  // namespace robot
}  // namespace pvegas