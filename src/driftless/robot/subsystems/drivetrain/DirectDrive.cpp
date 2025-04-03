#include "driftless/robot/subsystems/drivetrain/DirectDrive.hpp"

namespace driftless {
namespace robot {
namespace subsystems {
namespace drivetrain {
void directDrive::init() {
  m_left_motors.init();
  m_right_motors.init();
}

void directDrive::run() {}

void directDrive::setVelocity(Velocity velocity) {
  double left_voltage{};
  double right_voltage{};

  if(velocity.left_velocity != 0)
    left_voltage = m_left_motor_feed_forward.getControlValue(velocity.left_velocity);
  if(velocity.right_velocity != 0)
    right_voltage = m_right_motor_feed_forward.getControlValue(velocity.right_velocity);

  m_left_motors.setVoltage(left_voltage);
  m_right_motors.setVoltage(right_voltage);
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

void directDrive::setLeftMotorFeedForward(control::FeedForward left_motor_feed_forward) {
  m_left_motor_feed_forward = left_motor_feed_forward;
}

void directDrive::setRightMotorFeedForward(control::FeedForward right_motor_feed_forward) {
  m_right_motor_feed_forward = right_motor_feed_forward;
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
}  // namespace driftless