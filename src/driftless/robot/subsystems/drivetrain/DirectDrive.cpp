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

void directDrive::startClimb() {
  is_climbing = true;
  m_climb_pistons.extend();

  m_left_motors.setPosition(0.0);
  m_right_motors.setPosition(0.0);
}

void directDrive::climb(double voltage) {
  if(is_climbing) {
    double left_voltage{voltage};
    double right_voltage{voltage};

    double left_position{m_left_motors.getPosition()};
    double right_position{m_right_motors.getPosition()};
    double avg_position{(left_position + right_position) / 2.0};

    double left_voltage_scale{avg_position / left_position};
    double right_voltage_scale{avg_position / right_position};

    left_voltage *= left_voltage_scale;
    right_voltage *= right_voltage_scale;

    m_left_motors.setVoltage(left_voltage);
    m_right_motors.setVoltage(right_voltage);

  }
}
}  // namespace drivetrain
}  // namespace subsystems
}  // namespace robot
}  // namespace driftless