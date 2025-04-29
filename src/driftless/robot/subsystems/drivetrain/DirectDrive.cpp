#include "driftless/robot/subsystems/drivetrain/DirectDrive.hpp"

#include "pros/screen.hpp"
#include "pros/screen.hpp"
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
  if (!is_climbing) {
    m_left_motors.setVoltage(left_voltage);
    m_right_motors.setVoltage(right_voltage);
  }
  if (!is_climbing) {
    m_left_motors.setVoltage(left_voltage);
    m_right_motors.setVoltage(right_voltage);
  }
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

double directDrive::getLeftMotorPosition() {
  return m_left_motors.getPosition();
}

double directDrive::getRightMotorPosition() {
  return m_right_motors.getPosition();
}

void directDrive::toggleClimb() {
  is_climbing = !is_climbing;

  m_left_motors.setPosition(0.0);
  m_right_motors.setPosition(0.0);
}

void directDrive::climb(double voltage) {
  if (is_climbing) {
    double left_voltage{voltage};
    double right_voltage{voltage};

    double left_position{m_left_motors.getPosition()};
    if (left_position < 0) {
      m_left_motors.setPosition(0.0);
      left_position = 0.0;
    } else if (left_position > 51.0) {
      m_left_motors.setPosition(51.0);
      left_position = 51.0;
    }
    double right_position{m_right_motors.getPosition()};
    if (right_position < 0) {
      m_right_motors.setPosition(0.0);
      right_position = 0.0;
    } else if (right_position > 51.0) {
      m_right_motors.setPosition(51.0);
      right_position = 51.0;
    }

    pros::screen::print(pros::E_TEXT_LARGE_CENTER, 4, "L: %7.2f R: %7.2f",
                        left_position, right_position);

    double position_difference{left_position - right_position};
    double voltage_modifier{position_difference / 5.0};

    left_voltage -= voltage_modifier;
    right_voltage += voltage_modifier;

    if(voltage < 0) {
      left_voltage = std::min(left_voltage, 0.0);
      right_voltage = std::min(right_voltage, 0.0);
    } else if (voltage >= 0) {
      left_voltage = std::max(left_voltage, 0.0);
      right_voltage = std::max(right_voltage, 0.0);
    }

    m_left_motors.setVoltage(left_voltage);
    m_right_motors.setVoltage(right_voltage);
  }
}
}  // namespace drivetrain
}  // namespace subsystems
}  // namespace robot
}  // namespace driftless