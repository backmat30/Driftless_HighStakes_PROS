#include "driftless/robot/subsystems/drivetrain/DirectDriveBuilder.hpp"

namespace driftless {
namespace robot {
namespace subsystems {
namespace drivetrain {
DirectDriveBuilder *DirectDriveBuilder::withLeftMotor(
    std::unique_ptr<io::IMotor> &motor) {
  m_left_motors.addMotor(motor);
  return this;
}

DirectDriveBuilder *DirectDriveBuilder::withRightMotor(
    std::unique_ptr<io::IMotor> &motor) {
  m_right_motors.addMotor(motor);
  return this;
}

DirectDriveBuilder *DirectDriveBuilder::withLeftFeedForward(
    control::FeedForward left_motor_feed_forward) {
  m_left_motor_feed_forward = left_motor_feed_forward;
  return this;
}

DirectDriveBuilder *DirectDriveBuilder::withRightFeedForward(
    control::FeedForward right_motor_feed_forward) {
  m_right_motor_feed_forward = right_motor_feed_forward;
  return this;
}

DirectDriveBuilder *DirectDriveBuilder::withWheelRadius(double wheel_radius) {
  m_wheel_radius = wheel_radius;
  return this;
}

DirectDriveBuilder *DirectDriveBuilder::withDriveRadius(double drive_radius) {
  m_drive_radius = drive_radius;
  return this;
}

std::unique_ptr<IDrivetrain> DirectDriveBuilder::build() {
  std::unique_ptr<directDrive> drivetrain{std::make_unique<directDrive>()};
  drivetrain->setLeftMotors(m_left_motors);
  drivetrain->setRightMotors(m_right_motors);
  drivetrain->setLeftMotorFeedForward(m_left_motor_feed_forward);
  drivetrain->setRightMotorFeedForward(m_right_motor_feed_forward);
  drivetrain->setWheelRadius(m_wheel_radius);
  drivetrain->setDriveRadius(m_drive_radius);
  return drivetrain;
}
}  // namespace drivetrain
}  // namespace subsystems
}  // namespace robot
}  // namespace driftless