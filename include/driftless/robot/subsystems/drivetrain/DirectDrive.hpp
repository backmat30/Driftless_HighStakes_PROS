#ifndef __DIRECT_DRIVE_HPP__
#define __DIRECT_DRIVE_HPP__

#include "driftless/control/FeedForward.hpp"
#include "driftless/hal/MotorGroup.hpp"
#include "driftless/robot/subsystems/drivetrain/IDriveTrain.hpp"

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

/// @brief Class representing the direct drive system
/// @author Matthew Backman
class directDrive : public IDrivetrain {
 private:
  hal::MotorGroup m_left_motors{};

  hal::MotorGroup m_right_motors{};

  control::FeedForward m_left_motor_feed_forward{};

  control::FeedForward m_right_motor_feed_forward{};

  double m_gear_ratio{};

  double m_wheel_radius{};

  double m_drive_radius{};

 public:
  /// @brief Initializes the direct drive
  void init() override;

  /// @brief Runs the direct drive
  void run() override;

  /// @brief Sets the velocity of the drive train
  /// @param velocity __Velocity__ The desired velocity
  void setVelocity(Velocity velocity) override;

  /// @brief Sets the voltage sent to the drive train motors
  /// @param left_voltage __double__ The voltage passed to the left motors
  /// @param right_voltage __double__ The voltage passed to the right motors
  void setVoltage(double left_voltage, double right_voltage) override;

  /// @brief Sets the left motors used by the drive train
  /// @param left_motors __hal::MotorGroup&__ The motors in the left of the
  /// drive train
  void setLeftMotors(hal::MotorGroup& left_motors);

  /// @brief Sets the right motors used by the drive train
  /// @param right_motors __hal::MotorGroup&__ The motors in the right of the
  /// drive train
  void setRightMotors(hal::MotorGroup& right_motors);

  /// @brief Sets the feed forward controller for the left motors
  /// @param left_motor_feed_forward __control::FeedForward&__ The feed forward
  /// controller for the left motors
  void setLeftMotorFeedForward(control::FeedForward left_motor_feed_forward);

  /// @brief Sets the feed forward controller for the right motors
  /// @param right_motor_feed_forward __control::FeedForward&__ The feed forward
  /// controller for the right motors
  void setRightMotorFeedForward(control::FeedForward right_motor_feed_forward);

  /// @brief Sets the gear ratio of the drive motors
  /// @param gear_ratio __double__ The gear ratio
  void setGearRatio(double gear_ratio);

  /// @brief Sets the radius of the wheels
  /// @param wheel_radius __double__ The wheel's radius, in inches
  void setWheelRadius(double wheel_radius);

  /// @brief Sets the radius of the drive train
  /// @param drive_radius __double__ The radius of the drive train, in inches
  void setDriveRadius(double drive_radius);

  /// @brief Gets the velocity of the drive train
  /// @return __Velocity__ The velocity of the drive train
  Velocity getVelocity() override;

  /// @brief Gets the radius of the drive train
  /// @return __double__ The radius of the drive train
  double getDriveRadius() override;
};
}  // namespace drivetrain
}  // namespace subsystems
}  // namespace robot
}  // namespace driftless
#endif