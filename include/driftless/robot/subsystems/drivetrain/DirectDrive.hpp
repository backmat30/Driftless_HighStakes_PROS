#ifndef __DIRECT_DRIVE_HPP__
#define __DIRECT_DRIVE_HPP__

#include "driftless/hal/MotorGroup.hpp"
#include "driftless/hal/PistonGroup.hpp"
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

  double m_velocity_to_voltage{1.0};

  double m_gear_ratio{};

  double m_wheel_radius{};

  double m_drive_radius{};

  bool is_climbing{};

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

  /// @brief Sets the conversion from velocity to voltage
  /// @param velocity_to_voltage __double__ The ratio between velocity and
  /// voltage
  void setVelocityToVoltage(double velocity_to_voltage);

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

  /// @brief Sets the drive train to climbing mode
  void toggleClimb() override;

  /// @brief Controls the climb mech using the drive train
  /// @param voltage __double__ The voltage to feed the drive train
  void climb(double voltage) override;
};
}  // namespace drivetrain
}  // namespace subsystems
}  // namespace robot
}  // namespace driftless
#endif