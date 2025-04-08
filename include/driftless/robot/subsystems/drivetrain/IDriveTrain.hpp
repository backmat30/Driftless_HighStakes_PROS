#ifndef __I_DRIVETRAIN_HPP__
#define __I_DRIVETRAIN_HPP__

#include "driftless/robot/subsystems/drivetrain/Velocity.hpp"

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

/// @brief Interface for drive train systems
/// @author Matthew Backman
class IDrivetrain {
 public:
  /// @brief Destroys the drive train object
  virtual ~IDrivetrain() = default;

  /// @brief Initializes the drive train
  virtual void init() = 0;

  /// @brief Runs the drive train
  virtual void run() = 0;

  /// @brief Gets the velocity of the drive train
  /// @return __Velocity__ the drive train velocity
  virtual Velocity getVelocity() = 0;

  /// @brief Sets the velocity of the drive train
  /// @param velocity __Velocity__ the desired velocity
  virtual void setVelocity(Velocity velocity) = 0;

  /// @brief Sets the voltage sent to the drive train
  /// @param leftVoltage __double__ The voltage of the left motors
  /// @param rightVoltage __double__ The voltage of the right motors
  virtual void setVoltage(double leftVoltage, double rightVoltage) = 0;

  /// @brief Gets the radius of the drive train
  /// @return __double__ The drive radius
  virtual double getDriveRadius() = 0;

  /// @brief Sets the drive triain to climb mode
  virtual void startClimb() = 0;

  /// @brief Controls the climb mech using the drive train
  /// @param voltage __double__ The voltage to feed the drive train
  virtual void climb(double voltage) = 0;
};
}  // namespace drivetrain
}  // namespace subsystems
}  // namespace robot
}  // namespace driftless
#endif