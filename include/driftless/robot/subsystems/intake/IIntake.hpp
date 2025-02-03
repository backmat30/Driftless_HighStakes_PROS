#ifndef __I_INTAKE_HPP__
#define __I_INTAKE_HPP__

/// @brief The namespace for driftless library code
/// @author Matthew Backman
namespace driftless {

/// @brief The namespace for robot code
/// @author Matthew Backman
namespace robot {

/// @brief The namespace for subsystems code
/// @author Matthew Backman
namespace subsystems {

/// @brief The namespace for intake subsystem code
/// @author Matthew Backman
namespace intake {

/// @brief Interface for intake mechanisms
/// @author Matthew Backman
class IIntake {
 public:
  /// @brief Destroys the intake object
  virtual ~IIntake() = default;

  /// @brief Initializes the intake
  virtual void init() = 0;

  /// @brief Runs the intake
  virtual void run() = 0;

  /// @brief Sets the voltage of the motor
  /// @param voltage __double__ The voltage to set
  virtual void setVoltage(double voltage) = 0;
};
}  // namespace intake
}  // namespace subsystems
}  // namespace robot
}  // namespace driftless
#endif