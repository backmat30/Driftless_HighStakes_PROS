#ifndef __I_ELEVATOR_HPP__
#define __I_ELEVATOR_HPP__

/// @brief The namespace for driftless library code
/// @author Matthew Backman
namespace driftless {

/// @brief The namespace for robot code
/// @author Matthew Backman
namespace robot {

/// @brief The namespace for subsystems code
/// @author Matthew Backman
namespace subsystems {

/// @brief The namespace for elevator subsystem code
/// @author Matthew Backman
namespace elevator {

/// @brief Interface for elevator systems
/// @author Matthew Backman
class IElevator {
 public:
  // destroyer
  virtual ~IElevator() = default;

  // initialize the elevator
  virtual void init() = 0;

  // run the elevator
  virtual void run() = 0;

  // set the voltage of the elevator motors
  virtual void setVoltage(double voltage) = 0;

  // set the position of the elevator
  virtual void setPosition(double position) = 0;

  // get the position of the elevator
  virtual double getPosition() = 0;
};
}  // namespace elevator
}  // namespace subsystems
}  // namespace robot
}  // namespace driftless
#endif