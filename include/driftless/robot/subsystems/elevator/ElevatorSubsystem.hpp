#ifndef __ELEVATOR_SUBSYSTEM_HPP__
#define __ELEVATOR_SUBSYSTEM_HPP__

#include <memory>

#include "driftless/robot/subsystems/ASubsystem.hpp"
#include "driftless/robot/subsystems/elevator/IElevator.hpp"
#include "driftless/robot/subsystems/elevator/IRingRejection.hpp"

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

/// @brief Class representing the elevator subsystem
/// @author Matthew Backman
class ElevatorSubsystem : public ASubsystem {
 private:
  // the elevator object being used
  std::unique_ptr<IElevator> m_elevator{};

  /// @brief The ring rejector
  std::unique_ptr<IRingRejection> m_ring_rejector{};

 public:
  // constructor
  ElevatorSubsystem(std::unique_ptr<IElevator>& elevator,
                    std::unique_ptr<IRingRejection>& ring_rejector);

  // initialize the subsystem
  void init() override;

  // run the subsystem
  void run() override;

  // send a command to the subsystem
  void command(ESubsystemCommand command_name, va_list& args) override;

  // get a state of the subsystem
  void* state(ESubsystemState state_name) override;
};
}  // namespace elevator
}  // namespace subsystems
}  // namespace robot
}  // namespace driftless
#endif