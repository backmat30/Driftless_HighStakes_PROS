#ifndef __ELEVATOR_SUBSYSTEM_HPP__
#define __ELEVATOR_SUBSYSTEM_HPP__

#include <memory>

#include "driftless/robot/subsystems/ASubsystem.hpp"
#include "driftless/robot/subsystems/elevator/IElevator.hpp"

namespace driftless {
namespace robot {
namespace subsystems {
namespace elevator {
class ElevatorSubsystem : public ASubsystem {
 private:
  // name of the subsystem
  static constexpr char SUBSYSTEM_NAME[]{"ELEVATOR"};

  // COMMAND NAMES

  // command to set the position of the elevator
  static constexpr char SET_POSITION_COMMAND_NAME[]{"SET POSITION"};

  // command to set the voltage of the elevator motors
  static constexpr char SET_VOLTAGE_COMMAND_NAME[]{"SET VOLTAGE"};

  // STATE NAMES

  // position of the elevator
  static constexpr char GET_POSITION_STATE_NAME[]{"GET POSITION"};

  // the elevator object being used
  std::unique_ptr<IElevator> m_elevator{};

 public:
  // constructor
  ElevatorSubsystem(std::unique_ptr<IElevator>& elevator);

  // initialize the subsystem
  void init() override;

  // run the subsystem
  void run() override;

  // send a command to the subsystem
  void command(std::string command_name, va_list& args) override;

  // get a state of the subsystem
  void* state(std::string state_name) override;
};
}  // namespace elevator
}  // namespace subsystems
}  // namespace robot
}  // namespace driftless
#endif