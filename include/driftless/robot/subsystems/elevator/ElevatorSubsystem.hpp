#ifndef __ELEVATOR_SUBSYSTEM_HPP__
#define __ELEVATOR_SUBSYSTEM_HPP__

#include <memory>

#include "driftless/robot/subsystems/ASubsystem.hpp"
#include "driftless/robot/subsystems/elevator/IElevator.hpp"
#include "driftless/robot/subsystems/elevator/IRingRejection.hpp"

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

  // command to deploy the ring rejector
  static constexpr char DEPLOY_REJECTOR_COMMAND_NAME[]{"DEPLOY REJECTOR"};

  // command to retract the ring rejector
  static constexpr char RETRACT_REJECTOR_COMMAND_NAME[]{"RETRACT REJECTOR"};

  // STATE NAMES

  // position of the elevator
  static constexpr char GET_POSITION_STATE_NAME[]{"GET POSITION"};

  // whether the ring rejector is deployed
  static constexpr char IS_DEPLOYED_STATE_NAME[]{"IS DEPLOYED"};

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
  void command(std::string command_name, va_list& args) override;

  // get a state of the subsystem
  void* state(std::string state_name) override;
};
}  // namespace elevator
}  // namespace subsystems
}  // namespace robot
}  // namespace driftless
#endif