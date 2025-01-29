#ifndef __I_AUTON_HPP__
#define __I_AUTON_HPP__

#include <string>

#include "driftless/alliance/EAlliance.hpp"
#include "driftless/alliance/IAlliance.hpp"
#include "driftless/control/ControlSystem.hpp"
#include "driftless/control/EControl.hpp"
#include "driftless/control/EControlCommand.hpp"
#include "driftless/control/EControlState.hpp"
#include "driftless/processes/EProcess.hpp"
#include "driftless/processes/EProcessCommand.hpp"
#include "driftless/processes/EProcessState.hpp"
#include "driftless/processes/ProcessSystem.hpp"
#include "driftless/robot/Robot.hpp"
#include "driftless/robot/subsystems/ESubsystem.hpp"
#include "driftless/robot/subsystems/ESubsystemCommand.hpp"
#include "driftless/robot/subsystems/ESubsystemState.hpp"
#include "driftless/rtos/IClock.hpp"
#include "driftless/rtos/IDelayer.hpp"

namespace driftless {
namespace auton {
class IAuton {
 public:
  virtual ~IAuton() = default;

  virtual std::string getName() = 0;

  virtual void init(
      std::shared_ptr<driftless::robot::Robot>& robot,
      std::shared_ptr<driftless::control::ControlSystem>& control_system,
      std::shared_ptr<driftless::processes::ProcessSystem>& process_system) = 0;

  virtual void run(
      std::shared_ptr<driftless::robot::Robot>& robot,
      std::shared_ptr<driftless::control::ControlSystem>& control_system,
      std::shared_ptr<driftless::processes::ProcessSystem>& process_system,
      std::shared_ptr<driftless::alliance::IAlliance>& alliance,
      std::shared_ptr<rtos::IClock>& clock,
      std::unique_ptr<rtos::IDelayer>& delayer) = 0;
};
}  // namespace auton
}  // namespace driftless
#endif