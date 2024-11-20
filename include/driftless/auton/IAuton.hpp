#ifndef __I_AUTON_HPP__
#define __I_AUTON_HPP__

#include <string>

#include "driftless/alliance/IAlliance.hpp"
#include "driftless/control/ControlSystem.hpp"
#include "driftless/robot/Robot.hpp"
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
      std::shared_ptr<driftless::control::ControlSystem>& control_system) = 0;

  virtual void run(
      std::shared_ptr<driftless::robot::Robot>& robot,
      std::shared_ptr<driftless::control::ControlSystem>& control_system,
      std::shared_ptr<driftless::alliance::IAlliance>& alliance,
      std::shared_ptr<rtos::IClock>& clock,
      std::unique_ptr<rtos::IDelayer>& delayer) = 0;
};
}  // namespace auton
}  // namespace driftless
#endif