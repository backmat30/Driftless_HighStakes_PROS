#ifndef __INTAKE_OPERATOR_HPP__
#define __INTAKE_OPERATOR_HPP__

#include <memory>

#include "driftless/io/IController.hpp"
#include "driftless/op_control/EControllerDigital.hpp"
#include "driftless/op_control/intake/EIntakeControlMode.hpp"
#include "driftless/profiles/IProfile.hpp"
#include "driftless/robot/Robot.hpp"
#include "driftless/robot/subsystems/ESubsystem.hpp"
#include "driftless/robot/subsystems/ESubsystemCommand.hpp"
#include "driftless/robot/subsystems/ESubsystemState.hpp"

namespace driftless {
namespace op_control {
namespace intake {
class IntakeOperator {
 private:
  // the controller used
  std::shared_ptr<driftless::io::IController> m_controller{};

  // the robot being controlled
  std::shared_ptr<driftless::robot::Robot> m_robot{};

  // update the intake height with split toggle
  void updateSplitToggle(EControllerDigital up, EControllerDigital down);

  // update the intake height with single toggle
  void updateSingleToggle(EControllerDigital toggle);

  // update the intake height with hold up
  void updateHoldUp(EControllerDigital up);

  // update the intake spinner
  void updateSpinner(EControllerDigital spin, EControllerDigital reverse);

 public:
  // create a new intake operator object
  IntakeOperator(const std::shared_ptr<driftless::io::IController>& controller,
                 const std::shared_ptr<driftless::robot::Robot>& robot);

  // update the intake
  void update(const std::unique_ptr<driftless::profiles::IProfile>& profile);
};
}  // namespace intake
}  // namespace op_control
}  // namespace driftless
#endif