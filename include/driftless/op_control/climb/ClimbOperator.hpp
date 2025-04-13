#ifndef __CLIMB_OPERATOR_HPP__
#define __CLIMB_OPERATOR_HPP__

#include <memory>

#include "driftless/io/IController.hpp"
#include "driftless/op_control/EControllerAnalog.hpp"
#include "driftless/op_control/EControllerDigital.hpp"
#include "driftless/profiles/IProfile.hpp"
#include "driftless/robot/Robot.hpp"
#include "driftless/robot/subsystems/ESubsystem.hpp"
#include "driftless/robot/subsystems/ESubsystemCommand.hpp"
#include "driftless/robot/subsystems/ESubsystemState.hpp"

namespace driftless {
namespace op_control {
namespace climb {
class ClimbOperator {
 private:
  std::shared_ptr<io::IController> m_controller{};

  std::shared_ptr<robot::Robot> m_robot{};

  void updateStiltState();

  void pullBackClimber();

  void pushForwardClimber();

  void climbDriveTrain(double voltage);

 public:
  ClimbOperator(const std::shared_ptr<io::IController>& controller,
                const std::shared_ptr<robot::Robot>& robot);

  void update(const std::unique_ptr<profiles::IProfile>& profile);
};
}  // namespace climb
}  // namespace op_control
}  // namespace driftless
#endif