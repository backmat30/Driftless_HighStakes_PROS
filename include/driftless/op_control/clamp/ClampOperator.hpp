#ifndef __CLAMP_OPERATOR_HPP__
#define __CLAMP_OPERATOR_HPP__

#include <memory>

#include "driftless/io/IController.hpp"
#include "driftless/op_control/EControllerDigital.hpp"
#include "driftless/op_control/clamp/EClampControlMode.hpp"
#include "driftless/profiles/IProfile.hpp"
#include "driftless/robot/Robot.hpp"

namespace driftless {
namespace op_control {
namespace clamp {
class ClampOperator {
 private:
  // name of the clamp subsystem
  static constexpr char CLAMP_SUBSYSTEM_NAME[]{"CLAMP"};

  // name of the command to set the state of the clamp
  static constexpr char CLAMP_SET_STATE_COMMAND_NAME[]{"SET STATE"};

  // name of the state of the clamp
  static constexpr char CLAMP_GET_STATE_STATE_NAME[]{"GET STATE"};

  // the controller being used
  std::shared_ptr<driftless::io::IController> m_controller{};

  // the robot being controlled
  std::shared_ptr<driftless::robot::Robot> m_robot{};

  // update the clamp using hold mode
  void updateHold(EControllerDigital hold);

  // update the clamp using single toggle mode
  void updateSingleToggle(EControllerDigital toggle);

  // update the clamp using slpit toggle mode
  void updateSplitToggle(EControllerDigital grab, EControllerDigital release);

  // update the state of the clamp
  void updateClampState(bool state);

 public:
  // construct a new ClampOperator object
  ClampOperator(const std::shared_ptr<driftless::io::IController>& controller,
                const std::shared_ptr<driftless::robot::Robot>& robot);

  // update the clamp based on the profile specifications
  void update(const std::unique_ptr<driftless::profiles::IProfile>& profile);
};
}  // namespace clamp
}  // namespace op_control
}  // namespace driftless
#endif