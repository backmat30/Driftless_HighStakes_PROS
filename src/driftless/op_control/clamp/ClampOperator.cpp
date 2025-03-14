#include "driftless/op_control/clamp/ClampOperator.hpp"

namespace driftless {
namespace op_control {
namespace clamp {
void ClampOperator::updateHold(EControllerDigital hold) {
  updateClampState(m_controller->getDigital(hold));
}

void ClampOperator::updateSingleToggle(EControllerDigital toggle) {
  if (m_controller->getNewDigital(toggle)) {
    bool current_state{*static_cast<bool*>(m_robot->getState(
        robot::subsystems::ESubsystem::CLAMP,
        robot::subsystems::ESubsystemState::CLAMP_GET_STATE))};
    updateClampState(!current_state);
  }
}

void ClampOperator::updateSplitToggle(EControllerDigital grab,
                                      EControllerDigital release) {
  bool grab_state{m_controller->getNewDigital(grab)};
  bool release_state(m_controller->getNewDigital(release));

  if (grab_state && !release_state) {
    updateClampState(true);
  } else if (!grab_state && release_state) {
    updateClampState(false);
  }
}

void ClampOperator::updateClampState(bool state) {
  m_robot->sendCommand(robot::subsystems::ESubsystem::CLAMP,
                       robot::subsystems::ESubsystemCommand::CLAMP_SET_STATE,
                       state);
}

ClampOperator::ClampOperator(
    const std::shared_ptr<driftless::io::IController>& controller,
    const std::shared_ptr<driftless::robot::Robot>& robot)
    : m_controller{std::move(controller)}, m_robot{std::move(robot)} {}

void ClampOperator::update(
    const std::unique_ptr<driftless::profiles::IProfile>& profile) {
  EControllerDigital hold{
      profile->getDigitalControlMapping(EControl::CLAMP_HOLD)};
  EControllerDigital toggle{
      profile->getDigitalControlMapping(EControl::CLAMP_TOGGLE)};
  EControllerDigital grab{
      profile->getDigitalControlMapping(EControl::CLAMP_GRAB)};
  EControllerDigital release{
      profile->getDigitalControlMapping(EControl::CLAMP_RELEASE)};

  switch (static_cast<EClampControlMode>(
      profile->getControlMode(EControlType::CLAMP))) {
    case EClampControlMode::HOLD:
      updateHold(hold);
      break;
    case EClampControlMode::SINGLE_TOGGLE:
      updateSingleToggle(toggle);
      break;
    case EClampControlMode::SPLIT_TOGGLE:
      updateSplitToggle(grab, release);
      break;
  }
}
}  // namespace clamp
}  // namespace op_control
}  // namespace driftless