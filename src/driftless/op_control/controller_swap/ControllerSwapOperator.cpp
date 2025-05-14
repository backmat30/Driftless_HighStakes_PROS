#include "driftless/op_control/controller_swap/ControllerSwapOperator.hpp"

namespace driftless::op_control::controller_swap {
ControllerSwapOperator::ControllerSwapOperator(
    const std::shared_ptr<io::IController>& controller)
    : m_controller{controller} {}

void ControllerSwapOperator::update(
    const std::unique_ptr<profiles::IProfile>& profile) {
  if (!m_controller) {
    return;
  }

  EControllerDigital swap_control{
      profile->getDigitalControlMapping(EControl::CONTROLLER_SWITCH_PROFILE)};

  EControllerDigital verify_control{
      profile->getDigitalControlMapping(EControl::CONTROLLER_VERIFY_PROFILE)};

  if (m_controller->getNewDigital(swap_control)) {
    int current_profile{profile->getControlMode(EControlType::DRIVE)};
    int new_profile = (current_profile + 1) % 2;

    profile->setControlMode(EControlType::DRIVE, new_profile);
    profile->setControlMode(EControlType::CLIMB, new_profile);

    if (new_profile) {
      m_controller->rumble("..");
    } else {
      m_controller->rumble("-");
    }
  }

  if(m_controller->getNewDigital(verify_control)) {
    if(profile->getControlMode(EControlType::DRIVE) == 0) {
      m_controller->rumble("-");
    } else {
      m_controller->rumble("..");
    }
  }
}
}  // namespace driftless::op_control::controller_swap