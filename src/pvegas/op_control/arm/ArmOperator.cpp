#include "pvegas/op_control/arm/ArmOperator.hpp"

namespace pvegas {
namespace op_control {
namespace arm {
bool ArmOperator::hasAllianceRing(const pvegas::alliance::Alliance alliance) {
  bool has_ring{*static_cast<bool*>(
      m_robot->getState(ARM_SUBSYSTEM_NAME, HAS_RING_STATE_NAME))};

  double ring_hue{*static_cast<double*>(
      m_robot->getState(ARM_SUBSYSTEM_NAME, GET_HUE_STATE_NAME))};

  bool has_alliance_ring{};
  if (ring_hue >= alliance.hue_range[0] && ring_hue <= alliance.hue_range[1]) {
    has_alliance_ring = has_ring;
  }

  return has_alliance_ring;
}
void ArmOperator::updateSplitToggle(EControllerDigital neutral,
                                    EControllerDigital load,
                                    EControllerDigital score,
                                    const pvegas::alliance::Alliance alliance) {
  bool go_neutral{m_controller->getNewDigital(neutral)};
  bool go_load{m_controller->getNewDigital(load)};
  bool go_score{m_controller->getNewDigital(score)};

  if (go_neutral && !go_load && !go_score) {
    m_robot->sendCommand(ARM_SUBSYSTEM_NAME, GO_NEUTRAL_COMMAND_NAME);

  } else if (!go_neutral && go_load && !go_score) {
    m_robot->sendCommand(ARM_SUBSYSTEM_NAME, GO_LOAD_COMMAND_NAME);

  } else if (!go_neutral && !go_load && go_score) {
    if (hasAllianceRing(alliance)) {
      m_robot->sendCommand(ARM_SUBSYSTEM_NAME, GO_SCORE_COMMAND_NAME);
    }
  }
}

void ArmOperator::updateSingleToggle(
    EControllerDigital toggle, const pvegas::alliance::Alliance alliance) {
  bool next_position{m_controller->getNewDigital(toggle)};

  bool is_neutral{*static_cast<bool*>(
      m_robot->getState(ARM_SUBSYSTEM_NAME, IS_NEUTRAL_STATE_NAME))};
  bool is_load{*static_cast<bool*>(
      m_robot->getState(ARM_SUBSYSTEM_NAME, IS_LOAD_STATE_NAME))};
  bool is_score{*static_cast<bool*>(
      m_robot->getState(ARM_SUBSYSTEM_NAME, IS_SCORE_STATE_NAME))};

  if (next_position) {
    if (is_neutral) {
      m_robot->sendCommand(ARM_SUBSYSTEM_NAME, GO_LOAD_COMMAND_NAME);
    } else if (is_load) {
      // if (hasAllianceRing(alliance)) {
        m_robot->sendCommand(ARM_SUBSYSTEM_NAME, GO_SCORE_COMMAND_NAME);
      // } else {
      //   m_robot->sendCommand(ARM_SUBSYSTEM_NAME, GO_NEUTRAL_COMMAND_NAME);
      // }
    } else if (is_score) {
      m_robot->sendCommand(ARM_SUBSYSTEM_NAME, GO_NEUTRAL_COMMAND_NAME);
    }
  }
}

ArmOperator::ArmOperator(
    const std::shared_ptr<pvegas::io::IController>& controller,
    const std::shared_ptr<pvegas::robot::Robot>& robot)
    : m_controller{controller}, m_robot{robot} {}

void ArmOperator::update(
    const std::unique_ptr<pvegas::profiles::IProfile>& profile,
    const pvegas::alliance::Alliance alliance) {
  EControllerDigital neutral{
      profile->getDigitalControlMapping(EControl::ARM_NEUTRAL)};
  EControllerDigital load{
      profile->getDigitalControlMapping(EControl::ARM_LOAD)};
  EControllerDigital score{
      profile->getDigitalControlMapping(EControl::ARM_SCORE)};
  EControllerDigital toggle{
      profile->getDigitalControlMapping(EControl::ARM_TOGGLE)};

  switch (static_cast<EArmControlMode>(
      profile->getControlMode(EControlType::ARM))) {
    case EArmControlMode::SPLIT_TOGGLE:
      updateSplitToggle(neutral, load, score, alliance);
      break;
    case EArmControlMode::SINGLE_TOGGLE:
      updateSingleToggle(toggle, alliance);
      break;
  }
}
}  // namespace arm
}  // namespace op_control
}  // namespace pvegas