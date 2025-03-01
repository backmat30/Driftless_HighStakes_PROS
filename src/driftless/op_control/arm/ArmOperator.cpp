#include "driftless/op_control/arm/ArmOperator.hpp"

namespace driftless {
namespace op_control {
namespace arm {
bool ArmOperator::hasAllianceRing(
    const std::shared_ptr<alliance::IAlliance>& alliance) {
  bool has_ring{hasRing()};

  io::RGBValue ring_rgb{*static_cast<io::RGBValue*>(m_robot->getState(
      robot::subsystems::ESubsystem::RING_SORT,
      robot::subsystems::ESubsystemState::RING_SORT_GET_RGB))};

  bool has_alliance_ring{};

  if (has_ring) {
    if ((alliance->getAlliance() == alliance::EAlliance::BLUE &&
         ring_rgb.blue >= ring_rgb.red) ||
        (alliance->getAlliance() == alliance::EAlliance::RED &&
         ring_rgb.red >= ring_rgb.blue)) {
      has_alliance_ring = true;
    }
  }

  return has_alliance_ring;
}

bool ArmOperator::hasOpposingRing(
    const std::shared_ptr<alliance::IAlliance>& alliance) {
  bool has_ring{hasRing()};

  io::RGBValue ring_rgb{*static_cast<io::RGBValue*>(m_robot->getState(
      robot::subsystems::ESubsystem::RING_SORT,
      robot::subsystems::ESubsystemState::RING_SORT_GET_RGB))};

  bool has_opposing_ring{};

  if (has_ring) {
    if ((alliance->getAlliance() == alliance::EAlliance::BLUE &&
         ring_rgb.red >= ring_rgb.blue) ||
        (alliance->getAlliance() == alliance::EAlliance::RED &&
         ring_rgb.blue >= ring_rgb.red)) {
      has_opposing_ring = true;
    }
  }

  return has_opposing_ring;
}

bool ArmOperator::hasRing() {
  bool has_ring{*static_cast<bool*>(m_robot->getState(
      robot::subsystems::ESubsystem::RING_SORT,
      robot::subsystems::ESubsystemState::RING_SORT_HAS_RING))};

  return has_ring;
}

void ArmOperator::updateSplitToggle(
    EControllerDigital neutral, EControllerDigital load,
    EControllerDigital ready, EControllerDigital score,
    const std::shared_ptr<alliance::IAlliance>& alliance) {
  bool go_neutral{m_controller->getNewDigital(neutral)};
  bool go_load{m_controller->getNewDigital(load)};
  bool go_ready{m_controller->getNewDigital(ready)};
  bool go_score{m_controller->getNewDigital(score)};

  if (go_neutral && !go_load && !go_ready && !go_score) {
    bool at_load{*static_cast<bool*>(
        m_robot->getState(robot::subsystems::ESubsystem::ARM,
                          robot::subsystems::ESubsystemState::ARM_IS_LOAD))};
    if (!(hasAllianceRing(alliance) && at_load)) {
      m_robot->sendCommand(
          robot::subsystems::ESubsystem::ARM,
          robot::subsystems::ESubsystemCommand::ARM_GO_NEUTRAL);
    }

  } else if (!go_neutral && go_load && !go_ready && !go_score) {
    m_robot->sendCommand(robot::subsystems::ESubsystem::ARM,
                         robot::subsystems::ESubsystemCommand::ARM_GO_LOAD);

  } else if (!go_neutral && !go_load && go_ready && !go_score) {
    m_robot->sendCommand(robot::subsystems::ESubsystem::ARM,
                         robot::subsystems::ESubsystemCommand::ARM_GO_READY);
  } else if (!go_neutral && !go_load && go_score) {
    bool is_ready{*static_cast<bool*>(
        m_robot->getState(robot::subsystems::ESubsystem::ARM,
                          robot::subsystems::ESubsystemState::ARM_IS_READY))};

    if (is_ready) {
      m_robot->sendCommand(robot::subsystems::ESubsystem::ARM,
                           robot::subsystems::ESubsystemCommand::ARM_GO_SCORE);
    }
  }
}

void ArmOperator::updateSmartToggle(
    EControllerDigital toggle, EControllerDigital rush,
    EControllerDigital calibrate, EControllerDigital alliance_stake,
    const std::shared_ptr<alliance::IAlliance>& alliance) {
  bool next_position{m_controller->getNewDigital(toggle)};
  bool go_rush{m_controller->getNewDigital(rush)};
  bool calibrate_arm{m_controller->getNewDigital(calibrate)};
  bool go_alliance_stake{m_controller->getNewDigital(alliance_stake)};

  void* is_neutral_state{
      m_robot->getState(robot::subsystems::ESubsystem::ARM,
                        robot::subsystems::ESubsystemState::ARM_IS_NEUTRAL)};
  bool is_neutral{is_neutral_state != nullptr &&
                  *static_cast<bool*>(is_neutral_state)};

  void* is_going_neutral_state{m_robot->getState(
      robot::subsystems::ESubsystem::ARM,
      robot::subsystems::ESubsystemState::ARM_IS_GOING_NEUTRAL)};
  bool is_going_neutral{is_going_neutral_state != nullptr &&
                        *static_cast<bool*>(is_going_neutral_state)};

  void* is_load_state{
      m_robot->getState(robot::subsystems::ESubsystem::ARM,
                        robot::subsystems::ESubsystemState::ARM_IS_LOAD)};
  bool is_load{is_load_state != nullptr && *static_cast<bool*>(is_load_state)};

  void* is_going_load_state{
      m_robot->getState(robot::subsystems::ESubsystem::ARM,
                        robot::subsystems::ESubsystemState::ARM_IS_GOING_LOAD)};
  bool is_going_load{is_going_load_state != nullptr &&
                     *static_cast<bool*>(is_going_load_state)};

  void* is_ready_state{
      m_robot->getState(robot::subsystems::ESubsystem::ARM,
                        robot::subsystems::ESubsystemState::ARM_IS_READY)};
  bool is_ready{is_ready_state != nullptr &&
                *static_cast<bool*>(is_ready_state)};

  void* is_going_ready_state{m_robot->getState(
      robot::subsystems::ESubsystem::ARM,
      robot::subsystems::ESubsystemState::ARM_IS_GOING_READY)};
  bool is_going_ready{is_going_ready_state != nullptr &&
                      *static_cast<bool*>(is_going_ready_state)};

  void* is_score_state{
      m_robot->getState(robot::subsystems::ESubsystem::ARM,
                        robot::subsystems::ESubsystemState::ARM_IS_SCORE)};
  bool is_score{is_score_state != nullptr &&
                *static_cast<bool*>(is_score_state)};

  void* is_going_score_state{m_robot->getState(
      robot::subsystems::ESubsystem::ARM,
      robot::subsystems::ESubsystemState::ARM_IS_GOING_SCORE)};
  bool is_going_score{is_going_score_state != nullptr &&
                      *static_cast<bool*>(is_going_score_state)};

  void* is_rush_state{
      m_robot->getState(robot::subsystems::ESubsystem::ARM,
                        robot::subsystems::ESubsystemState::ARM_IS_RUSH)};
  bool is_rush{is_rush_state != nullptr && *static_cast<bool*>(is_rush_state)};

  void* is_going_rush_state{
      m_robot->getState(robot::subsystems::ESubsystem::ARM,
                        robot::subsystems::ESubsystemState::ARM_IS_GOING_RUSH)};
  bool is_going_rush{is_going_rush_state != nullptr &&
                     *static_cast<bool*>(is_going_rush_state)};

  bool has_alliance_ring{hasAllianceRing(alliance)};

  bool is_color_sorting_paused{*static_cast<bool*>(m_process_system->getState(
      processes::EProcess::AUTO_RING_REJECTION,
      processes::EProcessState::AUTO_RING_REJECTION_IS_PAUSED))};

  // Opposing ring logic moved to a process
  // bool has_opposing_ring{hasOpposingRing(alliance)};

  /*if (has_opposing_ring) {
    if (is_load) {
      m_robot->sendCommand(robot::subsystems::ESubsystem::ARM,
  robot::subsystems::ESubsystemCommand::ARM_GO_NEUTRAL);
    }
  } else {*/
  if (next_position && !go_rush && !calibrate_arm && !go_alliance_stake) {
    if (is_neutral) {
      m_robot->sendCommand(robot::subsystems::ESubsystem::ARM,
                           robot::subsystems::ESubsystemCommand::ARM_GO_LOAD);
    } else if (is_load &&
               (has_alliance_ring || (is_color_sorting_paused && hasRing()))) {
      m_robot->sendCommand(robot::subsystems::ESubsystem::ARM,
                           robot::subsystems::ESubsystemCommand::ARM_GO_READY);
    } else if (is_ready) {
      m_robot->sendCommand(robot::subsystems::ESubsystem::ARM,
                           robot::subsystems::ESubsystemCommand::ARM_GO_SCORE);
    } else if (is_rush) {
      m_robot->sendCommand(robot::subsystems::ESubsystem::ARM,
                           robot::subsystems::ESubsystemCommand::ARM_GO_LOAD);
    } else if (is_going_neutral || is_going_load || is_going_ready ||
               is_going_score || is_going_rush) {
      m_robot->sendCommand(
          robot::subsystems::ESubsystem::ARM,
          robot::subsystems::ESubsystemCommand::ARM_GO_PREVIOUS);
    } else {
      m_robot->sendCommand(
          robot::subsystems::ESubsystem::ARM,
          robot::subsystems::ESubsystemCommand::ARM_GO_NEUTRAL);
    }
  } else if (!next_position && go_rush && !calibrate_arm &&
             !go_alliance_stake) {
    if (is_rush) {
      m_robot->sendCommand(robot::subsystems::ESubsystem::ARM,
                           robot::subsystems::ESubsystemCommand::ARM_GO_LOAD);
    } else {
      m_robot->sendCommand(robot::subsystems::ESubsystem::ARM,
                           robot::subsystems::ESubsystemCommand::ARM_GO_RUSH);
    }
  } else if (!next_position && !go_rush && !calibrate_arm &&
             go_alliance_stake) {
    m_robot->sendCommand(
        robot::subsystems::ESubsystem::ARM,
        robot::subsystems::ESubsystemCommand::ARM_GO_ALLIANCE_STAKE);
  }
  //}
  if (!next_position && !go_rush && calibrate_arm && !go_alliance_stake) {
    m_robot->sendCommand(robot::subsystems::ESubsystem::ARM,
                         robot::subsystems::ESubsystemCommand::ARM_CALIBRATE);
  }
}

ArmOperator::ArmOperator(
    const std::shared_ptr<driftless::io::IController>& controller,
    const std::shared_ptr<driftless::robot::Robot>& robot,
    const std::shared_ptr<driftless::processes::ProcessSystem>& process_system)
    : m_controller{controller},
      m_robot{robot},
      m_process_system{process_system} {}

void ArmOperator::update(
    const std::unique_ptr<driftless::profiles::IProfile>& profile,
    const std::shared_ptr<alliance::IAlliance>& alliance) {
  EControllerDigital neutral{
      profile->getDigitalControlMapping(EControl::ARM_NEUTRAL)};
  EControllerDigital load{
      profile->getDigitalControlMapping(EControl::ARM_LOAD)};
  EControllerDigital ready{
      profile->getDigitalControlMapping(EControl::ARM_READY)};
  EControllerDigital score{
      profile->getDigitalControlMapping(EControl::ARM_SCORE)};
  EControllerDigital toggle{
      profile->getDigitalControlMapping(EControl::ARM_TOGGLE)};
  EControllerDigital rush{
      profile->getDigitalControlMapping(EControl::ARM_RUSH)};
  EControllerDigital calibrate{
      profile->getDigitalControlMapping(EControl::ARM_CALIBRATE)};
  EControllerDigital alliance_stake{
      profile->getDigitalControlMapping(EControl::ARM_ALLIANCE_STAKE)};

  switch (static_cast<EArmControlMode>(
      profile->getControlMode(EControlType::ARM))) {
    case EArmControlMode::SPLIT_TOGGLE:
      updateSplitToggle(neutral, load, ready, score, alliance);
      break;
    case EArmControlMode::SMART_TOGGLE:
      updateSmartToggle(toggle, rush, calibrate, alliance_stake, alliance);
      break;
  }
}
}  // namespace arm
}  // namespace op_control
}  // namespace driftless