#include "driftless/robot/subsystems/arm/ArmSubsystem.hpp"

namespace driftless {
namespace robot {
namespace subsystems {
namespace arm {
ArmSubsystem::ArmSubsystem(std::unique_ptr<IArmMotion>& arm_motion)
    : m_arm_motion{std::move(arm_motion)}, ASubsystem{ESubsystem::ARM} {}

void ArmSubsystem::init() { m_arm_motion->init(); }

void ArmSubsystem::run() { m_arm_motion->run(); }

void ArmSubsystem::command(ESubsystemCommand command_name, va_list& args) {
  if (command_name == ESubsystemCommand::ARM_CALIBRATE) {
    m_arm_motion->calibrate();
  } else if (command_name == ESubsystemCommand::ARM_GO_NEUTRAL) {
    m_arm_motion->goNeutral();
  } else if (command_name == ESubsystemCommand::ARM_GO_LOAD) {
    m_arm_motion->goLoad();
  } else if (command_name == ESubsystemCommand::ARM_GO_READY) {
    m_arm_motion->goReady();
  } else if (command_name == ESubsystemCommand::ARM_GO_SCORE) {
    m_arm_motion->goScore();
  } else if (command_name == ESubsystemCommand::ARM_GO_RUSH) {
    m_arm_motion->goRush();
  } else if (command_name == ESubsystemCommand::ARM_GO_PREVIOUS) {
    m_arm_motion->goPrevious();
  } else if (command_name == ESubsystemCommand::ARM_GO_ALLIANCE_STAKE) {
    m_arm_motion->goAllianceStake();
  }
}

void* ArmSubsystem::state(ESubsystemState state_name) {
  void* result{nullptr};

  if (state_name == ESubsystemState::ARM_IS_NEUTRAL) {
    result = new bool{m_arm_motion->isAtNeutral()};
  } else if (state_name == ESubsystemState::ARM_IS_GOING_NEUTRAL) {
    result = new bool{m_arm_motion->isGoingNeutral()};
  } else if (state_name == ESubsystemState::ARM_IS_LOAD) {
    result = new bool{m_arm_motion->isAtLoad()};
  } else if (state_name == ESubsystemState::ARM_IS_GOING_LOAD) {
    result = new bool{m_arm_motion->isGoingLoad()};
  } else if (state_name == ESubsystemState::ARM_IS_READY) {
    result = new bool{m_arm_motion->isAtReady()};
  } else if (state_name == ESubsystemState::ARM_IS_GOING_READY) {
    result = new bool{m_arm_motion->isGoingReady()};
  } else if (state_name == ESubsystemState::ARM_IS_SCORE) {
    result = new bool{m_arm_motion->isAtScore()};
  } else if (state_name == ESubsystemState::ARM_IS_GOING_SCORE) {
    result = new bool{m_arm_motion->isGoingScore()};
  } else if (state_name == ESubsystemState::ARM_IS_RUSH) {
    result = new bool{m_arm_motion->isAtRush()};
  } else if (state_name == ESubsystemState::ARM_IS_GOING_RUSH) {
    result = new bool{m_arm_motion->isGoingRush()};
  } else if (state_name == ESubsystemState::ARM_IS_ALLIANCE_STAKE) {
    result = new bool{m_arm_motion->isAtAllianceStake()};
  } else if (state_name == ESubsystemState::ARM_IS_GOING_ALLIANCE_STAKE) {
    result = new bool{m_arm_motion->isGoingAllianceStake()};
  }

  return result;
}
}  // namespace arm
}  // namespace subsystems
}  // namespace robot
}  // namespace driftless