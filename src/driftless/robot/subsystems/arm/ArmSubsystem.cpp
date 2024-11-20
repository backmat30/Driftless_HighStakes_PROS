#include "driftless/robot/subsystems/arm/ArmSubsystem.hpp"

namespace driftless {
namespace robot {
namespace subsystems {
namespace arm {
ArmSubsystem::ArmSubsystem(std::unique_ptr<IArmMotion>& arm_motion)
    : m_arm_motion{std::move(arm_motion)}, ASubsystem{SUBSYSTEM_NAME} {}

void ArmSubsystem::init() { m_arm_motion->init(); }

void ArmSubsystem::run() { m_arm_motion->run(); }

void ArmSubsystem::command(std::string command_name, va_list& args) {
  if (command_name == CALIBRATE_COMMAND_NAME) {
    m_arm_motion->calibrate();
  } else if (command_name == GO_NEUTRAL_COMMAND_NAME) {
    m_arm_motion->goNeutral();
  } else if (command_name == GO_LOAD_COMMAND_NAME) {
    m_arm_motion->goLoad();
  } else if (command_name == GO_READY_COMMAND_NAME) {
    m_arm_motion->goReady();
  } else if (command_name == GO_SCORE_COMMAND_NAME) {
    m_arm_motion->goScore();
  } else if (command_name == GO_RUSH_COMMAND_NAME) {
    m_arm_motion->goRush();
  } else if (command_name == GO_PREVIOUS_COMMAND_NAME) {
    m_arm_motion->goPrevious();
  } else if (command_name == GO_ALLIANCE_STAKE_COMMAND_NAME) {
    m_arm_motion->goAllianceStake();
  }
}

void* ArmSubsystem::state(std::string state_name) {
  void* result{nullptr};

  if (state_name == IS_NEUTRAL_STATE_NAME) {
    result = new bool{m_arm_motion->isAtNeutral()};
  } else if (state_name == IS_GOING_NEUTRAL_STATE_NAME) {
    result = new bool{m_arm_motion->isGoingNeutral()};
  } else if (state_name == IS_LOAD_STATE_NAME) {
    result = new bool{m_arm_motion->isAtLoad()};
  } else if (state_name == IS_GOING_LOAD_STATE_NAME) {
    result = new bool{m_arm_motion->isGoingLoad()};
  } else if (state_name == IS_READY_STATE_NAME) {
    result = new bool{m_arm_motion->isAtReady()};
  } else if (state_name == IS_GOING_READY_STATE_NAME) {
    result = new bool{m_arm_motion->isGoingReady()};
  } else if (state_name == IS_SCORE_STATE_NAME) {
    result = new bool{m_arm_motion->isAtScore()};
  } else if (state_name == IS_GOING_SCORE_STATE_NAME) {
    result = new bool{m_arm_motion->isGoingScore()};
  } else if (state_name == IS_RUSH_STATE_NAME) {
    result = new bool{m_arm_motion->isAtRush()};
  } else if (state_name == IS_GOING_RUSH_STATE_NAME) {
    result = new bool{m_arm_motion->isGoingRush()};
  } else if (state_name == IS_ALLIANCE_STAKE_STATE_NAME) {
    result = new bool{m_arm_motion->isAtAllianceStake()};
  } else if (state_name == IS_GOING_ALLIANCE_STAKE_STATE_NAME) {
    result = new bool{m_arm_motion->isGoingAllianceStake()};
  }

  return result;
}
}  // namespace arm
}  // namespace subsystems
}  // namespace robot
}  // namespace driftless