#include "driftless/robot/subsystems/arm/ArmSubsystem.hpp"

namespace driftless {
namespace robot {
namespace subsystems {
namespace arm {
ArmSubsystem::ArmSubsystem(std::unique_ptr<IArmMotion>& arm_motion,
                           std::unique_ptr<IRingSensor>& ring_sensor)
    : m_arm_motion{std::move(arm_motion)},
      m_ring_sensor{std::move(ring_sensor)},
      ASubsystem{SUBSYSTEM_NAME} {}

void ArmSubsystem::init() {
  m_arm_motion->init();
  m_ring_sensor->init();
}

void ArmSubsystem::run() {
  m_arm_motion->run();
  m_ring_sensor->run();
}

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
  } else if (state_name == HAS_RING_STATE_NAME) {
    result = new bool{m_ring_sensor->hasRing()};
  } else if (state_name == GET_HUE_STATE_NAME) {
    result = new double{m_ring_sensor->getHue()};
  }

  return result;
}
}  // namespace arm
}  // namespace subsystems
}  // namespace robot
}  // namespace driftless