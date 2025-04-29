#include "driftless/robot/subsystems/climb/ClimbSubsystem.hpp"

namespace driftless::robot::subsystems::climb {
  ClimbSubsystem::ClimbSubsystem(std::unique_ptr<IClimb>& climb) : ASubsystem{ESubsystem::CLIMB}, m_climb{std::move(climb)} {}

  void ClimbSubsystem::init() {
    m_climb->init();
  }
  void ClimbSubsystem::run() {
    m_climb->run();
  }

  void ClimbSubsystem::command(ESubsystemCommand command_name, va_list& args) {
    switch (command_name) {
      case ESubsystemCommand::CLIMB_TOGGLE_CLIMBING:
        m_climb->toggleClimbing();
        break;
      case ESubsystemCommand::CLIMB_PULL_BACK_CLIMBER:
        m_climb->pullBackClimber();
        break;
      case ESubsystemCommand::CLIMB_PUSH_FORWARD_CLIMBER:
        m_climb->pushForwardClimber();
        break;
      case ESubsystemCommand::CLIMB_PUSH_OUT_PASSIVE_HOOKS:
        m_climb->pushOutPassiveHooks();
        break;
      case ESubsystemCommand::CLIMB_PULL_IN_PASSIVE_HOOKS:
        m_climb->pullInPassiveHooks();
        break;
    }
  }

  void* ClimbSubsystem::state(ESubsystemState state_name) {
    switch(state_name) {
      case ESubsystemState::CLIMB_ARE_PASSIVES_OUT: {
        bool* are_passives_out{new bool{m_climb->arePassivesOut()}};
        return are_passives_out;
      }
      default:
        return nullptr;
    }
  }
}