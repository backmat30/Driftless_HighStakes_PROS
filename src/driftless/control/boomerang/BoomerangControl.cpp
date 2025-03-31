#include "driftless/control/boomerang/BoomerangControl.hpp"

namespace driftless::control::boomerang {

BoomerangControl::BoomerangControl(
    std::unique_ptr<driftless::control::boomerang::IBoomerang>& boomerang)
    : AControl(), m_boomerang(std::move(boomerang)) {}

void BoomerangControl::init() { m_boomerang->init(); }

void BoomerangControl::run() { m_boomerang->run(); }

void BoomerangControl::pause() { m_boomerang->pause(); }

void BoomerangControl::resume() { m_boomerang->resume(); }

void BoomerangControl::command(EControlCommand command_name, va_list& args) {
  if (command_name == EControlCommand::BOOMERANG_GO_TO_POSITION) {
    void* temp_robot{va_arg(args, void*)};
    std::shared_ptr<driftless::robot::Robot> robot{
        *static_cast<std::shared_ptr<driftless::robot::Robot>*>(temp_robot)};
    double velocity{va_arg(args, double)};
    double target_x{va_arg(args, double)};
    double target_y{va_arg(args, double)};
    double target_theta{va_arg(args, double)};

    m_boomerang->goToPosition(robot, velocity, target_x, target_y,
                              target_theta);

  } else if (command_name == EControlCommand::BOOMERANG_SET_VELOCITY) {
    double velocity{va_arg(args, double)};
    m_boomerang->setVelocity(velocity);
  }
}

void* BoomerangControl::state(EControlState state_name) {
  if (state_name == EControlState::BOOMERANG_TARGET_REACHED) {
    return reinterpret_cast<void*>(m_boomerang->targetReached());
  }
  return nullptr;
}
}  // namespace driftless::control::boomerang