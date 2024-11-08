#include "driftless/OpControlManager.hpp"

namespace driftless {
// constructor
OpControlManager::OpControlManager(
    const std::shared_ptr<rtos::IClock> &clock,
    const std::unique_ptr<rtos::IDelayer> &delayer)
    : m_clock{clock}, m_delayer{delayer->clone()} {}

// sets the controller's profile
void OpControlManager::setProfile(
    std::unique_ptr<profiles::IProfile> &profile) {
  m_profile = std::move(profile);
}

void OpControlManager::setAlliance(driftless::alliance::Alliance alliance) {
  m_alliance = alliance;
}

void OpControlManager::init(
    std::shared_ptr<control::ControlSystem> control_system,
    std::shared_ptr<io::IController> controller,
    std::shared_ptr<robot::Robot> robot) {}
void OpControlManager::run(
    std::shared_ptr<control::ControlSystem> control_system,
    std::shared_ptr<io::IController> controller,
    std::shared_ptr<robot::Robot> robot) {
  // pause control system to allow operator takeover
  control_system->pause();

  // set subsystems to driver control
  op_control::clamp::ClampOperator clamp_operator{controller, robot};
  op_control::drivetrain::DrivetrainOperator drive_operator{controller, robot};
  op_control::elevator::ElevatorOperator elevator_operator{controller, robot};
  op_control::intake::IntakeOperator intake_operator{controller, robot};
  op_control::arm::ArmOperator arm_operator{controller, robot};

  // variable to hold time for delayer
  uint32_t current_time{};

  // task loop
  while (true) {
    // grabs time for delayer
    current_time = m_clock->getTime();

    // updates all subsystems
    clamp_operator.update(m_profile);
    drive_operator.setDriveVoltage();
    elevator_operator.update(m_profile);
    intake_operator.update(m_profile);
    arm_operator.update(m_profile, m_alliance);

    // delay until 10 seconds after loop start
    // keeps time per loop consistent rather than delaying 10 seconds AFTER
    // commands
    m_delayer->delayUntil(current_time + CONTROL_DELAY);
  }
}
}  // namespace driftless