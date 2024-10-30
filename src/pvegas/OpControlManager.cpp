#include "pvegas/OpControlManager.hpp"

#include <cstdint>
#include <memory>

#include "pvegas/control/ControlSystem.hpp"
#include "pvegas/io/IController.hpp"
#include "pvegas/op_control/drivetrain/DrivetrainOperator.hpp"
#include "pvegas/profiles/IProfile.hpp"
#include "pvegas/robot/Robot.hpp"
#include "pvegas/rtos/IClock.hpp"
#include "pvegas/rtos/IDelayer.hpp"

namespace pvegas {
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
  op_control::drivetrain::DrivetrainOperator drive_operator{controller, robot};
  op_control::elevator::ElevatorOperator elevator_operator{controller, robot};

  // variable to hold time for delayer
  uint32_t current_time{};

  // task loop
  while (true) {
    // grabs time for delayer
    current_time = m_clock->getTime();

    // updates all subsystems
    drive_operator.setDriveVoltage();
    elevator_operator.update(m_profile);

    // delay until 10 seconds after loop start
    // keeps time per loop consistent rather than delaying 10 seconds AFTER
    // commands
    m_delayer->delayUntil(current_time + CONTROL_DELAY);
  }
}
}  // namespace pvegas