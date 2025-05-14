#include "driftless/OpControlManager.hpp"

namespace driftless {
// constructor
OpControlManager::OpControlManager(
    const std::shared_ptr<rtos::IClock>& clock,
    const std::unique_ptr<rtos::IDelayer>& delayer)
    : m_clock{clock}, m_delayer{delayer->clone()} {}

// sets the controller's profile
void OpControlManager::setProfile(
    std::unique_ptr<profiles::IProfile>& profile) {
  m_profile = std::move(profile);
}

void OpControlManager::setAlliance(
    std::shared_ptr<alliance::IAlliance>& alliance) {
  m_alliance = alliance;
}

void OpControlManager::init(
    std::shared_ptr<control::ControlSystem> control_system,
    std::shared_ptr<driftless::processes::ProcessSystem> process_system,
    std::shared_ptr<io::IController> controller,
    std::shared_ptr<robot::Robot> robot) {}
void OpControlManager::run(
    std::shared_ptr<control::ControlSystem> control_system,
    std::shared_ptr<driftless::processes::ProcessSystem> process_system,
    std::shared_ptr<io::IController> controller,
    std::shared_ptr<robot::Robot> robot) {
  // pause control system to allow operator takeover
  control_system->pause();

  process_system->sendCommand(
      processes::EProcess::AUTO_RING_REJECTION,
      processes::EProcessCommand::AUTO_RING_REJECTION_REJECT_RINGS, robot,
      m_alliance);
  process_system->resumeAll();

  // set subsystems to driver control

  op_control::clamp::ClampOperator clamp_operator{controller, robot};
  op_control::drivetrain::DrivetrainOperator drive_operator{controller, robot};
  op_control::elevator::ElevatorOperator elevator_operator{controller, robot};
  op_control::intake::IntakeOperator intake_operator{controller, robot};
  op_control::arm::ArmOperator arm_operator{controller, robot, process_system};
  op_control::color_sort::ColorSortOperator color_sort_operator{controller,
                                                                process_system};
  op_control::climb::ClimbOperator climb_operator{controller, robot};
  op_control::controller_swap::ControllerSwapOperator controller_swap_operator{
      controller};

  if (!m_profile->getStartupConfig(
          op_control::EStartupConfig::COLOR_SORT_DEFAULT)) {
    process_system->pause(processes::EProcess::AUTO_RING_REJECTION);
  }
  if (m_profile->getStartupConfig(op_control::EStartupConfig::ARM_CALLIBRATE)) {
    robot->sendCommand(robot::subsystems::ESubsystem::ARM,
                       robot::subsystems::ESubsystemCommand::ARM_CALIBRATE);
  }
  robot->sendCommand(robot::subsystems::ESubsystem::INTAKE,
                     robot::subsystems::ESubsystemCommand::INTAKE_PUSH_OUT);
  robot->sendCommand(robot::subsystems::ESubsystem::ELEVATOR,
                     robot::subsystems::ESubsystemCommand::ELEVATOR_REJECT_LEFT);

  // variable to hold time for delayer
  uint32_t current_time{};

  // task loop
  while (true) {
    // grabs time for delayer
    current_time = m_clock->getTime();

    // updates all subsystems
    clamp_operator.update(m_profile);
    climb_operator.update(m_profile);
    drive_operator.setDriveVoltage(m_profile);
    elevator_operator.update(m_profile, m_alliance);
    intake_operator.update(m_profile);
    arm_operator.update(m_profile, m_alliance);
    color_sort_operator.updateRingRejection(m_profile, m_alliance);
    controller_swap_operator.update(m_profile);    

    // delay until 10 seconds after loop start
    // keeps time per loop consistent rather than delaying 10 seconds AFTER
    // commands
    m_delayer->delayUntil(current_time + CONTROL_DELAY);
  }
}
}  // namespace driftless