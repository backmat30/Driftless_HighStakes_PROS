#include "driftless/AutonManager.hpp"

namespace driftless {
AutonManager::AutonManager(const std::shared_ptr<rtos::IClock>& clock,
                           const std::unique_ptr<rtos::IDelayer>& delayer)
    : m_clock{clock->clone()}, m_delayer{delayer->clone()} {}

void AutonManager::setAlliance(
    const std::shared_ptr<alliance::IAlliance>& alliance) {
  m_alliance = alliance;
}

void AutonManager::setAuton(std::unique_ptr<auton::IAuton>& auton) {
  m_auton = std::move(auton);
}

void AutonManager::initAuton(
    std::shared_ptr<robot::Robot>& robot,
    std::shared_ptr<control::ControlSystem>& control_system,
    std::shared_ptr<driftless::processes::ProcessSystem>& process_system) {
  m_auton->init(robot, control_system, process_system);
}

void AutonManager::runAuton(
    std::shared_ptr<driftless::robot::Robot>& robot,
    std::shared_ptr<driftless::control::ControlSystem>& control_system,
    std::shared_ptr<driftless::processes::ProcessSystem>& process_system) {
  m_auton->run(robot, control_system, process_system, m_alliance, m_clock,
               m_delayer);
}
}  // namespace driftless