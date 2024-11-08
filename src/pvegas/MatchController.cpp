#include "pvegas/MatchController.hpp"

#include "OpControlManager.hpp"
#include "SystemConfig.hpp"
namespace driftless {
MatchController::MatchController(std::unique_ptr<menu::IMenu> &new_menu,
                                 std::shared_ptr<rtos::IClock> &clock,
                                 std::unique_ptr<rtos::IDelayer> &delayer)
    : m_menu{std::move(new_menu)},
      m_clock{clock},
      m_delayer{std::move(delayer)},
      op_control_manager(m_clock, m_delayer) {}

void MatchController::init(bool fast_init) {
  // if the menu exists, display the menu
  if (m_menu) {
    m_menu->display();
    // delays until the menu is set up
    while (m_delayer && !m_menu->isStarted() && !fast_init) {
      m_delayer->delay(MENU_DELAY);
    }
  }

  // gather system configuration data
  SystemConfig system_config{};
  if (m_menu) {
    system_config = m_menu->getSystemConfig(fast_init);
  }
  // send the profile info to the op control manager
  op_control_manager.setProfile(system_config.profile);
  // get configuration settings
  control_system = system_config.config->buildControlSystem();
  controller = system_config.config->buildController();
  robot = system_config.config->buildRobot();

  // if fast init isn't being used, take time to initialize the parts of the
  // robot
  if (!fast_init) {
    if (robot) {
      robot->init();
    }
    if (control_system) {
      control_system->init();
    }
    if (controller) {
      controller->init();
    }
  }

  // run the robot
  if (robot) {
    robot->run();
  }
  if (control_system) {
    control_system->run();
  }
  if (controller) {
    controller->run();
  }

  // initialize the auton and op control managers
  op_control_manager.init(control_system, controller, robot);
}

void MatchController::disabled() {}

void MatchController::competitionInit() {}

void MatchController::autonomous() {}

void MatchController::operatorControl() {
  op_control_manager.run(control_system, controller, robot);
}

}  // namespace pvegas