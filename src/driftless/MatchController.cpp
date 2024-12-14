#include "driftless/MatchController.hpp"
namespace driftless {
MatchController::MatchController(std::unique_ptr<menu::IMenu> &new_menu,
                                 std::shared_ptr<rtos::IClock> &clock,
                                 std::unique_ptr<rtos::IDelayer> &delayer)
    : m_menu{std::move(new_menu)},
      m_clock{clock},
      m_delayer{std::move(delayer)},
      auton_manager{m_clock, m_delayer},
      op_control_manager{m_clock, m_delayer} {}

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
  auton_manager.setAlliance(system_config.alliance);
  auton_manager.setAuton(system_config.auton);
  op_control_manager.setProfile(system_config.profile);
  op_control_manager.setAlliance(system_config.alliance);
  // get configuration settings
  control_system = system_config.config->buildControlSystem();
  controller = system_config.config->buildController();
  robot = system_config.config->buildRobot();
  process_system = system_config.config->buildProcessSystem();

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
    if (process_system) {
      process_system->init();
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
  if (process_system) {
    process_system->run();
  }

  // initialize the auton and op control managers
  auton_manager.initAuton(robot, control_system, process_system);
  op_control_manager.init(control_system, process_system, controller, robot);
}

void MatchController::disabled() {}

void MatchController::competitionInit() {}

void MatchController::autonomous() {
  auton_manager.runAuton(robot, control_system, process_system);
}

void MatchController::operatorControl() {
  op_control_manager.run(control_system, process_system, controller, robot);
}

}  // namespace driftless