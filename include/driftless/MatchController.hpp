#ifndef __MATCHCONTROLLER_HPP__
#define __MATCHCONTROLLER_HPP__

// includes
#include <memory>

#include "driftless/AutonManager.hpp"
#include "driftless/OpControlManager.hpp"
#include "driftless/control/ControlSystem.hpp"
#include "driftless/io/IController.hpp"
#include "driftless/menu/IMenu.hpp"
#include "driftless/processes/ProcessSystem.hpp"
#include "driftless/robot/Robot.hpp"
#include "driftless/rtos/IClock.hpp"
#include "driftless/rtos/IDelayer.hpp"
#include "pros/misc.hpp"

namespace driftless {
class MatchController {
 private:
  static constexpr uint32_t MENU_DELAY{10};

  std::unique_ptr<menu::IMenu> m_menu{};

  std::shared_ptr<rtos::IClock> m_clock{};

  std::unique_ptr<rtos::IDelayer> m_delayer{};

  OpControlManager op_control_manager{m_clock, m_delayer};

  AutonManager auton_manager{m_clock, m_delayer};

  std::shared_ptr<control::ControlSystem> control_system{};

  std::shared_ptr<io::IController> controller{};

  std::shared_ptr<robot::Robot> robot{};

  std::shared_ptr<processes::ProcessSystem> process_system{};

 public:
  MatchController(std::unique_ptr<menu::IMenu> &new_menu,
                  std::shared_ptr<rtos::IClock> &clock,
                  std::unique_ptr<rtos::IDelayer> &delayer);

  void init(bool fast_init);

  void disabled();

  void competitionInit();

  void autonomous();

  void operatorControl();
};
}  // namespace driftless
#endif