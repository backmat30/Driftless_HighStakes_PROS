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

/// @brief Namespace for driftless library code
/// @author Matthew Backman
namespace driftless {
/// @brief Manages the match
/// @author Matthew Backman
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
  /// @brief Constructs a new MatchController object
  /// @param new_menu __std::unique_ptr<menu::IMenu>&__ The menu to use
  /// @param clock __std::shared_ptr<rtos::IClock>&__ The clock to use
  /// @param delayer __std::unique_ptr<rtos::IDelayer>&__ The delayer to use
  MatchController(std::unique_ptr<menu::IMenu> &new_menu,
                  std::shared_ptr<rtos::IClock> &clock,
                  std::unique_ptr<rtos::IDelayer> &delayer);

  /// @brief Initializes the MatchController
  /// @param fast_init __bool__ True if the MatchController should be initialized quickly, false otherwise
  void init(bool fast_init);

  /// @brief Runs the disabled sequence
  void disabled();

  /// @brief Runs the competition initialization sequence
  void competitionInit();

  /// @brief Runs the autonomous controller
  void autonomous();

  /// @brief Runs the operator control manager
  void operatorControl();
};
}  // namespace driftless
#endif