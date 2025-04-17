#ifndef __OP_CONTROL_MANAGER_HPP__
#define __OP_CONTROL_MANAGER_HPP__

#include <cstdint>
#include <memory>

#include "driftless/alliance/IAlliance.hpp"
#include "driftless/control/ControlSystem.hpp"
#include "driftless/io/IController.hpp"
#include "driftless/op_control/arm/ArmOperator.hpp"
#include "driftless/op_control/clamp/ClampOperator.hpp"
#include "driftless/op_control/climb/ClimbOperator.hpp"
#include "driftless/op_control/color_sort/ColorSortOperator.hpp"
#include "driftless/op_control/drivetrain/DrivetrainOperator.hpp"
#include "driftless/op_control/elevator/ElevatorOperator.hpp"
#include "driftless/op_control/intake/IntakeOperator.hpp"
#include "driftless/processes/ProcessSystem.hpp"
#include "driftless/profiles/IProfile.hpp"
#include "driftless/robot/Robot.hpp"
#include "driftless/rtos/IClock.hpp"
#include "driftless/rtos/IDelayer.hpp"

/// @brief Namespace for driftless library code
/// @author Matthew Backman
namespace driftless {
/// @brief Manages the operator control of the robot
/// @author Matthew Backman
class OpControlManager {
 private:
  static constexpr uint32_t CONTROL_DELAY{10};

  std::shared_ptr<rtos::IClock> m_clock{};

  std::unique_ptr<rtos::IDelayer> m_delayer{};

  std::unique_ptr<profiles::IProfile> m_profile{};

  std::shared_ptr<alliance::IAlliance> m_alliance{};

 public:
  /// @brief Constructs a new OpControlManager object
  /// @param clock __std::shared_ptr<rtos::IClock>&__ The clock to use
  /// @param delayer __std::unique_ptr<rtos::IDelayer>&__ The delayer to use
  OpControlManager(const std::shared_ptr<rtos::IClock>& clock,
                   const std::unique_ptr<rtos::IDelayer>& delayer);

  /// @brief Sets the profile used in the operator control
  /// @param profile __std::unique_ptr<profiles::IProfile>&__ The profile to use
  void setProfile(std::unique_ptr<profiles::IProfile>& profile);

  /// @brief Sets the alliance used in the operator control
  /// @param alliance __std::shared_ptr<alliance::IAlliance>&__ The alliance to
  /// use
  void setAlliance(std::shared_ptr<alliance::IAlliance>& alliance);

  /// @brief Initializes the operator control manager
  /// @param control_system __std::shared_ptr<control::ControlSystem>&__ The
  /// control system to use
  /// @param process_system
  /// __std::shared_ptr<driftless::processes::ProcessSystem>&__ The process
  /// system to use
  /// @param controller __std::shared_ptr<io::IController>&__ The controller to
  /// use
  /// @param robot __std::shared_ptr<robot::Robot>&__ The robot to use
  void init(std::shared_ptr<control::ControlSystem> control_system,
            std::shared_ptr<driftless::processes::ProcessSystem> process_system,
            std::shared_ptr<io::IController> controller,
            std::shared_ptr<robot::Robot> robot);

  /// @brief Runs the operator control manager
  /// @param control_system __std::shared_ptr<control::ControlSystem>&__ The
  /// control system to use
  /// @param process_system
  /// __std::shared_ptr<driftless::processes::ProcessSystem>&__ The process
  /// system to use
  /// @param controller __std::shared_ptr<io::IController>&__ The controller to
  /// use
  /// @param robot __std::shared_ptr<robot::Robot>&__ The robot to use
  void run(std::shared_ptr<control::ControlSystem> control_system,
           std::shared_ptr<driftless::processes::ProcessSystem> process_system,
           std::shared_ptr<io::IController> controller,
           std::shared_ptr<robot::Robot> robot);
};
}  // namespace driftless
#endif