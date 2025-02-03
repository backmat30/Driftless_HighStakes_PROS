#ifndef __AUTON_MANAGER_HPP__
#define __AUTON_MANAGER_HPP__

#include <memory>

#include "driftless/alliance/IAlliance.hpp"
#include "driftless/auton/IAuton.hpp"
#include "driftless/rtos/IClock.hpp"
#include "driftless/rtos/IDelayer.hpp"

/// @brief Namespace for driftless library code
/// @author Matthew Backman
namespace driftless {
/// @brief Manages the autonomous mode of the robot
/// @author Matthew Backman
class AutonManager {
 private:
  std::shared_ptr<alliance::IAlliance> m_alliance{};

  std::unique_ptr<auton::IAuton> m_auton{};

  std::shared_ptr<rtos::IClock> m_clock{};

  std::unique_ptr<rtos::IDelayer> m_delayer{};

 public:
  /// @brief Constructs a new AutonManager object
  /// @param clock __std::shared_ptr<rtos::IClock>&__ The clock to use
  /// @param delayer __std::unique_ptr<rtos::IDelayer>&__ The delayer to use
  AutonManager(const std::shared_ptr<rtos::IClock>& clock,
               const std::unique_ptr<rtos::IDelayer>& delayer);

  /// @brief Sets the alliance used in the autonomous manager
  /// @param alliance __std::shared_ptr<alliance::IAlliance>&__ The alliance to
  /// use
  void setAlliance(const std::shared_ptr<alliance::IAlliance>& alliance);

  /// @brief Sets the auton used in the autonomous manager
  /// @param auton __std::unique_ptr<auton::IAuton>&__ The auton to use
  void setAuton(std::unique_ptr<auton::IAuton>& auton);

  /// @brief Initializes the selected auton
  /// @param robot __std::shared_ptr<robot::Robot>&__ The robot to use
  /// @param control_system __std::shared_ptr<control::ControlSystem>&__ The
  /// control system to use
  /// @param process_system
  /// __std::shared_ptr<driftless::processes::ProcessSystem>&__ The process
  /// system to use
  void initAuton(
      std::shared_ptr<robot::Robot>& robot,
      std::shared_ptr<control::ControlSystem>& control_system,
      std::shared_ptr<driftless::processes::ProcessSystem>& process_system);

  /// @brief Runs the selected auton
  /// @param robot __std::shared_ptr<robot::Robot>&__ The robot to use
  /// @param control_system __std::shared_ptr<control::ControlSystem>&__ The
  /// control system to use
  /// @param process_system
  /// __std::shared_ptr<driftless::processes::ProcessSystem>&__ The process
  /// system to use
  void runAuton(
      std::shared_ptr<driftless::robot::Robot>& robot,
      std::shared_ptr<driftless::control::ControlSystem>& control_system,
      std::shared_ptr<driftless::processes::ProcessSystem>& process_system);
};
}  // namespace driftless
#endif