#ifndef __I_CONFIG_HPP__
#define __I_CONFIG_HPP__

#include <memory>
#include <string>

#include "driftless/control/ControlSystem.hpp"
#include "driftless/io/IController.hpp"
#include "driftless/processes/ProcessSystem.hpp"
#include "driftless/robot/Robot.hpp"

/// @brief Namespace for driftless library code
/// @author Matthew Backman
namespace driftless {

/// @brief Namespace for robot configurations
/// @author Matthew Backman
namespace config {

/// @brief Interface for a generic robot configuration
/// @author Matthew Backman
class IConfig {
 public:
  /// @brief deletes the config object
  virtual ~IConfig() = default;

  /// @brief gets the name of the config
  /// @return __std::string__ the name of the config as a string
  virtual std::string getName() = 0;

  /// @brief Builds a control system using the config values
  /// @return __std::shared_ptr<control::ControlSystem>__ The new control system
  virtual std::shared_ptr<control::ControlSystem> buildControlSystem() = 0;

  /// @brief Builds a controller object using the config values
  /// @return std::shared_ptr<io::IController>__ The new controller object
  virtual std::shared_ptr<io::IController> buildController() = 0;

  /// @brief Builds a robot object using the config values
  /// @return std::shared_ptr<robot::Robot>__ The new robot object
  virtual std::shared_ptr<robot::Robot> buildRobot() = 0;

  /// @brief Builds a process system using config values
  /// @return __shared_ptr<ProcessSystem>__ The new process system
  virtual std::shared_ptr<processes::ProcessSystem> buildProcessSystem() = 0;
};
}  // namespace config
}  // namespace driftless
#endif