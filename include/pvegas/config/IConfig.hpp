#ifndef __I_CONFIG_HPP__
#define __I_CONFIG_HPP__

#include <memory>
#include <string>

#include "pvegas/control/ControlSystem.hpp"
#include "pvegas/io/IController.hpp"
#include "pvegas/robot/Robot.hpp"
namespace pvegas {
namespace config {
class IConfig {
 public:
  virtual ~IConfig() = default;

  // gets the name of the config
  virtual std::string getName() = 0;

  // supplies a new control system built using this config
  virtual std::shared_ptr<control::ControlSystem> buildControlSystem() = 0;

  // supplies a new controller built using this config
  virtual std::shared_ptr<io::IController> buildController() = 0;

  // supplies a new robot built using this config
  virtual std::shared_ptr<robot::Robot> buildRobot() = 0;
};
}  // namespace config
}  // namespace pvegas
#endif