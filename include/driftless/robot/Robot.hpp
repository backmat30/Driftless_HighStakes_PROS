#ifndef __ROBOT_HPP__
#define __ROBOT_HPP__
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "driftless/robot/subsystems/ASubsystem.hpp"
#include "driftless/robot/subsystems/ESubsystem.hpp"
#include "driftless/robot/subsystems/ESubsystemCommand.hpp"
#include "driftless/robot/subsystems/ESubsystemState.hpp"

#include "pros/screen.hpp"

namespace driftless {
namespace robot {
class Robot {
 private:
  std::map<subsystems::ESubsystem, std::unique_ptr<subsystems::ASubsystem>> subsystems{};

 public:
  void addSubsystem(std::unique_ptr<subsystems::ASubsystem> &subsystem);

  bool removeSubsystem(subsystems::ESubsystem subsystem);

  void init();

  void run();

  void sendCommand(subsystems::ESubsystem subsystem_name,
                   subsystems::ESubsystemCommand command_name, ...);

  void *getState(subsystems::ESubsystem subsystem_name,
                 subsystems::ESubsystemState state_name);
};
}  // namespace robot
}  // namespace driftless
#endif