#ifndef __A_SUBSYSTEM_HPP__
#define __A_SUBSYSTEM_HPP__

#include <cstdarg>
#include <string>

#include "driftless/robot/subsystems/ESubsystem.hpp"
#include "driftless/robot/subsystems/ESubsystemCommand.hpp"
#include "driftless/robot/subsystems/ESubsystemState.hpp"

namespace driftless {
namespace robot {
namespace subsystems {
class ASubsystem {
 private:
  ESubsystem m_name{};

 public:
  ASubsystem() = default;

  ASubsystem(const ASubsystem& other) = default;

  ASubsystem(ESubsystem new_name) : m_name{new_name} {}

  virtual ~ASubsystem() = default;

  const ESubsystem& getName() const { return m_name; }

  virtual void init() = 0;

  virtual void run() = 0;

  virtual void command(ESubsystemCommand command_name, va_list& args) = 0;

  virtual void* state(ESubsystemState state_name) = 0;

  ASubsystem& operator=(const ASubsystem& rhs) = default;
};
}  // namespace subsystems
}  // namespace robot
}  // namespace driftless
#endif