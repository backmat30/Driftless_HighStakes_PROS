#ifndef __A_SUBSYSTEM_HPP__
#define __A_SUBSYSTEM_HPP__

#include <cstdarg>
#include <string>

namespace pvegas {
namespace robot {
class ASubsystem {
 private:
  std::string m_name{};

 public:
  ASubsystem() = default;

  ASubsystem(const ASubsystem& other) = default;

  ASubsystem(std::string new_name) : m_name{new_name} {}

  virtual ~ASubsystem() = default;

  const std::string& getName() const { return m_name; }

  virtual void init() = 0;

  virtual void run() = 0;

  virtual void command(std::string command_name, va_list& args) = 0;

  virtual void* state(std::string state_name) = 0;

  ASubsystem& operator=(const ASubsystem& rhs) = default;
};
}  // namespace robot
}  // namespace pvegas
#endif