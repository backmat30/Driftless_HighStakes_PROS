#ifndef __CLIMB_SUBSYSTEM_HPP__
#define __CLIMB_SUBSYSTEM_HPP__

#include "driftless/robot/subsystems/ASubsystem.hpp"
#include "driftless/robot/subsystems/climb/IClimb.hpp"

#include <memory>

namespace driftless {
namespace robot {
namespace subsystems {
namespace climb {
class ClimbSubsystem : public ASubsystem {
  private:
    std::unique_ptr<IClimb> m_climb{};

  public:
    ClimbSubsystem(std::unique_ptr<IClimb>& climb);

    void init() override;

    void run() override;

    void command(ESubsystemCommand command_name, va_list& args) override;

    void* state(ESubsystemState state_name) override;
};
}  // namespace climb
}  // namespace subsystems
}  // namespace robot
}  // namespace driftless
#endif