#ifndef __PNEUMATIC_CLIMB_HPP__
#define __PNEUMATIC_CLIMB_HPP__

#include "driftless/robot/subsystems/climb/IClimb.hpp"
#include "driftless/hal/PistonGroup.hpp"

namespace driftless {
namespace robot {
namespace subsystems {
namespace climb {
class PneumaticClimb : public IClimb {
  private:
    hal::PistonGroup m_stilt_pistons{};

    hal::PistonGroup m_climber_pistons{};

    bool is_climbing{};

  public:
    void init() override;

    void run() override;

    void toggleClimbing() override;

    void pullBackClimber() override;

    void pushForwardClimber() override;

    void setStiltPistons(hal::PistonGroup& pistons);

    void setClimberPistons(hal::PistonGroup& pistons);
};
}  // namespace climb
}  // namespace subsystems
}  // namespace robot
}  // namespace driftless

#endif