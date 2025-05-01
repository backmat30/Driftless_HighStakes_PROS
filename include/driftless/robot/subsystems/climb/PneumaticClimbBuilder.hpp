#ifndef __PNEUMATIC_CLIMB_BUILDER_HPP__
#define __PNEUMATIC_CLIMB_BUILDER_HPP__

#include "driftless/robot/subsystems/climb/PneumaticClimb.hpp"

namespace driftless {
  namespace robot {
    namespace subsystems {
      namespace climb {
        class PneumaticClimbBuilder {
          private:
            hal::PistonGroup m_stilt_pistons{};
            hal::PistonGroup m_climber_pistons{};
            hal::PistonGroup m_passive_hook_pistons{};
          public:
            PneumaticClimbBuilder* withStiltPiston(std::unique_ptr<io::IPiston>& stilt_piston);

            PneumaticClimbBuilder* withClimberPiston(std::unique_ptr<io::IPiston>& climber_piston);

            PneumaticClimbBuilder* withPassiveHookPiston(std::unique_ptr<io::IPiston>& passive_hook_piston);

            std::unique_ptr<IClimb> build();
        };
      }
    }
  }
}
#endif