#ifndef __PISTON_HEIGHT_CONTROL_BUILDER_HPP__
#define __PISTON_HEIGHT_CONTROL_BUILDER_HPP__

#include <memory>

#include "pvegas/robot/subsystems/intake/PistonHeightControl.hpp"

namespace pvegas {
namespace robot {
namespace subsystems {
namespace intake {
class PistonHeightControlBuilder {
 private:
  // the pistons used to build the height controller
  pvegas::hal::PistonGroup m_pistons{};

 public:
  // add a piston group to the builder
  PistonHeightControlBuilder* withPiston(
      std::unique_ptr<pvegas::io::IPiston>& piston);

  // build a new piston height controller
  std::unique_ptr<PistonHeightControl> build();
};
}  // namespace intake
}  // namespace subsystems
}  // namespace robot
}  // namespace pvegas
#endif