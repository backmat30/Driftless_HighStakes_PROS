#ifndef __PISTON_CLAMP_BUILDER_HPP__
#define __PISTON_CLAMP_BUILDER_HPP__

#include "pvegas/robot/subsystems/clamp/PistonClamp.hpp"

namespace pvegas {
namespace robot {
namespace subsystems {
namespace clamp {
class PistonClampBuilder {
 private:
  // piston group used to build the clamp
  pvegas::hal::PistonGroup m_pistons{};

 public:
  // add a piston to the builder
  PistonClampBuilder* withPiston(std::unique_ptr<pvegas::io::IPiston>& piston);

  // build a PistonClamp object
  std::unique_ptr<PistonClamp> build();
};
}  // namespace clamp
}  // namespace subsystems
}  // namespace robot
}  // namespace pvegas
#endif