#ifndef __PISTON_CLAMP_HPP__
#define __PISTON_CLAMP_HPP__

#include <memory>

#include "pvegas/hal/PistonGroup.hpp"
#include "pvegas/robot/subsystems/clamp/IClamp.hpp"

namespace driftless {
namespace robot {
namespace subsystems {
namespace clamp {
class PistonClamp : public IClamp {
 private:
  // the pistons controlling the clamp
  driftless::hal::PistonGroup m_pistons{};

  // whether the clamp is active or idle
  bool state{};

 public:
  // initialize the clamp
  void init() override;

  // run the clamp
  void run() override;

  // set the state of the clamp
  void setState(bool clamped) override;

  // gets the state of the clamp
  bool getState() override;

  // set the pistons used
  void setPistons(driftless::hal::PistonGroup& pistons);
};
}  // namespace clamp
}  // namespace subsystems
}  // namespace robot
}  // namespace pvegas
#endif