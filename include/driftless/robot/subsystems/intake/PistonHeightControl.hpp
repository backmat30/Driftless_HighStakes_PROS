#ifndef __PISTON_HEIGHT_CONTROL_HPP__
#define __PISTON_HEIGHT_CONTROL_HPP__

#include <memory>

#include "driftless/hal/PistonGroup.hpp"
#include "driftless/robot/subsystems/intake/IHeightControl.hpp"

namespace driftless {
namespace robot {
namespace subsystems {
namespace intake {
class PistonHeightControl : public IHeightControl {
 private:
  // the group of pistons being used
  driftless::hal::PistonGroup m_pistons{};

  // whether the intake is up or down
  bool raised{true};

 public:
  // initialize the height controller
  void init() override;

  // run the height controller
  void run() override;

  // sets the height of the intake
  void setHeight(bool up) override;

  // get the height of the intake
  bool isRaised() override;

  // set the pistons
  void setPistons(driftless::hal::PistonGroup& pistons);
};
}  // namespace intake
}  // namespace subsystems
}  // namespace robot
}  // namespace driftless
#endif