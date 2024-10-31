#ifndef __PISTON_HEIGHT_CONTROL_HPP__
#define __PISTON_HEIGHT_CONTROL_HPP__

#include <memory>

#include "pvegas/hal/PistonGroup.hpp"
#include "pvegas/robot/subsystems/intake/IHeightControl.hpp"

namespace pvegas {
namespace robot {
namespace subsystems {
namespace intake {
class PistonHeightControl : public IHeightControl {
 private:
  // the group of pistons being used
  pvegas::hal::PistonGroup m_pistons{};

  // whether the intake is up or down
  bool raised{};

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
  void setPistons(pvegas::hal::PistonGroup pistons);
};
}  // namespace intake
}  // namespace subsystems
}  // namespace robot
}  // namespace pvegas
#endif