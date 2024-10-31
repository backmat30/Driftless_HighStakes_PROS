#ifndef __PISTON_GROUP_HPP__
#define __PISTON_GROUP_HPP__

#include <vector>
#include <memory>

#include "pvegas/io/IPiston.hpp"

namespace pvegas {
namespace hal {
class PistonGroup {
 private:
  // vector of pistons in the group
  std::vector<std::unique_ptr<pvegas::io::IPiston>> m_pistons{};

  // whether the pistons are extended or retracted
  bool extended{};

 public:
  // add a piston to the group
  void addPiston(std::unique_ptr<pvegas::io::IPiston>& piston);

  // extend all pistons in the group
  void extend();

  // retract all pistons in the group
  void retract();

  // toggle the state of all pistons in the group
  void toggleState();

  // determines if the pistons are extended
  bool isExtended();
};
}
}
#endif