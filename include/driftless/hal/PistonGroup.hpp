#ifndef __PISTON_GROUP_HPP__
#define __PISTON_GROUP_HPP__

#include <memory>
#include <vector>

#include "driftless/io/IPiston.hpp"

/// @brief Namespace for driftless library code
/// @author Matthew Backman
namespace driftless {

/// @brief Namespace for the hardware abstraction layer
/// @author Matthew Backman
namespace hal {

/// @brief Class representing a group of linked pistons
/// @author Matthew Backman
class PistonGroup {
 private:
  // vector of pistons in the group
  std::vector<std::unique_ptr<driftless::io::IPiston>> m_pistons{};

  // whether the pistons are extended or retracted
  bool extended{};

 public:
  /// @brief Adds a piston to the group
  /// @param piston __std::unique_ptr<io::IPiston>&__ The piston to add
  void addPiston(std::unique_ptr<driftless::io::IPiston>& piston);

  /// @brief Extends all pistons in the group
  void extend();

  /// @brief Retracts all pistons in the group
  void retract();

  /// @brief Toggles the state of the pistons in the group
  void toggleState();

  /// @brief Determines if the pistons in the group are extended
  /// @return __bool__ True if extended, false if retracted
  bool isExtended();
};
}  // namespace hal
}  // namespace driftless
#endif