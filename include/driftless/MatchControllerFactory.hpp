#ifndef __MATCH_CONTROLLER_FACTORY_HPP__
#define __MATCH_CONTROLLER_FACTORY_HPP__

#include "driftless/MatchController.hpp"
#include "driftless/alliance/BlueAlliance.hpp"
#include "driftless/alliance/RedAlliance.hpp"
#include "driftless/auton/BlueRushAuton.hpp"
#include "driftless/auton/BlueSkillsAuton.hpp"
#include "driftless/auton/OrangeMidRushAuton.hpp"
#include "driftless/auton/OrangeRushAuton.hpp"
#include "driftless/auton/OrangeSkillsAuton.hpp"
#include "driftless/config/DefaultConfig.hpp"
#include "driftless/config/OrangeConfig.hpp"
#include "driftless/menu/MenuAdapter.hpp"
#include "driftless/profiles/EricProfile.hpp"
#include "driftless/profiles/EricSkillsProfile.hpp"
#include "driftless/profiles/JohnButArcade.hpp"
#include "driftless/profiles/JohnProfile.hpp"
#include "driftless/profiles/JohnSkillsProfile.hpp"

/// @brief Namespace for driftless library code
/// @author Matthew Backman
namespace driftless {
/// @brief Factory for creating a MatchController object
/// @author Matthew Backman
class MatchControllerFactory {
 public:
  /// @brief Creates a new MatchController object
  /// @return __driftless::MatchController__ The new MatchController object
  static driftless::MatchController createMatchController();
};
}  // namespace driftless
#endif