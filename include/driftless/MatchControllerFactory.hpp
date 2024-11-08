#ifndef __MATCH_CONTROLLER_FACTORY_HPP__
#define __MATCH_CONTROLLER_FACTORY_HPP__

#include "driftless/MatchController.hpp"
#include "driftless/config/DefaultConfig.hpp"
#include "driftless/menu/MenuAdapter.hpp"
#include "driftless/profiles/DefaultProfile.hpp"

namespace driftless {
class MatchControllerFactory {
 public:
  // creates a match controller
  static driftless::MatchController createMatchController();
};
}  // namespace driftless
#endif