#ifndef __MATCH_CONTROLLER_FACTORY_HPP__
#define __MATCH_CONTROLLER_FACTORY_HPP__

#include "MatchController.hpp"
namespace driftless {
class MatchControllerFactory {
 public:
  // creates a match controller
  static driftless::MatchController createMatchController();
};
}  // namespace pvegas
#endif