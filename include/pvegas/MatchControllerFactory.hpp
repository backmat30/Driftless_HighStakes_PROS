#ifndef __MATCH_CONTROLLER_FACTORY_HPP__
#define __MATCH_CONTROLLER_FACTORY_HPP__

#include "MatchController.hpp"
namespace pvegas {
class MatchControllerFactory {
 public:
  // creates a match controller
  static pvegas::MatchController createMatchController();
};
}  // namespace pvegas
#endif