#ifndef __OPTION_HPP__
#define __OPTION_HPP__

#include <string>
#include <vector>

/// @brief Namespace for driftless library code
/// @author Matthew Backman
namespace driftless {

/// @brief Namespace for menu handling
/// @author Matthew Backman
namespace menu {

/// @brief Struct representing an option type and potential values
/// @author Matthew Backman
struct Option {
  std::string name{};

  std::vector<std::string> choices{};

  int selected{};
};
}  // namespace menu
}  // namespace driftless
#endif