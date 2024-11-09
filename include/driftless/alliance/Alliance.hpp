#ifndef __ALLIANCE_HPP__
#define __ALLIANCE_HPP__

#include <string>

/// @brief Namespace for driftless library code
namespace driftless {
/// @brief Namespace for alliance code
namespace alliance {
/// @brief struct holding alliance information
struct Alliance {
  /// @brief name of the alliance
  std::string name{};

  /// @brief min and max hue for alliance ring hue
  double hue_range[2]{};
};
}  // namespace alliance
}  // namespace pvegas
#endif