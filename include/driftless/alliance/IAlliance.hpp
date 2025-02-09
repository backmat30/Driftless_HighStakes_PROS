#ifndef __I_ALLIANCE_HPP__
#define __I_ALLIANCE_HPP__

#include <string>

#include "driftless/alliance/EAlliance.hpp"
#include "driftless/io/RGBValue.hpp"
#include "driftless/utils/Range.hpp"

/// @brief Namespace for driftless library code
/// @author Matthew Backman
namespace driftless {

/// @brief Namespace for alliances
/// @author Matthew Backman
namespace alliance {

/// @brief Interface for a generic alliance
/// @author Matthew Backman
class IAlliance {
 public:
  /// @brief Deletes the current alliance object
  virtual ~IAlliance() = default;

  /// @brief Gets the enumerated value of the alliance
  /// @return __EAlliance__ The alliance
  virtual EAlliance getAlliance() = 0;

  /// @brief Gets the name of the alliance
  /// @return __std::string__ The name of the alliance
  virtual std::string getName() = 0;

  /// @brief Gets the color range of alliance rings
  /// @return __utils::Range<double>__ The range of color values
  virtual io::RGBValue getAllianceColor() = 0;

  /// @brief Gets the color range of the opponent's rings
  /// @return __utils::Range<double>__ The range of color values
  virtual io::RGBValue getOpposingColor() = 0;
};
}  // namespace alliance
}  // namespace driftless
#endif