#ifndef __RED_ALLIANCE_HPP__
#define __RED_ALLIANCE_HPP__

#include "driftless/alliance/IAlliance.hpp"

/// @brief Namespace for driftless library code
/// @author Matthew Backman
namespace driftless {

/// @brief Namespace for alliances
/// @author Matthew Backman
namespace alliance {

/// @brief Class representing the red alliance
/// @author Matthew Backman
class RedAlliance : public IAlliance {
 private:
  static constexpr io::RGBValue ALLIANCE_COLOR{1500.0, 0.0, 500.0, 0.0};

  static constexpr io::RGBValue OPPOSING_COLOR{500.0, 0.0, 1500.0, 0.0};

 public:
  /// @brief Gets the enumerated value of the alliance
  /// @return __EAlliance__ The alliance
  EAlliance getAlliance() override;

  /// @brief Gets the name of the alliance
  /// @return __std::string__ The name of the alliance
  std::string getName() override;

  /// @brief Gets the color range of alliance rings
  /// @return __utils::Range<double>__ The range of color values
  io::RGBValue getAllianceColor() override;

  /// @brief Gets the color range of the opponent's rings
  /// @return __utils::Range<double>__ The range of color values
  io::RGBValue getOpposingColor() override;
};
}  // namespace alliance
}  // namespace driftless
#endif