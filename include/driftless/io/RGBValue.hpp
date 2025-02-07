#ifndef __RGB_VALUE_HPP__
#define __RGB_VALUE_HPP__

/// @brief Namespace for driftless library code
/// @author Matthew Backman
namespace driftless {

/// @brief Namespace for IO device interfaces
/// @author Matthew Backman
namespace io {

/// @brief Holds data for an RGB value with brightness
/// @author Matthew Backman
struct RGBValue {
  double red{};

  double green{};

  double blue{};

  double brightness{};
};
}  // namespace io
}  // namespace driftless
#endif