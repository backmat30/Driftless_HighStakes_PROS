#ifndef __RGB_VALUE_HPP__
#define __RGB_VALUE_HPP__

namespace driftless {
namespace io {
/// @brief Holds data for an RGB value with brightness
struct RGBValue {
  double red{};

  double green{};

  double blue{};

  double brightness{};
};
}  // namespace io
}  // namespace driftless
#endif