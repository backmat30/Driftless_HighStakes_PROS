#ifndef __I_COLOR_SENSOR_HPP__
#define __I_COLOR_SENSOR_HPP__

#include <cstdint>

#include "driftless/io/RGBValue.hpp"

/// @brief Namespace for driftless library code
/// @author Matthew Backman
namespace driftless {

/// @brief Namespace for IO device interfaces
/// @author Matthew Backman
namespace io {

/// @brief Interface for a generic color sensor
/// @author Matthew Backman
class IColorSensor {
 public:
  /// @brief Deletes the color sensor
  virtual ~IColorSensor() = default;

  /// @brief Initializes the color sensor
  virtual void init() = 0;

  /// @brief Sets the brightness of the LEDs on the color sensor
  /// @param brightness __uint8_t__ The brightness, from 0 to 100
  virtual void setLEDBrightness(uint8_t brightness) = 0;

  /// @brief Gets the hue reading from the color sensor
  /// @return __double__ The hue, from 0 to 360
  virtual double getHue() = 0;

  /// @brief Gets the RGB value from the color sensor
  /// @return __RGBValue__ The rgb value of the color sensor
  virtual RGBValue getRGB() = 0;

  /// @brief Gets the proximity of the closest object to the color sensor
  /// @details Higher values mean closer objects, value of 0 for anything at or
  /// past ~1 foot
  /// @return __uint32_t__ The proximity value
  virtual uint32_t getProximity() = 0;
};
}  // namespace io
}  // namespace driftless
#endif