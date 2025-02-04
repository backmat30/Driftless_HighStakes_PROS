#ifndef __PROS_COLOR_SENSOR_HPP__
#define __PROS_COLOR_SENSOR_HPP__

#include <cstdint>
#include <memory>

#include "driftless/io/IColorSensor.hpp"
#include "pros/optical.hpp"

/// @brief The namespace for driftless library code
/// @author Matthew Backman
namespace driftless {

/// @brief The namespace for PROS adapters
/// @author Matthew Backman
namespace pros_adapters {

/// @brief Adapter class for the PROS color sensor
/// @author Matthew Backman
class ProsColorSensor : public driftless::io::IColorSensor {
 private:
  // the default brightness on the LEDs
  static constexpr uint8_t DEFAULT_LED_BRIGHTNESS{100};

  // the optical sensor being adapted
  std::unique_ptr<pros::Optical> m_optical_sensor{};

 public:
  /// @brief Constructor
  /// @param optical_sensor __std::unique_ptr<pros::Optical>&__ The optical
  /// sensor to adapt
  ProsColorSensor(std::unique_ptr<pros::Optical>& optical_sensor);

  /// @brief Initializes the color sensor
  void init() override;

  /// @brief Sets the brightness of the LEDs on the optical sensor
  /// @param brightness __uint8_t__ The brightness level
  void setLEDBrightness(uint8_t brightness) override;

  /// @brief Gets the hue from the color sensor
  /// @return __double__ The hue value
  double getHue() override;

  /// @brief Gets the RGB value from the color sensor
  /// @return __io::RGBValue__ The RGB value
  io::RGBValue getRGB() override;

  /// @brief Gets the proximity from the color sensor
  /// @return __uint32_t__ The proximity value
  uint32_t getProximity() override;
};
}  // namespace pros_adapters
}  // namespace driftless
#endif