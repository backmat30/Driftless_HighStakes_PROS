#ifndef __PROS_COLOR_SENSOR_HPP__
#define __PROS_COLOR_SENSOR_HPP__

#include <cstdint>
#include <memory>

#include "pros/optical.hpp"
#include "pvegas/io/IColorSensor.hpp"

namespace pvegas {
namespace pros_adapters {
class ProsColorSensor : public pvegas::io::IColorSensor {
 private:
  // the default brightness on the LEDs
  static constexpr uint8_t DEFAULT_LED_BRIGHTNESS{100};

  // the optical sensor being adapted
  std::unique_ptr<pros::Optical> m_optical_sensor{};

 public:
  // constructs a new pros color sensor
  ProsColorSensor(std::unique_ptr<pros::Optical>& optical_sensor);

  // initialize the color sensor
  void init() override;

  // set the brightness of the LEDs on the optical sensor
  void setLEDBrightness(uint8_t brightness) override;

  // get the hue from the color sensor
  double getHue() override;

  // get the proximity from the color sensor
  uint32_t getProximity() override;
};
}  // namespace pros_adapters
}  // namespace pvegas
#endif