#ifndef __I_COLOR_SENSOR_HPP__
#define __I_COLOR_SENSOR_HPP__

#include <cstdint>

namespace driftless {
namespace io {
class IColorSensor {
 public:
  // destroys the color sensor object
  virtual ~IColorSensor() = default;

  // initialize the color sensor
  virtual void init() = 0;

  // sets the brightness of the LEDs on the color sensor
  virtual void setLEDBrightness(uint8_t brightness) = 0;

  // gets the current hue from the color sensor
  virtual double getHue() = 0;

  // gets the proximity of the nearest object to the sensor
  virtual uint32_t getProximity() = 0;
};
}  // namespace io
}  // namespace pvegas
#endif