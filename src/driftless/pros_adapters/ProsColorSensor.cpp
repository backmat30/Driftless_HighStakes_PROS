#include "driftless/pros_adapters/ProsColorSensor.hpp"

namespace driftless {
namespace pros_adapters {
ProsColorSensor::ProsColorSensor(std::unique_ptr<pros::Optical>& optical_sensor)
    : m_optical_sensor{std::move(optical_sensor)} {}

void ProsColorSensor::init() {
  m_optical_sensor->disable_gesture();
  m_optical_sensor->set_integration_time(20);
  m_optical_sensor->set_led_pwm(100);
}

void ProsColorSensor::setLEDBrightness(uint8_t brightness) {
  m_optical_sensor->set_led_pwm(brightness);
}

double ProsColorSensor::getHue() { return m_optical_sensor->get_hue(); }

io::RGBValue ProsColorSensor::getRGB() {
  io::RGBValue rgb{};

  pros::c::optical_rgb_s_t pros_rgb{m_optical_sensor->get_rgb()};
  rgb.red = pros_rgb.red;
  rgb.green = pros_rgb.green;
  rgb.blue = pros_rgb.blue;
  rgb.brightness = pros_rgb.brightness;

  return rgb;
}

uint32_t ProsColorSensor::getProximity() {
  return m_optical_sensor->get_proximity();
}
}  // namespace pros_adapters
}  // namespace driftless