#include "pvegas/pros_adapters/ProsColorSensor.hpp"

namespace pvegas {
namespace pros_adapters {
ProsColorSensor::ProsColorSensor(std::unique_ptr<pros::Optical>& optical_sensor)
    : m_optical_sensor{std::move(optical_sensor)} {}

void ProsColorSensor::init() {
  m_optical_sensor->disable_gesture();
  m_optical_sensor->set_led_pwm(DEFAULT_LED_BRIGHTNESS);
}

void ProsColorSensor::setLEDBrightness(uint8_t brightness) {
  m_optical_sensor->set_led_pwm(brightness);
}

double ProsColorSensor::getHue() {
  return m_optical_sensor->get_hue();
}
}  // namespace pros_adapters
}  // namespace pvegas