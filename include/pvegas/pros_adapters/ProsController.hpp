#ifndef __PROS_CONTROLLER_HPP__
#define __PROS_CONTROLLER_HPP__

#include "pros/misc.h"
#include "pros/misc.hpp"
#include "pros/rtos.hpp"

#include "pvegas/io/IController.hpp"

#include <cstdint>
#include <map>
#include <memory>

namespace pvegas {
namespace pros_adapters {
class ProsController : public io::IController{
private:
  static constexpr uint8_t TASK_DELAY{10};

  static constexpr uint8_t RUMBLE_REFRESH_RATE{50};

  static constexpr uint8_t MAX_RUMBLE_LENGTH{8};

  static constexpr double ANALOG_CONVERSION{1.0 / 127};

  static void taskLoop(void *params);

  const std::map<io::EControllerAnalog, pros::controller_analog_e_t> ANALOGUE_MAP{
      {io::EControllerAnalog::JOYSTICK_LEFT_X, pros::E_CONTROLLER_ANALOG_LEFT_X},
      {io::EControllerAnalog::JOYSTICK_LEFT_Y, pros::E_CONTROLLER_ANALOG_LEFT_Y},
      {io::EControllerAnalog::JOYSTICK_RIGHT_X, pros::E_CONTROLLER_ANALOG_RIGHT_X},
      {io::EControllerAnalog::JOYSTICK_RIGHT_Y, pros::E_CONTROLLER_ANALOG_RIGHT_Y}};

  const std::map<io::EControllerDigital, pros::controller_digital_e_t> DIGITAL_MAP{
      {io::EControllerDigital::BUTTON_A, pros::E_CONTROLLER_DIGITAL_A},
      {io::EControllerDigital::BUTTON_B, pros::E_CONTROLLER_DIGITAL_B},
      {io::EControllerDigital::BUTTON_X, pros::E_CONTROLLER_DIGITAL_X},
      {io::EControllerDigital::BUTTON_Y, pros::E_CONTROLLER_DIGITAL_Y},
      {io::EControllerDigital::DPAD_DOWN, pros::E_CONTROLLER_DIGITAL_DOWN},
      {io::EControllerDigital::DPAD_LEFT, pros::E_CONTROLLER_DIGITAL_LEFT},
      {io::EControllerDigital::DPAD_RIGHT, pros::E_CONTROLLER_DIGITAL_RIGHT},
      {io::EControllerDigital::DPAD_UP, pros::E_CONTROLLER_DIGITAL_UP},
      {io::EControllerDigital::TRIGGER_LEFT_BOTTOM, pros::E_CONTROLLER_DIGITAL_L2},
      {io::EControllerDigital::TRIGGER_LEFT_TOP, pros::E_CONTROLLER_DIGITAL_L1},
      {io::EControllerDigital::TRIGGER_RIGHT_BOTTOM, pros::E_CONTROLLER_DIGITAL_R2},
      {io::EControllerDigital::TRIGGER_RIGHT_TOP, pros::E_CONTROLLER_DIGITAL_R1}};

  std::unique_ptr<pros::Controller> m_controller{};

  pros::Mutex mutex{};

  char rumble_pattern[MAX_RUMBLE_LENGTH]{};

  bool new_rumble_pattern{};

  uint32_t last_rumble_refresh{};

  void updateRumble();

  void taskUpdate();

public:
  ProsController(std::unique_ptr<pros::Controller> &controller);

  void init() override;

  void run() override;

  double getAnalog(io::EControllerAnalog channel) override;

  bool getDigital(io::EControllerDigital channel) override;

  bool getNewDigital(io::EControllerDigital channel) override;

  void rumble(std::string pattern) override;
};
} // namespace pros_adapters
} // namespace pvegas
#endif