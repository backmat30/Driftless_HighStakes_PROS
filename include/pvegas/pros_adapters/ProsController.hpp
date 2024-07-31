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

  const std::map<op_control::EControllerAnalog, pros::controller_analog_e_t> ANALOGUE_MAP{
      {op_control::EControllerAnalog::JOYSTICK_LEFT_X, pros::E_CONTROLLER_ANALOG_LEFT_X},
      {op_control::EControllerAnalog::JOYSTICK_LEFT_Y, pros::E_CONTROLLER_ANALOG_LEFT_Y},
      {op_control::EControllerAnalog::JOYSTICK_RIGHT_X, pros::E_CONTROLLER_ANALOG_RIGHT_X},
      {op_control::EControllerAnalog::JOYSTICK_RIGHT_Y, pros::E_CONTROLLER_ANALOG_RIGHT_Y}};

  const std::map<op_control::EControllerDigital, pros::controller_digital_e_t> DIGITAL_MAP{
      {op_control::EControllerDigital::BUTTON_A, pros::E_CONTROLLER_DIGITAL_A},
      {op_control::EControllerDigital::BUTTON_B, pros::E_CONTROLLER_DIGITAL_B},
      {op_control::EControllerDigital::BUTTON_X, pros::E_CONTROLLER_DIGITAL_X},
      {op_control::EControllerDigital::BUTTON_Y, pros::E_CONTROLLER_DIGITAL_Y},
      {op_control::EControllerDigital::DPAD_DOWN, pros::E_CONTROLLER_DIGITAL_DOWN},
      {op_control::EControllerDigital::DPAD_LEFT, pros::E_CONTROLLER_DIGITAL_LEFT},
      {op_control::EControllerDigital::DPAD_RIGHT, pros::E_CONTROLLER_DIGITAL_RIGHT},
      {op_control::EControllerDigital::DPAD_UP, pros::E_CONTROLLER_DIGITAL_UP},
      {op_control::EControllerDigital::TRIGGER_LEFT_BOTTOM, pros::E_CONTROLLER_DIGITAL_L2},
      {op_control::EControllerDigital::TRIGGER_LEFT_TOP, pros::E_CONTROLLER_DIGITAL_L1},
      {op_control::EControllerDigital::TRIGGER_RIGHT_BOTTOM, pros::E_CONTROLLER_DIGITAL_R2},
      {op_control::EControllerDigital::TRIGGER_RIGHT_TOP, pros::E_CONTROLLER_DIGITAL_R1}};

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

  double getAnalog(op_control::EControllerAnalog channel) override;

  bool getDigital(op_control::EControllerDigital channel) override;

  bool getNewDigital(op_control::EControllerDigital channel) override;

  void rumble(std::string pattern) override;
};
} // namespace pros_adapters
} // namespace pvegas
#endif