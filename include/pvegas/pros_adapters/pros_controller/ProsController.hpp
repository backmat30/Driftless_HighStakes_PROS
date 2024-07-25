#ifndef __PROS_CONTROLLER_HPP__
#define __PROS_CONTROLLER_HPP__

#include "EControllerAnalog.hpp"
#include "pros/misc.h"
#include "pros/misc.hpp"
#include "pros/rtos.hpp"
#include "pvegas/pros_adapters/pros_controller/EControllerAnalog.hpp"
#include "pvegas/pros_adapters/pros_controller/EControllerDigital.hpp"
#include <cstdint>
#include <map>
#include <memory>

namespace pvegas {
namespace pros_adapters {
namespace pros_controller {
class ProsController {
private:
  static constexpr uint8_t TASK_DELAY{10};

  static constexpr uint8_t RUMBLE_REFRESH_RATE{50};

  static constexpr uint8_t MAX_RUMBLE_LENGTH{8};

  static constexpr double ANALOG_CONVERSION{1.0 / 127};

  static void taskLoop(void* params);

  const std::map<EControllerAnalog, pros::controller_analog_e_t> ANALOGUE_MAP{
      {EControllerAnalog::JOYSTICK_LEFT_X, pros::E_CONTROLLER_ANALOG_LEFT_X},
      {EControllerAnalog::JOYSTICK_LEFT_Y, pros::E_CONTROLLER_ANALOG_LEFT_Y},
      {EControllerAnalog::JOYSTICK_RIGHT_X, pros::E_CONTROLLER_ANALOG_RIGHT_X},
      {EControllerAnalog::JOYSTICK_RIGHT_Y, pros::E_CONTROLLER_ANALOG_RIGHT_Y}};

  const std::map<EControllerDigital, pros::controller_digital_e_t> DIGITAL_MAP{
      {EControllerDigital::BUTTON_A, pros::E_CONTROLLER_DIGITAL_A},
      {EControllerDigital::BUTTON_B, pros::E_CONTROLLER_DIGITAL_B},
      {EControllerDigital::BUTTON_X, pros::E_CONTROLLER_DIGITAL_X},
      {EControllerDigital::BUTTON_Y, pros::E_CONTROLLER_DIGITAL_Y},
      {EControllerDigital::DPAD_DOWN, pros::E_CONTROLLER_DIGITAL_DOWN},
      {EControllerDigital::DPAD_LEFT, pros::E_CONTROLLER_DIGITAL_LEFT},
      {EControllerDigital::DPAD_RIGHT, pros::E_CONTROLLER_DIGITAL_RIGHT},
      {EControllerDigital::DPAD_UP, pros::E_CONTROLLER_DIGITAL_UP},
      {EControllerDigital::TRIGGER_LEFT_BOTTOM, pros::E_CONTROLLER_DIGITAL_L2},
      {EControllerDigital::TRIGGER_LEFT_TOP, pros::E_CONTROLLER_DIGITAL_L1},
      {EControllerDigital::TRIGGER_RIGHT_BOTTOM, pros::E_CONTROLLER_DIGITAL_R2},
      {EControllerDigital::TRIGGER_RIGHT_TOP, pros::E_CONTROLLER_DIGITAL_R1}};
    
    std::unique_ptr<pros::Controller> m_controller{};

    pros::Mutex mutex{};

    char rumble_pattern[MAX_RUMBLE_LENGTH]{};

    bool new_rumble_pattern{};

    uint32_t last_rumble_refresh{};

    void updateRumble();

    void taskUpdate();

    public:
    ProsController(std::unique_ptr<pros::Controller>& controller);

    void init();

    void run();

    double getAnalog(EControllerAnalog channel);

    bool getDigital(EControllerDigital channel);

    bool getNewDigital(EControllerDigital channel);

    void rumble(std::string pattern);
};
} // namespace pros_controller
} // namespace pros_adapters
} // namespace pvegas
#endif