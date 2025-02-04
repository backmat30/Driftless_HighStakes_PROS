#ifndef __PROS_CONTROLLER_HPP__
#define __PROS_CONTROLLER_HPP__

#include <cstdint>
#include <map>
#include <memory>

#include "driftless/io/IController.hpp"
#include "pros/misc.hpp"
#include "pros/rtos.hpp"

/// @brief The namespace for driftless library code
/// @author Matthew Backman
namespace driftless {

/// @brief The namespace for PROS adapters
/// @author Matthew Backman
namespace pros_adapters {

/// @brief Adapter class for the PROS controller
/// @author Matthew Backman
class ProsController : public io::IController {
 private:
  static constexpr uint8_t TASK_DELAY{10};

  static constexpr uint8_t RUMBLE_REFRESH_RATE{50};

  static constexpr uint8_t MAX_RUMBLE_LENGTH{8};

  static constexpr double ANALOG_CONVERSION{1.0 / 127};

  static void taskLoop(void *params);

  const std::map<op_control::EControllerAnalog, pros::controller_analog_e_t>
      ANALOGUE_MAP{{op_control::EControllerAnalog::JOYSTICK_LEFT_X,
                    pros::E_CONTROLLER_ANALOG_LEFT_X},
                   {op_control::EControllerAnalog::JOYSTICK_LEFT_Y,
                    pros::E_CONTROLLER_ANALOG_LEFT_Y},
                   {op_control::EControllerAnalog::JOYSTICK_RIGHT_X,
                    pros::E_CONTROLLER_ANALOG_RIGHT_X},
                   {op_control::EControllerAnalog::JOYSTICK_RIGHT_Y,
                    pros::E_CONTROLLER_ANALOG_RIGHT_Y}};

  const std::map<op_control::EControllerDigital, pros::controller_digital_e_t>
      DIGITAL_MAP{{op_control::EControllerDigital::BUTTON_A,
                   pros::E_CONTROLLER_DIGITAL_A},
                  {op_control::EControllerDigital::BUTTON_B,
                   pros::E_CONTROLLER_DIGITAL_B},
                  {op_control::EControllerDigital::BUTTON_X,
                   pros::E_CONTROLLER_DIGITAL_X},
                  {op_control::EControllerDigital::BUTTON_Y,
                   pros::E_CONTROLLER_DIGITAL_Y},
                  {op_control::EControllerDigital::DPAD_DOWN,
                   pros::E_CONTROLLER_DIGITAL_DOWN},
                  {op_control::EControllerDigital::DPAD_LEFT,
                   pros::E_CONTROLLER_DIGITAL_LEFT},
                  {op_control::EControllerDigital::DPAD_RIGHT,
                   pros::E_CONTROLLER_DIGITAL_RIGHT},
                  {op_control::EControllerDigital::DPAD_UP,
                   pros::E_CONTROLLER_DIGITAL_UP},
                  {op_control::EControllerDigital::TRIGGER_LEFT_BOTTOM,
                   pros::E_CONTROLLER_DIGITAL_L2},
                  {op_control::EControllerDigital::TRIGGER_LEFT_TOP,
                   pros::E_CONTROLLER_DIGITAL_L1},
                  {op_control::EControllerDigital::TRIGGER_RIGHT_BOTTOM,
                   pros::E_CONTROLLER_DIGITAL_R2},
                  {op_control::EControllerDigital::TRIGGER_RIGHT_TOP,
                   pros::E_CONTROLLER_DIGITAL_R1}};

  std::unique_ptr<pros::Controller> m_controller{};

  pros::Mutex mutex{};

  char rumble_pattern[MAX_RUMBLE_LENGTH]{};

  bool new_rumble_pattern{};

  uint32_t last_rumble_refresh{};

  void updateRumble();

  void taskUpdate();

 public:
  /// @brief Constructor
  /// @param controller __std::unique_ptr<pros::Controller>&__ The controller to
  /// adapt
  ProsController(std::unique_ptr<pros::Controller> &controller);

  /// @brief Initializes the controller
  void init() override;

  /// @brief Runs the controller
  void run() override;

  /// @brief Gets the analog value from the controller
  /// @param channel __op_control::EControllerAnalog__ The analog channel
  /// @return __double__ The analog value
  double getAnalog(op_control::EControllerAnalog channel) override;

  /// @brief Gets the digital value from the controller
  /// @param channel __op_control::EControllerDigital__ The digital channel
  /// @return __bool__ The digital value
  bool getDigital(op_control::EControllerDigital channel) override;

  /// @brief Gets the new digital value from the controller
  /// @param channel __op_control::EControllerDigital__ The digital channel
  /// @return __bool__ The new digital value
  bool getNewDigital(op_control::EControllerDigital channel) override;

  /// @brief Sets the rumble pattern for the controller
  /// @param pattern __std::string__ The rumble pattern
  void rumble(std::string pattern) override;
};
}  // namespace pros_adapters
}  // namespace driftless
#endif