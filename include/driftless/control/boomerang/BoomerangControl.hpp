#ifndef __BOOMERANG_CONTROL_HPP__
#define __BOOMERANG_CONTROL_HPP__

#include "driftless/control/AControl.hpp"
#include "driftless/control/boomerang/IBoomerang.hpp"

/// @brief Namespace for driftless library code
/// @author Matthew Backman
namespace driftless {

/// @brief Namespace for control algorithms
/// @author Matthew Backman
namespace control {

/// @brief Namespace for the boomerang control
/// @author Matthew Backman
namespace boomerang {

/// @brief Class representing the boomerang control
/// @author Matthew Backman
class BoomerangControl : public control::AControl {
 private:
  std::unique_ptr<driftless::control::boomerang::IBoomerang> m_boomerang{};

 public:
  /// @brief Constructs a new boomerang controller
  /// @param boomerang __std::unique_ptr<control::boomerang::IBoomerang>&__ The
  /// boomerang controller to control
  BoomerangControl(
      std::unique_ptr<driftless::control::boomerang::IBoomerang>& boomerang);

  /// @brief Initializes the boomerang control
  void init() override;

  /// @brief Runs the boomerang control
  void run() override;

  /// @brief Pauses the boomerang control
  void pause() override;

  /// @brief Resumes the boomerang control
  void resume() override;

  /// @brief Sends a command to the boomerang control
  /// @param command_name __EControlCommand__ The command being sent
  /// @param args __va_list&__ Potential arguments for the command
  void command(EControlCommand command_name, va_list& args) override;

  /// @brief Gets a state of the boomerang control
  /// @param state_name __EControlState__ The state to get
  /// @return __void*__ The state of the boomerang control
  void* state(EControlState state_name) override;
};
}  // namespace boomerang
}  // namespace control
}  // namespace driftless
#endif