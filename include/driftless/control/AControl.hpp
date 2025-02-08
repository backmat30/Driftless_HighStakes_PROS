#ifndef __A_CONTROL_HPP__
#define __A_CONTROL_HPP__

#include <string>

#include "driftless/control/EControl.hpp"
#include "driftless/control/EControlCommand.hpp"
#include "driftless/control/EControlState.hpp"

/// @brief Namespace for driftless library code
/// @author Matthew Backman
namespace driftless {

/// @brief Namespace for control algorithms
/// @author Matthew Backman
namespace control {

/// @brief Abstract class for a generic control
/// @author Matthew Backman
class AControl {
 private:
  EControl m_name{};

 public:
  /// @brief Constructs a new control
  AControl() = default;

  /// @brief Constructs a new control
  /// @param other __const AControl&__ The control being copied
  AControl(const AControl &other) = default;

  /// @brief Constructs a new control
  /// @param other __AControl&&__ The control being moved
  AControl(AControl &&other) = default;

  /// @brief Constructs a new control
  /// @param name __EControl__ The name of the control
  AControl(EControl name) : m_name{name} {}

  /// @brief Destroys the control
  virtual ~AControl() = default;

  /// @brief Gets the name of the control
  /// @return __EControl&__ The name of the control
  const EControl &getName() const { return m_name; }

  /// @brief Initializes the control
  virtual void init() = 0;

  /// @brief Runs the control
  virtual void run() = 0;

  /// @brief Pauses the control
  virtual void pause() = 0;

  /// @brief Resumes the control
  virtual void resume() = 0;

  /// @brief Runs a command
  /// @param command_name __EControlCommand__ The command to run
  /// @param args __va_list&__ Any potential arguements for the command
  virtual void command(EControlCommand command_name, va_list &args) = 0;

  /// @brief Gets a state of the control
  /// @param state_name __EControlState__ The desired state
  /// @return __void*__ The state of the control
  virtual void *state(EControlState state_name) = 0;

  /// @brief Copies another control
  /// @param rhs __const AControl&__ The control being copied
  /// @return __AControl&__ Reference to the new control
  AControl &operator=(const AControl &rhs) = default;

  /// @brief Moves another control
  /// @param rhs __AControl&&__ The control being moved
  /// @return __AControl&__ Reference to the new control
  AControl &operator=(AControl &&rhs) = default;
};
}  // namespace control
}  // namespace driftless
#endif