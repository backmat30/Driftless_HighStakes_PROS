#ifndef __I_CONTROLLER_HPP__
#define __I_CONTROLLER_HPP__

#include <string>

#include "driftless/op_control/EControllerAnalog.hpp"
#include "driftless/op_control/EControllerDigital.hpp"

/// @brief Namespace for driftless library code
/// @author Matthew Backman
namespace driftless {

/// @brief Namespace for IO device interfaces
/// @author Matthew Backman
namespace io {

/// @brief Interface for a generic controller
/// @author Matthew Backman
class IController {
 public:
  /// @brief Destroys the controller
  virtual ~IController() = default;

  /// @brief Initializes the controller
  virtual void init() = 0;

  /// @brief Runs the controller
  virtual void run() = 0;

  /// @brief Gets the reading from an analog channel
  /// @param channel __op_control::EControllerAnalog__ The channel to read from
  /// @return __double__ The reading from the analog channel, -127 to 127
  virtual double getAnalog(op_control::EControllerAnalog channel) = 0;

  /// @brief Gets the reading from a digital channel
  /// @param channel __op_control::EControllerDigital__ The channel to read from
  /// @return __bool__ True if there is a signal from the channel, false if none
  virtual bool getDigital(op_control::EControllerDigital channel) = 0;

  /// @brief Determines if there is a new signal from a digital channel
  /// @param channel __op_control::EControllerDigital__ The channel to read from
  /// @return __bool__ True if there is a new signal, false if no signal or
  /// preexisting signal
  virtual bool getNewDigital(op_control::EControllerDigital channel) = 0;

  /// @brief Rumbles the controller using a pattern
  /// @details signal using . for a short burst, and a - for a long burst
  /// @param pattern __std::string__ The rumble pattern to use
  virtual void rumble(std::string pattern) = 0;
};
}  // namespace io
}  // namespace driftless
#endif