#ifndef __I_DELAYER_HPP__
#define __I_DELAYER_HPP__

#include <memory>
/// @brief The namespace for driftless library code
/// @author Matthew Backman
namespace driftless {

/// @brief The namespace for real-time operating system code
/// @author Matthew Backman
namespace rtos {

/// @brief Interface for a delayer object
/// @author Matthew Backman
class IDelayer {
 public:
  /// @brief Deletes the delayer object
  virtual ~IDelayer() = default;
  
  /// @brief Clones the delayer object
  /// @return __unique_ptr<IDelayer>__ A unique pointer to a new delayer object
  virtual std::unique_ptr<IDelayer> clone() const = 0;

  /// @brief Delays for a specified number of milliseconds
  /// @param millis __uint32_t__ The number of milliseconds to delay
  virtual void delay(uint32_t millis) = 0;

  /// @brief Delays until a specified time
  /// @param time __uint32_t__ The time, im milliseconds, to delay until
  virtual void delayUntil(uint32_t time) = 0;
};
}  // namespace rtos
}  // namespace driftless
#endif