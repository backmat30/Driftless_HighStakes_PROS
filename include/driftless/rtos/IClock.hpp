#ifndef __I_CLOCK_HPP__
#define __I_CLOCK_HPP__

#include <cstdint>
#include <memory>

/// @brief The namespace for driftless library code
/// @author Matthew Backman
namespace driftless {

/// @brief The namespace for real-time operating system code
/// @author Matthew Backman
namespace rtos {

/// @brief Interface for a clock object
/// @author Matthew Backman
class IClock {
 public:
  /// @brief Deletes the clock object
  virtual ~IClock() = default;

  /// @brief Clones the clock object
  /// @return __unique_ptr<IClock>__ A unique pointer to a new clock object
  virtual std::unique_ptr<IClock> clone() const = 0;

  /// @brief Gets the current time
  /// @return __uint32_t__ The current time in milliseconds
  virtual uint32_t getTime() = 0;
};
}  // namespace rtos
}  // namespace driftless
#endif