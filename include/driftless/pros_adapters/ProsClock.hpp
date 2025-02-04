#ifndef __PROS_CLOCK_HPP__
#define __PROS_CLOCK_HPP__

#include <cstdint>
#include <memory>

#include "driftless/rtos/IClock.hpp"
#include "pros/rtos.hpp"

/// @brief The namespace for driftless library code
/// @author Matthew Backman
namespace driftless {

/// @brief The namespace for PROS adapters
/// @author Matthew Backman
namespace pros_adapters {

/// @brief Adapter class for the PROS clock
/// @author Matthew Backman
class ProsClock : public rtos::IClock {
 public:
  /// @brief Clones the clock
  /// @return __std::unique_ptr<rtos::IClock>__ The cloned clock
  std::unique_ptr<rtos::IClock> clone() const override;

  /// @brief Gets the current time
  /// @return __uint32_t__ The current time
  uint32_t getTime() override;
};
}  // namespace pros_adapters
}  // namespace driftless

#endif