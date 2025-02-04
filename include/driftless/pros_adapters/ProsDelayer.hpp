#ifndef __PROS_DELAYER_HPP__
#define __PROS_DELAYER_HPP__

#include <cstdint>
#include <memory>

#include "driftless/rtos/IDelayer.hpp"
#include "pros/rtos.hpp"

/// @brief The namespace for driftless library code
/// @author Matthew Backman
namespace driftless {

/// @brief The namespace for PROS adapters
/// @author Matthew Backman
namespace pros_adapters {

/// @brief Adapter class for the PROS delayer
/// @author Matthew Backman
class ProsDelayer : public rtos::IDelayer {
 public:
  /// @brief Clones the delayer
  /// @return __std::unique_ptr<rtos::IDelayer>__ The cloned delayer
  std::unique_ptr<rtos::IDelayer> clone() const override;

  /// @brief Delays for a specified amount of time
  /// @param millis __uint32_t__ The delay time in milliseconds
  void delay(uint32_t millis) override;

  /// @brief Delays until a specified time
  /// @param time __uint32_t__ The time to delay until
  void delayUntil(uint32_t time) override;
};
}  // namespace pros_adapters
}  // namespace driftless
#endif