#include "driftless/pros_adapters/ProsDelayer.hpp"

namespace driftless {
namespace pros_adapters {
std::unique_ptr<rtos::IDelayer> ProsDelayer::clone() const {
  return std::unique_ptr<rtos::IDelayer>(std::make_unique<ProsDelayer>(*this));
}

void ProsDelayer::delay(uint32_t millis) { pros::Task::delay(millis); }

void ProsDelayer::delayUntil(uint32_t time) {
  uint32_t currentTime{pros::millis()};
  pros::Task::delay_until(&currentTime, time - currentTime);
}
}  // namespace pros_adapters
}  // namespace driftless