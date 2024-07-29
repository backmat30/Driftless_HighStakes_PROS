#include "pvegas/pros_adapters/ProsClock.hpp"

namespace pvegas {
namespace pros_adapters {
std::unique_ptr<ProsClock> ProsClock::clone() const {
  return std::unique_ptr<ProsClock>(std::make_unique<ProsClock>(*this));
}

uint32_t ProsClock::getTime() { return pros::millis(); }
} // namespace pros_adapters
} // namespace pvegas