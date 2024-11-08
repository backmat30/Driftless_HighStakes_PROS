#include "driftless/pros_adapters/ProsMutex.hpp"

namespace driftless {
namespace pros_adapters {

void ProsMutex::take() { mutex.take(); }

void ProsMutex::give() { mutex.give(); }
}  // namespace pros_adapters
}  // namespace driftless