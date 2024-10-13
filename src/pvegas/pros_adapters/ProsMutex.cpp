#include "pvegas/pros_adapters/ProsMutex.hpp"

namespace pvegas {
namespace pros_adapters {

void ProsMutex::take() { mutex.take(); }

void ProsMutex::give() { mutex.give(); }
}  // namespace pros_adapters
}  // namespace pvegas