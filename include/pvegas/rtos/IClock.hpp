#ifndef __I_CLOCK_HPP__
#define __I_CLOCK_HPP__

#include <cstdint>
#include <memory>
namespace pvegas {
namespace rtos {
class IClock {
 public:
  virtual std::unique_ptr<IClock> clone() const = 0;

  virtual uint32_t getTime() = 0;
};
}  // namespace rtos
}  // namespace pvegas
#endif