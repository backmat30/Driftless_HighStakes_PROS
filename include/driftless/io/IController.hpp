#ifndef __I_CONTROLLER_HPP__
#define __I_CONTROLLER_HPP__

#include <string>

#include "driftless/op_control/EControllerAnalog.hpp"
#include "driftless/op_control/EControllerDigital.hpp"
namespace driftless {
namespace io {
class IController {
 public:
  virtual ~IController() = default;

  virtual void init() = 0;

  virtual void run() = 0;

  virtual double getAnalog(op_control::EControllerAnalog channel) = 0;

  virtual bool getDigital(op_control::EControllerDigital channel) = 0;

  virtual bool getNewDigital(op_control::EControllerDigital channel) = 0;

  virtual void rumble(std::string pattern) = 0;
};
}  // namespace io
}  // namespace driftless
#endif