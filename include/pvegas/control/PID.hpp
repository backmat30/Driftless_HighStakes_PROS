#ifndef __PID_HPP__
#define __PID_HPP__

#include <memory>

#include "pvegas/rtos/IClock.hpp"

namespace pvegas {
namespace control {
class PID {
 private:
  // system clock
  std::unique_ptr<pvegas::rtos::IClock> m_clock{};

  // proportional coefficient
  double m_kp{};

  // integral coefficient
  double m_ki{};

  // derivative coefficient
  double m_kd{};

  // the accumulated error
  double accumulated_error{};

  // the latest error value
  double last_error{};

  // the timestamp of the latest action
  double last_time{};

 public:
  // default constructor
  PID() = default;

  // constructor
  PID(const std::unique_ptr<pvegas::rtos::IClock>& clock, double kp, double ki,
      double kd);

  // copy constructor
  PID(const PID& other);

  // move constructor
  PID(PID&& other) = default;

  // gets the control value from the PID controller
  double getControlValue(double current, double target);

  // reset the PID controller
  void reset();

  // copy assignment
  PID& operator=(const PID& rhs);

  // default move assignment
  PID& operator=(PID&& rhs);
};
}  // namespace control
}  // namespace pvegas
#endif