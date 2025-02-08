#ifndef __PID_HPP__
#define __PID_HPP__

#include <memory>

#include "driftless/rtos/IClock.hpp"

/// @brief Namespace for driftless library code
/// @author Matthew Backman
namespace driftless {

/// @brief Namespace for control algorithms
/// @author Matthew Backman
namespace control {

/// @brief Class for a basic PID controller
/// @author Matthew Backman
class PID {
 private:
  // system clock
  std::unique_ptr<driftless::rtos::IClock> m_clock{};

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
  /// @brief Constructs a new PID controller
  PID() = default;

  /// @brief Constructs a new PID controller
  /// @param clock __std::unique_ptr<rtos::IClock>&__ The system clock used
  /// @param kp __double__ The proportional coefficient
  /// @param ki __double__ The integral coefficient
  /// @param kd __double__ The derivitive coefficient
  PID(const std::unique_ptr<driftless::rtos::IClock>& clock, double kp,
      double ki, double kd);

  /// @brief Copies another PID controller
  /// @param other __const PID&__ The PID controller being copied
  PID(const PID& other);

  /// @brief Moves a PID controller
  /// @param other __PID&&__ The PID controller being moved
  PID(PID&& other) = default;

  /// @brief Gets the control value of the PID controller
  /// @param current __double__ The current value
  /// @param target __double__ The target value
  /// @return __double__ The control value
  double getControlValue(double current, double target);

  /// @brief Resets the PID controller
  void reset();

  /// @brief Copies another PID controller
  /// @param rhs __const PID&__ The PID controller being copied
  /// @return __PID&__ Reference to the new PID controller
  PID& operator=(const PID& rhs);

  /// @brief Moves another PID controller
  /// @param rhs __PID&&__ The PID controller being moved
  /// @return __PID&__ Reference to the new PID controller
  PID& operator=(PID&& rhs);
};
}  // namespace control
}  // namespace driftless
#endif