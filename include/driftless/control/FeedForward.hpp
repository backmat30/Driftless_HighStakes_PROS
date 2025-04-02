#ifndef __FEED_FORWARD_HPP__
#define __FEED_FORWARD_HPP__

#include <cmath>

/// @brief Namespace for driftless library code
/// @author Matthew Backman
namespace driftless {

/// @brief Namespace for control algorithms
/// @author Matthew Backman
namespace control {

/// @brief Class for a basic feed forward controller
/// @author Matthew Backman
class FeedForward {
 private:
  double m_kS{};

  double m_kV{};

  double m_kA{};

 public:
  /// @brief Constructs a new FeedForward controller
  FeedForward() = default;

  /// @brief Constructs a new FeedForward controller
  /// @param kS __double__ The static coefficient
  /// @param kV __double__ The velocity coefficient
  /// @param kA __double__ The acceleration coefficient
  FeedForward(double kS, double kV, double kA);

  /// @brief Copies another FeedForward controller
  /// @param other __const FeedForward&__ The FeedForward controller being
  /// copied
  FeedForward(const FeedForward& other) = default;

  /// @brief Moves a FeedForward controller
  /// @param other __FeedForward&&__ The FeedForward controller being moved
  FeedForward(FeedForward&& other) = default;

  /// @brief Gets the control value of the FeedForward controller
  /// @param target_velocity __double__ The desired velocity
  /// @param target_acceleration __double__ The desired acceleration
  /// @return __double__ The control value
  double getControlValue(double target_velocity, double target_acceleration);

  /// @brief Copies another FeedForward controller
  /// @param rhs __const FeedForward&__ The FeedForward controller being copied
  /// @return __FeedForward&__ A reference to this FeedForward controller
  FeedForward& operator=(const FeedForward& rhs);

  /// @brief Moves a FeedForward controller
  /// @param rhs __FeedForward&&__ The FeedForward controller being moved
  /// @return __FeedForward&__ A reference to this FeedForward controller
  FeedForward& operator=(FeedForward&& rhs) = default;
};
}  // namespace control
}  // namespace driftless
#endif