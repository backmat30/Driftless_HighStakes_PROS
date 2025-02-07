#ifndef __I_DISTANCE_TRACKER_HPP__
#define __I_DISTANCE_TRACKER_HPP__

/// @brief Namespace for driftless library code
/// @author Matthew Backman
namespace driftless {

/// @brief Namespace for IO device interfaces
/// @author Matthew Backman
namespace io {

/// @brief Interface for a generic distance tracker
/// @author Matthew Backman
class IDistanceTracker {
 public:
  /// @brief Destroys the distance tracker
  virtual ~IDistanceTracker() = default;

  /// @brief Initializes the distance tracker
  virtual void init() = 0;

  /// @brief Resets the distance tracker
  virtual void reset() = 0;

  /// @brief Gets the distance, in inches, from the distance tracker
  /// @return __double__ The distance
  virtual double getDistance() = 0;

  /// @brief Sets the distance of the distance tracker
  /// @param distance __double__ The new distance
  virtual void setDistance(double distance) = 0;
};
}  // namespace io
}  // namespace driftless
#endif