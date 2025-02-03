#ifndef __DISTANCE_POSITION_RESETTER_BUILDER_HPP__
#define __DISTANCE_POSITION_RESETTER_BUILDER_HPP__

#include <memory>

#include "driftless/robot/subsystems/odometry/DistancePositionResetter.hpp"

/// @brief The namespace for driftless library code
/// @author Matthew Backman
namespace driftless {

/// @brief The namespace for robot code
/// @author Matthew Backman
namespace robot {

/// @brief The namespace for subsystems code
/// @author Matthew Backman
namespace subsystems {

/// @brief The namespace for odometry code
/// @author Matthew Backman
namespace odometry {

/// @brief Builder class for creating a DistancePositionResetter
/// @author Matthew Backman
class DistancePositionResetterBuilder {
 private:
  // the resetter being built
  std::unique_ptr<driftless::io::IDistanceSensor> m_distance_sensor{};

  // the local X offset
  double m_local_x{};

  // the local y offset
  double m_local_y{};

  // the local angular offset
  double m_local_theta{};

 public:
  /// @brief Adds a distance sensor to the position resetter
  /// @param distance_sensor __std::unique_ptr<driftless::io::IDistanceSensor>&__ The distance sensor to add
  /// @return __DistancePositionResetterBuilder*__ The builder instance
  DistancePositionResetterBuilder* withDistanceSensor(std::unique_ptr<driftless::io::IDistanceSensor>& distance_sensor);

  /// @brief Adds a local x offset to the position resetter
  /// @param local_x __double__ The local x offset to add
  /// @return __DistancePositionResetterBuilder*__ The builder instance
  DistancePositionResetterBuilder* withLocalX(double local_x);

  /// @brief Adds a local y offset to the position resetter
  /// @param local_y __double__ The local y offset to add
  /// @return __DistancePositionResetterBuilder*__ The builder instance
  DistancePositionResetterBuilder* withLocalY(double local_y);

  /// @brief Adds a local angular offset to the position resetter
  /// @param local_theta __double__ The local angular offset to add
  /// @return __DistancePositionResetterBuilder*__ The builder instance
  DistancePositionResetterBuilder* withLocalTheta(double local_theta);

  /// @brief Builds the position resetter
  /// @return __std::unique_ptr<IPositionResetter>__ The built position resetter
  std::unique_ptr<IPositionResetter> build();
};
}  // namespace odometry
}  // namespace subsystems
}  // namespace robot
}  // namespace driftless
#endif