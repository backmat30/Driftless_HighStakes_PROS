#ifndef __DISTANCE_POSITION_RESETTER_HPP__
#define __DISTANCE_POSITION_RESETTER_HPP__

#include <cmath>
#include <memory>

#include "driftless/io/IDistanceSensor.hpp"
#include "driftless/robot/subsystems/odometry/IPositionResetter.hpp"
#include "driftless/utils/UtilityFunctions.hpp"

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

/// @brief Class for resetting position using a distance sensor
/// @author Matthew Backman
class DistancePositionResetter
    : public driftless::robot::subsystems::odometry::IPositionResetter {
 private:
  // coordinate of the alliance-side wall
  static constexpr double NEAR_WALL{0};

  // coordinate of the opposing wall
  static constexpr double FAR_WALL{144};

  // the distance sensor used to reset position
  std::unique_ptr<driftless::io::IDistanceSensor> m_distance_sensor{};

  // local x offset
  double m_local_x{};

  // local y offset
  double m_local_y{};

  // local angle offset
  double m_local_theta{};

 public:
  /// @brief Initializes the position resetter
  void init() override;

  /// @brief Runs the position resetter
  void run() override;

  /// @brief Gets the x position after reset
  /// @param theta __double__ The angle in radians
  /// @return __double__ The x position
  double getResetX(double theta) override;

  /// @brief Gets the y position after reset
  /// @param theta __double__ The angle in radians
  /// @return __double__ The y position
  double getResetY(double theta) override;

  /// @brief Gets the raw value from the distance sensor
  /// @return __double__ The raw distance value
  double getRawValue() override;

  /// @brief Changes the distance sensor reference
  /// @param distance_sensor __std::unique_ptr<driftless::io::IDistanceSensor>&__ The new distance sensor
  void setDistanceSensor(
      std::unique_ptr<driftless::io::IDistanceSensor>& distance_sensor);

  /// @brief Sets the local x offset
  /// @param local_x __double__ The local x offset
  void setLocalX(double local_x);

  /// @brief Sets the local y offset
  /// @param local_y __double__ The local y offset
  void setLocalY(double local_y);

  /// @brief Sets the local angular offset
  /// @param local_theta __double__ The local angular offset
  void setLocalTheta(double local_theta);
};
}  // namespace odometry
}  // namespace subsystems
}  // namespace robot
}  // namespace driftless
#endif