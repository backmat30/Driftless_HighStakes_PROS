#ifndef __DISTANCE_POSITION_RESETTER_HPP__
#define __DISTANCE_POSITION_RESETTER_HPP__

#include <cmath>
#include <memory>

#include "pvegas/io/IDistanceSensor.hpp"
#include "pvegas/robot/subsystems/odometry/IPositionResetter.hpp"
#include "pvegas/utils/UtilityFunctions.hpp"

namespace driftless {
namespace robot {
namespace subsystems {
namespace odometry {
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
  // initialize the position resetter
  void init() override;

  // run the position resetter
  void run() override;

  // gets the x position after reset
  double getResetX(double theta) override;

  // gets the y position after reset
  double getResetY(double theta) override;

  // gets the raw value from the distance sensor
  double getRawValue() override;

  // changes the distance sensor reference
  void setDistanceSensor(
      std::unique_ptr<driftless::io::IDistanceSensor>& distance_sensor);

  // set the local x offset
  void setLocalX(double local_x);

  // set the local y offset
  void setLocalY(double local_y);

  // set the local angular offset
  void setLocalTheta(double local_theta);
};
}  // namespace odometry
}  // namespace subsystems
}  // namespace robot
}  // namespace pvegas
#endif