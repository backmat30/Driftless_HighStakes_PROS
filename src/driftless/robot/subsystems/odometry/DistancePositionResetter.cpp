#include "driftless/robot/subsystems/odometry/DistancePositionResetter.hpp"

namespace driftless {
namespace robot {
namespace subsystems {
namespace odometry {
void DistancePositionResetter::init() {
  if (m_distance_sensor) {
    m_distance_sensor->init();
  }
}

void DistancePositionResetter::run() {}

double DistancePositionResetter::getResetX(double theta) {
  double reset_x{};
  if (m_distance_sensor) {
    theta = bindRadians(theta);
    double distance{m_distance_sensor->getDistance()};
    double wall_theta{bindRadians(theta + m_local_theta)};
    bool facing_near_wall{std::abs(wall_theta) > M_PI / 2};
    // if the robot is facing the near wall, run calculations using the data
    // from the alliance-side wall
    if (facing_near_wall) {
      theta = bindRadians(theta + M_PI);
      wall_theta = bindRadians(wall_theta + M_PI);
    }
    double sensor_x_distance(distance * std::cos(wall_theta));
    // the distance added from the local x offset of the sensor on the robot
    double local_x_distance{m_local_x * std::cos(theta)};
    // the distance added from the local y offset of the sensor on the robot
    double local_y_distance{m_local_y * std::cos(theta + (M_PI / 2))};
    // distance from robot center to the wall
    double wall_distance{sensor_x_distance + local_x_distance +
                         local_y_distance};
    if (facing_near_wall) {
      reset_x = NEAR_WALL + wall_distance;
    } else {
      reset_x = FAR_WALL - wall_distance;
    }
  }
  return reset_x;
}

double DistancePositionResetter::getResetY(double theta) {
  // rotate theta 90 degrees to flip sin/cos
  theta -= (M_PI / 2);
  return getResetX(theta);
}

double DistancePositionResetter::getRawValue() {
  double raw_value{};
  if (m_distance_sensor) {
    raw_value = m_distance_sensor->getDistance();
  }
  return raw_value;
}

void DistancePositionResetter::setDistanceSensor(
    std::unique_ptr<driftless::io::IDistanceSensor>& distance_sensor) {
  if (distance_sensor) {
    m_distance_sensor = std::move(distance_sensor);
  }
}

void DistancePositionResetter::setLocalX(double local_x) {
  m_local_x = local_x;
}

void DistancePositionResetter::setLocalY(double local_y) {
  m_local_y = local_y;
}

void DistancePositionResetter::setLocalTheta(double local_theta) {
  m_local_theta = local_theta;
}
}  // namespace odometry
}  // namespace subsystems
}  // namespace robot
}  // namespace driftless