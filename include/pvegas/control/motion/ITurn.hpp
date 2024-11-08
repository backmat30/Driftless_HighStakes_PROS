#ifndef __I_TURN_HPP__
#define __I_TURN_HPP__

#include <memory>

#include "pvegas/control/Point.hpp"
#include "pvegas/control/motion/ETurnDirection.hpp"
#include "pvegas/robot/Robot.hpp"

namespace driftless {
namespace control {
namespace motion {
class ITurn {
 public:
  // destroyer
  virtual ~ITurn() = default;

  // initialize the control
  virtual void init() = 0;

  // run the control
  virtual void run() = 0;

  // pause the control
  virtual void pause() = 0;

  // resume the control
  virtual void resume() = 0;

  // tell the robot to turn in to a given angle
  virtual void turnToAngle(const std::shared_ptr<driftless::robot::Robot>& robot,
                           double velocity, double theta,
                           ETurnDirection direction = ETurnDirection::AUTO) = 0;

  // tell the robot to turn towards a point on the field
  virtual void turnToPoint(const std::shared_ptr<driftless::robot::Robot>& robot,
                           double velocity, Point point,
                           ETurnDirection direction = ETurnDirection::AUTO) = 0;

  // determines if the robot has reached the target
  virtual bool targetReached() = 0;
};
}  // namespace motion
}  // namespace control
}  // namespace pvegas
#endif