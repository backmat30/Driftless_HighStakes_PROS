#ifndef __I_DRIVE_STRAIGHT_HPP__
#define __I_DRIVE_STRAIGHT_HPP__

#include <memory>

#include "pvegas/robot/Robot.hpp"

namespace driftless {
namespace control {
namespace motion {
class IDriveStraight {
 public:
  // destroyer
  virtual ~IDriveStraight() = default;

  // initialize the control
  virtual void init() = 0;

  // run the control
  virtual void run() = 0;

  // pause the control
  virtual void pause() = 0;

  // resume the control
  virtual void resume() = 0;

  // tell the robot to drive straight for a given distance in inches
  virtual void driveStraight(std::shared_ptr<driftless::robot::Robot>& robot,
                             double velocity, double distance,
                             double theta) = 0;

  // set the velocity to run the control at
  virtual void setVelocity(double velocity) = 0;

  // return if the robot has reached the target
  virtual bool targetReached() = 0;
};
}  // namespace motion
}  // namespace control
}  // namespace pvegas
#endif