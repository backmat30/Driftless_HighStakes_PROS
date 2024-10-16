#ifndef __I_POSITION_TRACKER_HPP__
#define __I_POSITION_TRACKER_HPP__

#include "pvegas/robot/subsystems/odometry/Position.hpp"
namespace pvegas {
namespace robot {
namespace subsystems {
namespace odometry {
class IPositionTracker {
 public:
  // deleter
  virtual ~IPositionTracker() = default;

  // initialize the position tracker
  virtual void init() = 0;

  // run the position tracker
  virtual void run() = 0;

  // sets the position of the tracking system
  virtual void setPosition(Position position) = 0;

  // get the position of the robot
  virtual Position getPosition() = 0;

  // set the x position
  virtual void setX(double x) = 0;

  // set the y position
  virtual void setY(double y) = 0;

  // set the angle
  virtual void setTheta(double theta) = 0;
};
}  // namespace odometry
}  // namespace subsystems
}  // namespace robot
}  // namespace pvegas
#endif