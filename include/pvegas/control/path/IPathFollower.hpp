#ifndef __I_PATH_FOLLOWER_HPP__
#define __I_PATH_FOLLOWER_HPP__

#include <memory>
#include <vector>

#include "pvegas/control/Point.hpp"
#include "pvegas/robot/Robot.hpp"

namespace pvegas {
namespace control {
namespace path {
class IPathFollower {
 public:
  // destroyer
  virtual ~IPathFollower() = default;

  // initialize the path follower
  virtual void init() = 0;

  // run the path follower
  virtual void run() = 0;

  // pause the algorithm
  virtual void pause() = 0;

  // resume the algorithm
  virtual void resume() = 0;

  // follow a given path
  virtual void followPath(const std::shared_ptr<pvegas::robot::Robot>& robot,
                          const std::vector<Point>& path, double velocity) = 0;

  // set the velocity to follow the path at
  virtual void setVelocity(double velocity) = 0;

  // check if the robot has reached the target
  virtual bool targetReached() = 0;
};
}  // namespace path
}  // namespace control
}  // namespace pvegas
#endif