#ifndef __I_ARM_MOTION_HPP__
#define __I_ARM_MOTION_HPP__

namespace driftless {
namespace robot {
namespace subsystems {
namespace arm {
class IArmMotion {
 public:
  // destroys the object
  virtual ~IArmMotion() = default;

  // initializes the arm motion controller
  virtual void init() = 0;

  // runs the arm motion controller
  virtual void run() = 0;

  // goes to the neutral position
  virtual void goNeutral() = 0;

  // goes to the loading position
  virtual void goLoad() = 0;

  // goes to the ready position
  virtual void goReady() = 0;

  // goes to the score position
  virtual void goScore() = 0;

  // goes to the rush position
  virtual void goRush() = 0;

  // determines if the arm is in the neutral position
  virtual bool isAtNeutral() = 0;

  // determines if the arm is going to the neutral position
  virtual bool isGoingNeutral() = 0;

  // determines if the arm is in the loading position
  virtual bool isAtLoad() = 0;

  // determines if the arm is going to the loading position
  virtual bool isGoingLoad() = 0;

  // determines if the arm is in the ready position
  virtual bool isAtReady() = 0;

  // determines if the arm is going to the ready position
  virtual bool isGoingReady() = 0;

  // determines if the arm is in the score position
  virtual bool isAtScore() = 0;

  // determines if the arm is going to the score position
  virtual bool isGoingScore() = 0;

  // determines if the arm is at the rush position
  virtual bool isAtRush() = 0;

  // determines if the arm is going to the rush position
  virtual bool isGoingRush() = 0;
};
}  // namespace arm
}  // namespace subsystems
}  // namespace robot
}  // namespace driftless
#endif