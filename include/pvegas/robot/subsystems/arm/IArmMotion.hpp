#ifndef __I_ARM_MOTION_HPP__
#define __I_ARM_MOTION_HPP__

namespace pvegas {
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

  // goes to the scoring position
  virtual void goScore() = 0;

  // determines if the arm is in the neutral position
  virtual bool isAtNeutral() = 0;

  // determines if the arm is in the loading position
  virtual bool isAtLoad() = 0;

  // determines if the arm is in the scoring position
  virtual bool isAtScore() = 0;
};
}  // namespace arm
}  // namespace subsystems
}  // namespace robot
}  // namespace pvegas
#endif