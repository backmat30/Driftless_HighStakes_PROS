#ifndef __I_HEIGHT_CONTROL_HPP__
#define __I_HEIGHT_CONTROL_HPP__

namespace pvegas {
namespace robot {
namespace subsystems {
namespace intake {
class IHeightControl {
 public:
  // destroys the height control object
  virtual ~IHeightControl() = default;

  // initializes the height controller
  virtual void init() = 0;

  // runs the height controller
  virtual void run() = 0;

  // set the height of the intake
  virtual void setHeight(bool up) = 0;

  // get the position of the intake
  virtual bool isRaised() = 0;
};
}  // namespace intake
}  // namespace subsystems
}  // namespace robot
}  // namespace pvegas
#endif