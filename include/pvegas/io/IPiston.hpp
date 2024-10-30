#ifndef __I_PISTON_HPP__
#define __I_PISTON_HPP__

namespace pvegas {
namespace io {
class IPiston {
 public:
  // destroyer
  virtual ~IPiston() = default;

  // extend the piston
  virtual void extend() = 0;

  // retract the piston
  virtual void retract() = 0;

  // toggle the piston between states
  virtual void toggleState() = 0;

  // finds if the piston is extended or retracted
  virtual bool isExtended() = 0;
};
}  // namespace io
}  // namespace pvegas
#endif