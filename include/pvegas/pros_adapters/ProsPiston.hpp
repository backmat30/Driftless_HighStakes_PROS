#ifndef __PROS_PISTON_HPP__
#define __PROS_PISTON_HPP__

#include <memory>

#include "pros/adi.hpp"
#include "pvegas/io/IPiston.hpp"

namespace pvegas {
namespace pros_adapters {
class ProsPiston : public pvegas::io::IPiston {
 private:
  // the ADI port used by the piston
  std::unique_ptr<pros::ADIDigitalOut> m_adi_digital_out{};

  // whether the piston is extended or retracted
  bool extended{};

 public:
  // constructs a new pros piston object
  ProsPiston(std::unique_ptr<pros::ADIDigitalOut>& adi_digital_out);

  // extends the piston
  void extend() override;

  // retracts the piston
  void retract() override;

  // toggles the state of the piston
  void toggleState() override;

  // determines if the piston is extended
  bool isExtended() override;
};
}  // namespace pros_adapters
}  // namespace pvegas
#endif