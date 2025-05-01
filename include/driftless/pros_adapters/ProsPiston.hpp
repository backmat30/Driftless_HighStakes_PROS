#ifndef __PROS_PISTON_HPP__
#define __PROS_PISTON_HPP__

#include <memory>

#include "driftless/io/IPiston.hpp"
#include "pros/adi.hpp"

/// @brief The namespace for driftless library code
/// @author Matthew Backman
namespace driftless {

/// @brief The namespace for PROS adapters
/// @author Matthew Backman
namespace pros_adapters {

/// @brief The class for adapting a PROS piston
/// @author Matthew Backman
class ProsPiston : public driftless::io::IPiston {
 private:
  // the ADI port used by the piston
  std::unique_ptr<pros::adi::DigitalOut> m_adi_digital_out{};

  bool m_extended_value{};

  // whether the piston is extended or retracted
  bool extended{};

 public:
  /// @brief Constructs a new ProsPiston object
  /// @param adi_digital_out __std::unique_ptr<pros::adi::DigitalOut>&__ The ADI
  /// port used by the piston
  ProsPiston(std::unique_ptr<pros::adi::DigitalOut>& adi_digital_out, bool extended_value=true);

  /// @brief Extends the piston
  void extend() override;

  /// @brief Retracts the piston
  void retract() override;

  /// @brief Toggles the state of the piston
  void toggleState() override;

  /// @brief Determines if the piston is extended
  /// @return __bool__ True if extended, false otherwise
  bool isExtended() override;
};
}  // namespace pros_adapters
}  // namespace driftless
#endif