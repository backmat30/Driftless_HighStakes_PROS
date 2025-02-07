#ifndef __I_PISTON_HPP__
#define __I_PISTON_HPP__

/// @brief Namespace for driftless library code
/// @author Matthew Backman
namespace driftless {

/// @brief Namespace for IO device interfaces
/// @author Matthew Backman
namespace io {

/// @brief Interface for a generic piston
/// @author Matthew Backman
class IPiston {
 public:
  /// @brief Destroys the piston
  virtual ~IPiston() = default;

  /// @brief Extends the piston
  virtual void extend() = 0;

  /// @brief Retracts the piston
  virtual void retract() = 0;

  /// @brief Toggles the state of the piston
  virtual void toggleState() = 0;

  /// @brief Determines if the piston is extended
  /// @return __bool__ True if extended, false if retracted
  virtual bool isExtended() = 0;
};
}  // namespace io
}  // namespace driftless
#endif